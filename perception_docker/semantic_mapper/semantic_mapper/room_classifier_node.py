#!/usr/bin/env python3
"""
room_classifier_node.py — objects per room → LLM → room type label.

Subscribes
----------
/scene_graph  (std_msgs/String, JSON from semantic_mapper_node)
    Each room in the graph carries its objects list:
        {"id": 3, "objects": [{"class": "bed", ...}, {"class": "chair", ...}]}

Publishes
---------
/semantic_mapper/room_labels  (std_msgs/String, JSON)
    {
      "stamp": 1731427...,
      "labels": {
        "0": {"label": "kitchen", "confidence": 0.88,
              "reasoning": "has refrigerator + sink + table"},
        "1": {"label": "bedroom", "confidence": 0.72, ...}
      }
    }

Caching
-------
The prompt for a room is a function of its OBSERVED OBJECT CLASSES.
We cache the LLM answer by (frozenset of classes) — as long as a
room's object set doesn't change, no new LLM call is made.  That
means the classifier quietly re-uses past answers once it has seen a
room's "signature" once. A room with zero objects is labelled
"unknown" without calling the LLM.

Why a separate node?
-------------------
Keeps the semantic_mapper tight-looped at 2 Hz; the LLM is slow and
irregular. This node ticks at its own rate (default 1 Hz) and does
nothing on most ticks thanks to the cache.
"""

import json
import os
from typing import Dict, FrozenSet, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from semantic_mapper.llm_client import LLMClient


# ─────────────────────────────────────────────────────────────────────
#  Prompting
# ─────────────────────────────────────────────────────────────────────
# Default candidate labels. Override via ROS param `room_label_set`.
DEFAULT_LABEL_SET = [
    "kitchen", "bedroom", "bathroom", "living_room", "dining_room",
    "office", "hallway", "storage_closet", "laundry_room",
    "lobby", "waiting_area", "patient_room", "exam_room",
    "reception", "unknown",
]

SYSTEM_PROMPT_TEMPLATE = """You are a scene-understanding assistant that \
classifies indoor rooms from the objects observed inside them.

You will be given a list of objects observed in one room. Based ONLY on \
those objects and common sense about where they occur, output a single \
best room label.

Choose the label from this set (do not invent new ones):
{label_set}

Reply with a JSON object of the form:
{{"label": "<one label from the set>",
  "confidence": <float between 0 and 1>,
  "reasoning": "<one short sentence>"}}"""


USER_PROMPT_TEMPLATE = """Room observed objects:
{obj_list}

Classify this room."""


def _format_object_list(classes: List[str]) -> str:
    """Collapse duplicates into counts for the prompt."""
    from collections import Counter
    if not classes:
        return "(no objects observed yet)"
    c = Counter(classes)
    return "\n".join(f"- {name} x{n}" for name, n in sorted(c.items()))


# ─────────────────────────────────────────────────────────────────────
#  Node
# ─────────────────────────────────────────────────────────────────────
class RoomClassifierNode(Node):
    def __init__(self):
        super().__init__("room_classifier")

        P = self.declare_parameter
        P("scene_graph_topic",    "/scene_graph")
        P("out_topic",            "/semantic_mapper/room_labels")
        P("tick_rate_hz",         1.0)
        P("min_objects_for_call", 1)   # rooms with fewer objects: "unknown"
        P("room_label_set",       DEFAULT_LABEL_SET)

        g = lambda n: self.get_parameter(n).value
        self.tick_rate    = float(g("tick_rate_hz"))
        self.min_objects  = int(g("min_objects_for_call"))
        self.label_set    = [str(s) for s in g("room_label_set")]
        self.scene_topic  = str(g("scene_graph_topic"))
        self.out_topic    = str(g("out_topic"))

        # LLM backend.
        self.llm = LLMClient.from_env()
        self.get_logger().info(
            f"room_classifier LLM backend={self.llm.cfg.backend}  "
            f"model={self.llm.cfg.model}  url={self.llm.cfg.base_url}")

        # Cache: frozenset(class names) → {"label", "confidence", "reasoning"}
        self._sig_cache: Dict[FrozenSet[str], Dict] = {}
        # Latest per-room label, keyed by pid (as str for JSON).
        self._labels: Dict[str, Dict] = {}

        self._latest_sg = None

        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(String, self.scene_topic,
                                 self._sg_cb, latched)
        self.pub = self.create_publisher(String, self.out_topic, latched)
        self.pub_mk = self.create_publisher(
            MarkerArray, '/semantic_mapper/room_labels/markers', 1)

        self.create_timer(1.0 / self.tick_rate, self._tick)
        self.create_timer(10.0, self._hb)
        self._n = dict(ticks=0, calls=0, cache_hits=0,
                       rooms_labeled=0, errors=0)

        # Warn (but don't fail) if the LLM server is unreachable at startup.
        if not self.llm.ping():
            self.get_logger().warn(
                f"LLM server at {self.llm.cfg.base_url} did not respond "
                f"to a ping. Will retry on each tick.")

        self.get_logger().info(
            f"room_classifier ready  in={self.scene_topic}  "
            f"out={self.out_topic}  labels={len(self.label_set)}")

    # ── Subs ──────────────────────────────────────────────────────
    def _sg_cb(self, msg: String):
        try:
            self._latest_sg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"bad scene graph JSON: {e}",
                                   throttle_duration_sec=5.0)

    # ── Tick ──────────────────────────────────────────────────────
    def _tick(self):
        self._n["ticks"] += 1
        if self._latest_sg is None:
            return
        rooms = self._latest_sg.get("rooms", [])
        dirty = False
        for r in rooms:
            pid = str(r.get("id"))
            objs = r.get("objects", []) or []
            classes = [str(o.get("class", "")).strip().lower()
                       for o in objs if o.get("class")]

            if len(classes) < self.min_objects:
                # Don't call the LLM for an empty room; tag unknown.
                new = {"label": "unknown",
                       "confidence": 0.0,
                       "reasoning": "no objects observed yet"}
            else:
                sig = frozenset(classes)   # signature = *which* classes
                if sig in self._sig_cache:
                    self._n["cache_hits"] += 1
                    new = self._sig_cache[sig]
                else:
                    # Show what's being sent to the LLM. Only fires on
                    # a fresh object-set signature, so this log appears
                    # once per "new kind of room", not once per tick.
                    from collections import Counter
                    summary = ", ".join(
                        f"{n}×{name}" for name, n in
                        sorted(Counter(classes).items()))
                    self.get_logger().info(
                        f">>> LLM classifier call  room=R{pid}  "
                        f"objects: {summary}")
                    new = self._classify(classes)
                    if new is not None:
                        self._sig_cache[sig] = new
                    else:
                        # LLM failure — skip update, keep whatever we had.
                        continue

            prev = self._labels.get(pid)
            if prev != new:
                self._labels[pid] = new
                dirty = True

        # Drop labels for rooms that vanished (registry resets etc.)
        live_ids = {str(r["id"]) for r in rooms}
        for gone in list(self._labels.keys()):
            if gone not in live_ids:
                self._labels.pop(gone, None)
                dirty = True

        if dirty:
            self._publish()
            self._n["rooms_labeled"] = len(self._labels)

        # Markers are published every tick so labels track moving
        # centroids even when the label text itself hasn't changed.
        self._publish_markers(rooms)

    # ── LLM call ──────────────────────────────────────────────────
    def _classify(self, classes: List[str]):
        system = SYSTEM_PROMPT_TEMPLATE.format(
            label_set=", ".join(self.label_set))
        user = USER_PROMPT_TEMPLATE.format(
            obj_list=_format_object_list(classes))
        try:
            self._n["calls"] += 1
            reply = self.llm.chat_json(system, user)
        except Exception as e:
            self._n["errors"] += 1
            self.get_logger().warn(
                f"LLM classify failed ({len(classes)} objs): {e}",
                throttle_duration_sec=5.0)
            return None

        label = str(reply.get("label", "unknown")).strip().lower()
        if label not in self.label_set:
            # Coerce unknown-to-us labels into 'unknown' so downstream
            # stays in-set. Log so the user can broaden label_set.
            self.get_logger().warn(
                f"LLM returned out-of-set label {label!r}; coercing to 'unknown'.",
                throttle_duration_sec=10.0)
            label = "unknown"
        try:
            conf = float(reply.get("confidence", 0.0))
        except (TypeError, ValueError):
            conf = 0.0
        reasoning = str(reply.get("reasoning", ""))[:200]
        return {"label": label, "confidence": conf, "reasoning": reasoning}

    # ── Publishing ────────────────────────────────────────────────
    def _publish(self):
        out = {
            "stamp":  self.get_clock().now().nanoseconds * 1e-9,
            "labels": self._labels,
        }
        self.pub.publish(String(data=json.dumps(out)))

    def _publish_markers(self, sg_rooms):
        """Text marker above each room centroid showing the LLM label.
        Placed at z=3.7, between the semantic_mapper's 'R<id> τ=...' at
        z=3.0 and the oracle's 'P=...' at z=4.5, so they stack cleanly."""
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        stamp = self.get_clock().now().to_msg()
        nid = [0]
        for r in sg_rooms:
            pid = str(r.get("id"))
            entry = self._labels.get(pid)
            if not entry:
                continue
            cx, cy = r.get("centroid", [0.0, 0.0])
            mk = Marker()
            mk.header.frame_id = 'world'
            mk.header.stamp = stamp
            mk.ns, mk.id = 'room_labels_llm', nid[0]; nid[0] += 1
            mk.type, mk.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            mk.pose.orientation.w = 1.0
            mk.pose.position = Point(x=float(cx), y=float(cy), z=3.7)
            mk.scale.z = 0.45
            # Muted gold so labels read against room-coloured fills.
            mk.color = ColorRGBA(r=1.0, g=0.85, b=0.3, a=1.0)
            conf = float(entry.get("confidence", 0.0))
            mk.text = f"{entry['label']}  ({conf:.2f})"
            arr.markers.append(mk)
        self.pub_mk.publish(arr)

    def _hb(self):
        self.get_logger().info(
            f"room_classifier hb  ticks={self._n['ticks']}  "
            f"llm_calls={self._n['calls']} cache_hits={self._n['cache_hits']} "
            f"errors={self._n['errors']}  "
            f"labeled_rooms={len(self._labels)} sig_cache={len(self._sig_cache)}")
        self._n = dict(ticks=0, calls=0, cache_hits=0,
                       rooms_labeled=len(self._labels),
                       errors=0)


def main():
    rclpy.init()
    node = RoomClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()