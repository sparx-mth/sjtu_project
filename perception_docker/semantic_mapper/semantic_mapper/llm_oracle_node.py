#!/usr/bin/env python3
"""
llm_oracle_node.py — target + rooms (label, τ_r, F_r) → normalized room probs.

This is the LLM oracle from the Method doc. It asks an LLM: "Given the
target object and, for each room, its type, how long we've searched
there, and how many frontier clusters remain, what is the probability
the target is in each room?"  Raw LLM outputs often don't sum to 1 —
we normalise afterwards. If the LLM returns nothing usable, we fall
back to a uniform distribution over rooms as a conservative default.

Subscribes
----------
/scene_graph                (std_msgs/String, JSON with τ_r + F_r)
/semantic_mapper/room_labels (std_msgs/String, JSON of room labels)

Publishes
---------
/llm_oracle/probabilities   (std_msgs/String, JSON)
    {
      "stamp": 1731427...,
      "target": "car keys",
      "model":  "qwen2.5:3b-instruct",
      "rooms": [
         {"id": 0, "label": "kitchen", "raw": 0.6, "prob": 0.60},
         {"id": 1, "label": "living_room", "raw": 0.3, "prob": 0.30},
         {"id": 2, "label": "bathroom", "raw": 0.1, "prob": 0.10}
      ],
      "raw_reply": { ... },   # for debugging
      "source": "llm" | "uniform_fallback"
    }

Parameters
----------
target_object         Name of the target to search for. Settable at runtime:
                         ros2 param set /llm_oracle target_object "apple"
tick_period_s         How often to refresh the probabilities. Default 10s.
include_undiscovered  If True, rooms with 0 visit time are still included
                      (the drone may not have entered yet). Default True.
"""

import json
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from std_msgs.msg import String, ColorRGBA, Bool
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from semantic_mapper.llm_client import LLMClient


SYSTEM_PROMPT = """You are a commonsense search-planning oracle for an indoor drone.

You are told a target object and a list of rooms. For each room you are \
given:
  - its type (e.g. "kitchen", "bedroom", "bathroom"),
  - how many seconds the drone has already searched there,
  - how many unexplored "frontier clusters" still remain in that room \
(each cluster is a separate region of free-but-unscanned space).

Your job is to output, for EACH room, the probability that the target is \
currently somewhere in that room. Use your commonsense about which rooms \
typically contain which objects. Also account for the given effort:
  - A lot of prior search time with no success should LOWER the probability.
  - More remaining frontier clusters means more unscanned area, so the \
object is more likely to still be there if it belongs in that room.
  - A room that has not been searched at all but matches the object \
semantically should get a high probability.

The numbers do NOT have to sum to 1 — just give your best per-room estimate \
between 0 and 1. The caller will normalise.

Reply with a JSON object of the form:
{"rooms": [
   {"id": <room_id>, "probability": <float in [0,1]>, "reason": "<one short sentence>"},
   ...
 ]}

Include EVERY room you were given, with its original id. Do not invent rooms."""


USER_PROMPT_TEMPLATE = """Target object: {target}

Rooms:
{rooms_block}

Return probabilities for all {n_rooms} rooms as described."""


def _format_rooms_block(rooms: List[Dict]) -> str:
    lines = []
    for i, r in enumerate(rooms, start=1):
        lbl = r.get("label", "unknown")
        pid = r.get("id", -1)
        tau = float(r.get("time_in_room_s", 0.0))
        F   = int(r.get("frontier_clusters", 0))
        objs = r.get("objects", []) or []
        cls_names = sorted({o.get("class", "?") for o in objs
                            if o.get("class")})
        obj_blurb = (f" observed: {', '.join(cls_names)}" if cls_names
                     else " observed: (none)")
        lines.append(
            f"{i}. Room id={pid}  type={lbl}  searched={tau:.0f}s  "
            f"remaining_frontier_clusters={F}.{obj_blurb}")
    return "\n".join(lines)


class LLMOracleNode(Node):
    def __init__(self):
        super().__init__("llm_oracle")

        P = self.declare_parameter
        P("target_object",        "car keys")
        P("scene_graph_topic",    "/scene_graph")
        P("labels_topic",         "/semantic_mapper/room_labels")
        P("out_topic",            "/llm_oracle/probabilities")
        P("tick_period_s",        10.0)
        P("include_undiscovered", True)
        P("min_rooms_for_call",   1)   # skip if fewer than this many rooms

        g = lambda n: self.get_parameter(n).value
        self.target        = str(g("target_object"))
        self.scene_topic   = str(g("scene_graph_topic"))
        self.labels_topic  = str(g("labels_topic"))
        self.out_topic     = str(g("out_topic"))
        self.tick_period   = max(1.0, float(g("tick_period_s")))
        self.include_undiscovered = bool(g("include_undiscovered"))
        self.min_rooms     = int(g("min_rooms_for_call"))

        # React to runtime param changes on target_object so the user
        # can retarget without relaunching.
        self.add_on_set_parameters_callback(self._on_param_set)

        self.llm = LLMClient.from_env()
        self.get_logger().info(
            f"llm_oracle LLM backend={self.llm.cfg.backend}  "
            f"model={self.llm.cfg.model}  url={self.llm.cfg.base_url}")

        self._latest_sg = None
        self._latest_labels = {}   # {pid_str: {"label": "...", ...}}
        self._target_seen = False   # set by /target_seen — pauses ticks

        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(String, self.scene_topic,
                                 self._sg_cb, latched)
        self.create_subscription(String, self.labels_topic,
                                 self._labels_cb, latched)
        self.create_subscription(Bool, "/target_seen",
                                 self._target_seen_cb, latched)
        self.pub = self.create_publisher(String, self.out_topic, latched)
        # Separate MarkerArray topic so the user can toggle it in RViz
        # independently of the scene-graph markers.
        self.pub_mk = self.create_publisher(
            MarkerArray, '/llm_oracle/markers', 1)

        self.create_timer(self.tick_period, self._tick)
        self.create_timer(15.0, self._hb)
        self._n = dict(ticks=0, calls=0, errors=0, publishes=0,
                       fallbacks=0)

        if not self.llm.ping():
            self.get_logger().warn(
                f"LLM server at {self.llm.cfg.base_url} did not respond "
                f"to ping. Will retry on each tick.")

        self.get_logger().info(
            f"llm_oracle ready  target={self.target!r}  "
            f"in=({self.scene_topic}, {self.labels_topic})  "
            f"out={self.out_topic}  period={self.tick_period:.1f}s")

    # ── Runtime param update ──────────────────────────────────────
    def _on_param_set(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "target_object":
                self.target = str(p.value)
                self.get_logger().info(f"target_object -> {self.target!r}")
        return SetParametersResult(successful=True)

    # ── Subs ──────────────────────────────────────────────────────
    def _sg_cb(self, msg: String):
        try:
            self._latest_sg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"bad scene graph JSON: {e}",
                                   throttle_duration_sec=5.0)

    def _labels_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._latest_labels = data.get("labels", {}) or {}
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"bad labels JSON: {e}",
                                   throttle_duration_sec=5.0)

    def _target_seen_cb(self, msg: Bool):
        if msg.data and not self._target_seen:
            self.get_logger().info(
                "received /target_seen=True — pausing LLM oracle ticks.")
        self._target_seen = bool(msg.data)

    # ── Tick ──────────────────────────────────────────────────────
    def _tick(self):
        self._n["ticks"] += 1
        # Once the target is found, stop burning LLM cycles. Stays
        # paused for the lifetime of the process.
        if self._target_seen:
            return
        if self._latest_sg is None:
            return
        rooms_raw = self._latest_sg.get("rooms", []) or []
        # Merge labels in.
        rooms = []
        for r in rooms_raw:
            pid = r.get("id")
            if pid is None:
                continue
            tau = float(r.get("time_in_room_s", 0.0))
            if not self.include_undiscovered and tau <= 0.0:
                continue
            lbl_entry = self._latest_labels.get(str(pid), {})
            label = str(lbl_entry.get("label", "unknown"))
            rooms.append({
                "id":                   int(pid),
                "label":                label,
                "time_in_room_s":       tau,
                "frontier_clusters":    int(r.get("frontier_clusters", 0)),
                "objects":              r.get("objects", []) or [],
            })

        if len(rooms) < self.min_rooms:
            return

        # Show the user what we are about to send to the LLM.
        # Multi-line info log so the whole graph appears in one block
        # (ROS2 only prefixes the first line with the logger tag).
        self.get_logger().info(
            f">>> LLM oracle call  target={self.target!r}  "
            f"rooms={len(rooms)}\n{_format_rooms_block(rooms)}")

        result = self._ask_llm(rooms)
        if result is None:
            # Uniform fallback so downstream (RPT*) is never starved.
            self._n["fallbacks"] += 1
            u = 1.0 / len(rooms)
            result = {
                "source": "uniform_fallback",
                "raw_reply": None,
                "rooms": [{"id": r["id"], "label": r["label"],
                           "raw": u, "prob": u} for r in rooms],
            }
        else:
            result["source"] = "llm"

        out = {
            "stamp":  self.get_clock().now().nanoseconds * 1e-9,
            "target": self.target,
            "model":  self.llm.cfg.model,
            "rooms":  result["rooms"],
            "source": result["source"],
            "raw_reply": result.get("raw_reply"),
        }
        self.pub.publish(String(data=json.dumps(out)))
        self._n["publishes"] += 1

        # RViz visualization: a text marker above each room's centroid.
        self._publish_markers(rooms_raw, result["rooms"])

        # One-line summary for operator readability.
        top = sorted(result["rooms"], key=lambda r: -r["prob"])[:3]
        top_str = ", ".join(
            f"R{r['id']}({r['label']})={r['prob']:.2f}" for r in top)
        self.get_logger().info(
            f"probs  target={self.target!r}  source={result['source']}  "
            f"top3: {top_str}")

    def _ask_llm(self, rooms: List[Dict]) -> Optional[Dict]:
        user = USER_PROMPT_TEMPLATE.format(
            target=self.target,
            rooms_block=_format_rooms_block(rooms),
            n_rooms=len(rooms),
        )
        try:
            self._n["calls"] += 1
            reply = self.llm.chat_json(SYSTEM_PROMPT, user)
        except Exception as e:
            self._n["errors"] += 1
            self.get_logger().warn(f"LLM oracle call failed: {e}",
                                   throttle_duration_sec=5.0)
            return None

        raw_entries = reply.get("rooms") if isinstance(reply, dict) else None
        if not isinstance(raw_entries, list) or not raw_entries:
            self._n["errors"] += 1
            self.get_logger().warn(
                f"LLM oracle: reply missing 'rooms' list. Raw: {reply}",
                throttle_duration_sec=5.0)
            return None

        # Build id -> raw prob. Missing rooms get 0 so normalisation
        # distributes mass only over the ones the LLM scored.
        got: Dict[int, float] = {}
        reasons: Dict[int, str] = {}
        for e in raw_entries:
            try:
                rid = int(e.get("id"))
                val = float(e.get("probability", 0.0))
            except (TypeError, ValueError):
                continue
            val = max(0.0, min(1.0, val))
            got[rid] = val
            reasons[rid] = str(e.get("reason", ""))[:200]

        # Gather raw values in room order; if the LLM missed a room,
        # assign it 0 and note so in the reason.
        room_ids = [r["id"] for r in rooms]
        raw_vec = [got.get(rid, 0.0) for rid in room_ids]

        total = sum(raw_vec)
        if total <= 1e-9:
            # All zeros → fall back to uniform rather than publishing
            # a degenerate distribution.
            self.get_logger().warn(
                "LLM returned all-zero probabilities; falling back to uniform.",
                throttle_duration_sec=5.0)
            return None

        probs = [v / total for v in raw_vec]

        out_rooms = []
        for r, raw, p in zip(rooms, raw_vec, probs):
            out_rooms.append({
                "id":    r["id"],
                "label": r["label"],
                "raw":   raw,
                "prob":  p,
                "reason": reasons.get(r["id"], ""),
                "time_in_room_s":    r["time_in_room_s"],
                "frontier_clusters": r["frontier_clusters"],
            })
        return {"rooms": out_rooms, "raw_reply": reply}

    # ── Markers ───────────────────────────────────────────────────
    def _publish_markers(self, sg_rooms, prob_rooms):
        """Text marker above each room centroid: "P=0.45 (kitchen)".
        Colour interpolates blue (low) → red (high) by probability rank,
        so the most-likely room is visually obvious at a glance.
        `sg_rooms`   = latest scene graph's rooms list (has centroids).
        `prob_rooms` = our normalised probability output for those rooms.
        """
        if not sg_rooms or not prob_rooms:
            return
        centroid_of = {int(r["id"]): r.get("centroid", [0.0, 0.0])
                       for r in sg_rooms}
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        stamp = self.get_clock().now().to_msg()
        nid = [0]

        # Bright red for the top room, fading to muted for the rest.
        pmax = max((r["prob"] for r in prob_rooms), default=1e-9) or 1e-9
        for pr in prob_rooms:
            rid = int(pr["id"])
            if rid not in centroid_of:
                continue
            cx, cy = centroid_of[rid]
            # Colour: mix from cool grey-blue (low) to hot red (high).
            t = max(0.0, min(1.0, pr["prob"] / pmax))
            r, g, b = (0.50 + 0.50 * t,
                       0.55 - 0.40 * t,
                       0.75 - 0.55 * t)

            mk = Marker()
            mk.header.frame_id = 'world'   # semantic_mapper uses 'world'
            mk.header.stamp = stamp
            mk.ns, mk.id = 'llm_oracle', nid[0]; nid[0] += 1
            mk.type, mk.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            mk.pose.orientation.w = 1.0
            mk.pose.position = Point(x=float(cx), y=float(cy), z=4.5)
            mk.scale.z = 0.55
            mk.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            mk.text = f"P={pr['prob']:.2f}  ({pr.get('label','?')})"
            arr.markers.append(mk)
        self.pub_mk.publish(arr)

    def _hb(self):
        self.get_logger().info(
            f"llm_oracle hb  ticks={self._n['ticks']} calls={self._n['calls']} "
            f"pubs={self._n['publishes']} fallbacks={self._n['fallbacks']} "
            f"errors={self._n['errors']}  target={self.target!r}")
        self._n = dict(ticks=0, calls=0, errors=0,
                       publishes=0, fallbacks=0)


def main():
    rclpy.init()
    node = LLMOracleNode()
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