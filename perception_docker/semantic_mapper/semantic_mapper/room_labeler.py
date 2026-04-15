#!/usr/bin/env python3
"""
room_labeler.py (ROS2)

Subscribes to /scene_graph, assigns semantic labels, republishes on
/scene_graph/labeled.

Back-ends:
  • 'rules' (default)  — keyword classifier
  • 'llm'              — OpenAI-compatible endpoint

Compatible with the new MORE-style scene-graph schema: passes through the
`kind`, `neighbors`, and `edges` fields untouched. Rooms whose `kind` is
"open_space" are NOT relabeled — they keep that label.
"""

import json
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


RULES = [
    ("kitchen",     {"refrigerator", "microwave", "oven", "sink", "toaster",
                     "stove", "fridge", "dishwasher"}),
    ("bathroom",    {"toilet", "bathtub", "shower", "towel"}),
    ("bedroom",     {"bed", "pillow", "nightstand", "wardrobe"}),
    ("living_room", {"couch", "sofa", "tv", "television", "remote",
                     "coffee-table", "armchair"}),
    ("dining_room", {"dining-table", "fork", "plate", "wine", "bowl"}),
    ("office",      {"laptop", "keyboard", "mouse", "monitor", "desk"}),
    ("hallway",     {"picture"}),
]


def rule_label(object_classes):
    cs = {c.lower() for c in object_classes}
    best, best_score = "unknown", 0
    for label, kws in RULES:
        score = sum(1 for k in kws if any(k in c for c in cs))
        if score > best_score:
            best, best_score = label, score
    return best if best_score > 0 else "unknown"


def llm_label(object_classes, api_key, base_url, model, logger):
    try:
        from openai import OpenAI
    except Exception as e:
        logger.warn(f"openai SDK not installed: {e}")
        return None
    client = OpenAI(api_key=api_key, base_url=base_url)
    objs = ", ".join(sorted(set(object_classes))) or "(empty)"
    prompt = (
        "You are labeling indoor rooms from the objects observed inside. "
        "Reply with ONE lowercase snake_case label from this closed set: "
        "kitchen, bathroom, bedroom, living_room, dining_room, office, "
        "hallway, laundry, garage, closet, unknown.\n"
        f"Objects in the room: {objs}\n"
        "Label:"
    )
    try:
        r = client.chat.completions.create(
            model=model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.0, max_tokens=8,
        )
        return r.choices[0].message.content.strip().lower().split()[0]
    except Exception as e:
        logger.warn(f"LLM call failed: {e}")
        return None


class RoomLabeler(Node):
    def __init__(self):
        super().__init__("room_labeler")
        self.declare_parameter("use_llm", False)
        self.declare_parameter("api_key", os.environ.get("OPENAI_API_KEY", ""))
        self.declare_parameter("base_url",
            os.environ.get("OPENAI_BASE_URL", "https://api.openai.com/v1"))
        self.declare_parameter("model", "gpt-4o-mini")
        self.declare_parameter("min_objs_for_llm", 2)

        self.use_llm   = bool(self.get_parameter("use_llm").value)
        self.api_key   = str(self.get_parameter("api_key").value)
        self.base_url  = str(self.get_parameter("base_url").value)
        self.model     = str(self.get_parameter("model").value)
        self.min_objs  = int(self.get_parameter("min_objs_for_llm").value)

        self.cache = {}  # (room_id, sorted tuple of classes) -> label

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub = self.create_publisher(String, "/scene_graph/labeled", latched)
        self.create_subscription(String, "/scene_graph", self._cb, latched)

        self.get_logger().info(
            f"room_labeler: backend={'llm(' + self.model + ')' if self.use_llm else 'rules'}"
        )

    def _cb(self, msg):
        try:
            sg = json.loads(msg.data)
        except Exception:
            return
        obj_by_id = {o["id"]: o for o in sg["objects"]}
        for r in sg["rooms"]:
            # Open-space regions keep their kind-derived label; they are not
            # rooms in the conventional sense and shouldn't be classified as
            # kitchen/bedroom/etc.
            if r.get("kind") == "open_space":
                if not r.get("label") or r["label"] == "unknown":
                    r["label"] = "open_space"
                continue

            classes = [obj_by_id[i]["cls"] for i in r["objects"] if i in obj_by_id]
            label = rule_label(classes)
            if self.use_llm and len(classes) >= self.min_objs:
                key = (r["id"], tuple(sorted(classes)))
                cached = self.cache.get(key)
                if cached is not None:
                    label = cached
                else:
                    llm = llm_label(classes, self.api_key, self.base_url,
                                    self.model, self.get_logger())
                    if llm:
                        label = llm
                        self.cache[key] = llm
            r["label"] = label
        # `edges`, `neighbors`, `kind`, `current_room`, `doors`, `objects`,
        # `stamp` all pass through untouched.
        self.pub.publish(String(data=json.dumps(sg)))


def main():
    rclpy.init()
    node = RoomLabeler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()