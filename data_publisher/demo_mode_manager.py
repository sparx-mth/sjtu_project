#!/usr/bin/env python3
"""
demo_mode_manager.py  --  standalone copy of the office xtend_drone_demo_manager.

Holds the system state and lets FALCON (or anything) change it exactly the same way
as the real demo manager:

  * subscribes  /xtend/demo_mode_request  (std_msgs/String)
        accepts a bare mode string ("turning") or JSON ({"mode": "...", "source": ...})
  * publishes   /xtend/demo_mode          (std_msgs/String)
        RELIABLE + TRANSIENT_LOCAL (latched) so late joiners get the current mode,
        re-published every publish_period_sec and immediately on every change.
  * on FINISH:  publishes stop -> land on /xtend/cmd_nav, then disarm after a delay.
  * prepares (does not auto-send) /xtend/reset_odom (std_msgs/Empty).

DemoMode is inlined here so this file has no sparx_agency dependency.
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String


class DemoMode(Enum):
    IDLE = "idle"
    FLY_STRAIGHT = "fly_straight"
    TURNING = "turning"
    VISUAL_SERVOING = "visual_servoing"
    FINISH = "finish"

    @classmethod
    def from_text(cls, text: str):
        if text is None:
            return None
        t = str(text).strip().lower()
        for m in cls:
            if m.value == t:
                return m
        return None


@dataclass
class DemoModeEvent:
    mode: DemoMode
    source: str = "unknown"
    reason: str = ""


class DemoModeManager(Node):
    def __init__(self, request_topic, mode_topic, cmd_nav_topic, reset_odom_topic,
                 disarm_delay_sec, publish_period_sec, initial_mode):
        super().__init__("demo_mode_manager")

        self.request_topic = request_topic
        self.mode_topic = mode_topic
        self.cmd_nav_topic = cmd_nav_topic
        self.reset_odom_topic = reset_odom_topic
        self.disarm_delay_sec = float(disarm_delay_sec)

        self.current_mode = initial_mode
        self.finish_started = False
        self.disarm_timer = None

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        default_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.mode_pub = self.create_publisher(String, self.mode_topic, latched_qos)
        self.cmd_nav_pub = self.create_publisher(String, self.cmd_nav_topic, default_qos)
        self.reset_odom_pub = self.create_publisher(Empty, self.reset_odom_topic, default_qos)
        self.request_sub = self.create_subscription(
            String, self.request_topic, self.mode_request_cb, default_qos)

        self.create_timer(float(publish_period_sec), self.publish_current_mode)

        self.get_logger().info(f"request topic : {self.request_topic}")
        self.get_logger().info(f"mode topic    : {self.mode_topic} (latched)")
        self.get_logger().info(f"initial mode  : {self.current_mode.value}")
        self.publish_current_mode()

    def parse_request(self, raw: str):
        text = str(raw).strip()
        if not text:
            return None
        if text.startswith("{"):
            try:
                data = json.loads(text)
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid JSON mode request: {text}")
                return None
            mode = DemoMode.from_text(str(data.get("mode", "")))
            if mode is None:
                return None
            return DemoModeEvent(mode, str(data.get("source", "unknown")),
                                 str(data.get("reason", "")))
        mode = DemoMode.from_text(text)
        if mode is None:
            return None
        return DemoModeEvent(mode, "string_request", text)

    def mode_request_cb(self, msg: String):
        event = self.parse_request(msg.data)
        if event is None:
            self.get_logger().warn(f"Unknown demo mode request: {msg.data}")
            return
        self.set_mode(event.mode, event.source, event.reason)

    def set_mode(self, new_mode: DemoMode, source="unknown", reason=""):
        if new_mode == self.current_mode:
            self.publish_current_mode()
            return
        old = self.current_mode
        self.current_mode = new_mode
        self.get_logger().info(
            f"Mode changed: {old.value} -> {new_mode.value} source={source} reason={reason}")
        if new_mode == DemoMode.FINISH:
            self.start_finish_sequence()
        self.publish_current_mode()

    def publish_current_mode(self):
        msg = String()
        msg.data = self.current_mode.value
        self.mode_pub.publish(msg)

    def publish_cmd_nav(self, action: str, value: int = 0):
        msg = String()
        msg.data = json.dumps({"action": action, "value": int(value)})
        self.cmd_nav_pub.publish(msg)
        self.get_logger().info(f"cmd_nav: {msg.data}")

    def start_finish_sequence(self):
        if self.finish_started:
            return
        self.finish_started = True
        self.get_logger().warn("FINISH: stop -> land -> (delay) -> disarm")
        self.publish_cmd_nav("stop", 0)
        self.publish_cmd_nav("land", 0)
        self.disarm_timer = self.create_timer(self.disarm_delay_sec, self.disarm_once_cb)

    def disarm_once_cb(self):
        self.publish_cmd_nav("disarm", 0)
        if self.disarm_timer is not None:
            self.disarm_timer.cancel()
            self.destroy_timer(self.disarm_timer)
            self.disarm_timer = None


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--request-topic", default="/xtend/demo_mode_request")
    p.add_argument("--mode-topic", default="/xtend/demo_mode")
    p.add_argument("--cmd-nav-topic", default="/xtend/cmd_nav")
    p.add_argument("--reset-odom-topic", default="/xtend/reset_odom")
    p.add_argument("--disarm-delay-sec", type=float, default=8.0)
    p.add_argument("--publish-period-sec", type=float, default=1.0)
    p.add_argument("--initial-mode", default="idle")
    return p.parse_args()


def main():
    args = parse_args()
    initial_mode = DemoMode.from_text(args.initial_mode)
    if initial_mode is None:
        raise ValueError(f"Invalid initial mode: {args.initial_mode}")

    rclpy.init()
    node = DemoModeManager(
        request_topic=args.request_topic,
        mode_topic=args.mode_topic,
        cmd_nav_topic=args.cmd_nav_topic,
        reset_odom_topic=args.reset_odom_topic,
        disarm_delay_sec=args.disarm_delay_sec,
        publish_period_sec=args.publish_period_sec,
        initial_mode=initial_mode,
    )
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