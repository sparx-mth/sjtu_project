#!/usr/bin/env python3
"""
WaypointController
------------------
A lightweight proportional waypoint navigator for a drone.

Responsibilities:
 - Subscribe to /simple_drone/gt_pose
 - Publish /simple_drone/cmd_vel
 - Provide goto(x, y, z) for higher-level agents
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist


class WaypointController(Node):
    """
    A reusable controller for flying a drone toward a target waypoint.

    Features:
        - Pose subscription
        - Proportional velocity control (no PID)
        - Velocity clamping
        - Arrival detection using tolerance + stability counter
    """

    def __init__(self, name: str = "waypoint_controller"):
        super().__init__(name)

        # Internal state
        self.pose = Pose()

        # ROS interfaces
        self.pose_sub = self.create_subscription(
            Pose,
            "/simple_drone/gt_pose",
            self.pose_cb,
            10,
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            "/simple_drone/cmd_vel",
            10,
        )

        # Controller parameters
        self.kp = 0.8          # proportional gain
        self.tol = 0.05        # arrival threshold (meters)
        self.z_fixed = 1.5     # default altitude (meters)

        self.get_logger().info("Waiting for initial pose data...")

        # Wait until pose is available (blocking inside __init__)
        # This is OK because we instantiate this node once before spinning.
        while (
            rclpy.ok()
            and self.pose.position.x == 0.0
            and self.pose.position.y == 0.0
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.get_logger().info("Pose received. Controller ready.")

    # ------------------------------------------------------------------ #

    def pose_cb(self, msg: Pose) -> None:
        """Callback for receiving drone pose updates."""
        self.pose = msg

    # ------------------------------------------------------------------ #

    def goto(self, tx: float, ty: float, tz: float | None = None) -> None:
        """
        Move the drone toward the target (x, y, z).

        Args:
            tx: target X coordinate (world frame, meters)
            ty: target Y coordinate (world frame, meters)
            tz: target Z coordinate (altitude, meters).
                If None, uses self.z_fixed.
        """
        if tz is None:
            tz = self.z_fixed

        self.get_logger().info(
            f"Navigating to ({tx:.2f}, {ty:.2f}, {tz:.2f})"
        )

        stable_counter = 0
        rate = 0.02  # 50 Hz

        while rclpy.ok():
            rclpy.spin_once(self)
            x = self.pose.position.x
            y = self.pose.position.y
            z = self.pose.position.z

            dx = tx - x
            dy = ty - y
            dz = tz - z
            dist = math.sqrt(dx * dx + dy * dy)

            # Arrival detection
            if dist < self.tol:
                stable_counter += 1
                if stable_counter > 10:
                    self.stop()
                    self.get_logger().info("Reached target ✓")
                    return
            else:
                stable_counter = 0
                twist = Twist()
                twist.linear.x = self.clamp(self.kp * dx)
                twist.linear.y = self.clamp(self.kp * dy)
                twist.linear.z = self.clamp(self.kp * dz)
                self.cmd_pub.publish(twist)

            time.sleep(rate)

    # ------------------------------------------------------------------ #

    @staticmethod
    def clamp(value: float, limit: float = 0.3) -> float:
        """Clamp a value to ±limit."""
        return max(min(value, limit), -limit)

    # ------------------------------------------------------------------ #

    def stop(self) -> None:
        """
        Stop all drone motion by publishing zero Twist several times.
        This helps the drone settle before the next command.
        """
        twist = Twist()
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
