#!/usr/bin/env python3
"""
WaypointController
------------------
A lightweight proportional waypoint navigator for a drone.

Responsibilities:
 - Subscribe to /simple_drone/gt_pose
 - Publish /simple_drone/cmd_vel
 - Provide goto(x, y, z) for higher-level agents

IMPORTANT: When using with MultiThreadedExecutor, the pose subscription
uses a ReentrantCallbackGroup to allow concurrent updates while goto() blocks.
"""

import math
import time
import threading

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Twist


class WaypointController(Node):
    """
    A reusable controller for flying a drone toward a target waypoint.

    Features:
        - Pose subscription with ReentrantCallbackGroup for concurrency
        - Proportional velocity control (no PID)
        - Velocity clamping
        - Arrival detection using tolerance + stability counter
        - Thread-safe pose access
    """

    def __init__(self, name: str = "waypoint_controller"):
        super().__init__(name)

        # Thread-safe pose storage
        self._pose_lock = threading.Lock()
        self._pose = Pose()
        self._pose_received = False

        # Use ReentrantCallbackGroup so pose_cb can run while goto() is blocking
        # This is CRITICAL for MultiThreadedExecutor!
        self.pose_cb_group = ReentrantCallbackGroup()

        # ROS interfaces
        self.pose_sub = self.create_subscription(
            Pose,
            "/simple_drone/gt_pose",
            self.pose_cb,
            10,
            callback_group=self.pose_cb_group,  # Allow concurrent execution
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            "/simple_drone/cmd_vel",
            10,
        )

        # Controller parameters (matching old working code)
        self.kp = 0.8          # proportional gain
        self.tol = 0.05        # arrival threshold (meters)
        self.z_fixed = 1.5     # default altitude (meters)

        self.get_logger().info("WaypointController initialized.")

    # ------------------------------------------------------------------ #
    # Thread-safe pose access
    # ------------------------------------------------------------------ #

    @property
    def pose(self) -> Pose:
        """Thread-safe getter for current pose."""
        with self._pose_lock:
            return self._pose

    @property
    def pose_received(self) -> bool:
        """Thread-safe check if pose has been received."""
        with self._pose_lock:
            return self._pose_received

    def pose_cb(self, msg: Pose) -> None:
        """Callback for receiving drone pose updates (runs on executor thread)."""
        with self._pose_lock:
            self._pose = msg
            self._pose_received = True

    # ------------------------------------------------------------------ #

    def goto(self, tx: float, ty: float, tz: float | None = None) -> None:
        """
        Move the drone toward the target (x, y, z).

        This method blocks until the drone reaches the target.
        With MultiThreadedExecutor + ReentrantCallbackGroup, the pose_cb
        will continue to be called by the executor on another thread,
        so we don't need to call spin_once() manually.

        Args:
            tx: target X coordinate (world frame, meters)
            ty: target Y coordinate (world frame, meters)
            tz: target Z coordinate (altitude, meters).
                If None, uses self.z_fixed.
        """
        if tz is None:
            tz = self.z_fixed

        # Wait for initial pose
        wait_count = 0
        while not self.pose_received:
            time.sleep(0.1)
            wait_count += 1
            if wait_count % 10 == 0:
                self.get_logger().info("Waiting for initial pose...")
            if wait_count > 100:  # 10 second timeout
                self.get_logger().error("Timeout waiting for pose!")
                return

        # Get initial pose for logging
        current = self.pose
        self.get_logger().info(
            f"Navigating to ({tx:.2f}, {ty:.2f}, {tz:.2f}) "
            f"from ({current.position.x:.2f}, {current.position.y:.2f}, {current.position.z:.2f})"
        )

        stable_counter = 0
        loop_count = 0

        while True:
            # Get current pose (thread-safe)
            current = self.pose
            x = current.position.x
            y = current.position.y
            z = current.position.z

            dx = tx - x
            dy = ty - y
            dz = tz - z
            dist_xy = math.hypot(dx, dy)

            # Debug logging every 50 iterations (~1 second)
            loop_count += 1
            if loop_count % 50 == 0:
                self.get_logger().info(
                    f"  pos=({x:.2f}, {y:.2f}, {z:.2f}) "
                    f"target=({tx:.2f}, {ty:.2f}, {tz:.2f}) "
                    f"dist_xy={dist_xy:.3f} dz={dz:.3f}"
                )

            # Arrival detection (same logic as old code)
            if dist_xy < self.tol:
                stable_counter += 1
                if stable_counter > 10:
                    self.stop()
                    self.get_logger().info(
                        f"Reached target ✓ at ({x:.2f}, {y:.2f}, {z:.2f})"
                    )
                    return
            else:
                stable_counter = 0

            # Always send velocity command (even when close, to maintain altitude)
            twist = Twist()
            twist.linear.x = self.clamp(self.kp * dx)
            twist.linear.y = self.clamp(self.kp * dy)
            twist.linear.z = self.clamp(self.kp * dz)
            self.cmd_pub.publish(twist)

            time.sleep(0.02)  # 50 Hz control loop

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