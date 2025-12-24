#!/usr/bin/env python3
"""
Smooth Path Follower
--------------------
Follows a smooth trajectory using Pure Pursuit algorithm.

Key Features:
 - Never stops at intermediate waypoints
 - Naturally creates smooth, rounded turns
 - Adaptive lookahead based on speed and curvature
 - Velocity profiling for safe navigation
 - Compatible with WaypointController base class
"""

import math
import time
import threading
from typing import Tuple, Optional, Callable

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Twist

from autonomous_system.planning.trajectory_smoother import SmoothTrajectory


class SmoothPathFollower(Node):
    """
    Pure Pursuit path follower for smooth trajectory tracking.

    Instead of stopping at each waypoint, continuously follows a smooth
    trajectory by always steering toward a lookahead point.
    """

    def __init__(self, name: str = "smooth_path_follower"):
        super().__init__(name)

        # Thread-safe pose storage
        self._pose_lock = threading.Lock()
        self._pose = Pose()
        self._pose_received = False

        # Abort mechanism
        self._abort_lock = threading.Lock()
        self._abort_flag = False

        # Clearance callback for obstacle-aware speed
        self._clearance_lock = threading.Lock()
        self._clearance_callback: Optional[Callable[[float, float], float]] = None
        self._min_clearance_for_full_speed = 1.0
        self._min_clearance_threshold = 0.15

        # ROS interfaces
        self.pose_cb_group = ReentrantCallbackGroup()
        self.pose_sub = self.create_subscription(
            Pose, "/simple_drone/gt_pose", self._pose_cb, 10,
            callback_group=self.pose_cb_group,
        )
        self.cmd_pub = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)

        # ========================================
        # Pure Pursuit Parameters
        # ========================================

        # Lookahead distance (how far ahead to look on path)
        self.base_lookahead = 0.6  # Base lookahead distance (m)
        self.min_lookahead = 0.3  # Minimum lookahead (m)
        self.max_lookahead = 1.5  # Maximum lookahead (m)
        self.lookahead_speed_gain = 0.5  # Lookahead scales with speed

        # Speed control
        self.cruise_speed = 0.4  # Desired cruise speed (m/s)
        self.min_speed = 0.1  # Minimum speed (m/s)
        self.max_speed = 0.5  # Maximum speed (m/s)

        # Curvature-based speed reduction
        self.curvature_speed_factor = 0.3  # Slow down on tight turns

        # Goal tolerance
        self.goal_tolerance = 0.15  # Stop within this distance of goal (m)
        self.path_tolerance = 0.8  # Max allowed distance from path (m)

        # Altitude control (simple P controller)
        self.altitude_kp = 1.2
        self.max_vertical_speed = 0.3

        # Control loop
        self.control_rate = 50
        self.control_period = 1.0 / self.control_rate

        self.get_logger().info("SmoothPathFollower initialized")
        self.get_logger().info(f"  Lookahead: {self.min_lookahead}-{self.max_lookahead}m")
        self.get_logger().info(f"  Speed: {self.min_speed}-{self.max_speed}m/s")

    # ----------------------------------------------------------------
    # Pose and abort management
    # ----------------------------------------------------------------

    @property
    def pose(self) -> Pose:
        with self._pose_lock:
            return self._pose

    @property
    def pose_received(self) -> bool:
        with self._pose_lock:
            return self._pose_received

    def _pose_cb(self, msg: Pose) -> None:
        with self._pose_lock:
            self._pose = msg
            self._pose_received = True

    def trigger_abort(self):
        with self._abort_lock:
            self._abort_flag = True

    def clear_abort(self):
        with self._abort_lock:
            self._abort_flag = False

    def is_aborted(self) -> bool:
        with self._abort_lock:
            return self._abort_flag

    def stop(self) -> None:
        """Stop the drone."""
        twist = Twist()
        for _ in range(5):
            self.cmd_pub.publish(twist)
            time.sleep(0.02)

    # ----------------------------------------------------------------
    # Clearance-based speed control
    # ----------------------------------------------------------------

    def set_clearance_callback(
            self,
            callback: Callable[[float, float], float],
            min_clearance_for_full_speed: float = 1.0,
            min_clearance_threshold: float = 0.15,
    ):
        with self._clearance_lock:
            self._clearance_callback = callback
            self._min_clearance_for_full_speed = min_clearance_for_full_speed
            self._min_clearance_threshold = min_clearance_threshold

    def _get_clearance_factor(self, x: float, y: float) -> float:
        with self._clearance_lock:
            if self._clearance_callback is None:
                return 1.0
            try:
                clearance = self._clearance_callback(x, y)
                if clearance >= self._min_clearance_for_full_speed:
                    return 1.0
                if clearance <= self._min_clearance_threshold:
                    return 0.3
                ratio = (clearance - self._min_clearance_threshold) / (
                        self._min_clearance_for_full_speed - self._min_clearance_threshold
                )
                return 0.3 + 0.7 * ratio
            except Exception:
                return 1.0

    # ----------------------------------------------------------------
    # Pure Pursuit Core
    # ----------------------------------------------------------------

    def _compute_lookahead(self, current_speed: float, curvature: float) -> float:
        """Compute adaptive lookahead distance."""
        # Base lookahead + speed-proportional term
        lookahead = self.base_lookahead + self.lookahead_speed_gain * current_speed

        # Reduce lookahead on tight curves for better tracking
        if curvature > 0.5:
            lookahead *= 0.7

        return max(self.min_lookahead, min(lookahead, self.max_lookahead))

    def _compute_speed(
            self,
            dist_to_goal: float,
            curvature: float,
            clearance_factor: float,
    ) -> float:
        """Compute target speed based on distance, curvature, and clearance."""
        speed = self.cruise_speed

        # Slow down near goal
        if dist_to_goal < 1.0:
            speed *= (0.3 + 0.7 * (dist_to_goal / 1.0))

        # Slow down on curves
        curve_factor = 1.0 / (1.0 + self.curvature_speed_factor * curvature)
        speed *= curve_factor

        # Apply clearance factor
        speed *= clearance_factor

        return max(self.min_speed, min(speed, self.max_speed))

    # ----------------------------------------------------------------
    # Main follow method
    # ----------------------------------------------------------------

    def follow_trajectory(
            self,
            trajectory: SmoothTrajectory,
            target_altitude: float = 1.5,
            timeout: float = 120.0,
    ) -> Tuple[bool, bool]:
        """
        Follow a smooth trajectory using Pure Pursuit.

        Args:
            trajectory: SmoothTrajectory to follow
            target_altitude: Desired flight altitude (m)
            timeout: Maximum time to reach goal (s)

        Returns:
            Tuple of (success, aborted)
        """
        self.clear_abort()

        # Wait for pose
        if not self._wait_for_pose():
            return False, False

        goal_x, goal_y = trajectory.end
        current_s = 0.0  # Current progress along trajectory

        start_time = time.time()
        loop_count = 0
        current_speed = 0.0

        self.get_logger().info(
            f"Following trajectory: {trajectory.total_length:.2f}m to "
            f"({goal_x:.2f}, {goal_y:.2f})"
        )

        while True:
            loop_start = time.time()

            # Check abort
            if self.is_aborted():
                self.stop()
                self.get_logger().warn("Trajectory following ABORTED")
                return False, True

            # Get current position
            current = self.pose
            px, py, pz = current.position.x, current.position.y, current.position.z

            # Distance to goal
            dist_to_goal = math.hypot(goal_x - px, goal_y - py)

            # Check if goal reached
            if dist_to_goal < self.goal_tolerance:
                self.stop()
                elapsed = time.time() - start_time
                self.get_logger().info(
                    f"Goal reached in {elapsed:.1f}s, error={dist_to_goal * 100:.1f}cm"
                )
                return True, False

            # Find closest point on trajectory (search forward from current progress)
            closest_s, cross_track_error = trajectory.find_closest_point(
                px, py,
                search_start=max(0, current_s - 0.3),
                search_window=2.0,
            )

            # Update progress (only move forward, prevent backtracking)
            if closest_s > current_s:
                current_s = closest_s

            # Check if too far from path
            if cross_track_error > self.path_tolerance:
                self.get_logger().warn(
                    f"Too far from path: {cross_track_error:.2f}m > {self.path_tolerance}m"
                )
                self.stop()
                return False, False

            # Get trajectory info at current position
            traj_point = trajectory.get_point(current_s)
            curvature = traj_point.curvature

            # Compute adaptive lookahead
            lookahead = self._compute_lookahead(current_speed, curvature)

            # Get lookahead point on trajectory
            lookahead_s = min(current_s + lookahead, trajectory.total_length)
            target_point = trajectory.get_point(lookahead_s)
            target_x, target_y = target_point.x, target_point.y

            # Compute velocity toward lookahead point
            dx = target_x - px
            dy = target_y - py
            dist_to_target = math.hypot(dx, dy)

            # Get clearance-based speed factor
            clearance_factor = self._get_clearance_factor(px, py)

            # Compute target speed
            target_speed = self._compute_speed(dist_to_goal, curvature, clearance_factor)

            # Smooth speed changes
            speed_alpha = 0.3
            current_speed = speed_alpha * target_speed + (1 - speed_alpha) * current_speed

            # Compute velocity components
            if dist_to_target > 0.01:
                vx = (dx / dist_to_target) * current_speed
                vy = (dy / dist_to_target) * current_speed
            else:
                vx, vy = 0.0, 0.0

            # Altitude control (simple P)
            error_z = target_altitude - pz
            vz = self.altitude_kp * error_z
            vz = max(-self.max_vertical_speed, min(vz, self.max_vertical_speed))

            # Publish velocity command
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz
            self.cmd_pub.publish(twist)

            # Logging
            loop_count += 1
            if loop_count % 50 == 0:
                progress = (current_s / trajectory.total_length) * 100
                self.get_logger().info(
                    f"Progress: {progress:.0f}% | "
                    f"pos=({px:.2f},{py:.2f}) | "
                    f"speed={current_speed:.2f}m/s | "
                    f"lookahead={lookahead:.2f}m"
                )

            # Timeout check
            if time.time() - start_time > timeout:
                self.get_logger().warn("Trajectory following timeout!")
                self.stop()
                return False, False

            # Maintain control rate
            elapsed = time.time() - loop_start
            sleep_time = self.control_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        return False, False

    def _wait_for_pose(self, timeout: float = 10.0) -> bool:
        """Wait for initial pose."""
        start = time.time()
        while not self.pose_received:
            if time.time() - start > timeout:
                self.get_logger().error("Timeout waiting for pose!")
                return False
            time.sleep(0.1)
        return True