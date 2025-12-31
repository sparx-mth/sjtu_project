#!/usr/bin/env python3
"""
Minimum-Snap Trajectory Tracker (Full State Feedback)
======================================================
Feedforward + Feedback controller using position AND velocity feedback.

Control law:
    v_cmd = v_feedforward + Kp*(pos_error) + Kd*(vel_error)

This provides:
- Feedforward: Uses trajectory velocity directly (smooth, predictive)
- Position feedback: Corrects drift and disturbances
- Velocity feedback: Adds damping, prevents overshoot
"""

import math
import time
import threading
from typing import Tuple
from dataclasses import dataclass

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Twist

from autonomous_system.planning.minsnap_trajectory_smoother import SmoothTrajectory


@dataclass
class TrackerGains:
    """Controller gains."""
    kp_xy: float = 1.5   # Position gain (horizontal)
    kp_z: float = 1.2    # Position gain (vertical)
    kd_xy: float = 0.5   # Velocity gain (horizontal) - damping
    kd_z: float = 0.3    # Velocity gain (vertical)


class MinSnapTracker(Node):
    """
    Full state feedback trajectory tracker for minimum-snap trajectories.
    Uses both position and velocity feedback for improved tracking.
    """

    def __init__(self, name: str = "minsnap_tracker"):
        super().__init__(name)

        # State storage (thread-safe)
        self._pose_lock = threading.Lock()
        self._pose = Pose()
        self._vel = Twist()
        self._pose_received = False

        # Abort mechanism
        self._abort_flag = False

        # Gains
        self.gains = TrackerGains()

        # Limits
        self.max_speed_xy = 0.6
        self.max_speed_z = 0.3
        self.max_yaw_rate = 0.5

        # Tolerances
        self.goal_tolerance = 0.15
        self.max_tracking_error = 1.5

        # Yaw control
        self.yaw_kp = 0.8
        self.yaw_deadband = 0.1

        # Control rate
        self.control_rate = 50
        self.control_period = 1.0 / self.control_rate

        # ROS interfaces
        self.cb_group = ReentrantCallbackGroup()
        self.pose_sub = self.create_subscription(
            Pose, "/simple_drone/gt_pose", self._pose_cb, 10,
            callback_group=self.cb_group,
        )
        self.vel_sub = self.create_subscription(
            Twist, "/simple_drone/gt_vel", self._vel_cb, 10,
            callback_group=self.cb_group,
        )
        self.cmd_pub = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)

        self.get_logger().info("MinSnapTracker initialized (Full State Feedback)")
        self.get_logger().info(f"  Kp={self.gains.kp_xy}, Kd={self.gains.kd_xy}")

    # ----------------------------------------------------------------
    # State callbacks
    # ----------------------------------------------------------------

    @property
    def pose(self) -> Pose:
        with self._pose_lock:
            return self._pose

    @property
    def vel(self) -> Twist:
        with self._pose_lock:
            return self._vel

    @property
    def pose_received(self) -> bool:
        with self._pose_lock:
            return self._pose_received

    def _pose_cb(self, msg: Pose) -> None:
        with self._pose_lock:
            self._pose = msg
            self._pose_received = True

    def _vel_cb(self, msg: Twist) -> None:
        with self._pose_lock:
            self._vel = msg

    def trigger_abort(self):
        self._abort_flag = True

    def clear_abort(self):
        self._abort_flag = False

    def is_aborted(self) -> bool:
        return self._abort_flag

    def stop(self) -> None:
        twist = Twist()
        for _ in range(5):
            self.cmd_pub.publish(twist)
            time.sleep(0.02)

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------

    def _quaternion_to_yaw(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _world_to_body(self, vx: float, vy: float, yaw: float) -> Tuple[float, float]:
        c, s = math.cos(yaw), math.sin(yaw)
        return vx * c + vy * s, -vx * s + vy * c

    def _clamp(self, val: float, limit: float) -> float:
        return max(-limit, min(limit, val))

    # ----------------------------------------------------------------
    # Main tracking method
    # ----------------------------------------------------------------

    def follow_trajectory(
            self,
            trajectory: SmoothTrajectory,
            target_altitude: float = 1.5,
            timeout: float = 120.0,
    ) -> Tuple[bool, bool]:
        """
        Follow trajectory using full state feedback.

        Args:
            trajectory: SmoothTrajectory with full state
            target_altitude: Flight altitude (m)
            timeout: Max time (s)

        Returns:
            (reached_goal, was_aborted)
        """
        self.clear_abort()

        if not self._wait_for_pose():
            return False, False

        goal_x, goal_y, goal_z = trajectory.end
        if goal_z == 0:
            goal_z = target_altitude

        start_time = time.time()
        traj_start = time.time()
        loop_count = 0

        self.get_logger().info(
            f"Following: {trajectory.total_time:.1f}s, {trajectory.total_length:.1f}m"
        )

        while True:
            loop_start = time.time()

            if self.is_aborted():
                self.stop()
                return False, True

            # Current state
            p = self.pose
            v = self.vel
            px, py, pz = p.position.x, p.position.y, p.position.z
            vx_act, vy_act, vz_act = v.linear.x, v.linear.y, v.linear.z
            yaw = self._quaternion_to_yaw(p.orientation)

            # Goal check
            dist_goal = math.hypot(goal_x - px, goal_y - py)
            if dist_goal < self.goal_tolerance:
                self.stop()
                self.get_logger().info(f"Goal reached! err={dist_goal*100:.1f}cm")
                return True, False

            # Reference state
            t = time.time() - traj_start
            if t < trajectory.total_time:
                ref = trajectory.get_state_at_time(t)
            else:
                ref = trajectory.get_state_at_time(trajectory.total_time)

            # Position error
            e_px = self._clamp(ref.x - px, 1.0)
            e_py = self._clamp(ref.y - py, 1.0)
            e_pz = target_altitude - pz

            # Velocity error
            e_vx = ref.vx - vx_act
            e_vy = ref.vy - vy_act
            e_vz = -vz_act

            # Full state feedback: v = v_ff + Kp*e_p + Kd*e_v
            vx_cmd = ref.vx + self.gains.kp_xy * e_px + self.gains.kd_xy * e_vx
            vy_cmd = ref.vy + self.gains.kp_xy * e_py + self.gains.kd_xy * e_vy
            vz_cmd = self.gains.kp_z * e_pz + self.gains.kd_z * e_vz

            # Speed limits
            speed = math.hypot(vx_cmd, vy_cmd)
            if speed > self.max_speed_xy:
                vx_cmd *= self.max_speed_xy / speed
                vy_cmd *= self.max_speed_xy / speed
            vz_cmd = self._clamp(vz_cmd, self.max_speed_z)

            # Yaw control
            if speed > 0.05:
                des_yaw = math.atan2(vy_cmd, vx_cmd)
                yaw_err = self._normalize_angle(des_yaw - yaw)
                yaw_rate = self._clamp(self.yaw_kp * yaw_err, self.max_yaw_rate)
            else:
                yaw_rate = 0.0

            # Body frame
            vx_body, vy_body = self._world_to_body(vx_cmd, vy_cmd, yaw)

            # Publish
            twist = Twist()
            twist.linear.x = vx_body
            twist.linear.y = vy_body
            twist.linear.z = vz_cmd
            twist.angular.z = yaw_rate
            self.cmd_pub.publish(twist)

            # Log
            loop_count += 1
            if loop_count % 50 == 0:
                pos_err = math.hypot(e_px, e_py)
                vel_err = math.hypot(e_vx, e_vy)
                self.get_logger().info(
                    f"t={t:.1f}s pos_err={pos_err:.2f}m vel_err={vel_err:.2f}m/s"
                )

            # Timeout
            if time.time() - start_time > timeout:
                self.stop()
                return False, False

            # Rate control
            elapsed = time.time() - loop_start
            if elapsed < self.control_period:
                time.sleep(self.control_period - elapsed)

        return False, False

    def _wait_for_pose(self, timeout: float = 10.0) -> bool:
        start = time.time()
        while not self.pose_received:
            if time.time() - start > timeout:
                self.get_logger().error("Timeout waiting for pose!")
                return False
            time.sleep(0.1)
        return True