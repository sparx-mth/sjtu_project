#!/usr/bin/env python3
"""
Precision Forward Controller
----------------------------
Moves drone forward by exact distance using PID with trapezoidal velocity profiling.
Uses body-frame forward direction (along current yaw).
"""

import math
import time
import threading
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist


def quat_to_yaw(q) -> float:
    """Extract yaw from quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ForwardController(Node):
    """
    Precision 1D forward controller with:
    - PID control + anti-windup + derivative filtering
    - Trapezoidal velocity profiling (accel → cruise → decel)
    - Two-phase approach (coarse + fine)
    - Stability-based arrival detection
    """

    def __init__(self, name: str = "forward_controller"):
        super().__init__(name)

        # State
        self._lock = threading.Lock()
        self._pose = Pose()
        self._pose_ok = False

        # ROS interfaces
        self.pose_sub = self.create_subscription(Pose, "/simple_drone/gt_pose", self._pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)

        # PID gains (tuned for precision)
        self.kp, self.ki, self.kd = 1.2, 0.06, 0.35
        self.integral_limit = 0.2

        # Velocity profile
        self.v_max = 0.4        # m/s cruise
        self.v_min = 0.02       # m/s fine control
        self.accel = 0.5        # m/s² acceleration
        self.decel_dist = 0.3   # Start decel at 30cm

        # Precision
        self.tolerance = 0.015      # 1.5cm
        self.stable_required = 30   # ~0.6s stability

        self.get_logger().info("ForwardController ready (tolerance=1.5cm)")

    def _pose_cb(self, msg: Pose):
        with self._lock:
            self._pose = msg
            self._pose_ok = True

    @property
    def pose(self) -> Pose:
        with self._lock:
            return self._pose

    def move(self, distance: float, timeout: float = 30.0) -> bool:
        """
        Move forward by exact distance (meters). Positive = forward.
        Returns True if target reached within tolerance.
        """
        # Wait for pose
        t0 = time.time()
        while not self._pose_ok and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if not self._pose_ok:
            self.get_logger().error("No pose!")
            return False

        # Compute target in world frame
        p = self.pose
        yaw = quat_to_yaw(p.orientation)
        start_x, start_y = p.position.x, p.position.y
        target_x = start_x + distance * math.cos(yaw)
        target_y = start_y + distance * math.sin(yaw)

        self.get_logger().info(f"Forward {distance:.3f}m | yaw={math.degrees(yaw):.1f}°")

        # PID state
        integral, prev_err, filt_d, prev_t = 0.0, 0.0, 0.0, None
        traveled = 0.0  # For velocity profiling

        stable = 0
        t_start = time.time()

        while time.time() - t_start < timeout:
            loop_t = time.time()

            # Current position
            p = self.pose
            x, y = p.position.x, p.position.y

            # Error: project position error onto forward direction
            dx, dy = target_x - x, target_y - y
            error = dx * math.cos(yaw) + dy * math.sin(yaw)
            abs_err = abs(error)

            # Traveled distance (for accel phase)
            traveled = abs(distance) - abs_err

            # Trapezoidal velocity profile
            v_accel = math.sqrt(2 * self.accel * max(traveled, 0.01))  # Accel phase
            v_decel = math.sqrt(2 * self.accel * max(abs_err, 0.01))   # Decel phase
            v_limit = min(self.v_max, v_accel, v_decel)
            v_limit = max(v_limit, self.v_min)

            # Fine mode boost for integral
            fine_mode = abs_err < 0.08
            ki_eff = self.ki * (1.5 if fine_mode else 1.0)

            # PID
            now = time.time()
            dt = 0.02 if prev_t is None else max(now - prev_t, 0.001)
            prev_t = now

            p_term = self.kp * error
            integral = max(min(integral + error * dt, self.integral_limit), -self.integral_limit)
            i_term = ki_eff * integral
            raw_d = (error - prev_err) / dt
            filt_d = 0.15 * raw_d + 0.85 * filt_d
            d_term = self.kd * filt_d
            prev_err = error

            output = max(min(p_term + i_term + d_term, v_limit), -v_limit)

            # Convert to body-frame velocity
            twist = Twist()
            twist.linear.x = output * math.cos(yaw)
            twist.linear.y = output * math.sin(yaw)
            self.cmd_pub.publish(twist)

            # Arrival check
            if abs_err < self.tolerance:
                stable += 1
                if stable >= self.stable_required:
                    self._stop()
                    final_err = math.hypot(target_x - x, target_y - y)
                    self.get_logger().info(f"✓ Complete | error={final_err*100:.1f}cm")
                    return True
            else:
                stable = 0

            # Rate control ~50Hz
            time.sleep(max(0, 0.02 - (time.time() - loop_t)))

        self._stop()
        self.get_logger().warn("Timeout!")
        return False

    def _stop(self):
        twist = Twist()
        for _ in range(5):
            self.cmd_pub.publish(twist)
            time.sleep(0.02)