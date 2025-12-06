#!/usr/bin/env python3
"""
Precision Yaw Controller
------------------------
Rotates drone by exact degrees using PID with velocity profiling.
Positive = clockwise, Negative = counterclockwise.
"""

import math
import time
import threading
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist


def quat_to_yaw(q) -> float:
    """Extract yaw (radians) from quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(a: float) -> float:
    """Normalize angle to [-π, π]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class YawController(Node):
    """
    Precision yaw controller with:
    - PID control + anti-windup + derivative filtering
    - Two-phase approach (coarse + fine)
    - Stability-based arrival detection
    """

    def __init__(self, name: str = "yaw_controller"):
        super().__init__(name)

        # Pose state
        self._lock = threading.Lock()
        self._pose = Pose()
        self._pose_received = False

        # ROS interfaces
        self.pose_sub = self.create_subscription(Pose, "/simple_drone/gt_pose", self._pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)

        # PID gains - COARSE mode
        self.kp = 1.5
        self.ki = 0.03
        self.kd = 0.4       # Reduced from 0.7

        # Limits
        self.max_rate = 0.4          # rad/s (reduced)
        self.min_rate = 0.008        # rad/s for fine control
        self.integral_limit = 0.15
        self.decel_angle = 1.0       # Start slowing at ~57°

        # Fine phase threshold
        self.fine_angle = math.radians(8.0)  # Enter fine mode earlier at 8°

        # Precision thresholds
        self.tolerance = math.radians(0.25)  # 0.25° tolerance
        self.stable_required = 35

        # PID state
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._filtered_deriv = 0.0

        self.get_logger().info("YawController ready (tolerance=0.25°)")

    def _pose_cb(self, msg: Pose):
        with self._lock:
            self._pose = msg
            self._pose_received = True

    @property
    def yaw(self) -> float:
        with self._lock:
            return quat_to_yaw(self._pose.orientation)

    def rotate(self, degrees: float, timeout: float = 30.0) -> bool:
        """
        Rotate by specified degrees.
        Positive = clockwise, Negative = counterclockwise.
        Returns True if target reached.
        """
        self.get_logger().info(f"[DEBUG] rotate() called with degrees={degrees}")

        # Wait for pose
        t0 = time.time()
        while not self._pose_received and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if not self._pose_received:
            self.get_logger().error("[DEBUG] No pose received!")
            return False

        # Get initial yaw
        initial_yaw = self.yaw
        self.get_logger().info(f"[DEBUG] Initial yaw: {math.degrees(initial_yaw):.2f}°")

        # Compute target yaw (positive degrees = clockwise = negative yaw in ROS)
        delta_rad = math.radians(-degrees)
        target_yaw = normalize_angle(initial_yaw + delta_rad)

        self.get_logger().info(f"Rotating {degrees:+.1f}° | {math.degrees(initial_yaw):.1f}° → {math.degrees(target_yaw):.1f}°")

        # Reset PID
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._filtered_deriv = 0.0

        stable_count = 0
        start_time = time.time()
        loop_count = 0
        in_fine_mode = False

        while time.time() - start_time < timeout:
            loop_start = time.time()
            loop_count += 1

            # Get current yaw
            current_yaw = self.yaw

            # Compute error (shortest path)
            error = normalize_angle(target_yaw - current_yaw)
            abs_error = abs(error)

            # Enter fine mode
            if not in_fine_mode and abs_error < self.fine_angle:
                in_fine_mode = True
                self._integral *= 0.3          # Reduce integral
                self._filtered_deriv = 0.0     # Reset derivative to prevent spike
                self._prev_error = error       # Reset previous error
                self.get_logger().info(f"[DEBUG] Entering fine mode at {math.degrees(abs_error):.2f}°")

            # Debug every 25 iterations (~0.5s)
            if loop_count % 25 == 0:
                mode = "FINE" if in_fine_mode else "COARSE"
                self.get_logger().info(
                    f"[DEBUG] [{mode}] yaw: {math.degrees(current_yaw):.2f}° | "
                    f"err: {math.degrees(error):.2f}° | stable: {stable_count}/{self.stable_required}"
                )

            # Velocity profiling
            if in_fine_mode:
                # Very conservative in fine mode
                max_fine_rate = 0.05  # Much lower max rate in fine mode
                rate_limit = self.min_rate + (max_fine_rate - self.min_rate) * min(abs_error / self.fine_angle, 1.0)
            elif abs_error < self.decel_angle:
                rate_limit = 0.08 + (self.max_rate - 0.08) * (abs_error / self.decel_angle)
            else:
                rate_limit = self.max_rate

            # PID computation
            now = time.time()
            dt = 0.02 if self._prev_time is None else max(now - self._prev_time, 0.001)
            self._prev_time = now

            # Adaptive gains based on mode
            if in_fine_mode:
                kp = self.kp * 0.5    # 50% P gain
                ki = self.ki * 0.3    # 30% I gain
                kd = self.kd * 0.3    # 30% D gain - much lower to prevent oscillation
                deriv_alpha = 0.05    # Very heavy filtering
            else:
                kp = self.kp
                ki = self.ki
                kd = self.kd
                deriv_alpha = 0.1     # Standard filtering

            p_term = kp * error

            self._integral += error * dt
            self._integral = max(min(self._integral, self.integral_limit), -self.integral_limit)
            i_term = ki * self._integral

            raw_deriv = (error - self._prev_error) / dt
            self._filtered_deriv = deriv_alpha * raw_deriv + (1 - deriv_alpha) * self._filtered_deriv
            d_term = kd * self._filtered_deriv
            self._prev_error = error

            # Output with rate limiting
            output = p_term + i_term + d_term
            output = max(min(output, rate_limit), -rate_limit)

            # Debug PID in fine mode
            if in_fine_mode and loop_count % 15 == 0:
                self.get_logger().info(
                    f"[DEBUG] PID: P={p_term:.4f} I={i_term:.4f} D={d_term:.4f} | out={output:.4f} lim={rate_limit:.4f}"
                )

            # Publish angular velocity
            twist = Twist()
            twist.angular.z = output
            self.cmd_pub.publish(twist)

            # Arrival detection
            if abs_error < self.tolerance:
                stable_count += 1
                if stable_count >= self.stable_required:
                    self._stop()
                    final_yaw = self.yaw
                    final_err = math.degrees(abs(normalize_angle(target_yaw - final_yaw)))
                    actual_rotation = math.degrees(normalize_angle(final_yaw - initial_yaw))
                    self.get_logger().info(
                        f"✓ Complete | rotated: {-actual_rotation:.2f}° | error: {final_err:.2f}°"
                    )
                    return True
            else:
                stable_count = 0

            # Control rate ~50Hz
            sleep_time = 0.02 - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._stop()
        final_err = math.degrees(abs(normalize_angle(target_yaw - self.yaw)))
        self.get_logger().warn(f"Timeout! error: {final_err:.2f}°")
        return False

    def _stop(self):
        """Stop rotation."""
        self.get_logger().info("[DEBUG] Stopping...")
        twist = Twist()
        for _ in range(5):
            self.cmd_pub.publish(twist)
            time.sleep(0.02)