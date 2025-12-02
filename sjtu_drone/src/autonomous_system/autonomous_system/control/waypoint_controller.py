#!/usr/bin/env python3
"""
WaypointController (High Precision Version)
--------------------------------------------
A professional-grade waypoint navigator using PID control with velocity profiling.

Features:
 - Full PID control (Proportional + Integral + Derivative)
 - Velocity profiling with smooth deceleration
 - Anti-windup for integral term
 - Derivative filtering (low-pass) to reduce noise
 - Separate tuning for XY (horizontal) and Z (vertical)
 - HIGH PRECISION arrival detection (3cm tolerance)
 - TWO-PHASE APPROACH: normal + fine adjustment
 - ABORT MECHANISM for immediate stopping
 - CLEARANCE-BASED SPEED CONTROL for safe navigation near walls
"""

import math
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple, Callable

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Twist


@dataclass
class PIDGains:
    """PID controller gains."""
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


class PIDController:
    """
    A single-axis PID controller with anti-windup and derivative filtering.
    """

    def __init__(
            self,
            kp: float = 1.0,
            ki: float = 0.0,
            kd: float = 0.0,
            integral_limit: float = 1.0,
            output_limit: float = 1.0,
            derivative_filter_alpha: float = 0.2,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.derivative_filter_alpha = derivative_filter_alpha

        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.prev_time: Optional[float] = None

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.prev_time = None

    def compute(self, error: float, current_time: float) -> float:
        """Compute PID output for given error."""
        if self.prev_time is None:
            dt = 0.02
        else:
            dt = current_time - self.prev_time
            dt = max(dt, 0.001)

        self.prev_time = current_time

        p_term = self.kp * error

        self.integral += error * dt
        self.integral = self._clamp(self.integral, self.integral_limit)
        i_term = self.ki * self.integral

        raw_derivative = (error - self.prev_error) / dt
        self.filtered_derivative = (
                self.derivative_filter_alpha * raw_derivative +
                (1.0 - self.derivative_filter_alpha) * self.filtered_derivative
        )
        d_term = self.kd * self.filtered_derivative

        self.prev_error = error

        output = p_term + i_term + d_term
        return self._clamp(output, self.output_limit)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(min(value, limit), -limit)


class WaypointController(Node):
    """
    High-precision waypoint controller using PID with velocity profiling.

    Precision features:
    - 3cm position tolerance (configurable)
    - Two-phase approach: coarse then fine
    - Increased integral gain for zero steady-state error
    - Longer stability check for confirmed arrival
    """

    def __init__(self, name: str = "waypoint_controller"):
        super().__init__(name)

        # Thread-safe pose storage
        self._pose_lock = threading.Lock()
        self._pose = Pose()
        self._pose_received = False

        # ============================================================
        # ABORT MECHANISM
        # ============================================================
        self._abort_lock = threading.Lock()
        self._abort_flag = False

        # ============================================================
        # CLEARANCE-BASED SPEED CONTROL
        # ============================================================
        self._clearance_lock = threading.Lock()
        self._clearance_callback: Optional[Callable[[float, float], float]] = None
        self._min_clearance_for_full_speed = 1.0
        self._min_clearance_threshold = 0.15

        # Callback group for concurrent pose updates
        self.pose_cb_group = ReentrantCallbackGroup()

        # ROS interfaces
        self.pose_sub = self.create_subscription(
            Pose,
            "/simple_drone/gt_pose",
            self.pose_cb,
            10,
            callback_group=self.pose_cb_group,
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            "/simple_drone/cmd_vel",
            10,
        )

        # ============================================================
        # IMPROVED PID GAINS FOR HIGH PRECISION
        # ============================================================

        # XY (horizontal) PID gains - tuned for precision
        # Higher Ki eliminates steady-state error
        # Higher Kd provides damping for smooth approach
        self.pid_xy = PIDController(
            kp=1.0,  # Increased from 0.8 for faster response
            ki=0.08,  # Increased from 0.02 to eliminate steady-state error
            kd=0.4,  # Increased from 0.3 for better damping
            integral_limit=0.3,
            output_limit=0.5,
            derivative_filter_alpha=0.25,
        )

        # Z (vertical) PID gains
        self.pid_z = PIDController(
            kp=1.2,
            ki=0.08,
            kd=0.5,
            integral_limit=0.3,
            output_limit=0.4,
            derivative_filter_alpha=0.25,
        )

        # ============================================================
        # VELOCITY PROFILING - PRECISION APPROACH
        # ============================================================
        self.max_velocity_xy = 0.4  # Full speed in open areas
        self.min_velocity_xy = 0.08  # Minimum speed near walls
        self.max_velocity_z = 0.3

        # Two-phase approach radii
        self.coarse_decel_radius = 1.0  # Start slowing at 1m
        self.fine_approach_radius = 0.15  # Fine control within 15cm
        self.min_velocity = 0.03  # Very slow for fine approach

        # ============================================================
        # HIGH PRECISION ARRIVAL DETECTION
        # ============================================================
        self.position_tolerance = 0.03  # 3cm tolerance (was 8cm)
        self.fine_position_tolerance = 0.02  # 2cm for final check
        self.stable_count_required = 25  # More checks for stability (was 15)
        self.fine_stable_count = 15  # Additional fine stability checks

        # Control loop rate
        self.control_rate = 50
        self.control_period = 1.0 / self.control_rate

        self.get_logger().info(
            f"WaypointController initialized - HIGH PRECISION MODE"
        )
        self.get_logger().info(
            f"  Position tolerance: {self.position_tolerance * 100:.1f}cm, "
            f"Fine tolerance: {self.fine_position_tolerance * 100:.1f}cm"
        )

    # ------------------------------------------------------------------ #
    # CLEARANCE-BASED SPEED CONTROL
    # ------------------------------------------------------------------ #

    def set_clearance_callback(
            self,
            callback: Callable[[float, float], float],
            min_clearance_for_full_speed: float = 1.0,
            min_clearance_threshold: float = 0.15,
    ):
        """Set a callback function that returns clearance at a given position."""
        with self._clearance_lock:
            self._clearance_callback = callback
            self._min_clearance_for_full_speed = min_clearance_for_full_speed
            self._min_clearance_threshold = min_clearance_threshold

    def clear_clearance_callback(self):
        """Remove the clearance callback."""
        with self._clearance_lock:
            self._clearance_callback = None

    def get_clearance_speed_factor(self, wx: float, wy: float) -> float:
        """Get speed scaling factor based on clearance at position."""
        with self._clearance_lock:
            if self._clearance_callback is None:
                return 1.0

            try:
                clearance = self._clearance_callback(wx, wy)
            except Exception:
                return 1.0

            if clearance >= self._min_clearance_for_full_speed:
                return 1.0

            if clearance <= self._min_clearance_threshold:
                return 0.2

            ratio = (clearance - self._min_clearance_threshold) / (
                    self._min_clearance_for_full_speed - self._min_clearance_threshold
            )
            return 0.2 + 0.8 * ratio

    # ------------------------------------------------------------------ #
    # ABORT MECHANISM
    # ------------------------------------------------------------------ #

    def trigger_abort(self):
        """Signal the controller to abort current navigation immediately."""
        with self._abort_lock:
            self._abort_flag = True

    def clear_abort(self):
        """Clear the abort flag before starting new navigation."""
        with self._abort_lock:
            self._abort_flag = False

    def is_aborted(self) -> bool:
        """Check if abort was triggered."""
        with self._abort_lock:
            return self._abort_flag

    # ------------------------------------------------------------------ #
    # Thread-safe pose access
    # ------------------------------------------------------------------ #

    @property
    def pose(self) -> Pose:
        with self._pose_lock:
            return self._pose

    @property
    def pose_received(self) -> bool:
        with self._pose_lock:
            return self._pose_received

    def pose_cb(self, msg: Pose) -> None:
        with self._pose_lock:
            self._pose = msg
            self._pose_received = True

    # ------------------------------------------------------------------ #
    # IMPROVED Velocity Profiling
    # ------------------------------------------------------------------ #

    def compute_velocity_limit(self, distance: float, max_vel: float) -> float:
        """
        Compute velocity limit with two-phase approach.

        Phase 1 (distance > fine_approach_radius): Normal deceleration
        Phase 2 (distance <= fine_approach_radius): Fine slow approach
        """
        # Phase 2: Fine approach - very slow for precision
        if distance <= self.fine_approach_radius:
            # Linear from min_velocity at 0 to slightly higher at fine_approach_radius
            ratio = distance / self.fine_approach_radius
            return self.min_velocity + (0.08 - self.min_velocity) * ratio

        # Phase 1: Coarse approach with deceleration
        if distance > self.coarse_decel_radius:
            return max_vel

        # Smooth deceleration from coarse_decel_radius to fine_approach_radius
        range_dist = self.coarse_decel_radius - self.fine_approach_radius
        dist_in_range = distance - self.fine_approach_radius
        ratio = dist_in_range / range_dist

        min_for_phase1 = 0.08  # Speed at fine_approach_radius boundary
        velocity = min_for_phase1 + (max_vel - min_for_phase1) * ratio
        return max(velocity, min_for_phase1)

    # ------------------------------------------------------------------ #
    # Main Navigation Method - HIGH PRECISION
    # ------------------------------------------------------------------ #

    def goto(self, tx: float, ty: float, tz: float | None = None) -> Tuple[bool, bool]:
        """
        Navigate to target position with high precision.

        Uses two-phase approach:
        1. Coarse approach: Fast navigation to get close
        2. Fine approach: Slow, precise positioning

        Args:
            tx: Target X coordinate (world frame, meters)
            ty: Target Y coordinate (world frame, meters)
            tz: Target Z coordinate (altitude, meters). If None, uses 1.5m.

        Returns:
            Tuple of (reached, aborted):
                - reached: True if target was reached successfully
                - aborted: True if navigation was aborted externally
        """
        if tz is None:
            tz = 1.5

        # Wait for initial pose
        if not self._wait_for_pose(timeout=10.0):
            return False, False

        # Reset PID controllers for fresh start
        self.pid_xy.reset()
        self.pid_z.reset()
        self._reset_y_pid()

        # Get starting position
        start = self.pose
        initial_dist = math.hypot(tx - start.position.x, ty - start.position.y)

        self.get_logger().info(
            f"[PRECISION] Target: ({tx:.3f}, {ty:.3f}, {tz:.2f}) "
            f"from ({start.position.x:.3f}, {start.position.y:.3f}, {start.position.z:.2f}) "
            f"dist={initial_dist:.3f}m"
        )

        stable_counter = 0
        fine_stable_counter = 0
        loop_count = 0
        start_time = time.time()
        in_fine_approach = False

        while True:
            loop_start = time.time()

            # Check abort
            if self.is_aborted():
                self.stop()
                self.get_logger().warn("[PRECISION] Navigation ABORTED!")
                return False, True

            # Get current pose
            current = self.pose
            x, y, z = current.position.x, current.position.y, current.position.z

            # Compute errors
            error_x = tx - x
            error_y = ty - y
            error_z = tz - z
            dist_xy = math.hypot(error_x, error_y)

            # Detect phase transition
            if not in_fine_approach and dist_xy < self.fine_approach_radius:
                in_fine_approach = True
                self.get_logger().info(
                    f"[PRECISION] Entering fine approach at dist={dist_xy:.3f}m"
                )
                # Reset integral to avoid overshoot
                self.pid_xy.integral *= 0.5
                self._y_integral *= 0.5

            # Clearance-based speed scaling
            clearance_factor = self.get_clearance_speed_factor(x, y)
            effective_max_vel_xy = self.min_velocity_xy + (
                    self.max_velocity_xy - self.min_velocity_xy
            ) * clearance_factor

            # Compute velocity limits based on distance AND clearance
            vel_limit_xy = self.compute_velocity_limit(dist_xy, effective_max_vel_xy)
            vel_limit_z = self.compute_velocity_limit(abs(error_z), self.max_velocity_z)

            self.pid_xy.output_limit = vel_limit_xy
            self.pid_z.output_limit = vel_limit_z

            # Compute PID outputs
            current_time = time.time()
            vel_x_raw = self.pid_xy.compute(error_x, current_time)
            vel_y_raw = self._compute_y_pid(error_y, current_time)

            # Normalize XY velocity to respect limit
            vel_xy_mag = math.hypot(vel_x_raw, vel_y_raw)
            if vel_xy_mag > vel_limit_xy and vel_xy_mag > 0:
                scale = vel_limit_xy / vel_xy_mag
                vel_x = vel_x_raw * scale
                vel_y = vel_y_raw * scale
            else:
                vel_x = vel_x_raw
                vel_y = vel_y_raw

            vel_z = self.pid_z.compute(error_z, current_time)

            # Publish velocity command
            twist = Twist()
            twist.linear.x = vel_x
            twist.linear.y = vel_y
            twist.linear.z = vel_z
            self.cmd_pub.publish(twist)

            # ============================================================
            # HIGH PRECISION ARRIVAL DETECTION
            # ============================================================

            # Coarse arrival check
            if dist_xy < self.position_tolerance and abs(error_z) < self.position_tolerance:
                stable_counter += 1

                # Fine arrival check (tighter tolerance)
                if dist_xy < self.fine_position_tolerance and abs(error_z) < self.fine_position_tolerance:
                    fine_stable_counter += 1
                else:
                    fine_stable_counter = 0

                # Success conditions:
                # 1. Within coarse tolerance for stable_count_required cycles, OR
                # 2. Within fine tolerance for fine_stable_count cycles
                if stable_counter >= self.stable_count_required or fine_stable_counter >= self.fine_stable_count:
                    self.stop()
                    elapsed = time.time() - start_time
                    final_error = math.hypot(tx - x, ty - y)
                    self.get_logger().info(
                        f"[PRECISION] REACHED ({x:.3f}, {y:.3f}, {z:.2f}) "
                        f"error={final_error * 100:.1f}cm in {elapsed:.1f}s"
                    )
                    return True, False
            else:
                stable_counter = 0
                fine_stable_counter = 0

            # Periodic logging (more detail in fine approach)
            loop_count += 1
            log_interval = 25 if in_fine_approach else 50
            if loop_count % log_interval == 0:
                phase = "FINE" if in_fine_approach else "COARSE"
                self.get_logger().info(
                    f"  [{phase}] pos=({x:.3f}, {y:.3f}) "
                    f"err={dist_xy * 100:.1f}cm vel_lim={vel_limit_xy:.3f} "
                    f"stable={stable_counter}/{self.stable_count_required}"
                )

            # Timeout check
            if time.time() - start_time > 120.0:
                self.get_logger().warn("[PRECISION] Timeout!")
                self.stop()
                return False, False

            # Maintain control loop rate
            elapsed = time.time() - loop_start
            sleep_time = self.control_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        return False, False

    def _compute_y_pid(self, error_y: float, current_time: float) -> float:
        """Compute Y-axis PID using same gains as X but separate state."""
        if not hasattr(self, '_y_integral') or self._y_prev_time is None:
            self._y_integral = 0.0
            self._y_prev_error = 0.0
            self._y_prev_time = current_time
            self._y_filtered_deriv = 0.0
            return self.pid_xy.kp * error_y

        dt = current_time - self._y_prev_time
        dt = max(dt, 0.001)
        self._y_prev_time = current_time

        p_term = self.pid_xy.kp * error_y

        self._y_integral += error_y * dt
        self._y_integral = max(min(self._y_integral, self.pid_xy.integral_limit),
                               -self.pid_xy.integral_limit)
        i_term = self.pid_xy.ki * self._y_integral

        raw_deriv = (error_y - self._y_prev_error) / dt
        alpha = self.pid_xy.derivative_filter_alpha
        self._y_filtered_deriv = alpha * raw_deriv + (1 - alpha) * self._y_filtered_deriv
        d_term = self.pid_xy.kd * self._y_filtered_deriv

        self._y_prev_error = error_y

        return p_term + i_term + d_term

    def _reset_y_pid(self):
        """Reset Y-axis PID state."""
        self._y_integral = 0.0
        self._y_prev_error = 0.0
        self._y_filtered_deriv = 0.0
        self._y_prev_time = None

    # ------------------------------------------------------------------ #
    # Helper Methods
    # ------------------------------------------------------------------ #

    def _wait_for_pose(self, timeout: float = 10.0) -> bool:
        """Wait for initial pose with timeout."""
        start = time.time()
        while not self.pose_received:
            if time.time() - start > timeout:
                self.get_logger().error("Timeout waiting for pose!")
                return False
            time.sleep(0.1)
        return True

    def stop(self) -> None:
        """Stop drone by publishing zero velocity."""
        twist = Twist()
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        # Reset PID states
        self.pid_xy.reset()
        self.pid_z.reset()
        self._reset_y_pid()