#!/usr/bin/env python3
"""
WaypointController (Advanced PID Version)
------------------------------------------
A professional-grade waypoint navigator using PID control with velocity profiling.

Features:
 - Full PID control (Proportional + Integral + Derivative)
 - Velocity profiling with smooth deceleration
 - Anti-windup for integral term
 - Derivative filtering (low-pass) to reduce noise
 - Separate tuning for XY (horizontal) and Z (vertical)
 - Configurable arrival detection

Based on control strategies used in PX4 and ArduPilot autopilots.

References:
 - PX4 Position Controller: https://docs.px4.io/main/en/flight_stack/controller_diagrams.html
 - ArduPilot Loiter Mode: https://ardupilot.org/copter/docs/loiter-mode.html
"""

import math
import time
import threading
from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Twist


@dataclass
class PIDGains:
    """PID controller gains."""
    kp: float = 0.0  # Proportional gain
    ki: float = 0.0  # Integral gain
    kd: float = 0.0  # Derivative gain


class PIDController:
    """
    A single-axis PID controller with anti-windup and derivative filtering.

    Features:
        - Integral anti-windup (clamping)
        - Derivative low-pass filtering
        - Output clamping
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

        # State
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
        """
        Compute PID output for given error.

        Args:
            error: Current error (setpoint - measured)
            current_time: Current timestamp in seconds

        Returns:
            Control output (clamped to output_limit)
        """
        # Calculate dt
        if self.prev_time is None:
            dt = 0.02  # Default 50Hz
        else:
            dt = current_time - self.prev_time
            dt = max(dt, 0.001)  # Prevent division by zero

        self.prev_time = current_time

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = self._clamp(self.integral, self.integral_limit)
        i_term = self.ki * self.integral

        # Derivative term with low-pass filter
        raw_derivative = (error - self.prev_error) / dt
        self.filtered_derivative = (
                self.derivative_filter_alpha * raw_derivative +
                (1.0 - self.derivative_filter_alpha) * self.filtered_derivative
        )
        d_term = self.kd * self.filtered_derivative

        self.prev_error = error

        # Sum and clamp output
        output = p_term + i_term + d_term
        return self._clamp(output, self.output_limit)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(min(value, limit), -limit)


class WaypointController(Node):
    """
    Advanced waypoint controller using PID with velocity profiling.

    Control Architecture:
        Position Error → PID → Velocity Command → Drone

    The controller uses velocity profiling to smoothly decelerate
    as the drone approaches the target, preventing overshoot and
    oscillation.

    Tuning Guide:
        - kp: Responsiveness (higher = faster response, but more overshoot)
        - ki: Steady-state error correction (keep low to avoid wind-up)
        - kd: Damping (higher = less overshoot, smoother approach)
        - max_velocity: Maximum speed limit
        - decel_radius: Distance at which to start decelerating
    """

    def __init__(self, name: str = "waypoint_controller"):
        super().__init__(name)

        # Thread-safe pose storage
        self._pose_lock = threading.Lock()
        self._pose = Pose()
        self._pose_received = False

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

        # =============================================================
        # Controller Parameters (tune these for your drone!)
        # =============================================================

        # XY (horizontal) PID gains
        # - Higher kp = faster response
        # - kd provides damping to prevent oscillation
        # - ki helps with steady-state error (keep small)
        self.pid_xy = PIDController(
            kp=0.8,  # Proportional: position error → velocity
            ki=0.02,  # Integral: eliminates steady-state error
            kd=0.3,  # Derivative: damping, reduces overshoot
            integral_limit=0.5,
            output_limit=0.5,  # Max velocity component
            derivative_filter_alpha=0.3,
        )

        # Z (vertical) PID gains - typically needs different tuning
        self.pid_z = PIDController(
            kp=1.0,  # Altitude is usually more responsive
            ki=0.05,  # Small integral for hover accuracy
            kd=0.4,  # Good damping for altitude
            integral_limit=0.3,
            output_limit=0.4,
            derivative_filter_alpha=0.3,
        )

        # Velocity profiling parameters
        self.max_velocity_xy = 0.4  # m/s - max horizontal speed
        self.max_velocity_z = 0.3  # m/s - max vertical speed
        self.decel_radius = 0.8  # m - start decelerating at this distance
        self.min_velocity = 0.05  # m/s - minimum velocity near target

        # Arrival detection
        self.position_tolerance = 0.08  # m - position error threshold
        self.velocity_tolerance = 0.05  # m/s - velocity threshold for "stopped"
        self.stable_count_required = 15  # iterations to confirm arrival

        # Control loop rate
        self.control_rate = 50  # Hz
        self.control_period = 1.0 / self.control_rate

        self.get_logger().info(
            f"WaypointController initialized with PID control. "
            f"XY gains: P={self.pid_xy.kp}, I={self.pid_xy.ki}, D={self.pid_xy.kd}"
        )

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
    # Velocity Profiling
    # ------------------------------------------------------------------ #

    def compute_velocity_limit(self, distance: float, max_vel: float) -> float:
        """
        Compute velocity limit based on distance to target.

        Uses a smooth deceleration profile:
        - Far from target: max_vel
        - Within decel_radius: linearly decrease
        - Very close: min_velocity

        This prevents overshoot by slowing down as we approach.
        """
        if distance > self.decel_radius:
            return max_vel

        # Linear deceleration within radius
        # velocity = min_vel + (max_vel - min_vel) * (distance / decel_radius)
        ratio = distance / self.decel_radius
        velocity = self.min_velocity + (max_vel - self.min_velocity) * ratio
        return max(velocity, self.min_velocity)

    # ------------------------------------------------------------------ #
    # Main Navigation Method
    # ------------------------------------------------------------------ #

    def goto(self, tx: float, ty: float, tz: float | None = None) -> bool:
        """
        Navigate to target position using PID control with velocity profiling.

        Args:
            tx: Target X coordinate (world frame, meters)
            ty: Target Y coordinate (world frame, meters)
            tz: Target Z coordinate (altitude, meters). If None, uses 1.5m.

        Returns:
            True if target reached, False if aborted/timeout
        """
        if tz is None:
            tz = 1.5  # Default altitude

        # Wait for initial pose
        if not self._wait_for_pose(timeout=10.0):
            return False

        # Reset PID controllers for fresh start
        self.pid_xy.reset()
        self.pid_z.reset()

        # Get starting position
        start = self.pose
        self.get_logger().info(
            f"[PID] Navigating to ({tx:.2f}, {ty:.2f}, {tz:.2f}) "
            f"from ({start.position.x:.2f}, {start.position.y:.2f}, {start.position.z:.2f})"
        )

        stable_counter = 0
        loop_count = 0
        start_time = time.time()

        while True:
            loop_start = time.time()

            # Get current pose
            current = self.pose
            x, y, z = current.position.x, current.position.y, current.position.z

            # Compute errors
            error_x = tx - x
            error_y = ty - y
            error_z = tz - z
            dist_xy = math.hypot(error_x, error_y)
            dist_3d = math.sqrt(dist_xy ** 2 + error_z ** 2)

            # Compute velocity limits based on distance (velocity profiling)
            vel_limit_xy = self.compute_velocity_limit(dist_xy, self.max_velocity_xy)
            vel_limit_z = self.compute_velocity_limit(abs(error_z), self.max_velocity_z)

            # Temporarily adjust PID output limits for velocity profiling
            self.pid_xy.output_limit = vel_limit_xy
            self.pid_z.output_limit = vel_limit_z

            # Compute PID outputs
            current_time = time.time()

            # For XY, we compute PID on each axis separately
            # But we also normalize to respect the velocity limit as a vector magnitude
            vel_x_raw = self.pid_xy.compute(error_x, current_time)

            # Create a separate PID for Y (sharing gains with X)
            # Actually, let's compute both properly
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

            # Z velocity from PID
            vel_z = self.pid_z.compute(error_z, current_time)

            # Publish velocity command
            twist = Twist()
            twist.linear.x = vel_x
            twist.linear.y = vel_y
            twist.linear.z = vel_z
            self.cmd_pub.publish(twist)

            # Arrival detection
            vel_magnitude = math.sqrt(vel_x ** 2 + vel_y ** 2 + vel_z ** 2)

            if dist_xy < self.position_tolerance and abs(error_z) < self.position_tolerance:
                stable_counter += 1
                if stable_counter >= self.stable_count_required:
                    self.stop()
                    elapsed = time.time() - start_time
                    self.get_logger().info(
                        f"[PID] Reached target ✓ at ({x:.2f}, {y:.2f}, {z:.2f}) "
                        f"in {elapsed:.1f}s"
                    )
                    return True
            else:
                stable_counter = 0

            # Periodic logging (every ~1 second)
            loop_count += 1
            if loop_count % self.control_rate == 0:
                self.get_logger().info(
                    f"  pos=({x:.2f}, {y:.2f}, {z:.2f}) "
                    f"err_xy={dist_xy:.3f} err_z={error_z:.3f} "
                    f"vel=({vel_x:.2f}, {vel_y:.2f}, {vel_z:.2f})"
                )

            # Timeout check (60 seconds max per waypoint)
            if time.time() - start_time > 60.0:
                self.get_logger().warn("[PID] Timeout reaching waypoint!")
                self.stop()
                return False

            # Maintain control loop rate
            elapsed = time.time() - loop_start
            sleep_time = self.control_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        return False

    def _compute_y_pid(self, error_y: float, current_time: float) -> float:
        """
        Compute Y-axis PID using same gains as X but separate state.

        We need separate integral/derivative state for Y axis.
        This is a simplified approach - for production, use separate PID instances.
        """
        # Use a simple P-D control for Y to avoid state management complexity
        # The main pid_xy handles X, this handles Y with same gains
        if not hasattr(self, '_y_integral'):
            self._y_integral = 0.0
            self._y_prev_error = 0.0
            self._y_prev_time = current_time
            self._y_filtered_deriv = 0.0

        dt = current_time - self._y_prev_time
        dt = max(dt, 0.001)
        self._y_prev_time = current_time

        # P term
        p_term = self.pid_xy.kp * error_y

        # I term with anti-windup
        self._y_integral += error_y * dt
        self._y_integral = max(min(self._y_integral, self.pid_xy.integral_limit),
                               -self.pid_xy.integral_limit)
        i_term = self.pid_xy.ki * self._y_integral

        # D term with filtering
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
        if hasattr(self, '_y_integral'):
            self._reset_y_pid()