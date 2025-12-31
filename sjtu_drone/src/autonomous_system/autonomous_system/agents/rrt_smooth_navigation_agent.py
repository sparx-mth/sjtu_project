#!/usr/bin/env python3
"""
RRT Smooth Navigation Agent (v2 - Full State Feedback)
-------------------------------------------------------
Supports TWO tracking pipelines:

Pipeline 1 (legacy): RRT* → Cubic Spline → Pure Pursuit
Pipeline 2 (new):    RRT* → Minimum-Snap → Full State Feedback

Control law for MinSnap mode:
    v_cmd = v_feedforward + Kp*(pos_error) + Kd*(vel_error)

Select via parameter: tracking_mode = "pure_pursuit" | "minsnap"
"""

import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from autonomous_system.srv import NavigateToPose, PlanPath

# Use minsnap trajectory smoother (has total_time, full state info)
from autonomous_system.planning.minsnap_trajectory_smoother import SmoothTrajectory, smooth_waypoints

from geometry_msgs.msg import Pose, Twist


class RRTSmoothNavigationAgent(Node):
    """
    RRT Navigation Agent with selectable tracking mode.

    Modes:
    - "pure_pursuit": Uses Pure Pursuit (geometry-based, arc-length tracking)
    - "minsnap": Uses Full State Feedback (position + velocity feedback)
    """

    def __init__(self):
        super().__init__("rrt_smooth_navigation_agent")

        # Parameters
        self.declare_parameter("cruise_altitude", 1.5)
        self.declare_parameter("planner_timeout", 5.0)
        self.declare_parameter("cruise_speed", 0.4)
        self.declare_parameter("goal_tolerance", 0.15)
        self.declare_parameter("tracking_mode", "minsnap")  # "pure_pursuit" or "minsnap"

        self.cruise_altitude = float(self.get_parameter("cruise_altitude").value)
        self.planner_timeout = float(self.get_parameter("planner_timeout").value)
        self.cruise_speed = float(self.get_parameter("cruise_speed").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.tracking_mode = str(self.get_parameter("tracking_mode").value)

        # State storage
        self._pose = Pose()
        self._vel = Twist()
        self._pose_received = False
        self._vel_received = False
        self._abort_flag = False

        # ROS interfaces
        self.pose_cb_group = ReentrantCallbackGroup()
        self.pose_sub = self.create_subscription(
            Pose, "/simple_drone/gt_pose", self._pose_cb, 10,
            callback_group=self.pose_cb_group,
        )
        self.vel_sub = self.create_subscription(
            Twist, "/simple_drone/gt_vel", self._vel_cb, 10,
            callback_group=self.pose_cb_group,
        )
        self.cmd_pub = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)

        # ===========================================
        # Control Gains
        # ===========================================
        # Position feedback
        self.kp_xy = 1.5
        self.kp_z = 1.2

        # Velocity feedback (damping)
        self.kd_xy = 0.5
        self.kd_z = 0.3

        # Limits
        self.max_speed_xy = 0.6
        self.max_speed_z = 0.3
        self.max_yaw_rate = 0.5
        self.yaw_kp = 0.8
        self.control_rate = 50

        # Client to C++ RRT planner
        self.planner_cb_group = MutuallyExclusiveCallbackGroup()
        self.planner_client = self.create_client(
            PlanPath,
            "/plan_path_rrt",
            callback_group=self.planner_cb_group
        )

        # Navigation service
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(
            NavigateToPose,
            "/navigate_rrt_smooth",
            self._handle_navigation_request,
            callback_group=self.service_cb_group,
        )

        self._busy = False

        # Wait for planner service
        self.get_logger().info("Waiting for /plan_path_rrt service...")
        while not self.planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("  Still waiting...")

        self.get_logger().info(f"RRT Navigation Agent ready (mode: {self.tracking_mode})")
        self.get_logger().info(f"  Gains: Kp={self.kp_xy}, Kd={self.kd_xy}")

    # ----------------------------------------------------------------
    # State callbacks
    # ----------------------------------------------------------------

    @property
    def pose(self) -> Pose:
        return self._pose

    @property
    def vel(self) -> Twist:
        return self._vel

    @property
    def pose_received(self) -> bool:
        return self._pose_received

    def _pose_cb(self, msg: Pose) -> None:
        self._pose = msg
        self._pose_received = True

    def _vel_cb(self, msg: Twist) -> None:
        self._vel = msg
        self._vel_received = True

    def clear_abort(self):
        self._abort_flag = False

    def is_aborted(self) -> bool:
        return self._abort_flag

    def stop(self) -> None:
        twist = Twist()
        for _ in range(5):
            self.cmd_pub.publish(twist)
            time.sleep(0.02)

    def _wait_for_pose(self, timeout: float = 10.0) -> bool:
        start = time.time()
        while not self.pose_received:
            if time.time() - start > timeout:
                return False
            time.sleep(0.1)
        return True

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------

    def _quaternion_to_yaw(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _world_to_body(self, vx: float, vy: float, yaw: float) -> Tuple[float, float]:
        c, s = math.cos(yaw), math.sin(yaw)
        return vx * c + vy * s, -vx * s + vy * c

    def _clamp(self, val: float, limit: float) -> float:
        return max(-limit, min(limit, val))

    # ----------------------------------------------------------------
    # RRT Planner
    # ----------------------------------------------------------------

    def _call_planner(
            self,
            start_x: float,
            start_y: float,
            goal_x: float,
            goal_y: float,
    ) -> Tuple[bool, List[float], List[float], str]:
        """Call C++ RRT planner service."""
        request = PlanPath.Request()
        request.start_x = start_x
        request.start_y = start_y
        request.goal_x = goal_x
        request.goal_y = goal_y

        future = self.planner_client.call_async(request)

        start_time = time.time()
        while not future.done():
            if time.time() - start_time > self.planner_timeout:
                return False, [], [], "Planner timeout"
            time.sleep(0.05)

        result = future.result()
        if result is None:
            return False, [], [], "Planner call failed"

        if not result.success:
            return False, [], [], result.message

        return (
            True,
            list(result.waypoints_x),
            list(result.waypoints_y),
            result.message
        )

    # ----------------------------------------------------------------
    # MinSnap: Full State Feedback (Position + Velocity)
    # ----------------------------------------------------------------

    def _follow_minsnap(
            self,
            trajectory: SmoothTrajectory,
            target_altitude: float,
            timeout: float = 120.0,
    ) -> Tuple[bool, bool]:
        """
        Follow trajectory using full state feedback.

        Control law:
            v_cmd = v_feedforward + Kp*(pos_error) + Kd*(vel_error)
        """
        goal_x, goal_y, _ = trajectory.end
        start_time = time.time()
        traj_start = time.time()
        dt = 1.0 / self.control_rate

        self.get_logger().info(
            f"[MinSnap] Following: {trajectory.total_time:.1f}s, "
            f"{trajectory.total_length:.1f}m"
        )

        loop_count = 0
        while True:
            loop_start = time.time()

            if self.is_aborted():
                self.stop()
                return False, True

            # Current state
            p = self.pose
            v = self.vel
            px, py, pz = p.position.x, p.position.y, p.position.z
            vx_actual, vy_actual, vz_actual = v.linear.x, v.linear.y, v.linear.z
            yaw = self._quaternion_to_yaw(p.orientation)

            # Goal check
            dist_goal = math.hypot(goal_x - px, goal_y - py)
            if dist_goal < self.goal_tolerance:
                self.stop()
                elapsed = time.time() - start_time
                self.get_logger().info(
                    f"Goal reached in {elapsed:.1f}s! err={dist_goal*100:.1f}cm"
                )
                return True, False

            # Get reference from trajectory
            t = time.time() - traj_start
            if t < trajectory.total_time:
                ref = trajectory.get_state_at_time(t)
            else:
                ref = trajectory.get_state_at_time(trajectory.total_time)

            # ===========================================
            # Full State Feedback Control Law
            # v_cmd = v_ff + Kp*e_pos + Kd*e_vel
            # ===========================================

            # Position error
            e_px = ref.x - px
            e_py = ref.y - py
            e_pz = target_altitude - pz

            # Velocity error
            e_vx = ref.vx - vx_actual
            e_vy = ref.vy - vy_actual
            e_vz = 0.0 - vz_actual  # Target zero vertical velocity

            # Clamp position error contribution
            e_px = self._clamp(e_px, 1.0)
            e_py = self._clamp(e_py, 1.0)

            # Control output
            vx_cmd = ref.vx + self.kp_xy * e_px + self.kd_xy * e_vx
            vy_cmd = ref.vy + self.kp_xy * e_py + self.kd_xy * e_vy
            vz_cmd = self.kp_z * e_pz + self.kd_z * e_vz

            # Apply speed limits
            speed = math.hypot(vx_cmd, vy_cmd)
            if speed > self.max_speed_xy:
                vx_cmd *= self.max_speed_xy / speed
                vy_cmd *= self.max_speed_xy / speed
            vz_cmd = self._clamp(vz_cmd, self.max_speed_z)

            # Yaw control: face direction of motion
            if speed > 0.05:
                des_yaw = math.atan2(vy_cmd, vx_cmd)
                yaw_err = self._normalize_angle(des_yaw - yaw)
                yaw_rate = self._clamp(self.yaw_kp * yaw_err, self.max_yaw_rate)
            else:
                yaw_rate = 0.0

            # Transform to body frame
            vx_body, vy_body = self._world_to_body(vx_cmd, vy_cmd, yaw)

            # Publish command
            twist = Twist()
            twist.linear.x = vx_body
            twist.linear.y = vy_body
            twist.linear.z = vz_cmd
            twist.angular.z = yaw_rate
            self.cmd_pub.publish(twist)

            # Logging
            loop_count += 1
            if loop_count % 50 == 0:
                pos_err = math.hypot(e_px, e_py)
                vel_err = math.hypot(e_vx, e_vy)
                self.get_logger().info(
                    f"t={t:.1f}s | pos_err={pos_err:.2f}m vel_err={vel_err:.2f}m/s | "
                    f"v_ff=({ref.vx:.2f},{ref.vy:.2f}) v_cmd=({vx_cmd:.2f},{vy_cmd:.2f})"
                )

            # Timeout
            if time.time() - start_time > timeout:
                self.stop()
                return False, False

            # Rate control
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

        return False, False

    # ----------------------------------------------------------------
    # Pure Pursuit (Legacy)
    # ----------------------------------------------------------------

    def _follow_pure_pursuit(
            self,
            trajectory: SmoothTrajectory,
            target_altitude: float,
            timeout: float = 120.0,
    ) -> Tuple[bool, bool]:
        """Follow trajectory using Pure Pursuit (arc-length based)."""
        goal_x, goal_y, _ = trajectory.end
        start_time = time.time()
        dt = 1.0 / self.control_rate

        base_lookahead = 0.6
        current_s = 0.0
        current_speed = 0.0

        self.get_logger().info(f"[PurePursuit] Following: {trajectory.total_length:.1f}m")

        loop_count = 0
        while True:
            loop_start = time.time()

            if self.is_aborted():
                self.stop()
                return False, True

            p = self.pose
            px, py, pz = p.position.x, p.position.y, p.position.z
            yaw = self._quaternion_to_yaw(p.orientation)

            dist_goal = math.hypot(goal_x - px, goal_y - py)
            if dist_goal < self.goal_tolerance:
                self.stop()
                self.get_logger().info(f"Goal reached! err={dist_goal*100:.1f}cm")
                return True, False

            # Find closest point
            best_s, best_dist = current_s, float('inf')
            for s in [current_s + i * 0.05 for i in range(-5, 20)]:
                if 0 <= s <= trajectory.total_length:
                    pt = trajectory.get_point(s)
                    d = math.hypot(pt.x - px, pt.y - py)
                    if d < best_dist:
                        best_dist = d
                        best_s = s
            current_s = max(current_s, best_s)

            # Lookahead
            lookahead = base_lookahead + 0.5 * current_speed
            target_s = min(current_s + lookahead, trajectory.total_length)
            target = trajectory.get_point(target_s)

            dx, dy = target.x - px, target.y - py
            dist = math.hypot(dx, dy)

            # Speed
            target_speed = self.cruise_speed
            if dist_goal < 1.0:
                target_speed *= 0.3 + 0.7 * dist_goal
            current_speed = 0.3 * target_speed + 0.7 * current_speed

            if dist > 0.01:
                vx = (dx / dist) * current_speed
                vy = (dy / dist) * current_speed
                des_yaw = math.atan2(dy, dx)
            else:
                vx, vy = 0.0, 0.0
                des_yaw = yaw

            vz = self._clamp(self.kp_z * (target_altitude - pz), self.max_speed_z)
            yaw_rate = self._clamp(
                self.yaw_kp * self._normalize_angle(des_yaw - yaw),
                self.max_yaw_rate
            )

            vx_b, vy_b = self._world_to_body(vx, vy, yaw)

            twist = Twist()
            twist.linear.x = vx_b
            twist.linear.y = vy_b
            twist.linear.z = vz
            twist.angular.z = yaw_rate
            self.cmd_pub.publish(twist)

            loop_count += 1
            if loop_count % 50 == 0:
                progress = 100 * current_s / trajectory.total_length
                self.get_logger().info(f"progress={progress:.0f}% speed={current_speed:.2f}")

            if time.time() - start_time > timeout:
                self.stop()
                return False, False

            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

        return False, False

    # ----------------------------------------------------------------
    # Navigation Handler
    # ----------------------------------------------------------------

    def _handle_navigation_request(
            self,
            request: NavigateToPose.Request,
            response: NavigateToPose.Response,
    ) -> NavigateToPose.Response:
        """Handle navigation request."""

        if self._busy:
            response.success = False
            response.message = "Navigation agent is busy"
            return response

        self._busy = True
        self.clear_abort()

        goal_x = float(request.x)
        goal_y = float(request.y)
        goal_z = float(request.z) if request.z > 0 else self.cruise_altitude

        if not self._wait_for_pose(timeout=10.0):
            response.success = False
            response.message = "Timeout waiting for drone pose"
            self._busy = False
            return response

        current = self.pose
        start_x = float(current.position.x)
        start_y = float(current.position.y)

        self.get_logger().info(
            f"Navigation: ({start_x:.2f}, {start_y:.2f}) -> ({goal_x:.2f}, {goal_y:.2f})"
        )

        # Step 1: RRT* planning
        success, wp_x, wp_y, message = self._call_planner(
            start_x, start_y, goal_x, goal_y
        )

        if not success:
            response.success = False
            response.message = f"Planning failed: {message}"
            self._busy = False
            return response

        self.get_logger().info(f"RRT* returned {len(wp_x)} waypoints")

        # Step 2: Smooth trajectory
        trajectory = smooth_waypoints(wp_x, wp_y)

        if trajectory is None:
            response.success = False
            response.message = "Failed to create smooth trajectory"
            self._busy = False
            return response

        self.get_logger().info(
            f"Trajectory: {trajectory.total_length:.2f}m, {trajectory.total_time:.2f}s"
        )

        # Step 3: Follow using selected mode
        if self.tracking_mode == "minsnap":
            reached, aborted = self._follow_minsnap(trajectory, goal_z)
        else:
            reached, aborted = self._follow_pure_pursuit(trajectory, goal_z)

        if aborted:
            response.success = False
            response.message = "Navigation aborted"
        elif reached:
            response.success = True
            response.message = "Goal reached"
        else:
            response.success = False
            response.message = "Failed to reach goal"

        self._busy = False
        return response


def main():
    rclpy.init()
    node = RRTSmoothNavigationAgent()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()