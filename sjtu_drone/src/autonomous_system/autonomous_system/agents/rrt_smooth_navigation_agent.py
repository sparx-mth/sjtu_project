#!/usr/bin/env python3
"""
RRT Smooth Navigation Agent
----------------------------
Algorithm selection via parameter:
  1 = Cubic Spline + Pure Pursuit
  2 = Bezier + Pure Pursuit (default)
  3 = Minimum Snap + Feedforward
"""

import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from autonomous_system.srv import NavigateToPose, PlanPath
from geometry_msgs.msg import Pose, Twist

# Import smoothers
try:
    from autonomous_system.planning.trajectory_smoother import smooth_waypoints as cubic_smooth
except ImportError:
    cubic_smooth = None

try:
    from autonomous_system.planning.bezier_trajectory_smoother import smooth_waypoints as bezier_smooth
except ImportError:
    bezier_smooth = None

try:
    from autonomous_system.planning.minsnap_trajectory_smoother import smooth_waypoints as minsnap_smooth
except ImportError:
    minsnap_smooth = None


class RRTSmoothNavigationAgent(Node):
    """RRT Navigation Agent with selectable smoothing/tracking algorithm."""

    def __init__(self):
        super().__init__("rrt_smooth_navigation_agent")

        # Parameters
        self.declare_parameter("algorithm", 2)  # 1=cubic, 2=bezier, 3=minsnap
        self.declare_parameter("cruise_altitude", 1.5)
        self.declare_parameter("planner_timeout", 5.0)
        self.declare_parameter("cruise_speed", 0.4)
        self.declare_parameter("goal_tolerance", 0.15)

        self.algorithm = int(self.get_parameter("algorithm").value)
        self.cruise_altitude = float(self.get_parameter("cruise_altitude").value)
        self.planner_timeout = float(self.get_parameter("planner_timeout").value)
        self.cruise_speed = float(self.get_parameter("cruise_speed").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)

        # Select smoother based on algorithm
        smoothers = {1: cubic_smooth, 2: bezier_smooth, 3: minsnap_smooth}
        names = {1: "Cubic Spline", 2: "Bezier", 3: "MinSnap"}
        trackers = {1: "Pure Pursuit", 2: "Pure Pursuit", 3: "Feedforward"}

        if self.algorithm not in smoothers or smoothers[self.algorithm] is None:
            available = [k for k, v in smoothers.items() if v is not None]
            self.algorithm = available[0] if available else None
            if self.algorithm is None:
                raise RuntimeError("No trajectory smoother available!")

        self._smooth_waypoints = smoothers[self.algorithm]
        self._use_feedforward = (self.algorithm == 3)

        # State
        self._pose = Pose()
        self._vel = Twist()
        self._pose_received = False
        self._abort_flag = False
        self._busy = False

        # ROS interfaces
        self.cb_group = ReentrantCallbackGroup()
        self.pose_sub = self.create_subscription(
            Pose, "/simple_drone/gt_pose", self._pose_cb, 10, callback_group=self.cb_group)
        self.vel_sub = self.create_subscription(
            Twist, "/simple_drone/gt_vel", self._vel_cb, 10, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)

        # Control gains
        self.kp_xy, self.kp_z = 1.5, 1.2
        self.kd_xy, self.kd_z = 0.5, 0.3
        self.max_speed_xy, self.max_speed_z = 0.6, 0.3
        self.max_yaw_rate, self.yaw_kp = 0.5, 0.8
        self.control_rate = 50

        # Planner client
        self.planner_cb_group = MutuallyExclusiveCallbackGroup()
        self.planner_client = self.create_client(
            PlanPath, "/plan_path_rrt", callback_group=self.planner_cb_group)

        # Navigation service
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(
            NavigateToPose, "/navigate_rrt_smooth",
            self._handle_navigation_request, callback_group=self.service_cb_group)

        self.get_logger().info("Waiting for /plan_path_rrt service...")
        while not self.planner_client.wait_for_service(timeout_sec=1.0):
            pass

        self.get_logger().info(
            f"Agent ready: Algorithm {self.algorithm} ({names[self.algorithm]} + {trackers[self.algorithm]})")

    # ---- State callbacks ----
    def _pose_cb(self, msg): self._pose, self._pose_received = msg, True
    def _vel_cb(self, msg): self._vel = msg
    def clear_abort(self): self._abort_flag = False
    def is_aborted(self): return self._abort_flag

    def stop(self):
        twist = Twist()
        for _ in range(5):
            self.cmd_pub.publish(twist)
            time.sleep(0.02)

    def _wait_for_pose(self, timeout=10.0):
        start = time.time()
        while not self._pose_received:
            if time.time() - start > timeout:
                return False
            time.sleep(0.1)
        return True

    # ---- Helpers ----
    def _quat_to_yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

    def _normalize_angle(self, a):
        while a > math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _world_to_body(self, vx, vy, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return vx * c + vy * s, -vx * s + vy * c

    def _clamp(self, v, lim): return max(-lim, min(lim, v))

    # ---- RRT Planner ----
    def _call_planner(self, sx, sy, gx, gy) -> Tuple[bool, List[float], List[float], str]:
        req = PlanPath.Request()
        req.start_x, req.start_y, req.goal_x, req.goal_y = sx, sy, gx, gy
        future = self.planner_client.call_async(req)
        start = time.time()
        while not future.done():
            if time.time() - start > self.planner_timeout:
                return False, [], [], "Timeout"
            time.sleep(0.05)
        res = future.result()
        if res is None or not res.success:
            return False, [], [], res.message if res else "Failed"
        return True, list(res.waypoints_x), list(res.waypoints_y), res.message

    # ---- Feedforward Tracking (Algorithm 3) ----
    def _follow_feedforward(self, traj, alt, timeout=120.0):
        gx, gy, _ = traj.end
        start, traj_start = time.time(), time.time()
        dt = 1.0 / self.control_rate

        while True:
            t0 = time.time()
            if self.is_aborted():
                self.stop()
                return False, True

            p, v = self._pose, self._vel
            px, py, pz = p.position.x, p.position.y, p.position.z
            vx_act, vy_act = v.linear.x, v.linear.y
            yaw = self._quat_to_yaw(p.orientation)

            if math.hypot(gx - px, gy - py) < self.goal_tolerance:
                self.stop()
                return True, False

            t = time.time() - traj_start
            ref = traj.get_state_at_time(min(t, traj.total_time))

            e_px, e_py = self._clamp(ref.x - px, 1.0), self._clamp(ref.y - py, 1.0)
            e_vx, e_vy = ref.vx - vx_act, ref.vy - vy_act

            vx_cmd = ref.vx + self.kp_xy * e_px + self.kd_xy * e_vx
            vy_cmd = ref.vy + self.kp_xy * e_py + self.kd_xy * e_vy
            vz_cmd = self.kp_z * (alt - pz) + self.kd_z * (-v.linear.z)

            spd = math.hypot(vx_cmd, vy_cmd)
            if spd > self.max_speed_xy:
                vx_cmd, vy_cmd = vx_cmd * self.max_speed_xy / spd, vy_cmd * self.max_speed_xy / spd
            vz_cmd = self._clamp(vz_cmd, self.max_speed_z)

            yaw_rate = self._clamp(self.yaw_kp * self._normalize_angle(
                math.atan2(vy_cmd, vx_cmd) - yaw), self.max_yaw_rate) if spd > 0.05 else 0.0

            vx_b, vy_b = self._world_to_body(vx_cmd, vy_cmd, yaw)
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.linear.z = vx_b, vy_b, vz_cmd
            twist.angular.z = yaw_rate
            self.cmd_pub.publish(twist)

            if time.time() - start > timeout:
                self.stop()
                return False, False
            time.sleep(max(0, dt - (time.time() - t0)))

    # ---- Pure Pursuit Tracking (Algorithm 1 & 2) ----
    def _follow_pure_pursuit(self, traj, alt, timeout=120.0):
        gx, gy = traj.end[0], traj.end[1]
        start = time.time()
        dt = 1.0 / self.control_rate
        current_s, current_speed = 0.0, 0.0

        while True:
            t0 = time.time()
            if self.is_aborted():
                self.stop()
                return False, True

            p = self._pose
            px, py, pz = p.position.x, p.position.y, p.position.z
            yaw = self._quat_to_yaw(p.orientation)

            dist_goal = math.hypot(gx - px, gy - py)
            if dist_goal < self.goal_tolerance:
                self.stop()
                return True, False

            # Find closest point
            best_s, best_d = current_s, float('inf')
            for s in [current_s + i * 0.05 for i in range(-5, 20)]:
                if 0 <= s <= traj.total_length:
                    pt = traj.get_point(s)
                    d = math.hypot(pt.x - px, pt.y - py)
                    if d < best_d:
                        best_d, best_s = d, s
            current_s = max(current_s, best_s)

            # Lookahead
            lookahead = 0.6 + 0.5 * current_speed
            target = traj.get_point(min(current_s + lookahead, traj.total_length))
            dx, dy = target.x - px, target.y - py
            dist = math.hypot(dx, dy)

            # Speed
            target_speed = self.cruise_speed * (0.3 + 0.7 * min(1, dist_goal)) if dist_goal < 1.0 else self.cruise_speed
            current_speed = 0.3 * target_speed + 0.7 * current_speed

            vx, vy = (dx / dist * current_speed, dy / dist * current_speed) if dist > 0.01 else (0, 0)
            vz = self._clamp(self.kp_z * (alt - pz), self.max_speed_z)
            des_yaw = math.atan2(dy, dx) if dist > 0.01 else yaw
            yaw_rate = self._clamp(self.yaw_kp * self._normalize_angle(des_yaw - yaw), self.max_yaw_rate)

            vx_b, vy_b = self._world_to_body(vx, vy, yaw)
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.linear.z = vx_b, vy_b, vz
            twist.angular.z = yaw_rate
            self.cmd_pub.publish(twist)

            if time.time() - start > timeout:
                self.stop()
                return False, False
            time.sleep(max(0, dt - (time.time() - t0)))

    # ---- Navigation Handler ----
    def _handle_navigation_request(self, req, res):
        if self._busy:
            res.success, res.message = False, "Busy"
            return res

        self._busy = True
        self.clear_abort()

        gx, gy = float(req.x), float(req.y)
        gz = float(req.z) if req.z > 0 else self.cruise_altitude

        if not self._wait_for_pose():
            res.success, res.message = False, "No pose"
            self._busy = False
            return res

        sx, sy = self._pose.position.x, self._pose.position.y
        self.get_logger().info(f"Nav: ({sx:.2f},{sy:.2f}) -> ({gx:.2f},{gy:.2f})")

        # Plan
        ok, wp_x, wp_y, msg = self._call_planner(sx, sy, gx, gy)
        if not ok:
            res.success, res.message = False, f"Plan failed: {msg}"
            self._busy = False
            return res

        # Smooth
        traj = self._smooth_waypoints(wp_x, wp_y)
        if traj is None:
            res.success, res.message = False, "Smoothing failed"
            self._busy = False
            return res

        self.get_logger().info(f"Trajectory: {traj.total_length:.2f}m")

        # Follow
        if self._use_feedforward:
            reached, aborted = self._follow_feedforward(traj, gz)
        else:
            reached, aborted = self._follow_pure_pursuit(traj, gz)

        res.success = reached and not aborted
        res.message = "Aborted" if aborted else ("Reached" if reached else "Failed")
        self._busy = False
        return res


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