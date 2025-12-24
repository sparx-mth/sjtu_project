#!/usr/bin/env python3
"""
RRT Smooth Navigation Agent
----------------------------
Uses C++ RRT* planner for path planning, then follows the path
smoothly using trajectory smoothing and Pure Pursuit.

Key difference from original:
- Does NOT stop at each waypoint
- Converts waypoints to smooth cubic spline trajectory
- Uses Pure Pursuit for natural, curved motion
"""

import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from autonomous_system.srv import NavigateToPose, PlanPath
from autonomous_system.planning.trajectory_smoother import SmoothTrajectory, smooth_waypoints
from autonomous_system.control.smooth_path_follower import SmoothPathFollower


class RRTSmoothNavigationAgent(SmoothPathFollower):
    """
    RRT Navigation Agent with smooth trajectory following.

    Pipeline:
    1. Call /plan_path_rrt (C++) for RRT* path planning
    2. Convert waypoints to smooth spline trajectory
    3. Follow trajectory with Pure Pursuit (no stopping at waypoints)
    """

    def __init__(self):
        super().__init__(name="rrt_smooth_navigation_agent")

        # Parameters
        self.declare_parameter("cruise_altitude", 1.5)
        self.declare_parameter("planner_timeout", 5.0)
        self.declare_parameter("cruise_speed", 0.4)
        self.declare_parameter("goal_tolerance", 0.15)

        self.cruise_altitude = float(self.get_parameter("cruise_altitude").value)
        self.planner_timeout = float(self.get_parameter("planner_timeout").value)
        self.cruise_speed = float(self.get_parameter("cruise_speed").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)

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

        self.get_logger().info("RRT Smooth Navigation Agent ready on /navigate_rrt_smooth")

    def _call_planner(
            self,
            start_x: float,
            start_y: float,
            goal_x: float,
            goal_y: float,
    ) -> Tuple[bool, List[float], List[float], str]:
        """
        Call C++ RRT planner service.

        Returns:
            Tuple of (success, waypoints_x, waypoints_y, message)
        """
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

    def _handle_navigation_request(
            self,
            request: NavigateToPose.Request,
            response: NavigateToPose.Response,
    ) -> NavigateToPose.Response:
        """Handle navigation request using smooth trajectory following."""

        if self._busy:
            response.success = False
            response.message = "Navigation agent is busy"
            return response

        self._busy = True
        self.clear_abort()

        goal_x = float(request.x)
        goal_y = float(request.y)
        goal_z = float(request.z) if request.z > 0 else self.cruise_altitude

        # Wait for valid pose before planning
        if not self._wait_for_pose(timeout=10.0):
            response.success = False
            response.message = "Timeout waiting for drone pose"
            self._busy = False
            return response

        # Get current position
        current = self.pose
        start_x = float(current.position.x)
        start_y = float(current.position.y)

        self.get_logger().info(
            f"Navigation request: ({start_x:.2f}, {start_y:.2f}) -> "
            f"({goal_x:.2f}, {goal_y:.2f})"
        )

        # Step 1: Call RRT* planner
        success, wp_x, wp_y, message = self._call_planner(
            start_x, start_y, goal_x, goal_y
        )

        if not success:
            response.success = False
            response.message = f"Planning failed: {message}"
            self._busy = False
            return response

        self.get_logger().info(f"RRT* returned {len(wp_x)} waypoints")

        # Step 2: Create smooth trajectory
        trajectory = smooth_waypoints(wp_x, wp_y)

        if trajectory is None:
            response.success = False
            response.message = "Failed to create smooth trajectory"
            self._busy = False
            return response

        self.get_logger().info(
            f"Smooth trajectory: {trajectory.total_length:.2f}m"
        )

        # Step 3: Follow smooth trajectory
        reached, aborted = self.follow_trajectory(
            trajectory=trajectory,
            target_altitude=goal_z,
        )

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