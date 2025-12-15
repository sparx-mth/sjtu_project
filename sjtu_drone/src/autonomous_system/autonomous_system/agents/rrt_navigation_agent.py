#!/usr/bin/env python3
"""
RRT Navigation Agent (Service-based)
-------------------------------------
Uses C++ RRT* planner service for path planning,
then executes waypoints using WaypointController.
"""

import time
from typing import List, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from autonomous_system.srv import NavigateToPose, PlanPath
from autonomous_system.control.waypoint_controller import WaypointController


class RRTNavigationAgent(WaypointController):
    """
    RRT Navigation Agent:
    - Calls /plan_path_rrt (C++) for path planning
    - Executes waypoints using WaypointController
    """

    def __init__(self):
        super().__init__(name="rrt_navigation_agent")

        # Parameters
        self.declare_parameter("cruise_altitude", 1.5)
        self.declare_parameter("waypoint_tolerance", 0.3)
        self.declare_parameter("planner_timeout", 5.0)

        self.cruise_altitude = float(self.get_parameter("cruise_altitude").value)
        self.waypoint_tolerance = float(self.get_parameter("waypoint_tolerance").value)
        self.planner_timeout = float(self.get_parameter("planner_timeout").value)

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
            "/navigate_rrt",
            self.handle_navigation_request,
            callback_group=self.service_cb_group,
        )

        self.busy = False

        # Wait for planner service
        self.get_logger().info("Waiting for /plan_path_rrt service...")
        while not self.planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("  Still waiting...")

        self.get_logger().info("RRT Navigation Agent ready on /navigate_rrt")

    def call_planner(self, start_x: float, start_y: float,
                     goal_x: float, goal_y: float) -> Tuple[bool, List[Tuple[float, float]], str]:
        """Call C++ RRT planner service."""
        request = PlanPath.Request()
        request.start_x = start_x
        request.start_y = start_y
        request.goal_x = goal_x
        request.goal_y = goal_y

        future = self.planner_client.call_async(request)

        # Wait for result with timeout
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > self.planner_timeout:
                return False, [], "Planner timeout"
            time.sleep(0.05)

        result = future.result()
        if result is None:
            return False, [], "Planner call failed"

        if not result.success:
            return False, [], result.message

        # Convert arrays to list of tuples
        waypoints = list(zip(result.waypoints_x, result.waypoints_y))
        return True, waypoints, result.message

    def handle_navigation_request(
            self,
            request: NavigateToPose.Request,
            response: NavigateToPose.Response,
    ) -> NavigateToPose.Response:
        """Handle navigation request using RRT* planning."""

        if self.busy:
            response.success = False
            response.message = "RRT agent is busy."
            return response

        self.busy = True
        self.clear_abort()

        goal_x = float(request.x)
        goal_y = float(request.y)
        goal_z = float(request.z) if request.z > 0 else self.cruise_altitude

        # Get current position
        current = self.pose
        start_x = float(current.position.x)
        start_y = float(current.position.y)

        self.get_logger().info(
            f"Planning RRT*: ({start_x:.2f}, {start_y:.2f}) -> ({goal_x:.2f}, {goal_y:.2f})"
        )

        # Call C++ planner
        success, waypoints, message = self.call_planner(start_x, start_y, goal_x, goal_y)

        if not success:
            response.success = False
            response.message = f"Planning failed: {message}"
            self.busy = False
            return response

        self.get_logger().info(f"Path received: {len(waypoints)} waypoints")

        # Execute waypoints using WaypointController
        for i, (wx, wy) in enumerate(waypoints):
            self.get_logger().info(f"Waypoint {i + 1}/{len(waypoints)}: ({wx:.2f}, {wy:.2f})")

            reached, aborted = self.goto(wx, wy, goal_z)

            if aborted:
                response.success = False
                response.message = f"Aborted at waypoint {i + 1}"
                self.busy = False
                return response

            if not reached:
                response.success = False
                response.message = f"Failed to reach waypoint {i + 1}"
                self.busy = False
                return response

        # Check if we reached the goal
        current = self.pose
        dist_to_goal = ((current.position.x - goal_x) ** 2 +
                        (current.position.y - goal_y) ** 2) ** 0.5

        if dist_to_goal < self.waypoint_tolerance:
            response.success = True
            response.message = "Goal reached"
        else:
            response.success = True
            response.message = f"Completed path, {dist_to_goal:.2f}m from goal"

        self.busy = False
        return response


def main():
    rclpy.init()
    node = RRTNavigationAgent()

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