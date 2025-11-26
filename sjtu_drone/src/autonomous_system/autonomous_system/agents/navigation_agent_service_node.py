#!/usr/bin/env python3
"""
Navigation Agent (Service-based)
--------------------------------
Receives a navigation request (target pose in world coordinates),
plans a path using A* with turn penalty, simplifies it,
then executes the path using the WaypointController.

Coordinate Convention:
    - World coordinates: (wx, wy) in meters
    - Grid coordinates: (gx, gy) with origin at bottom-left

IMPORTANT: Uses MultiThreadedExecutor with proper callback groups
to allow pose updates while navigation is in progress.
"""

import time
from typing import List, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from autonomous_system.srv import NavigateToPose
from autonomous_system.control.waypoint_controller import WaypointController
from autonomous_system.planning.astar_planner import AStarPlanner
from autonomous_system.planning.path_simplifier import rdp_simplify, extract_turn_points

GridPoint = Tuple[int, int]
WorldPoint = Tuple[float, float]


class NavigationAgentService(WaypointController):
    """
    High-level navigation agent implemented as a ROS2 service.

    Service:
        /navigate_to_pose : NavigateToPose

    Behavior:
        1. Receives target XYZ (world coordinates)
        2. Reads current pose (from WaypointController)
        3. Runs A* with turn penalty
        4. Applies RDP + turn-based simplification
        5. Executes waypoints using goto()
        6. Returns success/failure to caller

    Threading Model:
        - Pose subscription: ReentrantCallbackGroup (inherited from WaypointController)
        - Service: MutuallyExclusiveCallbackGroup (can block without affecting pose)
        - MultiThreadedExecutor ensures both can run concurrently
    """

    def __init__(self):
        super().__init__(name="navigation_agent_service")

        # Parameters
        self.declare_parameter(
            "map_yaml",
            "/root/sjtu_project/sjtu_drone/maps/hospital_map_cropped.yaml",
        )
        self.declare_parameter("turn_penalty", 1.5)
        self.declare_parameter("rdp_eps", 1.5)
        self.declare_parameter("min_turn_dist", 6.0)
        self.declare_parameter("cruise_altitude", 1.5)

        map_yaml = self.get_parameter("map_yaml").value
        turn_penalty = float(self.get_parameter("turn_penalty").value)
        self.rdp_eps = float(self.get_parameter("rdp_eps").value)
        self.min_turn_dist = float(self.get_parameter("min_turn_dist").value)
        self.cruise_altitude = float(self.get_parameter("cruise_altitude").value)

        self.planner = AStarPlanner(map_yaml_path=map_yaml, turn_penalty=turn_penalty)

        # Service uses its own callback group so it doesn't block pose updates
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(
            NavigateToPose,
            "/navigate_to_pose",
            self.handle_navigation_request,
            callback_group=self.service_cb_group,
        )

        self.busy = False
        self.get_logger().info("NavigationAgentService ready.")

    def handle_navigation_request(
            self,
            request: NavigateToPose.Request,
            response: NavigateToPose.Response,
    ) -> NavigateToPose.Response:
        """Main navigation logic."""
        if self.busy:
            response.success = False
            response.message = "Navigation agent is busy."
            return response

        self.busy = True

        # Wait briefly to ensure we have fresh pose data
        time.sleep(0.1)

        # Current pose (world coordinates)
        current = self.pose
        start_wx = float(current.position.x)
        start_wy = float(current.position.y)

        # Goal pose (world coordinates)
        goal_wx = float(request.x)
        goal_wy = float(request.y)
        goal_wz = float(request.z)

        self.get_logger().info(
            f"New navigation task: "
            f"from=({start_wx:.2f}, {start_wy:.2f}) "
            f"to=({goal_wx:.2f}, {goal_wy:.2f}, {goal_wz:.2f})"
        )

        # Convert world -> grid
        start_grid: GridPoint = self.planner.world_to_map(start_wx, start_wy)
        goal_grid: GridPoint = self.planner.world_to_map(goal_wx, goal_wy)

        self.get_logger().info(f"Planning: start={start_grid}, goal={goal_grid}")

        # A* planning (grid coordinates)
        path: List[GridPoint] = self.planner.plan(start_grid, goal_grid)
        if not path:
            response.success = False
            response.message = "No path found."
            self.busy = False
            self.get_logger().error("No path found for navigation request.")
            return response

        # Simplify path (grid coordinates)
        simplified = rdp_simplify(path, eps=self.rdp_eps)
        waypoints_grid = extract_turn_points(simplified, min_dist=self.min_turn_dist)

        self.get_logger().info(
            f"Path: raw={len(path)}, simplified={len(simplified)}, "
            f"waypoints={len(waypoints_grid)}"
        )

        # Convert to world coordinates and execute
        for i, (gx, gy) in enumerate(waypoints_grid):
            wx, wy = self.planner.map_to_world(gx, gy)
            self.get_logger().info(
                f"→ Waypoint {i + 1}/{len(waypoints_grid)}: "
                f"({wx:.2f}, {wy:.2f}, {self.cruise_altitude:.2f})"
            )
            self.goto(wx, wy, tz=self.cruise_altitude)

            # Settling time between waypoints (from old working code)
            time.sleep(1.0)

        self.get_logger().info("Navigation task completed ✓")
        response.success = True
        response.message = "Reached target."
        self.busy = False
        return response


def main():
    rclpy.init()
    node = NavigationAgentService()

    # MultiThreadedExecutor with explicit thread count
    # This ensures pose callbacks can run while service is blocking in goto()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        node.get_logger().info("Starting executor with 4 threads...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()