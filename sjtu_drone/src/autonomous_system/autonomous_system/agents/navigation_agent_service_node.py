#!/usr/bin/env python3
"""
Navigation Agent (Service-based)
--------------------------------
Receives a navigation request (target pose in world coordinates),
plans a path using A* with turn penalty, simplifies it,
then executes the path using the WaypointController.

When finished, returns success/failure to the caller.
"""

from typing import List, Tuple

import rclpy
from rclpy.node import Node

from autonomous_system.srv import NavigateToPose
from autonomous_system.control.waypoint_controller import WaypointController
from autonomous_system.planning.astar_planner import AStarPlanner
from autonomous_system.planning.path_simplifier import (
    rdp_simplify,
    extract_turn_points,
)


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

        self.srv = self.create_service(
            NavigateToPose,
            "/navigate_to_pose",
            self.handle_navigation_request,
        )

        self.busy = False
        self.get_logger().info("NavigationAgentService ready.")

    # ------------------------------------------------------------------ #

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

        # Current pose (world)
        sx = float(self.pose.position.x)
        sy = float(self.pose.position.y)

        # Goal pose (world)
        gx = float(request.x)
        gy = float(request.y)
        gz = float(request.z)

        self.get_logger().info(
            f"New navigation task: target=({gx:.2f}, {gy:.2f}, {gz:.2f})"
        )

        # Convert world -> grid
        start: GridPoint = self.planner.world_to_map(sx, sy)
        goal: GridPoint = self.planner.world_to_map(gx, gy)

        self.get_logger().info(f"Planning in grid: start={start}, goal={goal}")

        # A* planning
        path: List[GridPoint] = self.planner.plan(start, goal)
        if not path:
            response.success = False
            response.message = "No path found."
            self.busy = False
            self.get_logger().error("No path found for navigation request.")
            return response

        # Simplify path
        simplified = rdp_simplify(path, eps=self.rdp_eps)
        waypoints_grid = extract_turn_points(
            simplified,
            min_dist=self.min_turn_dist,
        )

        self.get_logger().info(
            f"Path: raw={len(path)}, simplified={len(simplified)}, "
            f"waypoints={len(waypoints_grid)}"
        )

        # Convert to world waypoints
        waypoints_world: List[WorldPoint] = [
            self.planner.map_to_world(gx, gy) for gx, gy in waypoints_grid
        ]

        # Execute waypoints
        for i, (wx, wy) in enumerate(waypoints_world):
            self.get_logger().info(
                f"→ Executing waypoint {i+1}/{len(waypoints_world)}: "
                f"({wx:.2f}, {wy:.2f}, {self.cruise_altitude:.2f})"
            )
            self.goto(wx, wy, tz=self.cruise_altitude)

        self.get_logger().info("Navigation task completed ✓")
        response.success = True
        response.message = "Reached target."
        self.busy = False
        return response


def main():
    rclpy.init()
    node = NavigationAgentService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
