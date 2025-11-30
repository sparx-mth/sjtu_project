#!/usr/bin/env python3
"""
Doorway Traversal Agent (Service-based)
----------------------------------------
Receives a request to traverse through the nearest door.
The agent finds the closest door, determines if the drone is inside or outside
the room, then plans and executes a path to go through the door to the other side.

Coordinate Convention:
    - World coordinates: (wx, wy) in meters
    - Grid coordinates: (gx, gy) with origin at bottom-left (as in show_drone_map.py)
"""

import math
import time
from typing import List, Tuple, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from autonomous_system.srv import NavigateToPose
from autonomous_system.control.waypoint_controller import WaypointController
from autonomous_system.planning.astar_planner import AStarPlanner
from autonomous_system.planning.path_simplifier import rdp_simplify, extract_turn_points

GridPoint = Tuple[int, int]
WorldPoint = Tuple[float, float]


# Door coordinates in grid/map pixel coordinates (from show_drone_map.py)
# Each tuple represents the center of a door
DOOR_POSITIONS_GRID: List[GridPoint] = [
    (133, 75),
    (256, 75),
    (157, 252),
    (457, 225),
    (157, 299),
    (157, 649),
    (482, 298),
    (249, 475),
    (390, 475),
    (249, 624),
    (390, 624),
    (482, 649),
    (390, 875),
    (249, 875),
    (140, 862),
    (500, 862),
    (93, 996),
    (93, 1156),
    (150, 1175),
    (188, 1250),
    (545, 996),
    (545, 1156),
    (488, 1175),
    (448, 1250),
]


class DoorwayTraversalAgent(WaypointController):
    """
    Doorway traversal agent implemented as a ROS2 service.

    Service:
        /traverse_doorway : NavigateToPose (reusing the same service type)

    Behavior:
        1. Receives traversal request (can ignore target coords, or use them as hint)
        2. Finds the nearest door to current drone position
        3. Determines which side of the door the drone is on
        4. Plans a path through the door center to the other side
        5. Executes the path using PID-controlled goto()
        6. Returns success/failure to caller
    """

    def __init__(self):
        super().__init__(name="doorway_traversal_agent")

        # Parameters
        self.declare_parameter(
            "map_yaml",
            "/root/sjtu_project/sjtu_drone/maps/hospital_map_cropped.yaml",
        )
        self.declare_parameter("turn_penalty", 1.5)
        self.declare_parameter("rdp_eps", 1.5)
        self.declare_parameter("min_turn_dist", 4.0)
        self.declare_parameter("cruise_altitude", 1.5)
        self.declare_parameter("door_traverse_distance", 1.5)  # How far past door to go (meters)
        self.declare_parameter("door_approach_distance", 0.8)  # Approach point before door (meters)

        map_yaml = self.get_parameter("map_yaml").value
        turn_penalty = float(self.get_parameter("turn_penalty").value)
        self.rdp_eps = float(self.get_parameter("rdp_eps").value)
        self.min_turn_dist = float(self.get_parameter("min_turn_dist").value)
        self.cruise_altitude = float(self.get_parameter("cruise_altitude").value)
        self.door_traverse_distance = float(self.get_parameter("door_traverse_distance").value)
        self.door_approach_distance = float(self.get_parameter("door_approach_distance").value)

        self.planner = AStarPlanner(map_yaml_path=map_yaml, turn_penalty=turn_penalty)

        # Convert door positions from grid to world coordinates
        self.doors_world: List[WorldPoint] = []
        self.doors_grid: List[GridPoint] = DOOR_POSITIONS_GRID.copy()
        for gx, gy in DOOR_POSITIONS_GRID:
            wx, wy = self.planner.map_to_world(gx, gy)
            self.doors_world.append((wx, wy))

        # Service uses its own callback group
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(
            NavigateToPose,
            "/traverse_doorway",
            self.handle_traversal_request,
            callback_group=self.service_cb_group,
        )

        self.busy = False
        self.get_logger().info(
            f"DoorwayTraversalAgent ready. Tracking {len(self.doors_world)} doors."
        )

    def find_nearest_door(self, wx: float, wy: float) -> Tuple[int, float]:
        """
        Find the index of the nearest door to the given world position.

        Args:
            wx, wy: Current world position

        Returns:
            (door_index, distance) tuple
        """
        min_dist = float('inf')
        nearest_idx = 0

        for i, (door_wx, door_wy) in enumerate(self.doors_world):
            dist = math.hypot(door_wx - wx, door_wy - wy)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        return nearest_idx, min_dist

    def determine_door_orientation(self, door_gx: int, door_gy: int) -> str:
        """
        Determine the orientation of a door (horizontal or vertical) by
        analyzing the obstacle map around the door.

        A horizontal door has walls above and below (passage is left-right).
        A vertical door has walls left and right (passage is up-down).

        Returns:
            'horizontal' or 'vertical'
        """
        # Sample points around the door to detect wall orientation
        check_distance = 15  # grid cells to check

        # Check if walls are above/below (horizontal door)
        above_blocked = not self.planner.is_free(door_gx, door_gy + check_distance)
        below_blocked = not self.planner.is_free(door_gx, door_gy - check_distance)

        # Check if walls are left/right (vertical door)
        left_blocked = not self.planner.is_free(door_gx - check_distance, door_gy)
        right_blocked = not self.planner.is_free(door_gx + check_distance, door_gy)

        horizontal_score = int(above_blocked) + int(below_blocked)
        vertical_score = int(left_blocked) + int(right_blocked)

        if horizontal_score > vertical_score:
            return 'horizontal'
        elif vertical_score > horizontal_score:
            return 'vertical'
        else:
            # Ambiguous - use clearance to decide
            clearance_h = (
                self.planner.get_clearance(door_gx - check_distance, door_gy) +
                self.planner.get_clearance(door_gx + check_distance, door_gy)
            )
            clearance_v = (
                self.planner.get_clearance(door_gx, door_gy - check_distance) +
                self.planner.get_clearance(door_gx, door_gy + check_distance)
            )
            return 'horizontal' if clearance_h > clearance_v else 'vertical'

    def compute_traversal_points(
        self,
        drone_wx: float,
        drone_wy: float,
        door_wx: float,
        door_wy: float,
        door_gx: int,
        door_gy: int,
    ) -> Tuple[WorldPoint, WorldPoint, WorldPoint]:
        """
        Compute the approach point, door center, and exit point for traversal.

        The drone will:
        1. Go to the approach point (same side as drone, close to door)
        2. Go through the door center
        3. Continue to the exit point (opposite side of door)

        Returns:
            (approach_point, door_center, exit_point) in world coordinates
        """
        orientation = self.determine_door_orientation(door_gx, door_gy)
        self.get_logger().info(f"Door orientation: {orientation}")

        # Calculate direction from drone to door
        dx = door_wx - drone_wx
        dy = door_wy - drone_wy
        dist = math.hypot(dx, dy)

        if dist < 0.01:
            # Drone is at door center, use orientation to decide direction
            if orientation == 'horizontal':
                # Move left or right
                traverse_dx, traverse_dy = 1.0, 0.0
            else:
                # Move up or down
                traverse_dx, traverse_dy = 0.0, 1.0
        else:
            # Normalize direction from drone to door
            dx /= dist
            dy /= dist

            if orientation == 'horizontal':
                # Door passage is left-right (walls above/below)
                # The traversal direction should be along X axis
                traverse_dx = 1.0 if dx >= 0 else -1.0
                traverse_dy = 0.0
            else:
                # Door passage is up-down (walls left/right)
                # The traversal direction should be along Y axis
                traverse_dx = 0.0
                traverse_dy = 1.0 if dy >= 0 else -1.0

        # Determine which side the drone is on
        if orientation == 'horizontal':
            # Drone approaching from left or right
            if drone_wx < door_wx:
                # Drone is on the left, approach from left, exit to right
                approach_wx = door_wx - self.door_approach_distance
                exit_wx = door_wx + self.door_traverse_distance
            else:
                # Drone is on the right, approach from right, exit to left
                approach_wx = door_wx + self.door_approach_distance
                exit_wx = door_wx - self.door_traverse_distance
            approach_wy = door_wy
            exit_wy = door_wy
        else:
            # Drone approaching from above or below
            if drone_wy < door_wy:
                # Drone is below, approach from below, exit above
                approach_wy = door_wy - self.door_approach_distance
                exit_wy = door_wy + self.door_traverse_distance
            else:
                # Drone is above, approach from above, exit below
                approach_wy = door_wy + self.door_approach_distance
                exit_wy = door_wy - self.door_traverse_distance
            approach_wx = door_wx
            exit_wx = door_wx

        approach_point = (approach_wx, approach_wy)
        door_center = (door_wx, door_wy)
        exit_point = (exit_wx, exit_wy)

        return approach_point, door_center, exit_point

    def plan_path_to_point(
        self,
        start_wx: float,
        start_wy: float,
        goal_wx: float,
        goal_wy: float,
    ) -> Optional[List[GridPoint]]:
        """
        Plan a path from start to goal using A*.

        Returns:
            List of grid waypoints, or None if no path found
        """
        start_grid = self.planner.world_to_map(start_wx, start_wy)
        goal_grid = self.planner.world_to_map(goal_wx, goal_wy)

        path = self.planner.plan(start_grid, goal_grid)
        if not path:
            return None

        # Simplify path
        simplified = rdp_simplify(path, eps=self.rdp_eps)
        waypoints = extract_turn_points(simplified, min_dist=self.min_turn_dist)

        return waypoints

    def execute_waypoints(self, waypoints_grid: List[GridPoint]) -> bool:
        """
        Execute a sequence of waypoints.

        Args:
            waypoints_grid: List of grid coordinates to visit

        Returns:
            True if all waypoints reached, False otherwise
        """
        for i, (gx, gy) in enumerate(waypoints_grid):
            wx, wy = self.planner.map_to_world(gx, gy)
            self.get_logger().info(
                f"→ Waypoint {i + 1}/{len(waypoints_grid)}: ({wx:.2f}, {wy:.2f})"
            )

            success = self.goto(wx, wy, tz=self.cruise_altitude)
            if not success:
                return False

            # Brief settling time between waypoints
            time.sleep(0.3)

        return True

    def handle_traversal_request(
        self,
        request: NavigateToPose.Request,
        response: NavigateToPose.Response,
    ) -> NavigateToPose.Response:
        """Main traversal logic."""
        if self.busy:
            response.success = False
            response.message = "Doorway traversal agent is busy."
            return response

        self.busy = True
        time.sleep(0.1)  # Brief wait for fresh pose

        # Get current drone position
        current = self.pose
        drone_wx = float(current.position.x)
        drone_wy = float(current.position.y)

        self.get_logger().info(
            f"Doorway traversal requested. Drone at ({drone_wx:.2f}, {drone_wy:.2f})"
        )

        # Find nearest door
        door_idx, door_dist = self.find_nearest_door(drone_wx, drone_wy)
        door_gx, door_gy = self.doors_grid[door_idx]
        door_wx, door_wy = self.doors_world[door_idx]

        self.get_logger().info(
            f"Nearest door #{door_idx}: grid=({door_gx}, {door_gy}), "
            f"world=({door_wx:.2f}, {door_wy:.2f}), dist={door_dist:.2f}m"
        )

        # Compute traversal points
        approach_point, door_center, exit_point = self.compute_traversal_points(
            drone_wx, drone_wy, door_wx, door_wy, door_gx, door_gy
        )

        self.get_logger().info(
            f"Traversal plan:\n"
            f"  Approach: ({approach_point[0]:.2f}, {approach_point[1]:.2f})\n"
            f"  Door:     ({door_center[0]:.2f}, {door_center[1]:.2f})\n"
            f"  Exit:     ({exit_point[0]:.2f}, {exit_point[1]:.2f})"
        )

        # Phase 1: Navigate to approach point (if far from door)
        if door_dist > self.door_approach_distance * 1.5:
            self.get_logger().info("Phase 1: Navigating to approach point...")
            waypoints = self.plan_path_to_point(
                drone_wx, drone_wy, approach_point[0], approach_point[1]
            )
            if waypoints:
                if not self.execute_waypoints(waypoints):
                    response.success = False
                    response.message = "Failed to reach approach point"
                    self.busy = False
                    return response
            else:
                # No path found, try direct approach
                self.get_logger().warn("No path to approach point, trying direct")
                if not self.goto(approach_point[0], approach_point[1], tz=self.cruise_altitude):
                    response.success = False
                    response.message = "Failed to reach approach point (direct)"
                    self.busy = False
                    return response

        # Phase 2: Go through door center
        self.get_logger().info("Phase 2: Passing through door...")
        if not self.goto(door_center[0], door_center[1], tz=self.cruise_altitude):
            response.success = False
            response.message = "Failed to pass through door center"
            self.busy = False
            return response

        time.sleep(0.2)

        # Phase 3: Continue to exit point
        self.get_logger().info("Phase 3: Exiting through door...")
        if not self.goto(exit_point[0], exit_point[1], tz=self.cruise_altitude):
            response.success = False
            response.message = "Failed to reach exit point"
            self.busy = False
            return response

        self.get_logger().info("Doorway traversal completed ✓")
        response.success = True
        response.message = f"Successfully traversed door #{door_idx}"
        self.busy = False
        return response


def main():
    rclpy.init()
    node = DoorwayTraversalAgent()

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