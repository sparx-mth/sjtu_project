#!/usr/bin/env python3
"""
Doorway Traversal Agent
-----------------------------------------
Uses known door positions from DOOR_POSITIONS_GRID.
Only considers doors that are VISIBLE (observed_map != -1).
Uses ExplorationPlanner for path planning with observed_map.
"""

import math
import time
import threading
from typing import List, Tuple, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import numpy as np
from std_msgs.msg import Int8MultiArray

from autonomous_system.srv import NavigateToPose
from autonomous_system.control.waypoint_controller import WaypointController
from autonomous_system.planning.exploration_planner import ExplorationPlanner
from autonomous_system.planning.path_simplifier import rdp_simplify, extract_turn_points

GridPoint = Tuple[int, int]
WorldPoint = Tuple[float, float]

# Known door positions in grid coordinates
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
    Simple doorway traversal using known door positions.
    Only traverses doors that are visible in observed_map.
    """

    def __init__(self):
        super().__init__(name="doorway_traversal_agent")

        # Parameters
        self.declare_parameter("turn_penalty", 1.5)
        self.declare_parameter("rdp_eps", 1.5)
        self.declare_parameter("min_turn_dist", 4.0)
        self.declare_parameter("cruise_altitude", 1.5)
        self.declare_parameter("door_traverse_distance", 0.6)
        self.declare_parameter("door_approach_distance", 0.6)
        self.declare_parameter("min_safety_margin", 10)
        self.declare_parameter("preferred_clearance", 20)
        self.declare_parameter("wall_cost_weight", 0.8)

        self.turn_penalty = float(self.get_parameter("turn_penalty").value)
        self.rdp_eps = float(self.get_parameter("rdp_eps").value)
        self.min_turn_dist = float(self.get_parameter("min_turn_dist").value)
        self.cruise_altitude = float(self.get_parameter("cruise_altitude").value)
        self.door_traverse_distance = float(self.get_parameter("door_traverse_distance").value)
        self.door_approach_distance = float(self.get_parameter("door_approach_distance").value)
        self.min_safety_margin = int(self.get_parameter("min_safety_margin").value)
        self.preferred_clearance = int(self.get_parameter("preferred_clearance").value)
        self.wall_cost_weight = float(self.get_parameter("wall_cost_weight").value)

        # Planner (initialized on first map)
        self.planner: Optional[ExplorationPlanner] = None
        self.map_received = False

        # Door positions (will be converted to world coords after planner init)
        self.doors_grid: List[GridPoint] = DOOR_POSITIONS_GRID.copy()
        self.doors_world: List[WorldPoint] = []

        # Subscribe to observed map
        self.map_cb_group = ReentrantCallbackGroup()
        self.map_sub = self.create_subscription(
            Int8MultiArray,
            "/exploration/observed_map",
            self.map_callback,
            10,
            callback_group=self.map_cb_group,
        )

        # Service
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(
            NavigateToPose,
            "/traverse_doorway",
            self.handle_traversal_request,
            callback_group=self.service_cb_group,
        )

        self.busy = False
        self.get_logger().info(
            f"DoorwayTraversalAgent ready. Tracking {len(self.doors_grid)} known doors."
        )

    def map_callback(self, msg: Int8MultiArray):
        """Receive and update the exploration map."""
        if len(msg.layout.dim) < 2:
            return

        height = msg.layout.dim[0].size
        width = msg.layout.dim[1].size
        observed_map = np.array(msg.data, dtype=np.int8).reshape((height, width))

        if self.planner is None:
            try:
                parts = msg.layout.dim[0].label.split(",")
                resolution = float(parts[0])
                origin_x = float(parts[1])
                origin_y = float(parts[2])
            except (IndexError, ValueError):
                resolution = 0.05
                origin_x, origin_y = -25.0, -30.0
                self.get_logger().warn("Using default map metadata")

            self.planner = ExplorationPlanner(
                resolution=resolution,
                origin=(origin_x, origin_y, 0.0),
                turn_penalty=self.turn_penalty,
                min_safety_margin=self.min_safety_margin,
                preferred_clearance=self.preferred_clearance,
                wall_cost_weight=self.wall_cost_weight,
            )

            # Convert door grid coords to world coords
            self.doors_world = []
            for gx, gy in self.doors_grid:
                wx, wy = self.planner.map_to_world(gx, gy)
                self.doors_world.append((wx, wy))

            self.get_logger().info(
                f"Planner initialized. Doors converted to world coordinates."
            )

        self.planner.update_map(observed_map)

        if not self.map_received:
            self.map_received = True
            self.get_logger().info("First exploration map received!")

    def is_door_visible(self, door_idx: int) -> bool:
        """
        Check if a door is visible (observed) in the map.
        Returns True if observed_map at door position is NOT -1.
        """
        if self.planner is None or self.planner.observed_map is None:
            return False

        gx, gy = self.doors_grid[door_idx]

        # Check bounds
        h, w = self.planner.observed_map.shape
        if gx < 0 or gx >= w or gy < 0 or gy >= h:
            return False

        # -1 means unknown/unexplored
        return self.planner.observed_map[gy, gx] != -1

    def get_visible_doors(self) -> List[int]:
        """Get indices of all visible doors."""
        return [i for i in range(len(self.doors_grid)) if self.is_door_visible(i)]

    def find_nearest_visible_door(self, wx: float, wy: float) -> Tuple[Optional[int], float]:
        """
        Find the nearest VISIBLE door to the given position.
        Returns (door_index, distance) or (None, inf) if no visible doors.
        """
        min_dist = float('inf')
        nearest_idx = None

        for i, (door_wx, door_wy) in enumerate(self.doors_world):
            if not self.is_door_visible(i):
                continue

            dist = math.hypot(door_wx - wx, door_wy - wy)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        return nearest_idx, min_dist

    def determine_door_orientation(self, door_gx: int, door_gy: int) -> str:
        """
        Determine door orientation by checking walls around it.
        Returns 'horizontal' or 'vertical'.
        """
        if self.planner is None:
            return 'horizontal'

        check_distance = 15  # grid cells

        # Check walls above/below vs left/right
        above_blocked = not self.planner.is_passable(door_gx, door_gy + check_distance)
        below_blocked = not self.planner.is_passable(door_gx, door_gy - check_distance)
        left_blocked = not self.planner.is_passable(door_gx - check_distance, door_gy)
        right_blocked = not self.planner.is_passable(door_gx + check_distance, door_gy)

        horizontal_score = int(above_blocked) + int(below_blocked)
        vertical_score = int(left_blocked) + int(right_blocked)

        return 'horizontal' if horizontal_score >= vertical_score else 'vertical'

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
        Compute approach, door center, and exit points.
        """
        orientation = self.determine_door_orientation(door_gx, door_gy)
        self.get_logger().info(f"Door orientation: {orientation}")

        if orientation == 'horizontal':
            # Passage is left-right
            if drone_wx < door_wx:
                approach_wx = door_wx - self.door_approach_distance
                exit_wx = door_wx + self.door_traverse_distance
            else:
                approach_wx = door_wx + self.door_approach_distance
                exit_wx = door_wx - self.door_traverse_distance
            approach_wy = door_wy
            exit_wy = door_wy
        else:
            # Passage is up-down
            if drone_wy < door_wy:
                approach_wy = door_wy - self.door_approach_distance
                exit_wy = door_wy + self.door_traverse_distance
            else:
                approach_wy = door_wy + self.door_approach_distance
                exit_wy = door_wy - self.door_traverse_distance
            approach_wx = door_wx
            exit_wx = door_wx

        return (approach_wx, approach_wy), (door_wx, door_wy), (exit_wx, exit_wy)

    def plan_and_execute(self, start_wx: float, start_wy: float,
                         goal_wx: float, goal_wy: float) -> bool:
        """Plan path and execute waypoints. Returns True on success."""
        if self.planner is None:
            return False

        start_grid = self.planner.world_to_map(start_wx, start_wy)
        goal_grid = self.planner.world_to_map(goal_wx, goal_wy)

        path = self.planner.plan(start_grid, goal_grid)
        if not path:
            self.get_logger().warn("No path found, trying direct goto")
            success, _ = self.goto(goal_wx, goal_wy, self.cruise_altitude)
            return success

        # Simplify path
        simplified = rdp_simplify(path, eps=self.rdp_eps)
        waypoints = extract_turn_points(simplified, min_dist=self.min_turn_dist)

        # Execute waypoints
        for i, (gx, gy) in enumerate(waypoints):
            wx, wy = self.planner.map_to_world(gx, gy)
            self.get_logger().info(f"  Waypoint {i + 1}/{len(waypoints)}: ({wx:.2f}, {wy:.2f})")

            success, _ = self.goto(wx, wy, self.cruise_altitude)
            if not success:
                return False
            time.sleep(0.2)

        return True

    def handle_traversal_request(
            self,
            request: NavigateToPose.Request,
            response: NavigateToPose.Response,
    ) -> NavigateToPose.Response:
        """Handle doorway traversal request."""

        if self.busy:
            response.success = False
            response.message = "Agent is busy"
            return response

        if not self.map_received or self.planner is None:
            response.success = False
            response.message = "No map received yet"
            return response

        self.busy = True
        time.sleep(0.1)

        # Get drone position
        current = self.pose
        drone_wx = float(current.position.x)
        drone_wy = float(current.position.y)

        self.get_logger().info(f"Traversal requested. Drone at ({drone_wx:.2f}, {drone_wy:.2f})")

        # Find nearest visible door
        door_idx, door_dist = self.find_nearest_visible_door(drone_wx, drone_wy)

        if door_idx is None:
            # Log which doors are visible for debugging
            visible = self.get_visible_doors()
            self.get_logger().warn(f"No visible doors! Visible door indices: {visible}")
            response.success = False
            response.message = "No visible doors found"
            self.busy = False
            return response

        door_gx, door_gy = self.doors_grid[door_idx]
        door_wx, door_wy = self.doors_world[door_idx]

        self.get_logger().info(
            f"Nearest visible door #{door_idx}: "
            f"grid=({door_gx}, {door_gy}), world=({door_wx:.2f}, {door_wy:.2f}), "
            f"dist={door_dist:.2f}m"
        )

        # Compute traversal points
        approach, door_center, exit_pt = self.compute_traversal_points(
            drone_wx, drone_wy, door_wx, door_wy, door_gx, door_gy
        )

        self.get_logger().info(
            f"Plan: approach=({approach[0]:.2f}, {approach[1]:.2f}) -> "
            f"door=({door_center[0]:.2f}, {door_center[1]:.2f}) -> "
            f"exit=({exit_pt[0]:.2f}, {exit_pt[1]:.2f})"
        )

        # Phase 1: Navigate to approach point (if not already close)
        if door_dist > self.door_approach_distance * 1.5:
            self.get_logger().info("Phase 1: Going to approach point...")
            if not self.plan_and_execute(drone_wx, drone_wy, approach[0], approach[1]):
                response.success = False
                response.message = "Failed to reach approach point"
                self.busy = False
                return response

        # Phase 2: Through door center
        self.get_logger().info("Phase 2: Passing through door...")
        success, _ = self.goto(door_center[0], door_center[1], self.cruise_altitude)
        if not success:
            response.success = False
            response.message = "Failed at door center"
            self.busy = False
            return response
        time.sleep(0.2)

        # Phase 3: Exit
        self.get_logger().info("Phase 3: Exiting...")
        success, _ = self.goto(exit_pt[0], exit_pt[1], self.cruise_altitude)
        if not success:
            response.success = False
            response.message = "Failed to exit"
            self.busy = False
            return response

        self.get_logger().info("Doorway traversal complete!")
        response.success = True
        response.message = f"Traversed door #{door_idx}"
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