#!/usr/bin/env python3
"""
Doorway Traversal Agent (Partial Map / Exploration-based)
-----------------------------------------------------------
Receives a request to traverse through the nearest door.

Unlike the full-map version, this agent:
 - Uses the partial/exploration map (fog of war)
 - Does NOT know all door positions upfront
 - Detects doors dynamically as "visible slots" (narrow passages in the known map)
 - Uses ExplorationPlanner for navigation (treats unknown as free)

A "door" in this context is any narrow passage that has been discovered:
 - Known free cells flanked by known walls on opposite sides
 - Width roughly matching expected door size

Coordinate Convention:
    - World coordinates: (wx, wy) in meters
    - Grid coordinates: (gx, gy) with origin at bottom-left
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


class DoorwayTraversalAgent(WaypointController):
    """
    Doorway traversal agent using partial/exploration map.

    Service:
        /traverse_doorway : NavigateToPose (reusing the same service type)

    Behavior:
        1. Receives traversal request
        2. Scans the known map to find visible door-like passages
        3. Finds the closest detected door to current position
        4. Determines which side of the door the drone is on
        5. Plans and executes path through the door using ExplorationPlanner
        6. Returns success/failure to caller
    """

    def __init__(self):
        super().__init__(name="doorway_traversal_agent")

        # Parameters
        self.declare_parameter("turn_penalty", 1.5)
        self.declare_parameter("rdp_eps", 1.5)
        self.declare_parameter("min_turn_dist", 4.0)
        self.declare_parameter("cruise_altitude", 1.5)
        self.declare_parameter("door_traverse_distance", 0.8)  # How far past door to go (meters)
        self.declare_parameter("door_approach_distance", 0.8)  # Approach point before door (meters)

        # Safety margin parameters (matching navigation agent)
        self.declare_parameter("min_safety_margin", 8)
        self.declare_parameter("preferred_clearance", 20)
        self.declare_parameter("wall_cost_weight", 0.8)

        # Door detection parameters
        self.declare_parameter("min_door_width", 0.6)  # Minimum door width in meters
        self.declare_parameter("max_door_width", 2.0)  # Maximum door width in meters
        self.declare_parameter("door_search_radius", 5.0)  # How far to search for doors (meters)

        self.turn_penalty = float(self.get_parameter("turn_penalty").value)
        self.rdp_eps = float(self.get_parameter("rdp_eps").value)
        self.min_turn_dist = float(self.get_parameter("min_turn_dist").value)
        self.cruise_altitude = float(self.get_parameter("cruise_altitude").value)
        self.door_traverse_distance = float(self.get_parameter("door_traverse_distance").value)
        self.door_approach_distance = float(self.get_parameter("door_approach_distance").value)

        self.min_safety_margin = int(self.get_parameter("min_safety_margin").value)
        self.preferred_clearance = int(self.get_parameter("preferred_clearance").value)
        self.wall_cost_weight = float(self.get_parameter("wall_cost_weight").value)

        self.min_door_width = float(self.get_parameter("min_door_width").value)
        self.max_door_width = float(self.get_parameter("max_door_width").value)
        self.door_search_radius = float(self.get_parameter("door_search_radius").value)

        # Planner (will be configured when map received)
        self.planner: Optional[ExplorationPlanner] = None
        self.map_received = False
        self._map_lock = threading.Lock()

        # Current path being executed
        self._current_path: List[GridPoint] = []
        self._path_lock = threading.Lock()

        # Subscribe to exploration map
        self.map_cb_group = ReentrantCallbackGroup()
        self.map_sub = self.create_subscription(
            Int8MultiArray,
            "/exploration/observed_map",
            self.map_callback,
            10,
            callback_group=self.map_cb_group,
        )

        # Path monitoring timer
        self.declare_parameter("path_check_rate", 10.0)
        self.path_check_rate = float(self.get_parameter("path_check_rate").value)
        self.monitor_timer = self.create_timer(
            1.0 / self.path_check_rate,
            self.check_path_blocked,
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
        self.get_logger().info("DoorwayTraversalAgent ready (exploration mode).")
        self.get_logger().info("  Doors detected dynamically from visible map.")
        self.get_logger().info("Waiting for exploration map on /exploration/observed_map...")

    def map_callback(self, msg: Int8MultiArray):
        """Receive and update the exploration map."""
        if len(msg.layout.dim) < 2:
            return

        height = msg.layout.dim[0].size
        width = msg.layout.dim[1].size

        # Reconstruct 2D array
        observed_map = np.array(msg.data, dtype=np.int8).reshape((height, width))

        with self._map_lock:
            # Initialize planner on first map
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

                self.get_logger().info(
                    f"Planner initialized: res={resolution}, origin=({origin_x}, {origin_y})"
                )

            self.planner.update_map(observed_map)

            if not self.map_received:
                self.map_received = True
                self.get_logger().info("First exploration map received!")

    def check_path_blocked(self):
        """Check if current path is blocked by newly discovered walls."""
        if not self.map_received or self.planner is None:
            return

        with self._path_lock:
            if not self._current_path:
                return
            blocked_idx = self.planner.check_path_blocked(self._current_path)

        if blocked_idx is not None:
            self.get_logger().warn(f"Path blocked at index {blocked_idx}! Triggering abort...")
            self.trigger_abort()

    def detect_doors_near_position(
        self,
        wx: float,
        wy: float,
    ) -> List[Tuple[GridPoint, str]]:
        """
        Detect door-like passages in the known map near the given position.

        A door is detected as a narrow passage between known walls.
        We scan the area around the drone looking for:
        - Horizontal doors: known walls above and below, passage left-right
        - Vertical doors: known walls left and right, passage up-down

        Returns:
            List of (grid_position, orientation) tuples where orientation is 'horizontal' or 'vertical'
        """
        if self.planner is None or self.planner.observed_map is None:
            return []

        doors: List[Tuple[GridPoint, str]] = []

        # Convert search radius to grid cells
        search_cells = int(self.door_search_radius / self.planner.resolution)
        center_gx, center_gy = self.planner.world_to_map(wx, wy)

        # Door width in grid cells
        min_door_cells = int(self.min_door_width / self.planner.resolution)
        max_door_cells = int(self.max_door_width / self.planner.resolution)

        # Wall check distance
        wall_check = 15  # cells to check for walls on each side

        observed = self.planner.observed_map

        # Search in the area around the drone
        for gx in range(center_gx - search_cells, center_gx + search_cells + 1):
            for gy in range(center_gy - search_cells, center_gy + search_cells + 1):
                if not self.planner.in_bounds(gx, gy):
                    continue

                # Must be a known free cell
                if observed[gy, gx] != 0:
                    continue

                # Check for horizontal door (walls above/below, passage left/right)
                if self._is_horizontal_door(gx, gy, wall_check, min_door_cells, max_door_cells):
                    doors.append(((gx, gy), 'horizontal'))
                    continue

                # Check for vertical door (walls left/right, passage up/down)
                if self._is_vertical_door(gx, gy, wall_check, min_door_cells, max_door_cells):
                    doors.append(((gx, gy), 'vertical'))

        # Remove duplicates (keep one representative per cluster)
        doors = self._cluster_doors(doors)

        return doors

    def _is_horizontal_door(
        self,
        gx: int,
        gy: int,
        wall_check: int,
        min_width: int,
        max_width: int,
    ) -> bool:
        """Check if position is part of a horizontal door (passage is left-right)."""
        if self.planner is None or self.planner.observed_map is None:
            return False

        observed = self.planner.observed_map

        # Check for walls above
        above_wall = False
        for dy in range(1, wall_check + 1):
            check_y = gy + dy
            if not self.planner.in_bounds(gx, check_y):
                break
            if observed[check_y, gx] == 1:  # Known wall
                above_wall = True
                break
            if observed[check_y, gx] == 0:  # Known free, no wall yet
                continue
            # Unknown - can't confirm

        # Check for walls below
        below_wall = False
        for dy in range(1, wall_check + 1):
            check_y = gy - dy
            if not self.planner.in_bounds(gx, check_y):
                break
            if observed[check_y, gx] == 1:  # Known wall
                below_wall = True
                break
            if observed[check_y, gx] == 0:  # Known free
                continue

        if not (above_wall and below_wall):
            return False

        # Count passage width (free cells in vertical direction)
        passage_height = 1
        for dy in range(1, max_width + 1):
            if self.planner.in_bounds(gx, gy + dy) and observed[gy + dy, gx] == 0:
                passage_height += 1
            else:
                break
        for dy in range(1, max_width + 1):
            if self.planner.in_bounds(gx, gy - dy) and observed[gy - dy, gx] == 0:
                passage_height += 1
            else:
                break

        # Check if passage width is door-like
        return min_width <= passage_height <= max_width

    def _is_vertical_door(
        self,
        gx: int,
        gy: int,
        wall_check: int,
        min_width: int,
        max_width: int,
    ) -> bool:
        """Check if position is part of a vertical door (passage is up-down)."""
        if self.planner is None or self.planner.observed_map is None:
            return False

        observed = self.planner.observed_map

        # Check for walls to the left
        left_wall = False
        for dx in range(1, wall_check + 1):
            check_x = gx - dx
            if not self.planner.in_bounds(check_x, gy):
                break
            if observed[gy, check_x] == 1:  # Known wall
                left_wall = True
                break
            if observed[gy, check_x] == 0:  # Known free
                continue

        # Check for walls to the right
        right_wall = False
        for dx in range(1, wall_check + 1):
            check_x = gx + dx
            if not self.planner.in_bounds(check_x, gy):
                break
            if observed[gy, check_x] == 1:  # Known wall
                right_wall = True
                break
            if observed[gy, check_x] == 0:  # Known free
                continue

        if not (left_wall and right_wall):
            return False

        # Count passage width (free cells in horizontal direction)
        passage_width = 1
        for dx in range(1, max_width + 1):
            if self.planner.in_bounds(gx + dx, gy) and observed[gy, gx + dx] == 0:
                passage_width += 1
            else:
                break
        for dx in range(1, max_width + 1):
            if self.planner.in_bounds(gx - dx, gy) and observed[gy, gx - dx] == 0:
                passage_width += 1
            else:
                break

        return min_width <= passage_width <= max_width

    def _cluster_doors(
        self,
        doors: List[Tuple[GridPoint, str]],
        cluster_radius: int = 10,
    ) -> List[Tuple[GridPoint, str]]:
        """Cluster nearby door detections and return one representative per cluster."""
        if not doors:
            return []

        clustered = []
        used = set()

        for i, (pos_i, orient_i) in enumerate(doors):
            if i in used:
                continue

            # Find all doors in same cluster
            cluster_positions = [pos_i]
            used.add(i)

            for j, (pos_j, orient_j) in enumerate(doors):
                if j in used:
                    continue
                if orient_j != orient_i:
                    continue

                dist = math.hypot(pos_i[0] - pos_j[0], pos_i[1] - pos_j[1])
                if dist < cluster_radius:
                    cluster_positions.append(pos_j)
                    used.add(j)

            # Use center of cluster as representative
            avg_x = int(sum(p[0] for p in cluster_positions) / len(cluster_positions))
            avg_y = int(sum(p[1] for p in cluster_positions) / len(cluster_positions))
            clustered.append(((avg_x, avg_y), orient_i))

        return clustered

    def find_nearest_door(
        self,
        wx: float,
        wy: float,
    ) -> Optional[Tuple[GridPoint, str, float]]:
        """
        Find the nearest detected door to the given world position.

        Returns:
            (grid_position, orientation, distance) or None if no door found
        """
        doors = self.detect_doors_near_position(wx, wy)

        if not doors:
            return None

        min_dist = float('inf')
        nearest = None

        gx_drone, gy_drone = self.planner.world_to_map(wx, wy)

        for door_pos, orient in doors:
            dist = math.hypot(door_pos[0] - gx_drone, door_pos[1] - gy_drone)
            if dist < min_dist:
                min_dist = dist
                nearest = (door_pos, orient, dist * self.planner.resolution)

        return nearest

    def compute_traversal_points(
        self,
        drone_wx: float,
        drone_wy: float,
        door_gx: int,
        door_gy: int,
        orientation: str,
    ) -> Tuple[WorldPoint, WorldPoint, WorldPoint]:
        """
        Compute the approach point, door center, and exit point for traversal.

        Returns:
            (approach_point, door_center, exit_point) in world coordinates
        """
        door_wx, door_wy = self.planner.map_to_world(door_gx, door_gy)

        if orientation == 'horizontal':
            # Door passage is left-right
            if drone_wx < door_wx:
                approach_wx = door_wx - self.door_approach_distance
                exit_wx = door_wx + self.door_traverse_distance
            else:
                approach_wx = door_wx + self.door_approach_distance
                exit_wx = door_wx - self.door_traverse_distance
            approach_wy = door_wy
            exit_wy = door_wy
        else:
            # Door passage is up-down (vertical)
            if drone_wy < door_wy:
                approach_wy = door_wy - self.door_approach_distance
                exit_wy = door_wy + self.door_traverse_distance
            else:
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
        """Plan a path using the exploration planner."""
        if self.planner is None:
            return None

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
        """Execute a sequence of waypoints."""
        with self._path_lock:
            self._current_path = waypoints_grid[:]

        for i, (gx, gy) in enumerate(waypoints_grid):
            wx, wy = self.planner.map_to_world(gx, gy)
            self.get_logger().info(
                f"→ Waypoint {i + 1}/{len(waypoints_grid)}: ({wx:.2f}, {wy:.2f})"
            )

            reached, aborted = self.goto(wx, wy, tz=self.cruise_altitude)

            if aborted:
                self.get_logger().warn("Path aborted during execution!")
                with self._path_lock:
                    self._current_path = []
                return False

            if not reached:
                with self._path_lock:
                    self._current_path = []
                return False

            time.sleep(0.2)

        with self._path_lock:
            self._current_path = []
        return True

    def handle_traversal_request(
        self,
        request: NavigateToPose.Request,
        response: NavigateToPose.Response,
    ) -> NavigateToPose.Response:
        """Main traversal logic using partial map."""
        if self.busy:
            response.success = False
            response.message = "Doorway traversal agent is busy."
            return response

        if not self.map_received or self.planner is None:
            response.success = False
            response.message = "No exploration map received yet."
            return response

        self.busy = True
        self.clear_abort()
        time.sleep(0.1)

        # Get current drone position
        current = self.pose
        drone_wx = float(current.position.x)
        drone_wy = float(current.position.y)

        self.get_logger().info(
            f"Doorway traversal requested. Drone at ({drone_wx:.2f}, {drone_wy:.2f})"
        )

        # Find nearest door from visible map
        door_info = self.find_nearest_door(drone_wx, drone_wy)

        if door_info is None:
            self.get_logger().error("No visible doors detected nearby!")
            response.success = False
            response.message = "No visible doors found in the explored area."
            self.busy = False
            return response

        door_grid, orientation, door_dist = door_info
        door_gx, door_gy = door_grid
        door_wx, door_wy = self.planner.map_to_world(door_gx, door_gy)

        self.get_logger().info(
            f"Nearest door: grid=({door_gx}, {door_gy}), "
            f"world=({door_wx:.2f}, {door_wy:.2f}), "
            f"orientation={orientation}, dist={door_dist:.2f}m"
        )

        # Compute traversal points
        approach_point, door_center, exit_point = self.compute_traversal_points(
            drone_wx, drone_wy, door_gx, door_gy, orientation
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
                reached, aborted = self.goto(
                    approach_point[0], approach_point[1], tz=self.cruise_altitude
                )
                if aborted or not reached:
                    response.success = False
                    response.message = "Failed to reach approach point (direct)"
                    self.busy = False
                    return response

        # Phase 2: Go through door center
        self.get_logger().info("Phase 2: Passing through door...")
        reached, aborted = self.goto(door_center[0], door_center[1], tz=self.cruise_altitude)
        if aborted or not reached:
            response.success = False
            response.message = "Failed to pass through door center"
            self.busy = False
            return response

        time.sleep(0.2)

        # Phase 3: Continue to exit point
        self.get_logger().info("Phase 3: Exiting through door...")
        reached, aborted = self.goto(exit_point[0], exit_point[1], tz=self.cruise_altitude)
        if aborted or not reached:
            response.success = False
            response.message = "Failed to reach exit point"
            self.busy = False
            return response

        self.get_logger().info("Doorway traversal completed ✓")
        response.success = True
        response.message = f"Successfully traversed door at ({door_gx}, {door_gy})"
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