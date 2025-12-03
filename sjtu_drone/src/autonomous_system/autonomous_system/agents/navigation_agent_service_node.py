#!/usr/bin/env python3
"""
Navigation Agent with Exploration (Service-based) - FIXED VERSION
------------------------------------------------------------------
Fixes:
1. Added STUCK DETECTION - triggers replan if position doesn't change for N cycles
2. Checks drone's CURRENT POSITION for passability, not just path ahead
3. Checks cells BETWEEN current position and next waypoint
"""

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


class NavigationAgentService(WaypointController):
    """
    Navigation agent using exploration map with dynamic re-planning.

    FIXES APPLIED:
    - Stuck detection: triggers abort if drone position unchanged for stuck_threshold cycles
    - Current position check: verifies drone's current grid cell is passable
    - Path segment check: checks cells between drone and next waypoint
    """

    def __init__(self):
        super().__init__(name="navigation_agent_service")

        # Parameters
        self.declare_parameter("turn_penalty", 1.5)
        self.declare_parameter("rdp_eps", 1.5)
        self.declare_parameter("min_turn_dist", 6.0)
        self.declare_parameter("cruise_altitude", 1.5)
        self.declare_parameter("path_check_rate", 20.0)  # Hz

        # Adaptive safety margin parameters
        self.declare_parameter("min_safety_margin", 10)
        self.declare_parameter("preferred_clearance", 20)
        self.declare_parameter("wall_cost_weight", 0.8)
        self.declare_parameter("safety_margin", 10)

        # NEW: Stuck detection parameters
        self.declare_parameter("stuck_threshold", 300)  # cycles without movement
        self.declare_parameter("stuck_distance_threshold", 0.005)  # meters - movement less than this = stuck

        self.turn_penalty = float(self.get_parameter("turn_penalty").value)
        self.rdp_eps = float(self.get_parameter("rdp_eps").value)
        self.min_turn_dist = float(self.get_parameter("min_turn_dist").value)
        self.cruise_altitude = float(self.get_parameter("cruise_altitude").value)
        self.path_check_rate = float(self.get_parameter("path_check_rate").value)

        self.min_safety_margin = int(self.get_parameter("min_safety_margin").value)
        self.preferred_clearance = int(self.get_parameter("preferred_clearance").value)
        self.wall_cost_weight = float(self.get_parameter("wall_cost_weight").value)

        # Stuck detection
        self.stuck_threshold = int(self.get_parameter("stuck_threshold").value)
        self.stuck_distance_threshold = float(self.get_parameter("stuck_distance_threshold").value)

        legacy_margin = int(self.get_parameter("safety_margin").value)
        if self.min_safety_margin == 3 and legacy_margin != 3:
            self.min_safety_margin = legacy_margin

        # Planner
        self.planner: Optional[ExplorationPlanner] = None
        self.map_received = False

        # Current path being executed
        self._current_path: List[GridPoint] = []
        self._current_waypoint_idx = 0
        self._current_waypoint_world: Optional[Tuple[float, float]] = None  # NEW: track target waypoint
        self._path_lock = threading.Lock()

        # NEW: Stuck detection state
        self._last_position: Optional[Tuple[float, float]] = None
        self._stuck_counter = 0

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
        self.monitor_timer = self.create_timer(
            1.0 / self.path_check_rate,
            self.check_path_and_stuck,  # RENAMED: now checks both path AND stuck
            callback_group=self.map_cb_group,
        )

        # Service
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(
            NavigateToPose,
            "/navigate_to_pose",
            self.handle_navigation_request,
            callback_group=self.service_cb_group,
        )

        self.busy = False

        self.get_logger().info("NavigationAgentService ready (FIXED VERSION with stuck detection).")
        self.get_logger().info(
            f"  Stuck detection: {self.stuck_threshold} cycles, {self.stuck_distance_threshold}m threshold")

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

            self._setup_clearance_speed_control(resolution)
            self.get_logger().info(f"Planner initialized: res={resolution}, origin=({origin_x}, {origin_y})")

        self.planner.update_map(observed_map)

        if not self.map_received:
            self.map_received = True
            self.get_logger().info("First exploration map received!")

    def _setup_clearance_speed_control(self, resolution: float):
        """Configure clearance-based speed control."""
        min_clearance_for_full_speed = self.preferred_clearance * resolution
        min_clearance_threshold = self.min_safety_margin * resolution

        def get_clearance(wx: float, wy: float) -> float:
            if self.planner is None:
                return min_clearance_for_full_speed
            return self.planner.get_clearance_world(wx, wy)

        self.set_clearance_callback(
            callback=get_clearance,
            min_clearance_for_full_speed=min_clearance_for_full_speed,
            min_clearance_threshold=min_clearance_threshold,
        )

    def check_path_and_stuck(self):
        """
        Check if:
        1. Current drone position is in a wall (newly discovered)
        2. Path ahead is blocked
        3. Drone is stuck (no movement for N cycles)
        """
        if not self.map_received or self.planner is None:
            return

        # Get current drone position
        current = self.pose
        current_wx, current_wy = current.position.x, current.position.y
        current_grid = self.planner.world_to_map(current_wx, current_wy)

        with self._path_lock:
            if not self._current_path:
                self._stuck_counter = 0
                self._last_position = None
                return

            # ============================================================
            # FIX 1: Check if CURRENT POSITION is in a wall
            # ============================================================
            if not self.planner.is_passable(current_grid[0], current_grid[1]):
                self.get_logger().warn(
                    f"CURRENT POSITION {current_grid} is blocked! Triggering abort..."
                )
                self.trigger_abort()
                return

            # ============================================================
            # FIX 2: Check path from current position to next waypoint
            # ============================================================
            if self._current_waypoint_world is not None:
                target_grid = self.planner.world_to_map(
                    self._current_waypoint_world[0],
                    self._current_waypoint_world[1]
                )

                # Check cells on the line between current position and waypoint
                cells_to_check = self._get_line_cells(current_grid, target_grid)
                for cell in cells_to_check[:20]:  # Check first 20 cells (~1m at 0.05 resolution)
                    if not self.planner.is_passable(cell[0], cell[1]):
                        self.get_logger().warn(
                            f"Path segment blocked at {cell}! Triggering abort..."
                        )
                        self.trigger_abort()
                        return

            # ============================================================
            # FIX 3: Stuck detection - no movement for N cycles
            # ============================================================
            if self._last_position is not None:
                distance_moved = (
                                         (current_wx - self._last_position[0]) ** 2 +
                                         (current_wy - self._last_position[1]) ** 2
                                 ) ** 0.5

                if distance_moved < self.stuck_distance_threshold:
                    self._stuck_counter += 1

                    if self._stuck_counter >= self.stuck_threshold:
                        self.get_logger().warn(
                            f"STUCK DETECTED! No movement for {self._stuck_counter} cycles. "
                            f"Position: ({current_wx:.2f}, {current_wy:.2f}). Triggering abort..."
                        )
                        self._stuck_counter = 0
                        self.trigger_abort()
                        return
                else:
                    self._stuck_counter = 0

            self._last_position = (current_wx, current_wy)

            # Original path check (remaining path)
            remaining_path = self._current_path[self._current_waypoint_idx:]
            blocked_idx = self.planner.check_path_blocked(remaining_path)

        if blocked_idx is not None:
            self.get_logger().warn(f"Path blocked at index {blocked_idx}! Triggering abort...")
            self.trigger_abort()

    def _get_line_cells(self, start: GridPoint, end: GridPoint) -> List[GridPoint]:
        """Get grid cells along a line using Bresenham's algorithm."""
        cells = []
        x0, y0 = start
        x1, y1 = end

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        if dx > dy:
            err = dx / 2
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        cells.append((x1, y1))
        return cells

    def handle_navigation_request(
            self,
            request: NavigateToPose.Request,
            response: NavigateToPose.Response,
    ) -> NavigateToPose.Response:
        """Main navigation with re-planning on obstacle discovery."""

        if self.busy:
            response.success = False
            response.message = "Navigation agent is busy."
            return response

        if not self.map_received or self.planner is None:
            response.success = False
            response.message = "No exploration map received yet."
            return response

        self.busy = True
        self.clear_abort()
        self._stuck_counter = 0  # Reset stuck counter
        self._last_position = None
        time.sleep(0.1)

        goal_wx = float(request.x)
        goal_wy = float(request.y)
        goal_wz = float(request.z) if request.z > 0 else self.cruise_altitude

        max_replan_attempts = 20
        attempt = 0

        while attempt < max_replan_attempts:
            attempt += 1
            self.clear_abort()
            self._stuck_counter = 0  # Reset on each attempt
            self._last_position = None

            current = self.pose
            start_wx = float(current.position.x)
            start_wy = float(current.position.y)

            self.get_logger().info(
                f"[Attempt {attempt}] Planning: ({start_wx:.2f}, {start_wy:.2f}) -> "
                f"({goal_wx:.2f}, {goal_wy:.2f})"
            )

            start_grid = self.planner.world_to_map(start_wx, start_wy)
            goal_grid = self.planner.world_to_map(goal_wx, goal_wy)

            # Check if start position is blocked and find alternative
            if not self.planner.is_passable(start_grid[0], start_grid[1]):
                self.get_logger().warn(
                    f"Start position {start_grid} is blocked! Finding nearest free cell..."
                )
                start_grid = self.planner._find_nearest_passable(start_grid)
                if start_grid is None:
                    response.success = False
                    response.message = "Drone is stuck in wall, cannot find escape path"
                    self.busy = False
                    return response
                self.get_logger().info(f"Using alternative start: {start_grid}")

            path = self.planner.plan(start_grid, goal_grid)
            if not path:
                self.get_logger().error("Planner returned empty path!")
                response.success = False
                response.message = f"No path found from {start_grid} to {goal_grid}."
                self.busy = False
                return response

            min_clearance = self.planner.get_path_min_clearance(path)
            min_clearance_m = min_clearance * self.planner.resolution

            simplified = rdp_simplify(path, eps=self.rdp_eps)
            waypoints_grid = extract_turn_points(simplified, min_dist=self.min_turn_dist)

            with self._path_lock:
                self._current_path = path
                self._current_waypoint_idx = 0

            self.get_logger().info(
                f"Path: {len(path)} cells -> {len(waypoints_grid)} waypoints, "
                f"min clearance: {min_clearance_m:.2f}m"
            )

            # Execute waypoints
            navigation_complete = True
            for i, (gx, gy) in enumerate(waypoints_grid):
                with self._path_lock:
                    self._current_waypoint_idx = min(
                        i * (len(path) // max(len(waypoints_grid), 1)),
                        len(path) - 1
                    )
                    # Store current waypoint target for path segment checking
                    self._current_waypoint_world = self.planner.map_to_world(gx, gy)

                wx, wy = self._current_waypoint_world

                wp_clearance = self.planner.get_clearance(gx, gy)
                wp_clearance_m = wp_clearance * self.planner.resolution

                self.get_logger().info(
                    f"-> Waypoint {i + 1}/{len(waypoints_grid)}: "
                    f"({wx:.2f}, {wy:.2f}), clearance: {wp_clearance_m:.2f}m"
                )

                reached, aborted = self.goto(wx, wy, goal_wz)

                if aborted:
                    self.get_logger().warn("Aborted! Re-planning...")
                    navigation_complete = False
                    time.sleep(0.3)
                    break

                if not reached:
                    response.success = False
                    response.message = f"Failed to reach waypoint {i + 1}"
                    self.busy = False
                    with self._path_lock:
                        self._current_path = []
                        self._current_waypoint_world = None
                    return response

                time.sleep(0.2)

            if navigation_complete:
                current = self.pose
                dist_to_goal = ((current.position.x - goal_wx) ** 2 +
                                (current.position.y - goal_wy) ** 2) ** 0.5

                if dist_to_goal < 0.5:
                    self.get_logger().info("Navigation completed!")
                    response.success = True
                    response.message = "Reached target."
                    self.busy = False
                    with self._path_lock:
                        self._current_path = []
                        self._current_waypoint_world = None
                    return response

        response.success = False
        response.message = f"Failed after {max_replan_attempts} re-plan attempts."
        self.busy = False
        with self._path_lock:
            self._current_path = []
            self._current_waypoint_world = None
        return response


def main():
    rclpy.init()
    node = NavigationAgentService()

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