#!/usr/bin/env python3
"""
Navigation Agent with Exploration (Service-based)
--------------------------------------------------
Navigates using a partially known exploration map.
Re-plans when obstacles are discovered on the path.

Key behavior:
    - Unknown cells (-1) treated as free for planning
    - Continuously monitors path for newly discovered walls
    - Stops immediately and re-plans when path is blocked
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

    Uses WaypointController's built-in abort mechanism:
    - trigger_abort() stops the drone immediately during goto()
    - clear_abort() resets for new navigation
    - goto() returns (reached, aborted) tuple
    """

    def __init__(self):
        super().__init__(name="navigation_agent_service")

        # Parameters
        self.declare_parameter("turn_penalty", 1.5)
        self.declare_parameter("rdp_eps", 1.5)
        self.declare_parameter("min_turn_dist", 6.0)
        self.declare_parameter("cruise_altitude", 1.5)
        self.declare_parameter("safety_margin", 15)
        self.declare_parameter("path_check_rate", 10.0)  # Hz

        self.turn_penalty = float(self.get_parameter("turn_penalty").value)
        self.rdp_eps = float(self.get_parameter("rdp_eps").value)
        self.min_turn_dist = float(self.get_parameter("min_turn_dist").value)
        self.cruise_altitude = float(self.get_parameter("cruise_altitude").value)
        self.safety_margin = int(self.get_parameter("safety_margin").value)
        self.path_check_rate = float(self.get_parameter("path_check_rate").value)

        # Planner (will be configured when map received)
        self.planner: Optional[ExplorationPlanner] = None
        self.map_received = False

        # Current path being executed (grid coordinates)
        self._current_path: List[GridPoint] = []
        self._current_waypoint_idx = 0
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

        # Path monitoring timer - checks if path is blocked
        self.monitor_timer = self.create_timer(
            1.0 / self.path_check_rate,
            self.check_path_blocked,
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
        self.get_logger().info("NavigationAgentService ready (exploration mode).")
        self.get_logger().info("Waiting for exploration map on /exploration/observed_map...")

    def map_callback(self, msg: Int8MultiArray):
        """Receive and update the exploration map."""
        if len(msg.layout.dim) < 2:
            return

        height = msg.layout.dim[0].size
        width = msg.layout.dim[1].size

        # Reconstruct 2D array
        observed_map = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Initialize planner on first map
        if self.planner is None:
            # Get map metadata from message (stored in label field as "res,ox,oy")
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
                safety_margin=self.safety_margin,
            )
            self.get_logger().info(f"Planner initialized: res={resolution}, origin=({origin_x}, {origin_y})")

        self.planner.update_map(observed_map)

        if not self.map_received:
            self.map_received = True
            self.get_logger().info("First exploration map received!")

    def check_path_blocked(self):
        """
        Check if current path is blocked by newly discovered walls.
        Called periodically by timer. If blocked, triggers abort in the controller.
        """
        if not self.map_received or self.planner is None:
            return

        with self._path_lock:
            if not self._current_path:
                return

            # Check remaining path from current waypoint
            remaining_path = self._current_path[self._current_waypoint_idx:]
            blocked_idx = self.planner.check_path_blocked(remaining_path)

        if blocked_idx is not None:
            self.get_logger().warn(f"Path blocked at index {blocked_idx}! Triggering abort...")
            # This calls the WaypointController's trigger_abort() method
            # which will cause the current goto() to stop immediately
            self.trigger_abort()

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
        self.clear_abort()  # Reset abort flag from WaypointController
        time.sleep(0.1)

        goal_wx = float(request.x)
        goal_wy = float(request.y)
        goal_wz = float(request.z) if request.z > 0 else self.cruise_altitude

        max_replan_attempts = 20
        attempt = 0

        while attempt < max_replan_attempts:
            attempt += 1
            self.clear_abort()  # Clear abort before each attempt

            # Get current position
            current = self.pose
            start_wx = float(current.position.x)
            start_wy = float(current.position.y)

            self.get_logger().info(
                f"[Attempt {attempt}] Planning: ({start_wx:.2f}, {start_wy:.2f}) -> "
                f"({goal_wx:.2f}, {goal_wy:.2f})"
            )

            # Plan path
            start_grid = self.planner.world_to_map(start_wx, start_wy)
            goal_grid = self.planner.world_to_map(goal_wx, goal_wy)

            path = self.planner.plan(start_grid, goal_grid)
            if not path:
                response.success = False
                response.message = "No path found."
                self.busy = False
                return response

            # Simplify
            simplified = rdp_simplify(path, eps=self.rdp_eps)
            waypoints_grid = extract_turn_points(simplified, min_dist=self.min_turn_dist)

            # Store path for monitoring
            with self._path_lock:
                self._current_path = path
                self._current_waypoint_idx = 0

            self.get_logger().info(f"Path: {len(path)} cells -> {len(waypoints_grid)} waypoints")

            # Execute waypoints
            navigation_complete = True
            for i, (gx, gy) in enumerate(waypoints_grid):
                with self._path_lock:
                    # Update waypoint index (approximate position in full path)
                    self._current_waypoint_idx = min(
                        i * (len(path) // max(len(waypoints_grid), 1)),
                        len(path) - 1
                    )

                wx, wy = self.planner.map_to_world(gx, gy)
                self.get_logger().info(f"-> Waypoint {i + 1}/{len(waypoints_grid)}: ({wx:.2f}, {wy:.2f})")

                # Use WaypointController's goto() which now returns (reached, aborted)
                reached, aborted = self.goto(wx, wy, goal_wz)

                if aborted:
                    self.get_logger().warn("Aborted! Re-planning...")
                    navigation_complete = False
                    time.sleep(0.3)  # Brief pause before re-plan
                    break

                if not reached:
                    response.success = False
                    response.message = f"Failed to reach waypoint {i + 1}"
                    self.busy = False
                    with self._path_lock:
                        self._current_path = []
                    return response

                time.sleep(0.2)

            if navigation_complete:
                # Check if we're close to goal
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
                    return response

        response.success = False
        response.message = f"Failed after {max_replan_attempts} re-plan attempts."
        self.busy = False
        with self._path_lock:
            self._current_path = []
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