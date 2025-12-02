#!/usr/bin/env python3
"""
Room Frontier Exploration Agent
-------------------------------
Explores only within the current room/corridor.
Detects doors dynamically and refuses to cross them.
Uses flood-fill to determine room boundaries.

Map values: -1=Unknown, 0=Free, 1=Wall
"""

import math
import time
import threading
from typing import List, Tuple, Optional, Set
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8MultiArray
from autonomous_system.srv import NavigateToPose

GridPoint = Tuple[int, int]


class RoomExplorationAgent(Node):
    def __init__(self):
        super().__init__("room_exploration_agent")

        # Parameters
        self.declare_parameter("cruise_altitude", 1.5)
        self.declare_parameter("exploration_timeout", 120.0)
        self.declare_parameter("min_door_width", 0.6)
        self.declare_parameter("max_door_width", 2.0)

        self.cruise_altitude = float(self.get_parameter("cruise_altitude").value)
        self.timeout = float(self.get_parameter("exploration_timeout").value)
        self.min_door_width = float(self.get_parameter("min_door_width").value)
        self.max_door_width = float(self.get_parameter("max_door_width").value)

        # Map data
        self.map_lock = threading.Lock()
        self.map_data: Optional[np.ndarray] = None
        self.map_width = self.map_height = 0
        self.resolution = 0.05
        self.origin = (0.0, 0.0, 0.0)

        # Subscriptions
        self.map_sub = self.create_subscription(
            Int8MultiArray, "/exploration/observed_map", self._map_cb, 10,
            callback_group=ReentrantCallbackGroup()
        )
        self.pose: Optional[Pose] = None
        self.pose_sub = self.create_subscription(
            Pose, "/simple_drone/gt_pose", self._pose_cb, 10
        )

        # Navigation client
        self.nav_client = self.create_client(NavigateToPose, "/navigate_to_pose")
        self.get_logger().info("Waiting for /navigate_to_pose...")
        self.nav_client.wait_for_service()

        # Service
        self.srv = self.create_service(
            NavigateToPose, "/explore_room", self._handle_request,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.busy = False
        self.get_logger().info("RoomExplorationAgent ready")

    # ─────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────
    def _map_cb(self, msg: Int8MultiArray):
        if len(msg.layout.dim) < 2:
            return
        h, w = msg.layout.dim[0].size, msg.layout.dim[1].size
        try:
            parts = msg.layout.dim[0].label.split(",")
            self.resolution = float(parts[0])
            self.origin = (float(parts[1]), float(parts[2]), 0.0)
        except:
            pass
        with self.map_lock:
            self.map_width, self.map_height = w, h
            self.map_data = np.array(msg.data, dtype=np.int8).reshape((h, w))

    def _pose_cb(self, msg: Pose):
        self.pose = msg

    # ─────────────────────────────────────────────────────────────
    # Coordinate conversion
    # ─────────────────────────────────────────────────────────────
    def world_to_grid(self, wx: float, wy: float) -> GridPoint:
        ox, oy, _ = self.origin
        return int(round((wx - ox) / self.resolution)), int(round((wy - oy) / self.resolution))

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        ox, oy, _ = self.origin
        return gx * self.resolution + ox, gy * self.resolution + oy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.map_width and 0 <= gy < self.map_height

    # ─────────────────────────────────────────────────────────────
    # Door Detection (simplified)
    # ─────────────────────────────────────────────────────────────
    def detect_door_cells(self) -> Set[GridPoint]:
        """
        Find all cells that are part of a door (narrow passage between walls).
        Returns set of grid cells that should be treated as blocked.
        """
        if self.map_data is None:
            return set()

        door_cells: Set[GridPoint] = set()
        min_w = int(self.min_door_width / self.resolution)
        max_w = int(self.max_door_width / self.resolution)
        wall_check = 15

        obs = self.map_data

        for gy in range(1, self.map_height - 1):
            for gx in range(1, self.map_width - 1):
                if obs[gy, gx] != 0:  # Must be free
                    continue

                # Check horizontal door (walls above & below)
                if self._check_door(gx, gy, 0, 1, wall_check, min_w, max_w):
                    door_cells.update(self._get_door_segment(gx, gy, 0, 1, max_w))
                # Check vertical door (walls left & right)
                elif self._check_door(gx, gy, 1, 0, wall_check, min_w, max_w):
                    door_cells.update(self._get_door_segment(gx, gy, 1, 0, max_w))

        return door_cells

    def _check_door(self, gx: int, gy: int, dx: int, dy: int,
                    wall_check: int, min_w: int, max_w: int) -> bool:
        """Check if (gx,gy) is part of a door perpendicular to (dx,dy) direction."""
        obs = self.map_data

        # Check walls on both sides (perpendicular direction)
        wall_pos = wall_neg = False
        for d in range(1, wall_check + 1):
            nx, ny = gx + d * dy, gy + d * dx  # perpendicular
            if self.in_bounds(nx, ny) and obs[ny, nx] == 1:
                wall_pos = True
                break
        for d in range(1, wall_check + 1):
            nx, ny = gx - d * dy, gy - d * dx
            if self.in_bounds(nx, ny) and obs[ny, nx] == 1:
                wall_neg = True
                break

        if not (wall_pos and wall_neg):
            return False

        # Count passage width (in perpendicular direction)
        width = 1
        for d in range(1, max_w + 1):
            nx, ny = gx + d * dy, gy + d * dx
            if self.in_bounds(nx, ny) and obs[ny, nx] == 0:
                width += 1
            else:
                break
        for d in range(1, max_w + 1):
            nx, ny = gx - d * dy, gy - d * dx
            if self.in_bounds(nx, ny) and obs[ny, nx] == 0:
                width += 1
            else:
                break

        return min_w <= width <= max_w

    def _get_door_segment(self, gx: int, gy: int, dx: int, dy: int,
                          max_w: int) -> Set[GridPoint]:
        """Get all cells in the door segment perpendicular to (dx,dy)."""
        obs = self.map_data
        cells = {(gx, gy)}
        for d in range(1, max_w + 1):
            nx, ny = gx + d * dy, gy + d * dx
            if self.in_bounds(nx, ny) and obs[ny, nx] == 0:
                cells.add((nx, ny))
            else:
                break
        for d in range(1, max_w + 1):
            nx, ny = gx - d * dy, gy - d * dx
            if self.in_bounds(nx, ny) and obs[ny, nx] == 0:
                cells.add((nx, ny))
            else:
                break
        return cells

    # ─────────────────────────────────────────────────────────────
    # Room Boundary Detection (Flood Fill)
    # ─────────────────────────────────────────────────────────────
    def find_room_cells(self, start: GridPoint, door_cells: Set[GridPoint]) -> Set[GridPoint]:
        """
        Flood fill from start position.
        Stops at: walls (1), unknown (-1), and door cells.
        Returns all reachable free cells within the room.
        """
        if self.map_data is None:
            return set()

        obs = self.map_data
        room = set()
        queue = deque([start])
        visited = {start}

        while queue:
            gx, gy = queue.popleft()

            if not self.in_bounds(gx, gy):
                continue
            if obs[gy, gx] != 0:  # Only free cells
                continue
            if (gx, gy) in door_cells:  # Stop at doors
                continue

            room.add((gx, gy))

            for nx, ny in [(gx-1, gy), (gx+1, gy), (gx, gy-1), (gx, gy+1)]:
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        return room

    # ─────────────────────────────────────────────────────────────
    # Frontier Detection (Room-bounded)
    # ─────────────────────────────────────────────────────────────
    def find_room_frontiers(self, room_cells: Set[GridPoint]) -> List[GridPoint]:
        """Find frontiers only within the room (adjacent to unknown but inside room)."""
        if self.map_data is None:
            return []

        obs = self.map_data
        frontiers = []

        for gx, gy in room_cells:
            # Check if adjacent to unknown
            for nx, ny in [(gx-1, gy), (gx+1, gy), (gx, gy-1), (gx, gy+1)]:
                if self.in_bounds(nx, ny) and obs[ny, nx] == -1:
                    frontiers.append((gx, gy))
                    break

        return frontiers

    # ─────────────────────────────────────────────────────────────
    # Navigation
    # ─────────────────────────────────────────────────────────────
    def navigate_to(self, wx: float, wy: float, wz: float) -> bool:
        req = NavigateToPose.Request()
        req.x, req.y, req.z = wx, wy, wz
        future = self.nav_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None and future.result().success

    # ─────────────────────────────────────────────────────────────
    # Service Handler
    # ─────────────────────────────────────────────────────────────
    def _handle_request(self, request, response):
        if self.busy:
            response.success, response.message = False, "Busy"
            return response

        self.busy = True
        altitude = request.z if request.z > 0 else self.cruise_altitude
        start_time = time.time()

        # Wait for data
        for _ in range(100):
            if self.map_data is not None and self.pose is not None:
                break
            time.sleep(0.1)

        if self.map_data is None or self.pose is None:
            response.success, response.message = False, "No map/pose"
            self.busy = False
            return response

        visited = 0
        self.get_logger().info("Starting room exploration (no door crossing)...")

        while rclpy.ok() and (time.time() - start_time) < self.timeout:
            with self.map_lock:
                # Detect doors and room boundary
                door_cells = self.detect_door_cells()
                start_pos = self.world_to_grid(self.pose.position.x, self.pose.position.y)
                room_cells = self.find_room_cells(start_pos, door_cells)
                frontiers = self.find_room_frontiers(room_cells)

            self.get_logger().info(
                f"Room: {len(room_cells)} cells, Doors: {len(door_cells)} cells, "
                f"Frontiers: {len(frontiers)}"
            )

            if not frontiers:
                response.success = True
                response.message = f"Room fully explored! Visited {visited} frontiers."
                self.get_logger().info(response.message)
                self.busy = False
                return response

            # Pick closest frontier
            gx, gy = self.world_to_grid(self.pose.position.x, self.pose.position.y)
            target = min(frontiers, key=lambda f: (f[0]-gx)**2 + (f[1]-gy)**2)
            wx, wy = self.grid_to_world(*target)

            self.get_logger().info(f"  -> Going to frontier ({wx:.1f}, {wy:.1f})")

            if self.navigate_to(wx, wy, altitude):
                visited += 1

            time.sleep(0.3)

        response.success = False
        response.message = f"Timeout. Visited {visited} frontiers."
        self.busy = False
        return response


def main():
    rclpy.init()
    node = RoomExplorationAgent()
    executor = MultiThreadedExecutor(num_threads=2)
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