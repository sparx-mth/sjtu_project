#!/usr/bin/env python3
"""
A* Path Planner
---------------
Loads an occupancy grid map from a YAML + image file and performs A* search
on an inflated binary grid.

Coordinate Convention:
    - Grid origin (0, 0) is at the BOTTOM-LEFT of the map
    - gx increases to the RIGHT, gy increases UPWARD
    - Access pattern: map_data[gy, gx]

Includes a turn penalty in the cost function to discourage direction changes
and produce straighter paths with fewer turns.
"""

import os
import math
import heapq
from typing import List, Tuple, Optional

import cv2
import yaml
import numpy as np


GridPoint = Tuple[int, int]


class AStarPlanner:
    """
    A* planner operating on an inflated occupancy grid map.

    Grid values:
        0 = free
        1 = occupied / inflated obstacle

    Cost model:
        - Each step has base cost = 1
        - Changing direction from the previous step adds turn_penalty
    """

    def __init__(self, map_yaml_path: str, turn_penalty: float = 1.0):
        """
        Args:
            map_yaml_path: Path to a ROS-style map .yaml file.
            turn_penalty: Additional cost for changing direction between steps.
        """
        self.turn_penalty = float(turn_penalty)

        with open(map_yaml_path, "r") as f:
            info = yaml.safe_load(f)

        self.resolution: float = float(info["resolution"])
        self.origin: Tuple[float, float, float] = tuple(info["origin"])
        img_path = info["image"]

        if not img_path.startswith("/"):
            img_path = os.path.join(os.path.dirname(map_yaml_path), img_path)

        img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        assert img is not None, f"Failed to load map image: {img_path}"

        # Convert image to binary obstacle grid (0 = free, 1 = occupied)
        # Flip vertically so grid origin (0,0) is at BOTTOM-LEFT
        binary = np.zeros_like(img, dtype=np.uint8)
        binary[img < 50] = 1
        self.map_data = np.flipud(binary)

        # Inflate obstacles (~10 cells radius)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
        self.map_data = cv2.dilate(self.map_data, kernel)

        self.height, self.width = self.map_data.shape
        print(f"[AStarPlanner] Map loaded: {img_path}, shape={self.map_data.shape}")

    # ------------------------------------------------------------------ #
    #   Coordinate conversions
    # ------------------------------------------------------------------ #

    def world_to_map(self, wx: float, wy: float) -> GridPoint:
        """
        Convert world coordinates (meters) -> grid indices (gx, gy).

        Grid origin (0, 0) is at the bottom-left corner.
        """
        ox, oy, _ = self.origin
        gx = int(round((wx - ox) / self.resolution))
        gy = int(round((wy - oy) / self.resolution))
        return gx, gy

    def map_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """
        Convert grid indices (gx, gy) -> world coordinates (meters).

        Returns the center of the grid cell.
        """
        ox, oy, _ = self.origin
        wx = gx * self.resolution + ox
        wy = gy * self.resolution + oy
        return float(wx), float(wy)

    # ------------------------------------------------------------------ #
    #   Grid helpers
    # ------------------------------------------------------------------ #

    def in_bounds(self, gx: int, gy: int) -> bool:
        """Return True if (gx, gy) is inside the grid."""
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_free(self, gx: int, gy: int) -> bool:
        """Return True if cell (gx, gy) is free."""
        return self.in_bounds(gx, gy) and self.map_data[gy, gx] == 0

    # ------------------------------------------------------------------ #
    #   A* helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def heuristic(a: GridPoint, b: GridPoint) -> float:
        """Euclidean distance heuristic."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def neighbors(self, node: GridPoint) -> List[GridPoint]:
        """4-connected neighborhood."""
        gx, gy = node
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        return [(gx + dx, gy + dy) for dx, dy in moves if self.is_free(gx + dx, gy + dy)]

    # ------------------------------------------------------------------ #
    #   A* main function with turn penalty
    # ------------------------------------------------------------------ #

    def plan(self, start: GridPoint, goal: GridPoint) -> List[GridPoint]:
        """
        Run A* search with direction change penalty.

        Args:
            start: (gx, gy) start cell in grid coordinates.
            goal: (gx, gy) goal cell in grid coordinates.

        Returns:
            List of (gx, gy) grid cells from start to goal (inclusive).
            Returns [] if no path is found.
        """
        open_heap: List[Tuple[float, GridPoint, Optional[Tuple[int, int]]]] = []
        heapq.heappush(open_heap, (0.0, start, None))

        came_from: dict[GridPoint, GridPoint] = {}
        g_cost: dict[GridPoint, float] = {start: 0.0}

        while open_heap:
            _, current, prev_dir = heapq.heappop(open_heap)

            if current == goal:
                return self._reconstruct(came_from, current)

            for nx, ny in self.neighbors(current):
                dx = nx - current[0]
                dy = ny - current[1]
                cur_dir = (dx, dy)

                # Turn penalty: cost if direction changed
                turn_pen = 0.0 if (prev_dir is None or cur_dir == prev_dir) else self.turn_penalty

                step_cost = 1.0 + turn_pen
                tentative_g = g_cost[current] + step_cost

                if (nx, ny) not in g_cost or tentative_g < g_cost[(nx, ny)]:
                    g_cost[(nx, ny)] = tentative_g
                    came_from[(nx, ny)] = current
                    f = tentative_g + self.heuristic((nx, ny), goal)
                    heapq.heappush(open_heap, (f, (nx, ny), cur_dir))

        print("[AStarPlanner] No path found.")
        return []

    @staticmethod
    def _reconstruct(came_from: dict[GridPoint, GridPoint], cur: GridPoint) -> List[GridPoint]:
        """Reconstruct path from a 'came_from' dictionary."""
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path
