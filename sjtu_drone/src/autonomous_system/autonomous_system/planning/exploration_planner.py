#!/usr/bin/env python3
"""
Exploration A* Planner
----------------------
A* planner for partially known maps from fog-of-war exploration.

Cell values:
    -1 = Unknown (treated as FREE for optimistic planning)
     0 = Known free
     1 = Known wall (blocked)
"""

import math
import heapq
from typing import List, Tuple, Optional
import numpy as np

GridPoint = Tuple[int, int]


class ExplorationPlanner:
    """
    A* planner for exploration with partially known maps.

    Unknown cells are treated as passable (optimistic planning).
    Only known walls block the path.
    """

    def __init__(
            self,
            resolution: float,
            origin: Tuple[float, float, float],
            turn_penalty: float = 1.0,
            safety_margin: int = 3,
    ):
        self.resolution = resolution
        self.origin = origin
        self.turn_penalty = turn_penalty
        self.safety_margin = safety_margin

        # Map will be set dynamically
        self.observed_map: Optional[np.ndarray] = None
        self.inflated_walls: Optional[np.ndarray] = None
        self.height = 0
        self.width = 0

    def update_map(self, observed_map: np.ndarray):
        """
        Update the observed map.

        Args:
            observed_map: 2D array with values -1 (unknown), 0 (free), 1 (wall)
        """
        self.observed_map = observed_map.copy()
        self.height, self.width = observed_map.shape

        # Inflate only KNOWN walls (value == 1)
        self.inflated_walls = self._inflate_walls(observed_map)

    def _inflate_walls(self, observed_map: np.ndarray) -> np.ndarray:
        """Inflate known walls by safety margin."""
        walls = (observed_map == 1).astype(np.uint8)

        if self.safety_margin <= 0:
            return walls

        # Simple dilation using numpy
        inflated = walls.copy()
        for _ in range(self.safety_margin):
            padded = np.pad(inflated, 1, mode='constant', constant_values=0)
            inflated = (
                    padded[1:-1, 1:-1] |  # center
                    padded[:-2, 1:-1] |  # up
                    padded[2:, 1:-1] |  # down
                    padded[1:-1, :-2] |  # left
                    padded[1:-1, 2:]  # right
            ).astype(np.uint8)

        return inflated

    def world_to_map(self, wx: float, wy: float) -> GridPoint:
        """Convert world coordinates to grid indices."""
        ox, oy, _ = self.origin
        gx = int(round((wx - ox) / self.resolution))
        gy = int(round((wy - oy) / self.resolution))
        return gx, gy

    def map_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates."""
        ox, oy, _ = self.origin
        wx = gx * self.resolution + ox
        wy = gy * self.resolution + oy
        return float(wx), float(wy)

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_passable(self, gx: int, gy: int) -> bool:
        """
        Check if cell is passable for planning.
        Unknown (-1) and free (0) are passable.
        Only known walls (1) after inflation are blocked.
        """
        if not self.in_bounds(gx, gy):
            return False
        if self.inflated_walls is None:
            return True
        return self.inflated_walls[gy, gx] == 0

    def is_known_wall(self, gx: int, gy: int) -> bool:
        """Check if cell is a known wall (before inflation)."""
        if not self.in_bounds(gx, gy):
            return False
        if self.observed_map is None:
            return False
        return self.observed_map[gy, gx] == 1

    def check_path_blocked(self, path: List[GridPoint]) -> Optional[int]:
        """
        Check if any cell in the path is now a known wall.

        Returns:
            Index of first blocked cell, or None if path is clear.
        """
        if self.inflated_walls is None:
            return None

        for i, (gx, gy) in enumerate(path):
            if not self.is_passable(gx, gy):
                return i
        return None

    @staticmethod
    def heuristic(a: GridPoint, b: GridPoint) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def neighbors(self, node: GridPoint) -> List[GridPoint]:
        """4-connected neighbors that are passable."""
        gx, gy = node
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        return [(gx + dx, gy + dy) for dx, dy in moves
                if self.is_passable(gx + dx, gy + dy)]

    def plan(self, start: GridPoint, goal: GridPoint) -> List[GridPoint]:
        """
        A* search treating unknown cells as free.
        """
        if self.observed_map is None:
            print("[ExplorationPlanner] No map set!")
            return []

        if not self.is_passable(start[0], start[1]):
            start = self._find_nearest_passable(start)
            if start is None:
                return []

        if not self.is_passable(goal[0], goal[1]):
            goal = self._find_nearest_passable(goal)
            if goal is None:
                return []

        open_heap = [(0.0, start, None)]
        came_from = {}
        g_cost = {start: 0.0}

        while open_heap:
            _, current, prev_dir = heapq.heappop(open_heap)

            if current == goal:
                return self._reconstruct(came_from, current)

            for neighbor in self.neighbors(current):
                dx = neighbor[0] - current[0]
                dy = neighbor[1] - current[1]
                cur_dir = (dx, dy)

                step_cost = 1.0
                if prev_dir is not None and cur_dir != prev_dir:
                    step_cost += self.turn_penalty

                tentative_g = g_cost[current] + step_cost

                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f, neighbor, cur_dir))

        return []

    def _find_nearest_passable(self, point: GridPoint, max_search: int = 30) -> Optional[GridPoint]:
        gx, gy = point
        for radius in range(1, max_search):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:
                        nx, ny = gx + dx, gy + dy
                        if self.is_passable(nx, ny):
                            return (nx, ny)
        return None

    @staticmethod
    def _reconstruct(came_from: dict, cur: GridPoint) -> List[GridPoint]:
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path