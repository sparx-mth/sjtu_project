#!/usr/bin/env python3
"""
A* Path Planner (Enhanced with Wall Avoidance)
----------------------------------------------
Loads an occupancy grid map and performs A* search with:
 - Obstacle inflation for safety margin
 - Distance-to-obstacle cost for corridor centering
 - Turn penalty for smoother paths

The distance cost makes the planner prefer paths through the MIDDLE
of corridors rather than hugging walls.

Coordinate Convention:
    - Grid origin (0, 0) is at the BOTTOM-LEFT of the map
    - gx increases to the RIGHT, gy increases UPWARD
    - Access pattern: map_data[gy, gx]
"""

import os
import math
import heapq
from typing import List, Tuple, Optional

import cv2
import yaml
import numpy as np
from scipy import ndimage

GridPoint = Tuple[int, int]


class AStarPlanner:
    """
    A* planner with wall avoidance using distance transform.

    The planner uses three cost components:
    1. Base step cost (1.0 per cell)
    2. Turn penalty (discourage direction changes)
    3. Wall proximity cost (prefer staying away from obstacles)

    The wall proximity cost uses a distance transform - cells closer
    to walls have higher cost, pushing the path toward corridor centers.
    """

    def __init__(
            self,
            map_yaml_path: str,
            turn_penalty: float = 1.0,
            wall_cost_weight: float = 0.7,
            safety_margin: int = 20,
            preferred_clearance: int = 25,
    ):
        """
        Args:
            map_yaml_path: Path to a ROS-style map .yaml file.
            turn_penalty: Cost for changing direction between steps.
            wall_cost_weight: How much to penalize being near walls (0-1).
                             Higher = stronger preference for corridor center.
            safety_margin: Minimum cells to inflate obstacles (hard boundary).
            preferred_clearance: Cells of clearance for zero wall cost.
                                Beyond safety_margin, cost decreases linearly
                                until preferred_clearance where it's zero.
        """
        self.turn_penalty = float(turn_penalty)
        self.wall_cost_weight = float(wall_cost_weight)
        self.safety_margin = safety_margin
        self.preferred_clearance = preferred_clearance

        # Load map metadata
        with open(map_yaml_path, "r") as f:
            info = yaml.safe_load(f)

        self.resolution: float = float(info["resolution"])
        self.origin: Tuple[float, float, float] = tuple(info["origin"])
        img_path = info["image"]

        if not img_path.startswith("/"):
            img_path = os.path.join(os.path.dirname(map_yaml_path), img_path)

        img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        assert img is not None, f"Failed to load map image: {img_path}"

        # Convert to binary (0 = free, 1 = occupied)
        # Flip so origin is at bottom-left
        binary = np.zeros_like(img, dtype=np.uint8)
        binary[img < 50] = 1
        self.raw_obstacles = np.flipud(binary)

        # Compute distance transform BEFORE inflation
        # This gives distance to nearest obstacle for each cell
        free_space = (self.raw_obstacles == 0).astype(np.float32)
        self.distance_to_obstacle = ndimage.distance_transform_edt(free_space)

        # Inflate obstacles for safety margin (hard boundary)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * safety_margin + 1, 2 * safety_margin + 1)
        )
        self.map_data = cv2.dilate(self.raw_obstacles, kernel)

        # Compute wall cost map (0-1, higher near walls)
        # Cells within safety_margin are blocked (infinite cost via map_data)
        # Cells between safety_margin and preferred_clearance have decreasing cost
        # Cells beyond preferred_clearance have zero wall cost
        self.wall_cost_map = self._compute_wall_cost_map()

        self.height, self.width = self.map_data.shape
        print(f"[AStarPlanner] Map: {img_path}, shape={self.map_data.shape}")
        print(f"[AStarPlanner] Safety margin: {safety_margin} cells, "
              f"Preferred clearance: {preferred_clearance} cells")
        print(f"[AStarPlanner] Wall cost weight: {wall_cost_weight}")

    def _compute_wall_cost_map(self) -> np.ndarray:
        """
        Compute a cost map based on distance to obstacles.

        Returns array where:
        - 0.0 = far from walls (preferred)
        - 1.0 = close to walls (penalized)
        - Blocked cells (within safety margin) don't matter since they're not traversable
        """
        # Distance from safety margin to preferred clearance
        transition_zone = self.preferred_clearance - self.safety_margin

        if transition_zone <= 0:
            # No transition zone, just binary
            return np.zeros_like(self.distance_to_obstacle)

        # Cost decreases linearly from safety_margin to preferred_clearance
        # cost = 1.0 at safety_margin, 0.0 at preferred_clearance
        cost_map = np.zeros_like(self.distance_to_obstacle)

        in_transition = (
                (self.distance_to_obstacle > self.safety_margin) &
                (self.distance_to_obstacle < self.preferred_clearance)
        )

        # Linear interpolation: 1.0 at safety_margin, 0.0 at preferred_clearance
        cost_map[in_transition] = 1.0 - (
                (self.distance_to_obstacle[in_transition] - self.safety_margin) /
                transition_zone
        )

        return cost_map.astype(np.float32)

    # ------------------------------------------------------------------ #
    #   Coordinate conversions
    # ------------------------------------------------------------------ #

    def world_to_map(self, wx: float, wy: float) -> GridPoint:
        """Convert world coordinates (meters) -> grid indices (gx, gy)."""
        ox, oy, _ = self.origin
        gx = int(round((wx - ox) / self.resolution))
        gy = int(round((wy - oy) / self.resolution))
        return gx, gy

    def map_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid indices (gx, gy) -> world coordinates (meters)."""
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
        """Return True if cell (gx, gy) is free (not in inflated obstacle)."""
        return self.in_bounds(gx, gy) and self.map_data[gy, gx] == 0

    def get_clearance(self, gx: int, gy: int) -> float:
        """Get distance to nearest obstacle for a cell (in grid cells)."""
        if not self.in_bounds(gx, gy):
            return 0.0
        return float(self.distance_to_obstacle[gy, gx])

    def get_clearance_world(self, wx: float, wy: float) -> float:
        """Get distance to nearest obstacle in meters."""
        gx, gy = self.world_to_map(wx, wy)
        return self.get_clearance(gx, gy) * self.resolution

    # ------------------------------------------------------------------ #
    #   A* helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def heuristic(a: GridPoint, b: GridPoint) -> float:
        """Euclidean distance heuristic."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def neighbors(self, node: GridPoint) -> List[GridPoint]:
        """4-connected neighborhood of free cells."""
        gx, gy = node
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        return [(gx + dx, gy + dy) for dx, dy in moves if self.is_free(gx + dx, gy + dy)]

    def get_wall_cost(self, gx: int, gy: int) -> float:
        """Get wall proximity cost for a cell (0-1)."""
        if not self.in_bounds(gx, gy):
            return 1.0
        return float(self.wall_cost_map[gy, gx])

    # ------------------------------------------------------------------ #
    #   A* with wall avoidance
    # ------------------------------------------------------------------ #

    def plan(self, start: GridPoint, goal: GridPoint) -> List[GridPoint]:
        """
        Run A* search with turn penalty and wall avoidance cost.

        Cost function:
            step_cost = 1.0 + turn_penalty (if direction changed) + wall_cost_weight * wall_proximity

        This makes the planner:
        1. Prefer shorter paths (base cost)
        2. Prefer straighter paths (turn penalty)
        3. Prefer paths away from walls (wall proximity cost)
        """
        if not self.is_free(start[0], start[1]):
            print(f"[AStarPlanner] Start {start} is not free!")
            # Try to find nearest free cell
            start = self._find_nearest_free(start)
            if start is None:
                return []
            print(f"[AStarPlanner] Using nearest free cell: {start}")

        if not self.is_free(goal[0], goal[1]):
            print(f"[AStarPlanner] Goal {goal} is not free!")
            goal = self._find_nearest_free(goal)
            if goal is None:
                return []
            print(f"[AStarPlanner] Using nearest free cell: {goal}")

        open_heap: List[Tuple[float, GridPoint, Optional[Tuple[int, int]]]] = []
        heapq.heappush(open_heap, (0.0, start, None))

        came_from: dict[GridPoint, GridPoint] = {}
        g_cost: dict[GridPoint, float] = {start: 0.0}

        while open_heap:
            _, current, prev_dir = heapq.heappop(open_heap)

            if current == goal:
                return self._reconstruct(came_from, current)

            # Skip if we've already found a better path to this node
            if current in came_from and current != start:
                current_g = g_cost.get(current, float('inf'))
                if g_cost.get(current, float('inf')) < current_g:
                    continue

            for neighbor in self.neighbors(current):
                nx, ny = neighbor
                dx = nx - current[0]
                dy = ny - current[1]
                cur_dir = (dx, dy)

                # Base step cost
                step_cost = 1.0

                # Turn penalty
                if prev_dir is not None and cur_dir != prev_dir:
                    step_cost += self.turn_penalty

                # Wall proximity cost (prefer staying away from walls)
                wall_cost = self.get_wall_cost(nx, ny)
                step_cost += self.wall_cost_weight * wall_cost

                tentative_g = g_cost[current] + step_cost

                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f, neighbor, cur_dir))

        print("[AStarPlanner] No path found.")
        return []

    def _find_nearest_free(self, point: GridPoint, max_search: int = 50) -> Optional[GridPoint]:
        """Find the nearest free cell to a blocked point."""
        gx, gy = point
        for radius in range(1, max_search):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:
                        nx, ny = gx + dx, gy + dy
                        if self.is_free(nx, ny):
                            return (nx, ny)
        return None

    @staticmethod
    def _reconstruct(came_from: dict[GridPoint, GridPoint], cur: GridPoint) -> List[GridPoint]:
        """Reconstruct path from came_from dictionary."""
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path

    # ------------------------------------------------------------------ #
    #   Escape direction finding (for stuck recovery)
    # ------------------------------------------------------------------ #

    def find_escape_direction(
            self,
            wx: float,
            wy: float,
            num_samples: int = 16,
            search_distance: float = 1.0,
    ) -> Optional[Tuple[float, float]]:
        """
        Find the best direction to escape from current position.

        Samples directions around the current position and returns
        the direction with the most clearance.

        Args:
            wx, wy: Current world position
            num_samples: Number of directions to sample
            search_distance: How far to look in each direction (meters)

        Returns:
            (dx, dy) unit vector pointing toward best escape direction,
            or None if no good direction found.
        """
        best_direction = None
        best_clearance = 0.0

        search_cells = int(search_distance / self.resolution)

        for i in range(num_samples):
            angle = 2.0 * math.pi * i / num_samples
            dx = math.cos(angle)
            dy = math.sin(angle)

            # Check clearance along this direction
            total_clearance = 0.0
            valid_samples = 0

            for dist in range(1, search_cells + 1):
                check_wx = wx + dx * dist * self.resolution
                check_wy = wy + dy * dist * self.resolution
                gx, gy = self.world_to_map(check_wx, check_wy)

                if not self.in_bounds(gx, gy):
                    break

                if self.map_data[gy, gx] == 1:
                    # Hit obstacle, this direction is bad
                    break

                total_clearance += self.distance_to_obstacle[gy, gx]
                valid_samples += 1

            if valid_samples > 0:
                avg_clearance = total_clearance / valid_samples
                if avg_clearance > best_clearance:
                    best_clearance = avg_clearance
                    best_direction = (dx, dy)

        if best_clearance > self.safety_margin:
            return best_direction
        return None

    def find_safe_retreat_point(
            self,
            wx: float,
            wy: float,
            retreat_distance: float = 0.5,
    ) -> Optional[Tuple[float, float]]:
        """
        Find a safe point to retreat to from current position.

        Returns:
            (wx, wy) world coordinates of safe retreat point,
            or None if no safe point found.
        """
        direction = self.find_escape_direction(wx, wy)
        if direction is None:
            return None

        dx, dy = direction
        retreat_wx = wx + dx * retreat_distance
        retreat_wy = wy + dy * retreat_distance

        # Verify the retreat point is actually free
        gx, gy = self.world_to_map(retreat_wx, retreat_wy)
        if self.is_free(gx, gy):
            return (retreat_wx, retreat_wy)

        return None