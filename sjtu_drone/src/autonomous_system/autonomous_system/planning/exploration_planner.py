#!/usr/bin/env python3
"""
Exploration A* Planner
----------------------
A* planner for partially known maps from fog-of-war exploration.

Cell values:
    -1 = Unknown (treated as FREE for optimistic planning)
     0 = Known free
     1 = Known wall (blocked)

Enhanced with:
    - Minimum safety margin (hard boundary - drone size)
    - Preferred clearance (soft preference - stay centered in corridors)
    - Wall proximity cost (prefer routes far from walls)
    - Clearance query for speed control
"""

import math
import heapq
from typing import List, Tuple, Optional, Set
import numpy as np

# Try to import scipy for efficient distance transform, fall back to numpy-only if not available
try:
    from scipy import ndimage

    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

GridPoint = Tuple[int, int]


class ExplorationPlanner:
    """
    A* planner for exploration with partially known maps.

    Unknown cells are treated as passable (optimistic planning).
    Only known walls block the path.

    Features:
        - min_safety_margin: Hard boundary (drone can't pass closer than this)
        - preferred_clearance: Soft preference (zero wall cost beyond this distance)
        - wall_cost_weight: How much to penalize being near walls (0-1)

    The planner will:
        1. Allow passage through narrow spaces (doors) if wider than 2*min_safety_margin
        2. Prefer routes through corridor centers when possible
        3. Provide clearance info for speed control
    """

    def __init__(
            self,
            resolution: float,
            origin: Tuple[float, float, float],
            turn_penalty: float = 1.0,
            safety_margin: int = 3,  # Legacy parameter, now means min_safety_margin
            min_safety_margin: int = None,  # Hard boundary (drone size)
            preferred_clearance: int = 20,  # Soft preference (corridor centering)
            wall_cost_weight: float = 0.8,  # How much to penalize wall proximity
            centering_cost_weight: float = 1.0,  # How much to prefer passage center
    ):
        self.resolution = resolution
        self.origin = origin
        self.turn_penalty = turn_penalty

        # Use min_safety_margin if provided, otherwise fall back to safety_margin
        self.min_safety_margin = min_safety_margin if min_safety_margin is not None else safety_margin
        self.preferred_clearance = preferred_clearance
        self.wall_cost_weight = wall_cost_weight
        self.centering_cost_weight = centering_cost_weight

        # Legacy compatibility
        self.safety_margin = self.min_safety_margin

        # Map will be set dynamically
        self.observed_map: Optional[np.ndarray] = None
        self.inflated_walls: Optional[np.ndarray] = None
        self.distance_to_walls: Optional[np.ndarray] = None
        self.wall_cost_map: Optional[np.ndarray] = None
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

        # Compute distance transform for known walls ONLY
        self._compute_distance_transform(observed_map)

        # Inflate only KNOWN walls by minimum safety margin (hard boundary)
        self.inflated_walls = self._inflate_walls(observed_map)

        # Compute wall cost map (soft preference for staying away from walls)
        self._compute_wall_cost_map()

    def _compute_distance_transform(self, observed_map: np.ndarray):
        """
        Compute distance to nearest known wall for each cell.

        Uses scipy if available, otherwise falls back to a simpler approximation.
        Unknown cells (-1) and free cells (0) are treated as free space.
        Only known walls (1) are considered obstacles.
        """
        # Consider only known walls (value == 1) as obstacles for distance
        # Unknown (-1) and free (0) are treated as free space for distance calc
        known_walls = (observed_map == 1).astype(np.uint8)

        if SCIPY_AVAILABLE:
            # Create binary image: 1 = not a known wall, 0 = known wall
            free_space = (known_walls == 0).astype(np.float32)
            # Distance transform gives distance to nearest wall
            self.distance_to_walls = ndimage.distance_transform_edt(free_space)
        else:
            # Fallback: approximate distance using iterative dilation
            self.distance_to_walls = self._approximate_distance_transform(known_walls)

    def _inflate_walls(self, observed_map: np.ndarray) -> np.ndarray:
        """
        Inflate ONLY known walls by minimum safety margin.

        Unknown cells (-1) are NOT inflated - they remain passable.
        Only cells with value == 1 (known walls) are inflated.
        """
        # CRITICAL: Only inflate cells that are KNOWN walls (value == 1)
        # Unknown (-1) → 0 in this array → NOT inflated → remains passable
        # Free (0) → 0 in this array → NOT inflated → remains passable
        # Wall (1) → 1 in this array → WILL be inflated
        walls = (observed_map == 1).astype(np.uint8)

        if self.min_safety_margin <= 0:
            return walls

        # Simple dilation using numpy
        inflated = walls.copy()
        for _ in range(self.min_safety_margin):
            padded = np.pad(inflated, 1, mode='constant', constant_values=0)
            inflated = (
                    padded[1:-1, 1:-1] |  # center
                    padded[:-2, 1:-1] |  # up
                    padded[2:, 1:-1] |  # down
                    padded[1:-1, :-2] |  # left
                    padded[1:-1, 2:]  # right
            ).astype(np.uint8)

        return inflated

    def _approximate_distance_transform(self, walls: np.ndarray) -> np.ndarray:
        """
        Approximate distance transform using iterative dilation.

        This is a fallback when scipy is not available.
        """
        max_dist = self.preferred_clearance + 5

        # Start with large distance for free cells, 0 for walls
        distance = np.where(walls == 1, 0, max_dist).astype(np.float32)

        # Iteratively propagate distances
        for d in range(1, max_dist + 1):
            padded = np.pad(distance, 1, mode='constant', constant_values=max_dist)
            min_neighbor = np.minimum.reduce([
                padded[1:-1, 1:-1],
                padded[:-2, 1:-1],
                padded[2:, 1:-1],
                padded[1:-1, :-2],
                padded[1:-1, 2:]
            ])
            distance = np.minimum(distance, min_neighbor + 1)

        return distance

    def _compute_wall_cost_map(self):
        """
        Compute wall proximity cost map.

        Cost ranges from 0.0 (far from walls) to 1.0 (at min_safety_margin boundary).
        """
        if self.distance_to_walls is None:
            self.wall_cost_map = None
            return

        transition_zone = self.preferred_clearance - self.min_safety_margin

        if transition_zone <= 0:
            self.wall_cost_map = np.zeros_like(self.distance_to_walls, dtype=np.float32)
            return

        cost_map = np.zeros_like(self.distance_to_walls, dtype=np.float32)

        in_transition = (
                (self.distance_to_walls > self.min_safety_margin) &
                (self.distance_to_walls < self.preferred_clearance)
        )

        cost_map[in_transition] = 1.0 - (
                (self.distance_to_walls[in_transition] - self.min_safety_margin) /
                transition_zone
        )

        cost_map[self.distance_to_walls <= self.min_safety_margin] = 1.0

        self.wall_cost_map = cost_map

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

        Unknown (-1) and free (0) are passable unless within inflated wall zone.
        Only known walls (1) after inflation are blocked.
        """
        if not self.in_bounds(gx, gy):
            return False
        if self.inflated_walls is None:
            return True
        # Cell is passable if it's NOT in the inflated wall zone
        return self.inflated_walls[gy, gx] == 0

    def is_known_wall(self, gx: int, gy: int) -> bool:
        """Check if cell is a known wall (before inflation)."""
        if not self.in_bounds(gx, gy):
            return False
        if self.observed_map is None:
            return False
        return self.observed_map[gy, gx] == 1

    def get_clearance(self, gx: int, gy: int) -> float:
        """
        Get distance to nearest KNOWN wall in grid cells.
        """
        if not self.in_bounds(gx, gy):
            return 0.0
        if self.distance_to_walls is None:
            return float(self.preferred_clearance)

        distance = float(self.distance_to_walls[gy, gx])
        return min(distance, float(self.preferred_clearance) * 2)

    def get_clearance_world(self, wx: float, wy: float) -> float:
        """Get distance to nearest wall in meters."""
        gx, gy = self.world_to_map(wx, wy)
        return self.get_clearance(gx, gy) * self.resolution

    def get_wall_cost(self, gx: int, gy: int) -> float:
        """
        Get wall proximity cost for a cell (0-1).
        """
        if not self.in_bounds(gx, gy):
            return 1.0
        if self.wall_cost_map is None:
            return 0.0
        return float(self.wall_cost_map[gy, gx])

    def get_centering_cost(self, gx: int, gy: int) -> float:
        """
        Get centering cost - penalizes cells not at local maximum clearance.

        This makes the planner prefer passing through the exact center of
        narrow passages (like doors) rather than closer to one wall.

        Returns:
            0.0 if cell is at local maximum clearance (centered)
            Positive value if neighbors have higher clearance (off-center)
        """
        if not self.in_bounds(gx, gy):
            return 0.0
        if self.distance_to_walls is None:
            return 0.0

        my_clearance = self.distance_to_walls[gy, gx]

        # Only apply centering cost in narrow areas (where it matters)
        # In wide open areas, centering doesn't matter
        if my_clearance >= self.preferred_clearance:
            return 0.0

        # Find maximum clearance among neighbors
        max_neighbor_clearance = my_clearance
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = gx + dx, gy + dy
            if self.in_bounds(nx, ny):
                neighbor_clearance = self.distance_to_walls[ny, nx]
                max_neighbor_clearance = max(max_neighbor_clearance, neighbor_clearance)

        # If we're at local maximum (centered), no penalty
        if my_clearance >= max_neighbor_clearance:
            return 0.0

        # Penalty proportional to how far we are from being centered
        # Normalized to keep cost reasonable (0 to ~1)
        diff = max_neighbor_clearance - my_clearance
        return min(diff / max(self.min_safety_margin, 1), 1.0)

    def check_path_blocked(self, path: List[GridPoint]) -> Optional[int]:
        """
        Check if any cell in the path is now a known wall.
        """
        if self.inflated_walls is None:
            return None

        for i, (gx, gy) in enumerate(path):
            if not self.is_passable(gx, gy):
                return i
        return None

    def get_path_min_clearance(self, path: List[GridPoint]) -> float:
        """Get the minimum clearance along a path."""
        if not path:
            return 0.0

        min_clearance = float('inf')
        for gx, gy in path:
            clearance = self.get_clearance(gx, gy)
            min_clearance = min(min_clearance, clearance)

        return min_clearance

    @staticmethod
    def heuristic(a: GridPoint, b: GridPoint) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def neighbors(self, node: GridPoint) -> List[GridPoint]:
        """4-connected neighbors that are passable."""
        gx, gy = node
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        result = []
        for dx, dy in moves:
            nx, ny = gx + dx, gy + dy
            if self.is_passable(nx, ny):
                result.append((nx, ny))
        return result

    def plan(self, start: GridPoint, goal: GridPoint) -> List[GridPoint]:
        """
        A* search treating unknown cells as free, with wall avoidance cost.
        """
        if self.observed_map is None:
            print("[ExplorationPlanner] No map set!")
            return []

        # === DIAGNOSTIC OUTPUT ===
        print(f"[ExplorationPlanner] Planning from {start} to {goal}")
        print(f"[ExplorationPlanner] Map: {self.width}x{self.height}, res={self.resolution}m")

        # Check bounds
        if not self.in_bounds(start[0], start[1]):
            print(f"[ExplorationPlanner] ERROR: Start {start} OUT OF BOUNDS! "
                  f"Valid: x=[0,{self.width - 1}], y=[0,{self.height - 1}]")
            return []

        if not self.in_bounds(goal[0], goal[1]):
            print(f"[ExplorationPlanner] ERROR: Goal {goal} OUT OF BOUNDS! "
                  f"Valid: x=[0,{self.width - 1}], y=[0,{self.height - 1}]")
            return []

        # Check cell values
        start_obs = self.observed_map[start[1], start[0]]
        goal_obs = self.observed_map[goal[1], goal[0]]
        start_inf = self.inflated_walls[start[1], start[0]] if self.inflated_walls is not None else 0
        goal_inf = self.inflated_walls[goal[1], goal[0]] if self.inflated_walls is not None else 0

        print(f"[ExplorationPlanner] Start: obs={start_obs}(-1=unk,0=free,1=wall), inflated={start_inf}")
        print(f"[ExplorationPlanner] Goal: obs={goal_obs}, inflated={goal_inf}")

        # Find passable start/goal if needed
        if not self.is_passable(start[0], start[1]):
            print(f"[ExplorationPlanner] Start blocked, finding nearest passable...")
            start = self._find_nearest_passable(start)
            if start is None:
                print("[ExplorationPlanner] No passable cell near start!")
                return []
            print(f"[ExplorationPlanner] New start: {start}")

        if not self.is_passable(goal[0], goal[1]):
            print(f"[ExplorationPlanner] Goal blocked, finding nearest passable...")
            goal = self._find_nearest_passable(goal)
            if goal is None:
                print("[ExplorationPlanner] No passable cell near goal!")
                return []
            print(f"[ExplorationPlanner] New goal: {goal}")

        # === A* SEARCH ===
        # Using proper closed set to avoid revisiting nodes
        open_heap: List[Tuple[float, int, GridPoint, Optional[Tuple[int, int]]]] = []
        counter = 0  # Tie-breaker for heap
        heapq.heappush(open_heap, (0.0, counter, start, None))

        came_from: dict[GridPoint, GridPoint] = {}
        g_cost: dict[GridPoint, float] = {start: 0.0}
        closed_set: Set[GridPoint] = set()

        while open_heap:
            _, _, current, prev_dir = heapq.heappop(open_heap)

            # Skip if already fully processed
            if current in closed_set:
                continue
            closed_set.add(current)

            # Goal reached!
            if current == goal:
                path = self._reconstruct(came_from, current)
                min_clearance = self.get_path_min_clearance(path)
                print(f"[ExplorationPlanner] SUCCESS: {len(path)} cells, "
                      f"clearance={min_clearance:.1f}, explored={len(closed_set)}")
                return path

            # Expand neighbors
            for neighbor in self.neighbors(current):
                if neighbor in closed_set:
                    continue

                dx = neighbor[0] - current[0]
                dy = neighbor[1] - current[1]
                cur_dir = (dx, dy)

                # Compute step cost
                step_cost = 1.0
                if prev_dir is not None and cur_dir != prev_dir:
                    step_cost += self.turn_penalty
                wall_cost = self.get_wall_cost(neighbor[0], neighbor[1])
                step_cost += self.wall_cost_weight * wall_cost

                # Centering cost - prefer middle of narrow passages
                centering_cost = self.get_centering_cost(neighbor[0], neighbor[1])
                step_cost += self.centering_cost_weight * centering_cost

                tentative_g = g_cost[current] + step_cost

                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f = tentative_g + self.heuristic(neighbor, goal)
                    counter += 1
                    heapq.heappush(open_heap, (f, counter, neighbor, cur_dir))

        # No path found - diagnostics
        print(f"[ExplorationPlanner] FAILED: No path! Explored {len(closed_set)} cells")

        start_neighbors = self.neighbors(start)
        print(f"[ExplorationPlanner] Start neighbors: {len(start_neighbors)}")

        if len(closed_set) < 10:
            print(f"[ExplorationPlanner] WARNING: Very few cells explored!")
            print(f"[ExplorationPlanner] This suggests start is isolated or surrounded by walls")

            # Check what's around start
            gx, gy = start
            for dy in range(-2, 3):
                row = ""
                for dx in range(-2, 3):
                    nx, ny = gx + dx, gy + dy
                    if not self.in_bounds(nx, ny):
                        row += "X"
                    elif self.inflated_walls[ny, nx] == 1:
                        row += "#"
                    elif self.observed_map[ny, nx] == -1:
                        row += "?"
                    else:
                        row += "."
                print(f"[ExplorationPlanner]   {row}")

        return []

    def _find_nearest_passable(self, point: GridPoint, max_search: int = 50) -> Optional[GridPoint]:
        """Find nearest passable cell using BFS."""
        gx, gy = point

        # BFS to find nearest passable
        from collections import deque
        queue = deque([(gx, gy, 0)])
        visited = {(gx, gy)}

        while queue:
            x, y, dist = queue.popleft()

            if dist > max_search:
                break

            if self.is_passable(x, y):
                return (x, y)

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if (nx, ny) not in visited and self.in_bounds(nx, ny):
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))

        return None

    @staticmethod
    def _reconstruct(came_from: dict, cur: GridPoint) -> List[GridPoint]:
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path