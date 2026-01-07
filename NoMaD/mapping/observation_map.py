#!/usr/bin/env python3
"""
observation_map.py
------------------
A reusable Observation/Exploration Map module for drone navigation.

This module provides:
    1. ObservationMap - Core class for tracking explored areas
    2. ExplorationPlanner - Helper for exploration-based path planning

The map starts completely unknown (gray). As the drone moves,
areas within a sensor radius are revealed, showing the actual
occupancy data from the ground truth map.

Usage:
    from autonomous_system.mapping.observation_map import ObservationMap

    obs_map = ObservationMap(map_yaml_path, exploration_radius=30)
    obs_map.update_observation(drone_x, drone_y)  # Call each time drone moves

    # Check if a cell is explored
    if obs_map.is_explored(gx, gy):
        occupancy = obs_map.get_observed_value(gx, gy)
"""

import numpy as np
import yaml
import cv2
import os
from typing import Tuple, List, Optional
from dataclasses import dataclass
from enum import IntEnum


class CellState(IntEnum):
    """State of a cell in the observation map."""
    UNKNOWN = -1  # Not yet observed (gray)
    FREE = 0  # Observed as free (white)
    OCCUPIED = 1  # Observed as occupied (black)


@dataclass
class MapInfo:
    """Map metadata."""
    resolution: float
    origin: Tuple[float, float, float]
    width: int
    height: int


class ObservationMap:
    """
    Manages a fog-of-war style observation map for robot exploration.

    The map has three layers:
        1. Ground truth (loaded from occupancy grid YAML)
        2. Exploration mask (binary: explored or not)
        3. Observed map (revealed portions of ground truth)

    Attributes:
        ground_truth: The actual occupancy grid (0=free, 1=occupied)
        exploration_mask: Binary mask (0=unexplored, 1=explored)
        observed_map: What the robot "sees" (-1=unknown, 0=free, 1=occupied)
        exploration_radius: Sensor range in pixels
    """

    def __init__(
            self,
            map_yaml_path: str,
            exploration_radius: int = 30,
            exploration_radius_meters: Optional[float] = None
    ):
        """
        Initialize the observation map.

        Args:
            map_yaml_path: Path to the map YAML file
            exploration_radius: Sensor range in pixels
            exploration_radius_meters: If provided, overrides pixel radius
        """
        # Load map configuration
        with open(map_yaml_path, 'r') as f:
            info = yaml.safe_load(f)

        self.resolution = info['resolution']
        self.origin = tuple(info['origin'])

        # Load map image
        map_image_path = info['image']
        if not map_image_path.startswith('/'):
            map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_path)

        img = cv2.imread(map_image_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(f"Failed to load map image: {map_image_path}")

        # Create ground truth occupancy grid
        # 0 = free, 1 = occupied
        self.ground_truth = np.zeros_like(img, dtype=np.int8)
        self.ground_truth[img < 50] = CellState.OCCUPIED
        self.ground_truth = np.flipud(self.ground_truth)

        self.height, self.width = self.ground_truth.shape

        # Store map info
        self.map_info = MapInfo(
            resolution=self.resolution,
            origin=self.origin,
            width=self.width,
            height=self.height
        )

        # Set exploration radius
        if exploration_radius_meters is not None and exploration_radius_meters > 0:
            self.exploration_radius = int(exploration_radius_meters / self.resolution)
        else:
            self.exploration_radius = exploration_radius

        # Initialize exploration state
        self.exploration_mask = np.zeros((self.height, self.width), dtype=np.uint8)
        self.observed_map = np.full((self.height, self.width), CellState.UNKNOWN, dtype=np.int8)

        # Precompute circle mask for efficiency
        self._precompute_circle_mask()

        # Statistics
        self.total_cells = self.height * self.width
        self.total_free_cells = np.sum(self.ground_truth == CellState.FREE)
        self.explored_cells = 0
        self.explored_free_cells = 0

        # Track last update position to avoid redundant updates
        self._last_update_pos: Optional[Tuple[int, int]] = None

    def _precompute_circle_mask(self):
        """Precompute circular mask offsets for the exploration radius."""
        r = self.exploration_radius
        y, x = np.ogrid[-r:r + 1, -r:r + 1]
        circle_mask = x * x + y * y <= r * r
        self.circle_offsets = np.argwhere(circle_mask) - r

    def set_exploration_radius(self, radius_pixels: int = None, radius_meters: float = None):
        """
        Update the exploration radius.

        Args:
            radius_pixels: New radius in pixels
            radius_meters: New radius in meters (takes precedence)
        """
        if radius_meters is not None:
            self.exploration_radius = int(radius_meters / self.resolution)
        elif radius_pixels is not None:
            self.exploration_radius = radius_pixels
        self._precompute_circle_mask()

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        ox, oy, _ = self.origin
        gx = int(round((wx - ox) / self.resolution))
        gy = int(round((wy - oy) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        ox, oy, _ = self.origin
        wx = gx * self.resolution + ox
        wy = gy * self.resolution + oy
        return wx, wy

    def is_in_bounds(self, gx: int, gy: int) -> bool:
        """Check if grid coordinates are within map bounds."""
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_explored(self, gx: int, gy: int) -> bool:
        """Check if a cell has been explored."""
        if not self.is_in_bounds(gx, gy):
            return False
        return self.exploration_mask[gy, gx] == 1

    def get_observed_value(self, gx: int, gy: int) -> int:
        """
        Get the observed value of a cell.

        Returns:
            CellState.UNKNOWN (-1) if unexplored
            CellState.FREE (0) if observed as free
            CellState.OCCUPIED (1) if observed as occupied
        """
        if not self.is_in_bounds(gx, gy):
            return CellState.UNKNOWN
        return self.observed_map[gy, gx]

    def get_ground_truth(self, gx: int, gy: int) -> int:
        """Get the ground truth value (cheating - for debugging only)."""
        if not self.is_in_bounds(gx, gy):
            return CellState.OCCUPIED
        return self.ground_truth[gy, gx]

    def update_observation(self, wx: float = None, wy: float = None,
                           gx: int = None, gy: int = None,
                           force: bool = False) -> int:
        """
        Update the observation map by revealing area around the given position.

        Args:
            wx, wy: World coordinates (meters)
            gx, gy: Grid coordinates (pixels) - used if world coords not provided
            force: Force update even if position hasn't changed

        Returns:
            Number of newly revealed cells
        """
        # Convert world to grid if needed
        if wx is not None and wy is not None:
            gx, gy = self.world_to_grid(wx, wy)

        if gx is None or gy is None:
            raise ValueError("Must provide either world coords (wx, wy) or grid coords (gx, gy)")

        # Skip if position hasn't changed (optimization)
        if not force and self._last_update_pos == (gx, gy):
            return 0
        self._last_update_pos = (gx, gy)

        # Reveal cells within radius
        new_cells = 0
        new_free = 0

        for dy, dx in self.circle_offsets:
            px = gx + dx
            py = gy + dy

            if self.is_in_bounds(px, py) and self.exploration_mask[py, px] == 0:
                # Mark as explored
                self.exploration_mask[py, px] = 1
                # Reveal ground truth
                gt_value = self.ground_truth[py, px]
                self.observed_map[py, px] = gt_value
                new_cells += 1
                if gt_value == CellState.FREE:
                    new_free += 1

        self.explored_cells += new_cells
        self.explored_free_cells += new_free

        return new_cells

    def reset(self):
        """Reset exploration - make entire map unknown again."""
        self.exploration_mask.fill(0)
        self.observed_map.fill(CellState.UNKNOWN)
        self.explored_cells = 0
        self.explored_free_cells = 0
        self._last_update_pos = None

    @property
    def exploration_percentage(self) -> float:
        """Percentage of total map explored."""
        return (self.explored_cells / self.total_cells) * 100

    @property
    def free_space_exploration_percentage(self) -> float:
        """Percentage of free space explored (more meaningful metric)."""
        if self.total_free_cells == 0:
            return 0.0
        return (self.explored_free_cells / self.total_free_cells) * 100

    def get_frontier_cells(self) -> List[Tuple[int, int]]:
        """
        Find frontier cells - explored free cells adjacent to unknown cells.

        Frontiers are interesting for exploration planning.

        Returns:
            List of (gx, gy) frontier cell coordinates
        """
        frontiers = []

        # 4-connectivity neighbors
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        for gy in range(self.height):
            for gx in range(self.width):
                # Must be explored and free
                if self.observed_map[gy, gx] != CellState.FREE:
                    continue

                # Check if any neighbor is unknown
                for dx, dy in neighbors:
                    nx, ny = gx + dx, gy + dy
                    if self.is_in_bounds(nx, ny) and self.observed_map[ny, nx] == CellState.UNKNOWN:
                        frontiers.append((gx, gy))
                        break

        return frontiers

    def get_nearest_frontier(self, gx: int, gy: int) -> Optional[Tuple[int, int]]:
        """
        Find the nearest frontier cell to a given position.

        Args:
            gx, gy: Current grid position

        Returns:
            (gx, gy) of nearest frontier, or None if no frontiers exist
        """
        frontiers = self.get_frontier_cells()
        if not frontiers:
            return None

        min_dist = float('inf')
        nearest = None

        for fx, fy in frontiers:
            dist = (fx - gx) ** 2 + (fy - gy) ** 2
            if dist < min_dist:
                min_dist = dist
                nearest = (fx, fy)

        return nearest

    def is_path_explored(self, path: List[Tuple[int, int]]) -> bool:
        """Check if an entire path has been explored."""
        return all(self.is_explored(gx, gy) for gx, gy in path)

    def is_path_safe(self, path: List[Tuple[int, int]]) -> bool:
        """Check if a path goes only through explored free space."""
        for gx, gy in path:
            obs_val = self.get_observed_value(gx, gy)
            if obs_val != CellState.FREE:
                return False
        return True

    def get_visualization_array(self) -> np.ndarray:
        """
        Get an array suitable for visualization.

        Returns:
            Array with values: -1 (gray/unknown), 0 (white/free), 1 (black/occupied)
        """
        return self.observed_map.copy()

    def save(self, path_prefix: str):
        """
        Save the current exploration state.

        Args:
            path_prefix: Prefix for output files
        """
        # Save exploration mask
        mask_path = f"{path_prefix}_mask.png"
        cv2.imwrite(mask_path, (self.exploration_mask * 255).astype(np.uint8))

        # Save observed map as image
        obs_path = f"{path_prefix}_observed.png"
        obs_img = np.zeros((self.height, self.width), dtype=np.uint8)
        obs_img[self.observed_map == CellState.UNKNOWN] = 128  # Gray
        obs_img[self.observed_map == CellState.FREE] = 255  # White
        obs_img[self.observed_map == CellState.OCCUPIED] = 0  # Black
        cv2.imwrite(obs_path, np.flipud(obs_img))

        # Save numpy arrays
        np.save(f"{path_prefix}_mask.npy", self.exploration_mask)
        np.save(f"{path_prefix}_observed.npy", self.observed_map)

    def load(self, path_prefix: str):
        """
        Load a previously saved exploration state.

        Args:
            path_prefix: Prefix used when saving
        """
        self.exploration_mask = np.load(f"{path_prefix}_mask.npy")
        self.observed_map = np.load(f"{path_prefix}_observed.npy")
        self.explored_cells = np.sum(self.exploration_mask)
        self.explored_free_cells = np.sum(
            (self.exploration_mask == 1) & (self.ground_truth == CellState.FREE)
        )


class ExplorationAwarePlanner:
    """
    A planner that considers exploration state when planning paths.

    This can be used to:
        1. Plan paths only through explored areas (safe planning)
        2. Plan paths to frontiers for exploration
        3. Evaluate path safety based on observation map
    """

    def __init__(self, observation_map: ObservationMap):
        """
        Initialize with an observation map.

        Args:
            observation_map: The ObservationMap instance to use
        """
        self.obs_map = observation_map

    def get_safe_cost(self, gx: int, gy: int) -> float:
        """
        Get traversal cost for a cell considering exploration state.

        Returns:
            0.0 for explored free cells
            inf for occupied or unknown cells
        """
        obs_val = self.obs_map.get_observed_value(gx, gy)
        if obs_val == CellState.FREE:
            return 1.0
        elif obs_val == CellState.OCCUPIED:
            return float('inf')
        else:  # UNKNOWN
            return float('inf')  # Conservative: don't traverse unknown

    def get_exploration_cost(self, gx: int, gy: int,
                             unknown_cost: float = 2.0) -> float:
        """
        Get traversal cost for exploration (willing to traverse unknown).

        Args:
            gx, gy: Grid coordinates
            unknown_cost: Cost multiplier for unknown cells

        Returns:
            1.0 for free cells
            unknown_cost for unknown cells
            inf for occupied cells
        """
        obs_val = self.obs_map.get_observed_value(gx, gy)
        if obs_val == CellState.FREE:
            return 1.0
        elif obs_val == CellState.OCCUPIED:
            return float('inf')
        else:  # UNKNOWN - allow traversal with higher cost
            # Check ground truth for actual safety (robot has to take the risk)
            gt = self.obs_map.get_ground_truth(gx, gy)
            if gt == CellState.OCCUPIED:
                return float('inf')  # Actually blocked
            return unknown_cost

    def find_exploration_target(self) -> Optional[Tuple[int, int]]:
        """
        Find a good target for exploration.

        Returns the centroid of the largest frontier cluster,
        or the nearest frontier if clustering fails.
        """
        frontiers = self.obs_map.get_frontier_cells()
        if not frontiers:
            return None

        # Simple: return centroid of all frontiers
        if len(frontiers) < 10:
            # Few frontiers - just pick the first one
            return frontiers[0]

        # Calculate centroid
        cx = sum(f[0] for f in frontiers) // len(frontiers)
        cy = sum(f[1] for f in frontiers) // len(frontiers)

        # Find frontier closest to centroid
        min_dist = float('inf')
        best = frontiers[0]
        for fx, fy in frontiers:
            dist = (fx - cx) ** 2 + (fy - cy) ** 2
            if dist < min_dist:
                min_dist = dist
                best = (fx, fy)

        return best


# Example usage and testing
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors

    # Test with a sample map
    map_yaml = "/root/sjtu_project/sjtu_drone/maps/hospital_map_cropped.yaml"

    print("Creating ObservationMap...")
    obs_map = ObservationMap(map_yaml, exploration_radius=30)

    print(f"Map size: {obs_map.width} x {obs_map.height}")
    print(f"Resolution: {obs_map.resolution} m/pixel")
    print(f"Exploration radius: {obs_map.exploration_radius} pixels")

    # Simulate drone moving
    test_positions = [
        (0, 0), (5, 5), (10, 10), (15, 15), (20, 20)
    ]

    # Convert to world coords (roughly center of map)
    wx_base, wy_base = obs_map.grid_to_world(obs_map.width // 2, obs_map.height // 2)

    for i, (dx, dy) in enumerate(test_positions):
        wx = wx_base + dx * obs_map.resolution
        wy = wy_base + dy * obs_map.resolution
        new_cells = obs_map.update_observation(wx=wx, wy=wy)
        print(f"Position {i + 1}: revealed {new_cells} cells, "
              f"total explored: {obs_map.exploration_percentage:.1f}%")

    # Visualize
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # Ground truth
    axes[0].imshow(obs_map.ground_truth, cmap='gray', origin='lower')
    axes[0].set_title('Ground Truth')

    # Exploration mask
    axes[1].imshow(obs_map.exploration_mask, cmap='gray', origin='lower')
    axes[1].set_title('Exploration Mask')

    # Observed map
    cmap = mcolors.ListedColormap(['#808080', 'white', 'black'])
    bounds = [-1.5, -0.5, 0.5, 1.5]
    norm = mcolors.BoundaryNorm(bounds, cmap.N)
    axes[2].imshow(obs_map.observed_map, cmap=cmap, norm=norm, origin='lower')
    axes[2].set_title(f'Observed Map ({obs_map.exploration_percentage:.1f}% explored)')

    plt.tight_layout()
    plt.savefig('/tmp/observation_map_test.png')
    print("\nVisualization saved to /tmp/observation_map_test.png")