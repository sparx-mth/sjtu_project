#!/usr/bin/env python3
"""
RRT* Planner using OMPL
-----------------------
Minimal wrapper around OMPL's RRTstar for 2D occupancy grid navigation.
Uses the same map format as astar_planner.py for compatibility.
"""

import os
import cv2
import yaml
import numpy as np
from typing import List, Tuple, Optional

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    raise ImportError("Install OMPL: pip install ompl-thin --break-system-packages")

GridPoint = Tuple[int, int]


class RRTStarPlanner:
    """Minimal RRT* planner for 2D occupancy grids using OMPL."""

    def __init__(self, map_yaml_path: str, safety_margin: int = 10):
        # Load map (same as A* planner)
        with open(map_yaml_path, "r") as f:
            info = yaml.safe_load(f)

        self.resolution = float(info["resolution"])
        self.origin = tuple(info["origin"])

        img_path = info["image"]
        if not img_path.startswith("/"):
            img_path = os.path.join(os.path.dirname(map_yaml_path), img_path)

        img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        assert img is not None, f"Failed to load: {img_path}"

        # Binary map: 0=free, 1=obstacle (gray and black are obstacles)
        binary = np.zeros_like(img, dtype=np.uint8)
        binary[img < 250] = 1  # Only white (255) is free
        self.raw_map = np.flipud(binary)

        # Inflate obstacles
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * safety_margin + 1, 2 * safety_margin + 1)
        )
        self.map_data = cv2.dilate(self.raw_map, kernel)
        self.height, self.width = self.map_data.shape

        print(f"[RRTStarPlanner] Map loaded: {self.width}x{self.height}")

    def world_to_map(self, wx: float, wy: float) -> GridPoint:
        ox, oy, _ = self.origin
        return int(round((wx - ox) / self.resolution)), int(round((wy - oy) / self.resolution))

    def map_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        ox, oy, _ = self.origin
        return gx * self.resolution + ox, gy * self.resolution + oy

    def is_free(self, gx: int, gy: int) -> bool:
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.map_data[gy, gx] == 0
        return False

    def _is_state_valid(self, state):
        """OMPL validity checker callback."""
        gx, gy = int(state[0]), int(state[1])
        return self.is_free(gx, gy)

    def plan(self, start: GridPoint, goal: GridPoint, timeout: float = 2.0) -> List[GridPoint]:
        """Plan path using RRT*. Returns list of grid points."""
        if not self.is_free(*start) or not self.is_free(*goal):
            print("[RRTStarPlanner] Start or goal in obstacle!")
            return []

        # Setup OMPL state space (2D real vector space)
        space = ob.RealVectorStateSpace(2)
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, 0)
        bounds.setHigh(0, self.width - 1)
        bounds.setLow(1, 0)
        bounds.setHigh(1, self.height - 1)
        space.setBounds(bounds)

        # Setup space information with validity checker
        si = ob.SpaceInformation(space)
        si.setStateValidityChecker(ob.StateValidityCheckerFn(self._is_state_valid))
        si.setup()

        # Create start and goal states
        start_state = ob.State(space)
        start_state[0], start_state[1] = float(start[0]), float(start[1])

        goal_state = ob.State(space)
        goal_state[0], goal_state[1] = float(goal[0]), float(goal[1])

        # Setup problem definition
        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start_state, goal_state)

        # Use RRT* planner
        planner = og.RRTstar(si)
        planner.setProblemDefinition(pdef)
        planner.setup()

        # Solve
        solved = planner.solve(timeout)

        if solved:
            path = pdef.getSolutionPath()
            path.interpolate()  # Add intermediate points

            # Convert to grid points
            waypoints = []
            for i in range(path.getStateCount()):
                s = path.getState(i)
                waypoints.append((int(round(s[0])), int(round(s[1]))))

            print(f"[RRTStarPlanner] Found path with {len(waypoints)} waypoints")
            return waypoints

        print("[RRTStarPlanner] No path found")
        return []

    def plan_world(self, start_world: Tuple[float, float], goal_world: Tuple[float, float],
                   timeout: float = 2.0) -> List[Tuple[float, float]]:
        """Plan in world coordinates. Returns waypoints in world coords."""
        start_grid = self.world_to_map(*start_world)
        goal_grid = self.world_to_map(*goal_world)

        path_grid = self.plan(start_grid, goal_grid, timeout)

        return [self.map_to_world(gx, gy) for gx, gy in path_grid]