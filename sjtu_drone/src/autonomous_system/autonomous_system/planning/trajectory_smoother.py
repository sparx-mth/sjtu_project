#!/usr/bin/env python3
"""
Trajectory Smoother
-------------------
Converts discrete waypoints into a smooth, continuous trajectory
using cubic spline interpolation.

Features:
 - Cubic spline interpolation for C2 continuity (smooth position, velocity, acceleration)
 - Arc-length parameterization for uniform speed sampling
 - Velocity and curvature computation at any point
 - Handles corner cases (too few points, duplicate points)
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np
from scipy.interpolate import CubicSpline


@dataclass
class TrajectoryPoint:
    """A point on the smooth trajectory."""
    x: float
    y: float
    vx: float  # Velocity direction (normalized)
    vy: float
    curvature: float  # Path curvature at this point
    s: float  # Arc length from start


class SmoothTrajectory:
    """
    A smooth trajectory created from waypoints using cubic splines.

    Parameterized by arc length for uniform-speed sampling.
    """

    def __init__(
            self,
            waypoints_x: List[float],
            waypoints_y: List[float],
            min_point_spacing: float = 0.05,
    ):
        """
        Create smooth trajectory from waypoints.

        Args:
            waypoints_x: X coordinates of waypoints
            waypoints_y: Y coordinates of waypoints
            min_point_spacing: Minimum distance between points (removes duplicates)
        """
        # Filter duplicate/too-close points
        xs, ys = self._filter_points(waypoints_x, waypoints_y, min_point_spacing)

        if len(xs) < 2:
            raise ValueError("Need at least 2 distinct waypoints")

        self._xs = np.array(xs)
        self._ys = np.array(ys)

        # Compute cumulative arc length as parameter
        self._s = self._compute_arc_length(self._xs, self._ys)
        self.total_length = self._s[-1]

        # Create cubic splines parameterized by arc length
        if len(xs) >= 4:
            # Enough points for cubic spline
            self._spline_x = CubicSpline(self._s, self._xs, bc_type='natural')
            self._spline_y = CubicSpline(self._s, self._ys, bc_type='natural')
        elif len(xs) >= 2:
            # Fall back to linear for 2-3 points
            self._spline_x = CubicSpline(self._s, self._xs, bc_type='clamped')
            self._spline_y = CubicSpline(self._s, self._ys, bc_type='clamped')

        # Store start and end points
        self.start = (xs[0], ys[0])
        self.end = (xs[-1], ys[-1])

    @staticmethod
    def _filter_points(
            xs: List[float],
            ys: List[float],
            min_spacing: float
    ) -> Tuple[List[float], List[float]]:
        """Remove points that are too close together."""
        if len(xs) == 0:
            return [], []

        filtered_x = [xs[0]]
        filtered_y = [ys[0]]

        for x, y in zip(xs[1:], ys[1:]):
            dist = math.hypot(x - filtered_x[-1], y - filtered_y[-1])
            if dist >= min_spacing:
                filtered_x.append(x)
                filtered_y.append(y)

        # Always include the last point if different from current last
        if len(xs) > 1:
            dist = math.hypot(xs[-1] - filtered_x[-1], ys[-1] - filtered_y[-1])
            if dist >= min_spacing * 0.5:  # Slightly relaxed for endpoint
                filtered_x.append(xs[-1])
                filtered_y.append(ys[-1])

        return filtered_x, filtered_y

    @staticmethod
    def _compute_arc_length(xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Compute cumulative arc length along waypoints."""
        dx = np.diff(xs)
        dy = np.diff(ys)
        ds = np.sqrt(dx ** 2 + dy ** 2)
        s = np.zeros(len(xs))
        s[1:] = np.cumsum(ds)
        return s

    def get_point(self, s: float) -> TrajectoryPoint:
        """
        Get trajectory point at arc length s.

        Args:
            s: Arc length from start (0 to total_length)

        Returns:
            TrajectoryPoint with position, velocity direction, and curvature
        """
        # Clamp to valid range
        s = max(0.0, min(s, self.total_length))

        # Position
        x = float(self._spline_x(s))
        y = float(self._spline_y(s))

        # First derivative (tangent/velocity direction)
        dx = float(self._spline_x(s, 1))
        dy = float(self._spline_y(s, 1))

        # Normalize velocity direction
        speed = math.hypot(dx, dy)
        if speed > 1e-6:
            vx = dx / speed
            vy = dy / speed
        else:
            vx, vy = 1.0, 0.0

        # Second derivative for curvature
        ddx = float(self._spline_x(s, 2))
        ddy = float(self._spline_y(s, 2))

        # Curvature: |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
        if speed > 1e-6:
            curvature = abs(dx * ddy - dy * ddx) / (speed ** 3)
        else:
            curvature = 0.0

        return TrajectoryPoint(x=x, y=y, vx=vx, vy=vy, curvature=curvature, s=s)

    def get_position(self, s: float) -> Tuple[float, float]:
        """Get just the (x, y) position at arc length s."""
        s = max(0.0, min(s, self.total_length))
        return float(self._spline_x(s)), float(self._spline_y(s))

    def find_closest_point(
            self,
            px: float,
            py: float,
            search_start: float = 0.0,
            search_window: float = None,
    ) -> Tuple[float, float]:
        """
        Find the closest point on trajectory to (px, py).

        Args:
            px, py: Query point
            search_start: Start searching from this arc length
            search_window: Only search within this distance (None = full trajectory)

        Returns:
            Tuple of (arc_length, distance) of closest point
        """
        if search_window is None:
            search_window = self.total_length

        search_end = min(search_start + search_window, self.total_length)

        # Coarse search with fixed step
        step = 0.1  # 10cm steps
        best_s = search_start
        best_dist = float('inf')

        s = search_start
        while s <= search_end:
            x, y = self.get_position(s)
            dist = math.hypot(x - px, y - py)
            if dist < best_dist:
                best_dist = dist
                best_s = s
            s += step

        # Fine search around best point
        fine_start = max(search_start, best_s - step)
        fine_end = min(search_end, best_s + step)
        fine_step = 0.02  # 2cm steps

        s = fine_start
        while s <= fine_end:
            x, y = self.get_position(s)
            dist = math.hypot(x - px, y - py)
            if dist < best_dist:
                best_dist = dist
                best_s = s
            s += fine_step

        return best_s, best_dist

    def sample_trajectory(self, spacing: float = 0.1) -> List[TrajectoryPoint]:
        """
        Sample trajectory at regular arc-length intervals.

        Args:
            spacing: Distance between samples (meters)

        Returns:
            List of TrajectoryPoints
        """
        points = []
        s = 0.0
        while s <= self.total_length:
            points.append(self.get_point(s))
            s += spacing

        # Always include endpoint
        if points and points[-1].s < self.total_length - 0.01:
            points.append(self.get_point(self.total_length))

        return points


def smooth_waypoints(
        waypoints_x: List[float],
        waypoints_y: List[float],
) -> Optional[SmoothTrajectory]:
    """
    Convenience function to create a smooth trajectory from waypoints.

    Returns None if trajectory cannot be created.
    """
    try:
        return SmoothTrajectory(waypoints_x, waypoints_y)
    except ValueError:
        return None