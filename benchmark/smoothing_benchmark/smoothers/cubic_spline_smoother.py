#!/usr/bin/env python3
"""
Cubic Spline Trajectory Smoother
---------------------------------
Converts discrete waypoints into a smooth, continuous trajectory
using cubic spline interpolation.

Features:
 - Cubic spline interpolation for C2 continuity
 - Arc-length parameterization for uniform speed sampling
 - Velocity and curvature computation at any point

Standalone module - no ROS dependencies.
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


class CubicSplineSmoother:
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
            # Fall back to clamped for 2-3 points
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
        min_point_spacing: float = 0.05,
) -> Optional[CubicSplineSmoother]:
    """
    Convenience function to create a smooth trajectory from waypoints.

    Returns None if trajectory cannot be created.
    """
    try:
        return CubicSplineSmoother(waypoints_x, waypoints_y, min_point_spacing)
    except ValueError:
        return None


if __name__ == "__main__":
    # Quick test
    wx = [0, 1, 2, 3, 4, 5]
    wy = [0, 0.5, 0, 0.5, 0, 0.5]

    traj = smooth_waypoints(wx, wy)
    if traj:
        print(f"Cubic Spline Trajectory")
        print(f"Total length: {traj.total_length:.2f}m")
        print(f"Start: {traj.start}, End: {traj.end}")

        for pt in traj.sample_trajectory(spacing=1.0):
            heading = math.degrees(math.atan2(pt.vy, pt.vx))
            print(f"  s={pt.s:.1f}: ({pt.x:.2f}, {pt.y:.2f}) heading={heading:.0f}° curvature={pt.curvature:.3f}")