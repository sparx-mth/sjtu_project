#!/usr/bin/env python3
"""
Trajectory Smoother
-------------------
Converts discrete waypoints into a smooth, continuous trajectory.

Supports two modes:
 - Dubins mode: Waypoints already come from Dubins curves (pre-smoothed)
 - Spline mode: Uses cubic spline interpolation for C2 continuity

For Dubins paths, cubic splines are NOT needed since:
 - Dubins paths are already smooth (C1 continuous)
 - Curvature is bounded by the turning radius
 - Headings are provided directly from the planner

This simplified version focuses on:
 - Arc-length parameterization for uniform speed sampling
 - Closest point queries for path tracking
 - Optional spline smoothing when needed
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum
import numpy as np

# Optional scipy import for spline mode
try:
    from scipy.interpolate import CubicSpline
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class SmoothingMode(Enum):
    DUBINS = "dubins"    # Pre-smoothed, no interpolation needed
    SPLINE = "spline"    # Use cubic spline interpolation


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
    A smooth trajectory created from waypoints.

    For Dubins paths: Uses linear interpolation (path is already smooth)
    For regular waypoints: Uses cubic splines (if scipy available)
    """

    def __init__(
            self,
            waypoints_x: List[float],
            waypoints_y: List[float],
            headings: Optional[List[float]] = None,
            min_point_spacing: float = 0.05,
            mode: SmoothingMode = None,
    ):
        """
        Create smooth trajectory from waypoints.

        Args:
            waypoints_x: X coordinates of waypoints
            waypoints_y: Y coordinates of waypoints
            headings: Optional yaw angles (radians). If provided, uses Dubins mode.
            min_point_spacing: Minimum distance between points (removes duplicates)
            mode: Force a specific mode (auto-detected if None)
        """
        # Auto-detect mode
        if mode is None:
            mode = SmoothingMode.DUBINS if headings is not None else SmoothingMode.SPLINE

        self._mode = mode

        # Filter duplicate/too-close points
        if headings is not None:
            xs, ys, hs = self._filter_points_with_headings(
                waypoints_x, waypoints_y, headings, min_point_spacing)
            self._headings = np.array(hs)
        else:
            xs, ys = self._filter_points(waypoints_x, waypoints_y, min_point_spacing)
            self._headings = None

        if len(xs) < 2:
            raise ValueError("Need at least 2 distinct waypoints")

        self._xs = np.array(xs)
        self._ys = np.array(ys)

        # Compute cumulative arc length as parameter
        self._s = self._compute_arc_length(self._xs, self._ys)
        self.total_length = self._s[-1]

        # Create splines only if needed and available
        self._spline_x = None
        self._spline_y = None

        if mode == SmoothingMode.SPLINE and HAS_SCIPY and len(xs) >= 2:
            bc_type = 'natural' if len(xs) >= 4 else 'clamped'
            self._spline_x = CubicSpline(self._s, self._xs, bc_type=bc_type)
            self._spline_y = CubicSpline(self._s, self._ys, bc_type=bc_type)

        # Store start and end points
        self.start = (xs[0], ys[0])
        self.end = (xs[-1], ys[-1])

    @property
    def mode(self) -> SmoothingMode:
        return self._mode

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
            if dist >= min_spacing * 0.5:
                filtered_x.append(xs[-1])
                filtered_y.append(ys[-1])

        return filtered_x, filtered_y

    @staticmethod
    def _filter_points_with_headings(
            xs: List[float],
            ys: List[float],
            headings: List[float],
            min_spacing: float
    ) -> Tuple[List[float], List[float], List[float]]:
        """Remove points that are too close together, preserving headings."""
        if len(xs) == 0:
            return [], [], []

        filtered_x = [xs[0]]
        filtered_y = [ys[0]]
        filtered_h = [headings[0]]

        for x, y, h in zip(xs[1:], ys[1:], headings[1:]):
            dist = math.hypot(x - filtered_x[-1], y - filtered_y[-1])
            if dist >= min_spacing:
                filtered_x.append(x)
                filtered_y.append(y)
                filtered_h.append(h)

        # Always include the last point
        if len(xs) > 1:
            dist = math.hypot(xs[-1] - filtered_x[-1], ys[-1] - filtered_y[-1])
            if dist >= min_spacing * 0.5:
                filtered_x.append(xs[-1])
                filtered_y.append(ys[-1])
                filtered_h.append(headings[-1])

        return filtered_x, filtered_y, filtered_h

    @staticmethod
    def _compute_arc_length(xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Compute cumulative arc length along waypoints."""
        dx = np.diff(xs)
        dy = np.diff(ys)
        ds = np.sqrt(dx ** 2 + dy ** 2)
        s = np.zeros(len(xs))
        s[1:] = np.cumsum(ds)
        return s

    def _find_segment(self, s: float) -> Tuple[int, float]:
        """Find which segment contains arc length s, and the local parameter t."""
        s = max(0.0, min(s, self.total_length))

        # Binary search for segment
        idx = np.searchsorted(self._s, s, side='right') - 1
        idx = max(0, min(idx, len(self._s) - 2))

        # Local parameter within segment
        seg_start = self._s[idx]
        seg_end = self._s[idx + 1]
        seg_len = seg_end - seg_start

        if seg_len > 1e-6:
            t = (s - seg_start) / seg_len
        else:
            t = 0.0

        return idx, t

    def get_point(self, s: float) -> TrajectoryPoint:
        """
        Get trajectory point at arc length s.

        Args:
            s: Arc length from start (0 to total_length)

        Returns:
            TrajectoryPoint with position, velocity direction, and curvature
        """
        s = max(0.0, min(s, self.total_length))

        # Use splines if available (spline mode)
        if self._spline_x is not None:
            x = float(self._spline_x(s))
            y = float(self._spline_y(s))

            # First derivative (tangent)
            dx = float(self._spline_x(s, 1))
            dy = float(self._spline_y(s, 1))

            speed = math.hypot(dx, dy)
            if speed > 1e-6:
                vx, vy = dx / speed, dy / speed
            else:
                vx, vy = 1.0, 0.0

            # Second derivative for curvature
            ddx = float(self._spline_x(s, 2))
            ddy = float(self._spline_y(s, 2))

            if speed > 1e-6:
                curvature = abs(dx * ddy - dy * ddx) / (speed ** 3)
            else:
                curvature = 0.0

            return TrajectoryPoint(x=x, y=y, vx=vx, vy=vy, curvature=curvature, s=s)

        # Linear interpolation for Dubins mode (path is already smooth)
        idx, t = self._find_segment(s)

        x = self._xs[idx] + t * (self._xs[idx + 1] - self._xs[idx])
        y = self._ys[idx] + t * (self._ys[idx + 1] - self._ys[idx])

        # Use provided headings if available
        if self._headings is not None:
            # Interpolate heading (handle angle wraparound)
            h0, h1 = self._headings[idx], self._headings[idx + 1]
            dh = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
            heading = h0 + t * dh

            vx = math.cos(heading)
            vy = math.sin(heading)

            # Estimate curvature from heading change
            seg_len = self._s[idx + 1] - self._s[idx]
            if seg_len > 1e-6:
                curvature = abs(dh) / seg_len
            else:
                curvature = 0.0
        else:
            # Compute from position difference
            dx = self._xs[idx + 1] - self._xs[idx]
            dy = self._ys[idx + 1] - self._ys[idx]
            mag = math.hypot(dx, dy)

            if mag > 1e-6:
                vx, vy = dx / mag, dy / mag
            else:
                vx, vy = 1.0, 0.0

            curvature = 0.0  # Unknown without second derivative

        return TrajectoryPoint(x=x, y=y, vx=vx, vy=vy, curvature=curvature, s=s)

    def get_position(self, s: float) -> Tuple[float, float]:
        """Get just the (x, y) position at arc length s."""
        s = max(0.0, min(s, self.total_length))

        if self._spline_x is not None:
            return float(self._spline_x(s)), float(self._spline_y(s))

        idx, t = self._find_segment(s)
        x = self._xs[idx] + t * (self._xs[idx + 1] - self._xs[idx])
        y = self._ys[idx] + t * (self._ys[idx + 1] - self._ys[idx])
        return x, y

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
        headings: Optional[List[float]] = None,
) -> Optional[SmoothTrajectory]:
    """
    Convenience function to create a smooth trajectory from waypoints.

    Args:
        waypoints_x: X coordinates
        waypoints_y: Y coordinates
        headings: Optional yaw angles (for Dubins paths)

    Returns:
        SmoothTrajectory or None if trajectory cannot be created.
    """
    try:
        return SmoothTrajectory(waypoints_x, waypoints_y, headings=headings)
    except ValueError:
        return None


# =============================================================================
# Dubins-specific utilities
# =============================================================================

def compute_dubins_curvature(turning_radius: float) -> float:
    """Get curvature for a Dubins arc segment."""
    return 1.0 / turning_radius if turning_radius > 0 else 0.0


def is_dubins_path(headings: Optional[List[float]]) -> bool:
    """Check if this appears to be a Dubins path (has headings)."""
    return headings is not None and len(headings) > 0