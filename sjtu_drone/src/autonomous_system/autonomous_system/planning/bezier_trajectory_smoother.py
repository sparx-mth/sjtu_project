#!/usr/bin/env python3
"""
Bezier Trajectory Smoother
--------------------------
Converts discrete waypoints into a smooth trajectory using cubic Bezier curves
with heading continuity.

Uses scipy.interpolate.CubicHermiteSpline which allows specifying tangent
directions at each waypoint - giving us heading-aware smoothing.

Key Features:
 - G1 continuity: smooth heading transitions between segments
 - Entry/exit angle awareness: drone arrives facing the next waypoint
 - Arc-length parameterization for uniform speed sampling
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np
from scipy.interpolate import CubicHermiteSpline


@dataclass
class TrajectoryPoint:
    """A point on the smooth trajectory."""
    x: float
    y: float
    vx: float  # Velocity direction (normalized)
    vy: float
    curvature: float
    s: float  # Arc length from start


class SmoothTrajectory:
    """
    Smooth trajectory from waypoints using cubic Hermite splines with heading continuity.

    CubicHermiteSpline allows us to specify both position AND tangent (heading)
    at each waypoint, giving us control over the drone's orientation.
    """

    def __init__(
            self,
            waypoints_x: List[float],
            waypoints_y: List[float],
            min_point_spacing: float = 0.05,
            tangent_scale: float = 0.5,
            cruise_speed: float = 0.4,
    ):
        """
        Args:
            waypoints_x, waypoints_y: Waypoint coordinates
            min_point_spacing: Minimum distance between points
            tangent_scale: Scale factor for tangent magnitude (affects curve smoothness)
            cruise_speed: Desired cruise speed (m/s)
        """
        xs, ys = self._filter_points(waypoints_x, waypoints_y, min_point_spacing)
        if len(xs) < 2:
            raise ValueError("Need at least 2 distinct waypoints")

        self._xs = np.array(xs, dtype=float)
        self._ys = np.array(ys, dtype=float)
        self._cruise_speed = cruise_speed
        self._tangent_scale = tangent_scale

        # Compute parameter values (cumulative chord length)
        self._t = self._compute_parameters()

        # Compute tangents at each waypoint for heading continuity
        self._dx, self._dy = self._compute_tangents()

        # Create Hermite splines (position + tangent at each point)
        self._spline_x = CubicHermiteSpline(self._t, self._xs, self._dx)
        self._spline_y = CubicHermiteSpline(self._t, self._ys, self._dy)

        # Compute arc length lookup table
        self._arc_lengths, self._t_samples = self._compute_arc_length_table()
        self.total_length = self._arc_lengths[-1]
        self.total_time = self.total_length / cruise_speed if cruise_speed > 0 else 0.0

        self.start = (xs[0], ys[0], 0.0)
        self.end = (xs[-1], ys[-1], 0.0)

    @staticmethod
    def _filter_points(xs, ys, min_spacing):
        """Remove points too close together."""
        if not xs:
            return [], []
        filtered_x, filtered_y = [xs[0]], [ys[0]]
        for x, y in zip(xs[1:], ys[1:]):
            if math.hypot(x - filtered_x[-1], y - filtered_y[-1]) >= min_spacing:
                filtered_x.append(x)
                filtered_y.append(y)
        if len(xs) > 1 and math.hypot(xs[-1] - filtered_x[-1], ys[-1] - filtered_y[-1]) >= min_spacing * 0.5:
            filtered_x.append(xs[-1])
            filtered_y.append(ys[-1])
        return filtered_x, filtered_y

    def _compute_parameters(self) -> np.ndarray:
        """Compute parameter values using cumulative chord length."""
        diffs = np.sqrt(np.diff(self._xs) ** 2 + np.diff(self._ys) ** 2)
        t = np.zeros(len(self._xs))
        t[1:] = np.cumsum(diffs)
        return t

    def _compute_tangents(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute tangent vectors at each waypoint for heading continuity.

        At each interior point, tangent points from previous waypoint toward next,
        ensuring the drone arrives aligned to continue toward the next waypoint.
        """
        n = len(self._xs)
        dx = np.zeros(n)
        dy = np.zeros(n)

        for i in range(n):
            if i == 0:
                # First point: tangent toward next point
                direction = np.array([self._xs[1] - self._xs[0],
                                      self._ys[1] - self._ys[0]])
            elif i == n - 1:
                # Last point: tangent from previous point
                direction = np.array([self._xs[-1] - self._xs[-2],
                                      self._ys[-1] - self._ys[-2]])
            else:
                # Interior: average of incoming and outgoing directions
                incoming = np.array([self._xs[i] - self._xs[i - 1],
                                     self._ys[i] - self._ys[i - 1]])
                outgoing = np.array([self._xs[i + 1] - self._xs[i],
                                     self._ys[i + 1] - self._ys[i]])

                # Normalize and average
                in_len = np.linalg.norm(incoming)
                out_len = np.linalg.norm(outgoing)
                if in_len > 1e-6:
                    incoming /= in_len
                if out_len > 1e-6:
                    outgoing /= out_len
                direction = incoming + outgoing

            # Scale tangent by distance to neighbors
            norm = np.linalg.norm(direction)
            if norm > 1e-6:
                direction /= norm

            # Scale tangent magnitude based on segment lengths
            if i == 0:
                seg_len = self._t[1] - self._t[0]
            elif i == n - 1:
                seg_len = self._t[-1] - self._t[-2]
            else:
                seg_len = (self._t[i + 1] - self._t[i - 1]) / 2

            dx[i] = direction[0] * seg_len * self._tangent_scale
            dy[i] = direction[1] * seg_len * self._tangent_scale

        return dx, dy

    def _compute_arc_length_table(self, num_samples: int = 500) -> Tuple[np.ndarray, np.ndarray]:
        """Build lookup table for arc-length parameterization."""
        t_samples = np.linspace(self._t[0], self._t[-1], num_samples)

        # Compute cumulative arc length
        x_vals = self._spline_x(t_samples)
        y_vals = self._spline_y(t_samples)

        dx = np.diff(x_vals)
        dy = np.diff(y_vals)
        ds = np.sqrt(dx ** 2 + dy ** 2)

        arc_lengths = np.zeros(num_samples)
        arc_lengths[1:] = np.cumsum(ds)

        return arc_lengths, t_samples

    def _arc_length_to_t(self, s: float) -> float:
        """Convert arc length to spline parameter t."""
        s = np.clip(s, 0.0, self.total_length)
        # Linear interpolation in lookup table
        return np.interp(s, self._arc_lengths, self._t_samples)

    def get_point(self, s: float) -> TrajectoryPoint:
        """Get trajectory point at arc length s."""
        t = self._arc_length_to_t(s)

        # Position
        x = float(self._spline_x(t))
        y = float(self._spline_y(t))

        # First derivative (tangent)
        dx = float(self._spline_x(t, 1))
        dy = float(self._spline_y(t, 1))
        speed = math.hypot(dx, dy)
        vx, vy = (dx / speed, dy / speed) if speed > 1e-6 else (1.0, 0.0)

        # Second derivative for curvature
        ddx = float(self._spline_x(t, 2))
        ddy = float(self._spline_y(t, 2))
        curvature = abs(dx * ddy - dy * ddx) / (speed ** 3) if speed > 1e-6 else 0.0

        return TrajectoryPoint(x=x, y=y, vx=vx, vy=vy, curvature=curvature, s=s)

    def get_position(self, s: float) -> Tuple[float, float]:
        """Get (x, y) position at arc length s."""
        t = self._arc_length_to_t(s)
        return float(self._spline_x(t)), float(self._spline_y(t))

    def get_state_at_time(self, time: float) -> TrajectoryPoint:
        """Get state at time (constant speed). Compatible with time-based tracking."""
        s = (time / self.total_time) * self.total_length if self.total_time > 0 else 0.0
        point = self.get_point(np.clip(s, 0.0, self.total_length))
        point.vx *= self._cruise_speed
        point.vy *= self._cruise_speed
        return point

    def find_closest_point(self, px: float, py: float, search_start: float = 0.0,
                           search_window: float = None) -> Tuple[float, float]:
        """Find closest point on trajectory to (px, py)."""
        search_end = min(search_start + (search_window or self.total_length), self.total_length)
        best_s, best_dist = search_start, float('inf')

        # Coarse search
        for s in np.arange(search_start, search_end, 0.1):
            x, y = self.get_position(s)
            dist = math.hypot(x - px, y - py)
            if dist < best_dist:
                best_dist, best_s = dist, s

        # Fine search
        for s in np.arange(max(search_start, best_s - 0.1), min(search_end, best_s + 0.1), 0.02):
            x, y = self.get_position(s)
            dist = math.hypot(x - px, y - py)
            if dist < best_dist:
                best_dist, best_s = dist, s

        return best_s, best_dist

    def sample_trajectory(self, spacing: float = 0.1) -> List[TrajectoryPoint]:
        """Sample trajectory at regular arc-length intervals."""
        points = [self.get_point(s) for s in np.arange(0, self.total_length, spacing)]
        if points and points[-1].s < self.total_length - 0.01:
            points.append(self.get_point(self.total_length))
        return points

    def get_heading_at(self, s: float) -> float:
        """Get heading angle (radians) at arc length s."""
        pt = self.get_point(s)
        return math.atan2(pt.vy, pt.vx)


def smooth_waypoints(waypoints_x: List[float], waypoints_y: List[float],
                     tangent_scale: float = 0.5, cruise_speed: float = 0.4) -> Optional[SmoothTrajectory]:
    """Create smooth trajectory from waypoints. Returns None on failure."""
    try:
        return SmoothTrajectory(waypoints_x, waypoints_y,
                                tangent_scale=tangent_scale, cruise_speed=cruise_speed)
    except ValueError:
        return None


if __name__ == "__main__":
    # Quick test
    wx, wy = [0, 2, 4, 4, 4], [0, 0, 0, 2, 4]
    traj = smooth_waypoints(wx, wy)
    if traj:
        print(f"Length: {traj.total_length:.2f}m, Time: {traj.total_time:.2f}s")
        for pt in traj.sample_trajectory(spacing=1.0):
            heading = math.degrees(math.atan2(pt.vy, pt.vx))
            print(f"  s={pt.s:.1f}: ({pt.x:.2f}, {pt.y:.2f}) heading={heading:.0f}°")