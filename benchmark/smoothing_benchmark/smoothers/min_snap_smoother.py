#!/usr/bin/env python3
"""
Minimum-Snap Trajectory Smoother
================================
Full state trajectory generation using minimum-snap optimization.
Optimal for quadrotor drones.

Output includes: t, x, y, z, vx, vy, vz, ax, ay, az

Requires: pip install minsnap-trajectories

Standalone module - no ROS dependencies.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np

try:
    import minsnap_trajectories as ms

    MINSNAP_AVAILABLE = True
except ImportError:
    MINSNAP_AVAILABLE = False
    print("Warning: minsnap-trajectories not installed. Run: pip install minsnap-trajectories")


@dataclass
class TrajectoryPoint:
    """Full state at a point in trajectory."""
    t: float  # Time (seconds from start)
    x: float  # Position X (m)
    y: float  # Position Y (m)
    z: float  # Position Z (m)
    vx: float  # Velocity X (m/s)
    vy: float  # Velocity Y (m/s)
    vz: float  # Velocity Z (m/s)
    ax: float  # Acceleration X (m/s²)
    ay: float  # Acceleration Y (m/s²)
    az: float  # Acceleration Z (m/s²)
    s: float  # Arc length from start (m)
    curvature: float  # Path curvature (1/m)


@dataclass
class DroneConstraints:
    """Physical limits for the drone."""
    max_velocity: float = 0.5  # m/s
    max_acceleration: float = 1.0  # m/s²
    max_jerk: float = 5.0  # m/s³


class MinSnapSmoother:
    """
    Minimum-snap trajectory from waypoints.
    Provides full state (position, velocity, acceleration) at any point.
    """

    def __init__(
            self,
            waypoints_x: List[float],
            waypoints_y: List[float],
            waypoints_z: List[float] = None,
            constraints: DroneConstraints = None,
            min_point_spacing: float = 0.05,
    ):
        if not MINSNAP_AVAILABLE:
            raise ImportError("minsnap-trajectories is not installed")

        self.constraints = constraints or DroneConstraints()

        # Handle 2D case (default z=0)
        if waypoints_z is None:
            waypoints_z = [0.0] * len(waypoints_x)

        # Filter duplicate/too-close points
        xs, ys, zs = self._filter_points(waypoints_x, waypoints_y, waypoints_z, min_point_spacing)

        if len(xs) < 2:
            raise ValueError("Need at least 2 distinct waypoints")

        self._xs, self._ys, self._zs = xs, ys, zs
        self._trajectory = self._generate_minsnap(xs, ys, zs)

        if self._trajectory is None:
            raise ValueError("Failed to generate trajectory")

        # Set total_time FIRST (needed by _compute_arc_length)
        self.total_time = self._trajectory.time_reference[-1]
        self.total_length = self._compute_arc_length()

        self.start = (xs[0], ys[0], zs[0])
        self.end = (xs[-1], ys[-1], zs[-1])

    @staticmethod
    def _filter_points(
            xs: List[float],
            ys: List[float],
            zs: List[float],
            min_spacing: float
    ) -> Tuple[List[float], List[float], List[float]]:
        """Remove points that are too close together."""
        if not xs:
            return [], [], []

        fx, fy, fz = [xs[0]], [ys[0]], [zs[0]]

        for x, y, z in zip(xs[1:], ys[1:], zs[1:]):
            dist = math.sqrt((x - fx[-1]) ** 2 + (y - fy[-1]) ** 2 + (z - fz[-1]) ** 2)
            if dist >= min_spacing:
                fx.append(x)
                fy.append(y)
                fz.append(z)

        # Always include the last point
        if len(xs) > 1:
            dist = math.sqrt((xs[-1] - fx[-1]) ** 2 + (ys[-1] - fy[-1]) ** 2 + (zs[-1] - fz[-1]) ** 2)
            if dist >= min_spacing * 0.5:
                fx.append(xs[-1])
                fy.append(ys[-1])
                fz.append(zs[-1])

        return fx, fy, fz

    def _allocate_times(
            self,
            xs: List[float],
            ys: List[float],
            zs: List[float]
    ) -> List[float]:
        """Allocate time for each segment based on distance and constraints."""
        times = [0.0]

        # Conservative velocity for time allocation
        v_eff = self.constraints.max_velocity * 0.6
        a_max = self.constraints.max_acceleration

        for i in range(1, len(xs)):
            dist = math.sqrt(
                (xs[i] - xs[i - 1]) ** 2 +
                (ys[i] - ys[i - 1]) ** 2 +
                (zs[i] - zs[i - 1]) ** 2
            )

            if dist < 0.01:
                times.append(times[-1] + 0.2)
                continue

            # Trapezoidal velocity profile estimation
            t_accel = v_eff / a_max
            d_accel = 0.5 * a_max * t_accel ** 2

            if dist < 2 * d_accel:
                # Short segment: triangular profile
                seg_time = 2 * math.sqrt(dist / a_max)
            else:
                # Long segment: trapezoidal profile
                seg_time = 2 * t_accel + (dist - 2 * d_accel) / v_eff

            seg_time = max(seg_time * 1.8, 1.2)
            times.append(times[-1] + seg_time)

        return times

    def _generate_minsnap(
            self,
            xs: List[float],
            ys: List[float],
            zs: List[float]
    ) -> Optional["ms.PiecewisePolynomialTrajectory"]:
        """Generate minimum-snap trajectory."""
        times = self._allocate_times(xs, ys, zs)

        refs = []
        for i in range(len(xs)):
            pos = np.array([xs[i], ys[i], zs[i]])

            if i == 0 or i == len(xs) - 1:
                # Start/end: constrain position and velocity (zero velocity)
                refs.append(ms.Waypoint(
                    time=times[i],
                    position=pos,
                    velocity=np.zeros(3)
                ))
            else:
                # Intermediate: only constrain position
                refs.append(ms.Waypoint(
                    time=times[i],
                    position=pos
                ))

        try:
            return ms.generate_trajectory(
                refs,
                degree=7,  # 7th degree polynomial (standard for min-snap)
                idx_minimized_orders=(4,),  # Minimize snap (4th derivative)
                num_continuous_orders=4,  # C4 continuity
                algorithm="closed-form"  # Fast, stable
            )
        except Exception as e:
            print(f"MinSnap generation failed: {e}")
            return None

    def _compute_arc_length(self) -> float:
        """Compute total arc length by sampling trajectory."""
        if self._trajectory is None:
            return 0.0

        t_samples = np.linspace(0, self.total_time, 200)
        pos = ms.compute_trajectory_derivatives(self._trajectory, t_samples, order=1)[0]

        length = 0.0
        for i in range(1, len(t_samples)):
            dx = pos[i, 0] - pos[i - 1, 0]
            dy = pos[i, 1] - pos[i - 1, 1]
            dz = pos[i, 2] - pos[i - 1, 2]
            length += math.sqrt(dx * dx + dy * dy + dz * dz)

        return length

    def get_state_at_time(self, t: float) -> TrajectoryPoint:
        """
        Get full state at time t.

        Args:
            t: Time in seconds from trajectory start

        Returns:
            TrajectoryPoint with full state
        """
        # Clamp to valid range
        t = max(0.0, min(t, self.total_time))

        # Get position, velocity, acceleration
        pva = ms.compute_trajectory_derivatives(self._trajectory, np.array([t]), order=3)

        x, y, z = float(pva[0, 0, 0]), float(pva[0, 0, 1]), float(pva[0, 0, 2])
        vx, vy, vz = float(pva[1, 0, 0]), float(pva[1, 0, 1]), float(pva[1, 0, 2])
        ax, ay, az = float(pva[2, 0, 0]), float(pva[2, 0, 1]), float(pva[2, 0, 2])

        # Compute curvature: |v × a| / |v|³
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > 1e-6:
            # Cross product magnitude for 3D
            cross_x = vy * az - vz * ay
            cross_y = vz * ax - vx * az
            cross_z = vx * ay - vy * ax
            cross_mag = math.sqrt(cross_x ** 2 + cross_y ** 2 + cross_z ** 2)
            curvature = cross_mag / (speed ** 3)
        else:
            curvature = 0.0

        # Estimate arc length at this time (approximate)
        s = (t / self.total_time) * self.total_length if self.total_time > 0 else 0.0

        return TrajectoryPoint(
            t=t, x=x, y=y, z=z,
            vx=vx, vy=vy, vz=vz,
            ax=ax, ay=ay, az=az,
            s=s, curvature=curvature
        )

    def get_point(self, s: float) -> TrajectoryPoint:
        """
        Get state at arc length s (for compatibility with other smoothers).

        Args:
            s: Arc length from start (meters)
        """
        # Convert arc length to time (approximate linear mapping)
        t = (s / self.total_length) * self.total_time if self.total_length > 0 else 0.0
        return self.get_state_at_time(t)

    def get_position(self, s: float) -> Tuple[float, float]:
        """Get (x, y) position at arc length s."""
        pt = self.get_point(s)
        return pt.x, pt.y

    def sample_by_time(self, dt: float = 0.05) -> List[TrajectoryPoint]:
        """
        Sample trajectory at regular TIME intervals.

        Args:
            dt: Time step (seconds), default 50ms = 20Hz

        Returns:
            List of TrajectoryPoints
        """
        points = []
        t = 0.0

        while t <= self.total_time:
            points.append(self.get_state_at_time(t))
            t += dt

        # Always include endpoint
        if points and points[-1].t < self.total_time - 0.001:
            points.append(self.get_state_at_time(self.total_time))

        return points

    def sample_trajectory(self, spacing: float = 0.1) -> List[TrajectoryPoint]:
        """
        Sample trajectory at regular ARC LENGTH intervals.
        (For compatibility with other smoothers)

        Args:
            spacing: Distance between samples (meters)
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

    def validate_constraints(self) -> Tuple[bool, str]:
        """
        Check if trajectory satisfies dynamic constraints.

        Returns:
            (is_valid, message)
        """
        points = self.sample_by_time(dt=0.01)

        max_vel = 0.0
        max_acc = 0.0

        for pt in points:
            vel = math.sqrt(pt.vx ** 2 + pt.vy ** 2 + pt.vz ** 2)
            acc = math.sqrt(pt.ax ** 2 + pt.ay ** 2 + pt.az ** 2)
            max_vel = max(max_vel, vel)
            max_acc = max(max_acc, acc)

        is_valid = True
        msgs = []

        if max_vel > self.constraints.max_velocity * 1.1:
            is_valid = False
            msgs.append(f"vel {max_vel:.2f} > {self.constraints.max_velocity}")

        if max_acc > self.constraints.max_acceleration * 1.2:
            is_valid = False
            msgs.append(f"acc {max_acc:.2f} > {self.constraints.max_acceleration}")

        if is_valid:
            return True, f"OK (v_max={max_vel:.2f}, a_max={max_acc:.2f})"
        else:
            return False, ", ".join(msgs)


def smooth_waypoints(
        waypoints_x: List[float],
        waypoints_y: List[float],
        waypoints_z: List[float] = None,
        constraints: DroneConstraints = None,
) -> Optional[MinSnapSmoother]:
    """
    Convenience function to create a smooth trajectory from waypoints.

    Args:
        waypoints_x, waypoints_y: 2D path coordinates
        waypoints_z: Optional Z coordinates (default: 0)
        constraints: Optional drone constraints

    Returns:
        MinSnapSmoother or None if generation fails
    """
    if not MINSNAP_AVAILABLE:
        print("Error: minsnap-trajectories not installed")
        return None

    try:
        return MinSnapSmoother(waypoints_x, waypoints_y, waypoints_z, constraints)
    except ValueError as e:
        print(f"Trajectory generation failed: {e}")
        return None


if __name__ == "__main__":
    if not MINSNAP_AVAILABLE:
        print("Install minsnap-trajectories to run this test:")
        print("  pip install minsnap-trajectories")
    else:
        # Quick test
        wx = [0, 2, 4, 4, 4]
        wy = [0, 0, 0, 2, 4]

        traj = smooth_waypoints(wx, wy)
        if traj:
            print(f"Minimum-Snap Trajectory")
            print(f"Length: {traj.total_length:.2f}m, Time: {traj.total_time:.2f}s")
            print(f"Start: {traj.start}, End: {traj.end}")

            valid, msg = traj.validate_constraints()
            print(f"Constraints: {msg}")

            for pt in traj.sample_by_time(dt=1.0):
                print(
                    f"  t={pt.t:.1f}s: ({pt.x:.2f}, {pt.y:.2f}) v=({pt.vx:.2f}, {pt.vy:.2f}) a=({pt.ax:.2f}, {pt.ay:.2f})")