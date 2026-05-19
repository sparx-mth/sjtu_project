#!/usr/bin/env python3
"""
trajectory_tracker.py — the explicit 24-point tracker.

This is the "where is the tracking?" answer. It is the SAME turn-then-
fly algorithm as waypoint_follower.py (align to the point with pure
yaw, then fly straight to it with pure vx, then the next point), but:

  * it runs in the INFERENCE-LOCAL frame, not the world frame, and
  * it is re-zeroable: every new NavDP inference calls set_trajectory()
    with fresh points AND the controller resets the local origin to the
    current telemetry. So accumulated localization drift is dropped on
    every inference — exactly the behaviour you described ("I take the
    telemetry and this is my new zero").

No ROS here. The controller feeds it the drone's pose *in the local
frame* (telemetry delta since the inference snapshot) and gets back a
(vx, wz) command that already respects the platform invariant
vx==0 OR wz==0.

Frame: local. At the inference that produced `points`, the drone was
(0,0) heading +x. `points[i] = (x_i, y_i)` are NavDP's body-frame
waypoints in that same frame.
"""
import math


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class TrajectoryTracker:
    def __init__(self, pos_radius=0.35, yaw_settle=0.05,
                 vel_x=0.3, yaw_rate=0.7, yaw_kp=1.8,
                 skip_yaw_thresh=0.25, passed_bearing_deg=100.0,
                 yaw_realign_thresh=0.40, logger=None):
        self.pos_radius = float(pos_radius)
        self.yaw_settle = float(yaw_settle)
        self.vel_x = float(vel_x)
        self.yaw_rate = float(yaw_rate)
        self.yaw_kp = float(yaw_kp)
        self.skip_yaw = float(skip_yaw_thresh)
        self.passed_bearing = math.radians(float(passed_bearing_deg))
        self.yaw_realign = float(yaw_realign_thresh)
        self._log = logger or (lambda *a, **k: None)

        self._pts = []
        self._idx = 0
        self._aligned = False        # have we finished yaw-aligning to _idx?

    def _yaw_cmd(self, berr):
        """Proportional yaw, capped at yaw_rate. CRITICAL for the
        re-zeroed design: a small heading error must produce a small,
        brief correction that finishes inside one inference window —
        bang-bang yaw_rate would swing ~yaw_rate*infer_period per
        window, hugely overshoot a few-degree error, and limit-cycle
        forever as the zero resets each inference."""
        mag = min(self.yaw_rate, self.yaw_kp * abs(berr))
        return math.copysign(mag, berr)

    # ── new inference → new points (controller resets zero alongside) ──
    def set_trajectory(self, points):
        """points: iterable of (x, y) in the local frame. Index 0 is
        ~the drone itself (NavDP step 0); we drop it so we always have
        a meaningful bearing for the first real target."""
        pts = [(float(x), float(y)) for x, y in points]
        # Drop leading points that sit basically on the origin — NavDP's
        # step 0 is the current pose; aligning to it is meaningless.
        i0 = 0
        while i0 < len(pts) - 1 and math.hypot(*pts[i0]) < self.pos_radius:
            i0 += 1
        self._pts = pts[i0:]
        self._idx = 0
        self._aligned = False

    @property
    def finished(self):
        return self._idx >= len(self._pts)

    @property
    def n_points(self):
        return len(self._pts)

    @property
    def target_index(self):
        return self._idx

    # ── one control tick ─────────────────────────────────────────────
    def step(self, lx, ly, lyaw):
        """lx, ly, lyaw : drone pose in the inference-LOCAL frame
        (telemetry delta since the snapshot). Returns (vx, wz).
        Exactly one of vx / wz is non-zero, matching the platform
        invariant enforced by waypoint_follower."""
        if not self._pts or self.finished:
            return (0.0, 0.0)

        tx, ty = self._pts[self._idx]
        dx, dy = tx - lx, ty - ly
        d = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        berr = _wrap(desired_yaw - lyaw)

        # Reached this point, or slipped past it → advance.
        captured = d < self.pos_radius
        passed = abs(berr) > self.passed_bearing
        if captured or passed:
            self._log("traj: point %d/%d %s (d=%.2f berr=%+.0f deg)",
                      self._idx + 1, len(self._pts),
                      "PASSED" if (passed and not captured) else "reached",
                      d, math.degrees(berr))
            self._idx += 1
            self._aligned = False
            if self.finished:
                return (0.0, 0.0)
            # Glide test: if the next point is roughly straight ahead,
            # don't stop to re-align — keep flying (same as
            # waypoint_follower's skip_yaw_thresh GLIDE).
            ntx, nty = self._pts[self._idx]
            nberr = _wrap(math.atan2(nty - ly, ntx - lx) - lyaw)
            if abs(nberr) < self.skip_yaw:
                self._aligned = True
            return self.step(lx, ly, lyaw)        # re-evaluate immediately

        # ── ALIGN: pure yaw until pointed at the target ──
        # Skip the align phase entirely when the target is already
        # roughly ahead (same spirit as waypoint_follower's GLIDE, but
        # also applied to the first target so a straight-ahead goal
        # doesn't trigger a needless yaw cycle every re-zero).
        if not self._aligned and abs(berr) <= self.skip_yaw:
            self._aligned = True
        if not self._aligned:
            if abs(berr) <= self.yaw_settle:
                self._aligned = True
            else:
                return (0.0, self._yaw_cmd(berr))

        # ── ADVANCE: pure vx; re-align if heading drifts off the point ──
        if abs(berr) > self.yaw_realign:
            self._log("traj: drifted off point %d (berr=%+.0f deg) "
                      "→ re-align", self._idx + 1, math.degrees(berr))
            self._aligned = False
            return (0.0, self._yaw_cmd(berr))
        return (self.vel_x, 0.0)