#!/usr/bin/env python3
"""
flight_executor.py — translate a body-frame NavDP Path into cmd_vel.

Runs while the system is in visual_servoing mode (the NavDP mode);
this node IS the cmd_vel publisher during that mode. No mode
handshake, no pose feedback, no localization. The full loop is:

    /navdp/path  ──►  turn-then-go conversion  ──►  /cmd_vel @ 5 Hz

When a new Path arrives mid-stream it REPLACES the in-flight plan
(elapsed resets to 0). When the plan finishes, we publish zeros
until the next Path.

PLATFORM INVARIANTS (carried over from waypoint_follower.py — these
are platform-level, not state-machine-level, so they still apply):
  1. vy ≡ 0 in every published Twist
  2. vz ≡ 0 in every published Twist
  3. vx = 0  OR  wz = 0  in every published Twist  (never both)

Invariant #3 is enforced STRUCTURALLY by the turn-in-place /
go-straight conversion: each (vx, wz, dt) row has exactly one of
vx and wz non-zero. The Twist assembler also rechecks defensively
and zeros wz with a loud error log if both ever slip past.

Slew + saturation are kept (same defaults as waypoint_follower.py)
because they protect the autopilot from the step changes inherent
in the turn→go pulse pattern, regardless of which mode produced
the commands.

5 Hz quantization caveat:
  At 5 Hz, ticks are 0.2 s apart. NavDP emits 24 waypoints over
  ~2-3 m forward, so per-segment heading changes are small and
  the corresponding "turn" rows are often < 0.1 s. Two things
  swallow them:
    (a) ticks straddle the turn — it's never sampled
    (b) wz slew hasn't ramped up before the turn ends
  Net effect: the drone curves LESS than NavDP intended. NavDP
  REPLANS frequently, so per-plan drift is absorbed by the next
  plan — but if it bites visibly, raise _rate to 20 Hz (cheap,
  fixes (a) entirely).

Subscribes:
    ~path_topic         (nav_msgs/Path)        default "/navdp/path"
        Body-frame waypoints: pose.position.x = forward (m),
        pose.position.y = left (m). Orientation IGNORED.

Publishes:
    <drone_ns>/cmd_vel  (geometry_msgs/Twist)  every 1/~rate s

Run:
    rosrun <pkg> flight_executor.py \
        _drone_ns:=/simple_drone _path_topic:=/navdp/path \
        _vel_x:=0.3 _yaw_rate:=0.7 _rate:=5.0
"""
import json
import os
import datetime
import threading

import numpy as np

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path


def saturate(v, lim):
    if v >  lim: return  lim
    if v < -lim: return -lim
    return v


class FlightExecutor:
    def __init__(self):
        rospy.init_node("flight_executor")
        G = rospy.get_param

        self.drone_ns = G("~drone_ns", "/simple_drone")

        # Speed caps + publish rate.
        self.vel_x    = float(G("~vel_x",    0.3))
        self.yaw_rate = float(G("~yaw_rate", 0.7))
        self.rate_hz  = float(G("~rate",     5.0))

        # Slew + saturation. Same defaults as waypoint_follower.py
        # because the autopilot is the same; these are platform
        # concerns and have nothing to do with the state machine
        # the A* follower needs and we don't.
        self.vel_xy_sat      = float(G("~vel_xy_sat",      1.25))
        self.yaw_rate_sat    = float(G("~yaw_rate_sat",    2.4))
        self.accel_limit     = float(G("~accel_limit",     1.5))
        self.yaw_accel_limit = float(G("~yaw_accel_limit", 3.5))

        # Topics.
        self.t_cmd_vel = self.drone_ns + "/cmd_vel"
        self.t_path    = G("~path_topic", "/navdp/path")

        # Active plan and when we started it. Guarded by self.lock
        # because the Path subscriber writes them on a separate
        # thread and the control timer reads them every tick.
        self.lock      = threading.Lock()
        self.cmd_seq   = []        # list of (vx, wz, duration_s)
        self.seq_start = None      # rospy.Time when seq began

        # Slewer memory.
        self.last_vx = 0.0
        self.last_wz = 0.0

        # Optional cmd log (JSON Lines, same convention as A* follower).
        log_path = G("~cmd_log_path",
                     "/home/falcon/runs/flight_exec_log_{ts}.jsonl")
        if log_path and "{ts}" in log_path:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = log_path.replace("{ts}", ts)
        self.log_path  = log_path
        self._log_file = None
        if log_path:
            try:
                os.makedirs(os.path.dirname(log_path) or ".",
                            exist_ok=True)
                self._log_file = open(log_path, "w")
            except Exception as e:
                rospy.logwarn(
                    "flight_executor: failed to open log %s: %s",
                    log_path, e)

        # Publishers / subscribers.
        self.cmd_vel_pub = rospy.Publisher(self.t_cmd_vel, Twist,
                                           queue_size=1)
        # queue_size=1 on the path: only the latest plan matters.
        rospy.Subscriber(self.t_path, Path, self._path_cb,
                         queue_size=1)

        rospy.on_shutdown(self._on_shutdown)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._ctrl_loop)

        rospy.loginfo("=" * 72)
        rospy.loginfo("flight_executor ready  (NavDP turn-then-go @ %.1f Hz)",
                      self.rate_hz)
        rospy.loginfo("  drone_ns = %s", self.drone_ns)
        rospy.loginfo("  in   : %s (nav_msgs/Path, body frame)", self.t_path)
        rospy.loginfo("  out  : %s (geometry_msgs/Twist)",       self.t_cmd_vel)
        rospy.loginfo("  caps : vel_x=%.2f m/s  yaw_rate=%.2f rad/s",
                      self.vel_x, self.yaw_rate)
        rospy.loginfo("  PUBLISHED Twist invariants:  vy≡0  vz≡0  "
                      "(vx=0 OR wz=0)")
        rospy.loginfo("  cmd_log = %s",
                      self.log_path if self._log_file else "disabled")
        rospy.loginfo("=" * 72)

    # ── waypoints → cmd_vel sequence ───────────────────────────────
    def waypoints_to_cmd_vel(self, traj):
        """Turn-in-place then go-straight. Each emitted row has either
        vx=0 (pure turn) or wz=0 (pure go), so platform invariant #3
        (vx=0 OR wz=0) is satisfied STRUCTURALLY.

        traj : iterable of (forward, left) in body frame, metres.
        Returns: list of (vx, wz, dt).
        """
        traj = np.asarray(traj, dtype=float)
        cmds = []
        cx, cy, c_yaw = 0.0, 0.0, 0.0   # origin, facing +X
        for wp in traj:
            tx, ty = float(wp[0]), float(wp[1])
            dx, dy = tx - cx, ty - cy
            dist   = float(np.hypot(dx, dy))
            if dist < 1e-3:
                continue

            # 1) Pure turn to face the next waypoint (vx=0).
            target_yaw = float(np.arctan2(dy, dx))
            yaw_err    = float(np.arctan2(
                np.sin(target_yaw - c_yaw),
                np.cos(target_yaw - c_yaw)))
            if abs(yaw_err) > 1e-3:
                t_turn = abs(yaw_err) / self.yaw_rate
                wz     = self.yaw_rate * np.sign(yaw_err)
                cmds.append((0.0, float(wz), float(t_turn)))
                c_yaw  = target_yaw

            # 2) Pure forward to the waypoint (wz=0).
            t_go = dist / self.vel_x
            cmds.append((float(self.vel_x), 0.0, float(t_go)))
            cx, cy = tx, ty

        return cmds

    # ── path callback: replace the in-flight plan ──────────────────
    def _path_cb(self, msg):
        if not msg.poses:
            rospy.logwarn("flight_executor: empty Path on %s, ignoring",
                          self.t_path)
            return
        traj = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        cmds = self.waypoints_to_cmd_vel(traj)
        if not cmds:
            rospy.logwarn("flight_executor: Path has no executable "
                          "segments, ignoring")
            return
        total_t = sum(c[2] for c in cmds)
        with self.lock:
            self.cmd_seq   = cmds
            self.seq_start = rospy.Time.now()
        n_turn = sum(1 for c in cmds if c[0] == 0.0)
        n_go   = sum(1 for c in cmds if c[0] != 0.0)
        rospy.loginfo("[PATH] new plan: %d waypoints → %d cmds "
                      "(%d turns, %d gos), total %.2fs",
                      len(traj), len(cmds), n_turn, n_go, total_t)

    # ── slewer ─────────────────────────────────────────────────────
    @staticmethod
    def _slew(target, current, max_step):
        delta = target - current
        if delta >  max_step: return current + max_step
        if delta < -max_step: return current - max_step
        return target

    # ── the ONE Twist assembler ────────────────────────────────────
    def _publish_twist(self, vx, wz):
        """Single path through which a Twist can be assembled.
        Mirrors waypoint_follower._publish_twist (minus the
        startup_hold; we don't need it here).

        Hardwired guarantees:
          • linear.y = 0
          • linear.z = 0
          • angular.x = angular.y = 0
          • vx = 0 OR wz = 0  (defensive check; zeros wz if both set)
          • vx and wz are slew-limited and saturated
        """
        if abs(vx) > 1e-6 and abs(wz) > 1e-6:
            rospy.logerr_throttle(
                1.0,
                "flight_executor: INVARIANT VIOLATION  vx=%.3f wz=%.3f "
                "— zeroing wz", vx, wz)
            wz = 0.0

        vx = saturate(vx, self.vel_xy_sat)
        wz = saturate(wz, self.yaw_rate_sat)
        dt = 1.0 / self.rate_hz
        vx = self._slew(vx, self.last_vx, self.accel_limit     * dt)
        wz = self._slew(wz, self.last_wz, self.yaw_accel_limit * dt)

        m = Twist()
        m.linear.x  = vx
        m.linear.y  = 0.0   # HARDWIRED
        m.linear.z  = 0.0   # HARDWIRED
        m.angular.x = 0.0
        m.angular.y = 0.0
        m.angular.z = wz
        self.cmd_vel_pub.publish(m)

        self.last_vx = vx
        self.last_wz = wz

        if self._log_file is not None:
            entry = {
                "t":       rospy.Time.now().to_sec(),
                "linear":  {"x": float(vx), "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": float(wz)},
            }
            try:
                self._log_file.write(json.dumps(entry) + "\n")
                self._log_file.flush()
            except Exception as e:
                rospy.logwarn_throttle(
                    10.0,
                    "flight_executor: log write failed: %s", e)

    def _publish_zero(self):
        self._publish_twist(0.0, 0.0)

    # ── control loop @ rate_hz ────────────────────────────────────
    def _ctrl_loop(self, _):
        with self.lock:
            cmds  = self.cmd_seq
            start = self.seq_start

        # Idle: no plan, or finished plan. Publish a zero (still
        # goes through the slewer, so any residual motion from a
        # just-cleared plan ramps down gracefully).
        if not cmds or start is None:
            self._publish_zero()
            return

        elapsed = (rospy.Time.now() - start).to_sec()

        # Look up the active segment by elapsed time. O(N) where N
        # is ≤ ~50 for a 24-waypoint NavDP plan and the timer runs
        # at 5 Hz, so this costs nothing.
        t_acc   = 0.0
        current = None
        for vx, wz, dur in cmds:
            if elapsed < t_acc + dur:
                current = (vx, wz)
                break
            t_acc += dur

        if current is None:
            # Past the end → publish zero and clear (unless preempted
            # by a new plan since we read seq_start).
            self._publish_zero()
            with self.lock:
                if self.seq_start == start:
                    self.cmd_seq   = []
                    self.seq_start = None
                    rospy.loginfo("[PATH] plan complete")
            return

        vx, wz = current
        self._publish_twist(vx, wz)

    # ── shutdown: leave the bus quiet ──────────────────────────────
    def _on_shutdown(self):
        try:
            for _ in range(5):
                self._publish_zero()
                rospy.sleep(0.02)
        except Exception:
            pass
        if self._log_file is not None:
            try:
                self._log_file.close()
            except Exception:
                pass


def main():
    FlightExecutor()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass