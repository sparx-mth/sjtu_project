#!/usr/bin/env python3
"""
waypoint_follower.py  (v8 — XY=X-only, Z=fixed, YAW-or-X separation)

PLATFORM INVARIANTS (hard requirements):
  1. vy ≡ 0 in every published Twist.   (no lateral movement)
  2. vz ≡ 0 in every published Twist after takeoff.   (fixed altitude)
  3. vx = 0  OR  wz = 0  in every published Twist.    (never both)

The drone climbs to `takeoff_z` during the TAKING_OFF state by
re-publishing /takeoff (Empty) — this is the existing sjtu_drone
takeoff path and the code does NOT command vz to climb. Once
airborne, the underlying flight controller holds altitude on its own
when given linear.z = 0. From HOVER_SETTLE onwards, every Twist this
node publishes has vy = 0 AND vz = 0.

vs v7:
  • vz forced to 0 in _publish_twist (was: alt-hold P-controller).
  • Takeoff altitude is now an explicit ~takeoff_z argument (default
    1.0 m). cruise_z and the altitude-hold parameters are removed.
  • _publish_twist now takes (vx, wz) only — vy and vz are no longer
    even arguments. There is exactly one path through which a Twist
    can be assembled, and that path hardwires linear.y = linear.z = 0.

State machine (unchanged from v7):
    WAIT_POSE → TAKING_OFF → HOVER_SETTLE → WAIT_PATH →
    YAW_ALIGN → ADVANCE → (BRAKE → YAW_ALIGN → ADVANCE)* → DONE
"""
import math
import json
import os
import datetime
import rospy
import tf.transformations as tft

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool, Int8, String


def wrap_pi(a):    return math.atan2(math.sin(a), math.cos(a))
def quat_yaw(q):   return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
def saturate(v, lim):
    if v >  lim: return  lim
    if v < -lim: return -lim
    return v


class DemoMode:
    """The three operational flight modes the system can be in.
    Strings match the ROS2 DemoMode(str, Enum) payloads bridged over
    /xtend/demo_mode and /xtend/demo_mode_request."""
    FLY_STRAIGHT    = "fly_straight"
    TURNING         = "turning"
    VISUAL_SERVOING = "visual_servoing"


class S:
    WAIT_POSE    = "WAIT_POSE"
    TAKING_OFF   = "TAKING_OFF"
    HOVER_SETTLE = "HOVER_SETTLE"
    WAIT_PATH    = "WAIT_PATH"
    YAW_ALIGN    = "YAW_ALIGN"
    ADVANCE      = "ADVANCE"
    BRAKE        = "BRAKE"
    DONE         = "DONE"


class WaypointFollower:
    def __init__(self):
        rospy.init_node("waypoint_follower")
        G = rospy.get_param

        self.drone_ns = G("~drone_ns", "/simple_drone")

        # Speeds
        self.vel_x    = float(G("~vel_x",    0.3))
        self.yaw_rate = float(G("~yaw_rate", 0.7))

        # Acquisition radii / settle thresholds
        self.pos_radius = float(G("~pos_acquisition_radius", 0.35))
        self.yaw_radius = float(G("~yaw_acquisition_radius", 0.10))
        self.yaw_settle = float(G("~yaw_settle_thresh",      0.05))

        # Strict-separation thresholds
        self.yaw_drift_thresh = float(G("~yaw_drift_thresh", 0.40))
        self.vx_brake_thresh  = float(G("~vx_brake_thresh",  0.05))
        self.skip_yaw_thresh  = float(G("~skip_yaw_thresh",  0.25))
        self.brake_timeout_s  = float(G("~brake_timeout_s",  2.0))
        # Pass-through threshold: if the bearing to the current target
        # exceeds this, the drone is past it (waypoint is now mostly
        # behind/perpendicular). Advance to the next wp instead of
        # chasing it forever. 100° = 10° past perpendicular, clearly
        # committed to flying past. Without this, an off-axis YAW exit
        # makes the drone miss the pos_radius and fly off into nowhere.
        self.passed_bearing_rad = math.radians(
            float(G("~passed_bearing_deg", 100.0)))

        # Yaw-lead: stop the rotation early to compensate for inertia.
        # 0   = aim at the actual desired heading (drone overshoots due
        #       to slewer ramp-down on wz)
        # 10  = aim 10% short; e.g. for a 90° rotation, aim at 81°. The
        #       remaining ~9° is consumed by the inertia ramp-down.
        # Read live every YAW_ALIGN entry from /waypoint_follower/yaw_lead_pct
        # so you can `rosparam set /waypoint_follower/yaw_lead_pct 12`
        # mid-flight without restarting the node.
        self.yaw_lead_pct = float(G("~yaw_lead_pct", 10.0))
        # Per-yaw-alignment snapshot. Captured on entry to YAW_ALIGN
        # so the lead-offset is fixed at "X% of the *initial* sweep,"
        # not "X% of whatever's left this tick." Without the snapshot,
        # the lead would shrink with the error and never trigger.
        self._yaw_align_initial_eyaw = 0.0
        self._yaw_align_lead         = 0.0
        # Heading at the moment we enter ADVANCE. Used for actual-drift
        # detection (compare current yaw against this), not bearing-
        # to-target geometry.
        self._advance_yaw_at_entry   = 0.0

        # Takeoff
        self.auto_takeoff      = bool (G("~auto_takeoff",      True))
        # NEW v8 — explicit takeoff altitude. The TAKING_OFF state
        # publishes /takeoff (Empty) until the drone reaches
        # takeoff_z_thresh; after that we trust the platform's own
        # altitude hold and never command vz again. The argument name
        # and a launch override are documented in the README.
        self.takeoff_z         = float(G("~takeoff_z",         1.0))
        self.takeoff_z_thresh  = float(G("~takeoff_z_thresh",  0.5))
        self.takeoff_timeout   = float(G("~takeoff_timeout",   30.0))
        self.takeoff_retry_sec = float(G("~takeoff_retry_sec", 1.0))
        self.hover_settle_sec  = float(G("~hover_settle_sec",  2.5))

        # Slew + saturations
        self.vel_xy_sat       = float(G("~vel_xy_sat",       1.25))
        self.yaw_rate_sat     = float(G("~yaw_rate_sat",     2.4))
        self.accel_limit      = float(G("~accel_limit",      1.5))
        self.yaw_accel_limit  = float(G("~yaw_accel_limit",  2.0))

        # Behaviour
        self.freeze_during_yaw = bool (G("~freeze_during_yaw", True))
        self.ctrl_rate_hz      = float(G("~ctrl_rate_hz",      50.0))
        self.status_hz         = float(G("~status_hz",         1.0))

        # Startup hold: for the first `startup_hold_sec` seconds the
        # node refuses to command any motion (vx and wz forced to 0),
        # so the drone waits while the map warms up instead of cruising
        # into an all-unknown world. 0 disables.
        self.startup_hold_sec = float(G("~startup_hold_sec", 5.0))
        self._node_start_t    = rospy.Time.now()

        # Forward-only mode: skip YAW_ALIGN entirely (treat all transitions
        # to YAW_ALIGN as transitions to ADVANCE). Useful when the drone is
        # already pointed in the right direction and you just want it to
        # fly forward — e.g. straight down a corridor.
        self.forward_only = bool(G("~forward_only", False))

        # Optional cmd_vel logger. Every published Twist is appended to the
        # file as one JSON object per line (JSON Lines format). Empty path
        # disables logging. `{ts}` in the path expands to a YYYYMMDD_HHMMSS
        # timestamp at startup, so each run gets a unique file.
        log_path = G("~cmd_log_path", "/home/falcon/runs/cmd_log_{ts}.jsonl")
        if log_path and "{ts}" in log_path:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = log_path.replace("{ts}", ts)
        self.log_path = log_path
        self._log_file = None
        if log_path:
            try:
                os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
                self._log_file = open(log_path, "w")
            except Exception as e:
                rospy.logwarn("waypoint_follower: failed to open log %s: %s",
                              log_path, e)
                self._log_file = None

        # State
        self.state         = S.WAIT_POSE
        self.t_state       = rospy.Time.now()
        self.cur_pose      = None
        self.takeoff_pose  = None
        self.path_xy       = []
        self.wp_idx        = 0
        self.last_takeoff  = rospy.Time(0)
        self.takeoff_count = 0
        self.drone_state   = None
        self.last_freeze   = None
        # Slew memory. last_vy and last_vz are kept at 0 — never
        # written to anywhere except this initialization. This is
        # what makes the platform invariant on Y/Z hold.
        self.last_vx = self.last_wz = 0.0
        self.last_vy = 0.0   # NEVER MUTATED
        self.last_vz = 0.0   # NEVER MUTATED

        # Topics
        self.t_cmd_vel = self.drone_ns + "/cmd_vel"
        self.t_takeoff = self.drone_ns + "/takeoff"
        self.t_pose    = self.drone_ns + "/gt_pose"
        self.t_dstate  = self.drone_ns + "/state"
        self.t_path    = "/path/waypoints"

        self.cmd_vel_pub = rospy.Publisher(self.t_cmd_vel, Twist,
                                            queue_size=1)
        self.takeoff_pub = rospy.Publisher(self.t_takeoff, Empty,
                                            queue_size=1, latch=True)
        self.freeze_pub  = rospy.Publisher("/sensor_gate/freeze", Bool,
                                            queue_size=1, latch=True)
        # DemoMode handshake (bridged via ros1_bridge: see bridge.yaml).
        #
        # demo_mode_topic         : ROS2-owned current state (we read).
        # demo_mode_request_topic : ROS1-owned transition request
        #                           (we publish; system reacts).
        # request_repeat_sec      : while waiting for confirmation we
        #                           re-publish the request at this
        #                           cadence so a brief bridge stutter
        #                           doesn't deadlock the handshake.
        # request_timeout_sec     : log loudly if a request hasn't been
        #                           confirmed in this long. 0 disables.
        # current_demo_mode       : the last value seen on
        #                           demo_mode_topic. None until the
        #                           first message arrives (the
        #                           ~startup_delay_sec below covers
        #                           that window).
        # requested_demo_mode     : the mode we are currently asking
        #                           for. Used to detect "new request"
        #                           and reset the timeout/repeat clocks.
        self.demo_mode_topic         = G("~demo_mode_topic",
                                         "/xtend/demo_mode")
        self.demo_mode_request_topic = G("~demo_mode_request_topic",
                                         "/xtend/demo_mode_request")
        self.request_repeat_sec  = float(G("~request_repeat_sec",  0.5))
        self.request_timeout_sec = float(G("~request_timeout_sec", 5.0))
        self.current_demo_mode   = None
        self.requested_demo_mode = None
        self._last_request_pub_t = rospy.Time(0)
        self._request_entered_t  = rospy.Time(0)
        # Latched so the most recent request is visible to a late-
        # joining subscriber across the bridge.
        self.demo_req_pub = rospy.Publisher(self.demo_mode_request_topic,
                                             String, queue_size=1, latch=True)

        rospy.Subscriber(self.t_pose,   Pose, self._pose_cb,   queue_size=10)
        rospy.Subscriber(self.t_dstate, Int8, self._dstate_cb, queue_size=10)
        rospy.Subscriber(self.t_path,   Path, self._path_cb,   queue_size=1)
        rospy.Subscriber(self.demo_mode_topic, String,
                         self._demo_mode_cb, queue_size=10)

        # Brief stabilisation delay. The launcher only starts us once
        # the upstream inputs are wired up, but the bridge subscribers
        # need a moment to actually receive their first message (and
        # for latched publishers like demo_req_pub to register with the
        # bridge) before we start sending commands. 1s is plenty.
        rospy.sleep(float(G("~startup_delay_sec", 1.0)))

        rospy.on_shutdown(self._on_shutdown)
        rospy.Timer(rospy.Duration(1.0 / self.ctrl_rate_hz), self._ctrl_loop)
        rospy.Timer(rospy.Duration(1.0 / self.status_hz),    self._status)

        rospy.loginfo("=" * 64)
        rospy.loginfo("waypoint_follower v8 ready  (X+YAW only, fixed altitude)")
        rospy.loginfo("  drone_ns = %s", self.drone_ns)
        rospy.loginfo("  ctrl=%dHz  vel_x=%.2f m/s  yaw_rate=%.2f rad/s",
                      int(self.ctrl_rate_hz), self.vel_x, self.yaw_rate)
        rospy.loginfo("  takeoff_z=%.2f m  (Empty msgs to %s; no vz commands)",
                      self.takeoff_z, self.t_takeoff)
        rospy.loginfo("  YAW_ALIGN: yaw_rad=%.2f  yaw_settle=%.2f  "
                      "lead=%.1f%% (live: rosparam set ~yaw_lead_pct)",
                      self.yaw_radius, self.yaw_settle, self.yaw_lead_pct)
        rospy.loginfo("  ADVANCE  : drift_thresh=%.0f°  skip_yaw=%.0f°  "
                      "passed_bearing=%.0f°",
                      math.degrees(self.yaw_drift_thresh),
                      math.degrees(self.skip_yaw_thresh),
                      math.degrees(self.passed_bearing_rad))
        rospy.loginfo("  BRAKE    : vx_thresh=%.2f m/s  timeout=%.1fs",
                      self.vx_brake_thresh, self.brake_timeout_s)
        rospy.loginfo("  PUBLISHED Twist invariants:  vy≡0  vz≡0  "
                      "(vx=0 OR wz=0)")
        rospy.loginfo("  forward_only=%s   cmd_log=%s",
                      self.forward_only,
                      self.log_path if self._log_file else "disabled")
        rospy.loginfo("=" * 64)

    # ─── Callbacks ───────────────────────────────────────────────
    def _pose_cb(self, msg):
        if self.cur_pose is None:
            rospy.loginfo("waypoint_follower: first /gt_pose pose=(%.2f,%.2f,%.2f)",
                          msg.position.x, msg.position.y, msg.position.z)
        self.cur_pose = msg

    def _dstate_cb(self, msg): self.drone_state = msg.data

    def _demo_mode_cb(self, msg):
        new_mode = (msg.data or "").strip().lower()
        if new_mode != self.current_demo_mode:
            rospy.loginfo("waypoint_follower: DemoMode  %s → %s",
                          self.current_demo_mode, new_mode)
            self.current_demo_mode = new_mode

    def _request_demo_mode(self, mode):
        """Publish a DemoMode transition request (rate-limited + with
        a soft timeout). Switching `mode` resets the request clocks so
        the timeout measures "this attempt", not the cumulative
        history. We keep re-publishing on `request_repeat_sec` cadence
        until the system confirms; the latched publisher means a late-
        joining subscriber still sees the last request immediately.
        """
        if self.requested_demo_mode != mode:
            self.requested_demo_mode = mode
            self._request_entered_t  = rospy.Time.now()
            self._last_request_pub_t = rospy.Time(0)
            rospy.loginfo("waypoint_follower: DemoMode REQUEST → %s "
                          "(current=%s)", mode, self.current_demo_mode)
        # Already confirmed → stop re-publishing.
        if self.current_demo_mode == mode:
            return
        now = rospy.Time.now()
        if (now - self._last_request_pub_t).to_sec() >= self.request_repeat_sec:
            self.demo_req_pub.publish(String(data=mode))
            self._last_request_pub_t = now
        if (self.request_timeout_sec > 0.0 and
                (now - self._request_entered_t).to_sec()
                    > self.request_timeout_sec):
            rospy.logwarn_throttle(2.0,
                "waypoint_follower: DemoMode request '%s' not confirmed "
                "after %.1fs (current=%s) — holding zero velocity",
                mode, (now - self._request_entered_t).to_sec(),
                self.current_demo_mode)

    def _ensure_mode(self, mode):
        """Stop -> Request -> Wait -> Action.

        Hard-publish zero velocity (the "Stop"), then drive the
        request handshake. Returns True iff the system has officially
        confirmed `mode` via /xtend/demo_mode. Callers invoke this at
        the top of every motion state and bail out (continuing to
        hold zero velocity) on False — physical motion only runs once
        this returns True.
        """
        self._publish_twist(0.0, 0.0)
        self._request_demo_mode(mode)
        return self.current_demo_mode == mode

    def _path_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            rospy.logwarn("waypoint_follower: empty path"); return

        if self.cur_pose is not None and len(pts) >= 2:
            cx = self.cur_pose.position.x
            cy = self.cur_pose.position.y
            best_i, best_d = 0, float('inf')
            for i in range(len(pts) - 1):
                ax, ay = pts[i]; bx, by = pts[i + 1]
                ex, ey = bx - ax, by - ay
                L2 = ex * ex + ey * ey
                if L2 < 1e-9:
                    px, py = ax, ay
                else:
                    t = ((cx - ax) * ex + (cy - ay) * ey) / L2
                    if   t < 0.0: t = 0.0
                    elif t > 1.0: t = 1.0
                    px, py = ax + t * ex, ay + t * ey
                d = math.hypot(cx - px, cy - py)
                if d < best_d:
                    best_d, best_i = d, i
            drop = best_i + 1
            if drop < len(pts):
                ex, ey = pts[drop]
                if math.hypot(ex - cx, ey - cy) < self.pos_radius:
                    drop += 1
            if drop >= len(pts):
                pts = pts[-1:]
            else:
                pts = pts[drop:]

        self.path_xy = pts
        self.wp_idx  = 0
        rospy.loginfo("waypoint_follower: NEW PATH  %d wp  "
                      "first=(%.2f,%.2f)  last=(%.2f,%.2f)",
                      len(pts), pts[0][0], pts[0][1], pts[-1][0], pts[-1][1])
        if self.state in (S.YAW_ALIGN, S.ADVANCE, S.BRAKE, S.DONE):
            # Refresh per-state snapshots BEFORE deciding state. If the
            # new path's first waypoint requires a totally different
            # rotation (e.g. old sweep was +34° and new is -147°), the
            # YAW_ALIGN snapshot from the old target — including its
            # sign — would otherwise stay stale across the path change
            # because _enter() is a no-op when state doesn't change.
            # Same for advance_yaw_at_entry. _entry_after_new_path may
            # transition states below; _enter() will refresh again on
            # transition (idempotent), and skip if state is unchanged
            # (which is exactly when we need this pre-refresh).
            if self.state == S.YAW_ALIGN:
                self._snapshot_yaw_lead()
            elif self.state == S.ADVANCE and self.cur_pose is not None:
                self._advance_yaw_at_entry = quat_yaw(
                    self.cur_pose.orientation)
            self._entry_after_new_path()

    def _entry_after_new_path(self):
        if not self.path_xy or self.cur_pose is None:
            self._enter(S.YAW_ALIGN); return
        tx, ty = self.path_xy[0]
        cx, cy = self.cur_pose.position.x, self.cur_pose.position.y
        if math.hypot(tx - cx, ty - cy) < 1e-3:
            self._enter(S.YAW_ALIGN); return
        bearing = math.atan2(ty - cy, tx - cx)
        yaw_cur = quat_yaw(self.cur_pose.orientation)
        moving  = abs(self.last_vx) > 0.05
        if moving and abs(wrap_pi(bearing - yaw_cur)) < self.skip_yaw_thresh:
            self._enter(S.ADVANCE)
        else:
            self._enter(S.BRAKE if moving else S.YAW_ALIGN)

    # ─── Helpers ─────────────────────────────────────────────────
    def _enter(self, new):
        # Forward-only mode: never enter YAW_ALIGN; jump straight to ADVANCE.
        if new == S.YAW_ALIGN and self.forward_only:
            new = S.ADVANCE
        if new != self.state:
            rospy.loginfo("waypoint_follower: %s → %s", self.state, new)
            self.state   = new
            self.t_state = rospy.Time.now()
            # On YAW_ALIGN entry, snapshot the initial sweep and the
            # absolute lead offset for this rotation. We re-read
            # yaw_lead_pct live so it's tunable without restart.
            if new == S.YAW_ALIGN:
                self._snapshot_yaw_lead()
            # On ADVANCE entry, snapshot the heading we're committing
            # to. The drift check measures *actual* yaw deviation from
            # this snapshot, not bearing-to-target (which inflates as
            # we approach off-axis waypoints — pure geometry, not drift).
            if new == S.ADVANCE and self.cur_pose is not None:
                self._advance_yaw_at_entry = quat_yaw(
                    self.cur_pose.orientation)

    def _snapshot_yaw_lead(self):
        if self.cur_pose is None or not self.path_xy \
                or self.wp_idx >= len(self.path_xy):
            self._yaw_align_initial_eyaw = 0.0
            self._yaw_align_lead         = 0.0
            return
        # Re-read the param so live `rosparam set` takes effect.
        try:
            pct = float(rospy.get_param("~yaw_lead_pct", self.yaw_lead_pct))
        except Exception:
            pct = self.yaw_lead_pct
        # Clamp to a sane range. >40% would cancel most of the rotation.
        if pct < 0.0:
            pct = 0.0
        elif pct > 40.0:
            rospy.logwarn("waypoint_follower: yaw_lead_pct=%.1f clamped to 40", pct)
            pct = 40.0
        self.yaw_lead_pct = pct

        tx, ty = self.path_xy[self.wp_idx]
        cx, cy = self.cur_pose.position.x, self.cur_pose.position.y
        yaw_des = math.atan2(ty - cy, tx - cx)
        yaw_cur = quat_yaw(self.cur_pose.orientation)
        eyaw    = wrap_pi(yaw_des - yaw_cur)
        self._yaw_align_initial_eyaw = eyaw
        self._yaw_align_lead         = abs(eyaw) * (pct / 100.0)
        if pct > 0.0 and abs(eyaw) > self.yaw_radius:
            rospy.loginfo("waypoint_follower: YAW_ALIGN  sweep=%+.1f°  "
                          "lead=%.1f%% (%.1f°)  effective_target_err=%+.1f°",
                          math.degrees(eyaw), pct,
                          math.degrees(self._yaw_align_lead),
                          math.degrees(eyaw)
                              - math.degrees(self._yaw_align_lead) * (1 if eyaw>0 else -1))

    def _t_in(self): return (rospy.Time.now() - self.t_state).to_sec()

    def _set_freeze(self, want):
        if self.last_freeze is want: return
        self.freeze_pub.publish(Bool(data=bool(want)))
        self.last_freeze = want

    @staticmethod
    def _slew(target, current, max_step):
        delta = target - current
        if delta >  max_step: return current + max_step
        if delta < -max_step: return current - max_step
        return target

    def _publish_twist(self, vx, wz):
        """The ONE path through which a Twist can be assembled.

        Hardwired guarantees:
          • linear.y = 0  (no lateral movement, ever)
          • linear.z = 0  (fixed altitude — platform holds it on its own)
          • vx and wz are slew-limited and saturated
          • vx=0 OR wz=0 invariant is checked (logs error if violated)
        """
        if abs(vx) > 1e-6 and abs(wz) > 1e-6:
            rospy.logerr_throttle(1.0,
                "waypoint_follower: INVARIANT VIOLATION  vx=%.3f wz=%.3f "
                "in state %s — zeroing wz", vx, wz, self.state)
            wz = 0.0

        # Startup hold: swallow every motion command for the first
        # `startup_hold_sec` seconds. The state machine keeps running;
        # only the actuation is suppressed, so once the hold expires
        # the node resumes normally with no extra state.
        if (self.startup_hold_sec > 0.0
                and (rospy.Time.now() - self._node_start_t).to_sec()
                    < self.startup_hold_sec):
            vx = 0.0
            wz = 0.0

        vx = saturate(vx, self.vel_xy_sat)
        wz = saturate(wz, self.yaw_rate_sat)
        dt = 1.0 / self.ctrl_rate_hz
        vx = self._slew(vx, self.last_vx, self.accel_limit     * dt)
        wz = self._slew(wz, self.last_wz, self.yaw_accel_limit * dt)

        m = Twist()
        m.linear.x  = vx
        m.linear.y  = 0.0   # HARDWIRED — never set elsewhere
        m.linear.z  = 0.0   # HARDWIRED — never set elsewhere
        m.angular.x = 0.0
        m.angular.y = 0.0
        m.angular.z = wz
        self.cmd_vel_pub.publish(m)

        self.last_vx = vx
        self.last_wz = wz
        # last_vy and last_vz are not updated — they stay at 0 forever.

        # Log the published Twist to the JSON Lines file, if enabled.
        # One line per Twist; fields mirror the geometry_msgs/Twist
        # structure so the file can be replayed as-is.
        if self._log_file is not None:
            entry = {
                "t":       rospy.Time.now().to_sec(),
                "linear":  {"x": float(vx),  "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": float(wz)},
            }
            try:
                self._log_file.write(json.dumps(entry) + "\n")
                self._log_file.flush()
            except Exception as e:
                rospy.logwarn_throttle(10.0,
                    "waypoint_follower: log write failed: %s", e)

    def _publish_zero(self):
        self._publish_twist(0.0, 0.0)

    # ─── Control loop ────────────────────────────────────────────
    def _ctrl_loop(self, _):
        if self.state == S.WAIT_POSE:
            if self.cur_pose is not None:
                self._enter(S.TAKING_OFF if self.auto_takeoff
                            else S.HOVER_SETTLE)
            return

        if self.cur_pose is None:
            return

        # ── TAKING_OFF: re-publish /takeoff Empty until airborne. ──
        # We do NOT command vz to climb — sjtu_drone owns the vertical
        # actuation in response to the Empty msg. We pump zeros
        # (vx=wz=0) so the cmd_vel stream is continuous.
        if self.state == S.TAKING_OFF:
            now = rospy.Time.now()
            if (now - self.last_takeoff).to_sec() > self.takeoff_retry_sec:
                self.takeoff_pub.publish(Empty())
                self.last_takeoff   = now
                self.takeoff_count += 1
                rospy.loginfo("waypoint_follower: published /takeoff (#%d)  "
                              "target_z=%.2f", self.takeoff_count, self.takeoff_z)
            self._publish_zero()
            airborne = (self.drone_state == 1
                        or self.cur_pose.position.z >= self.takeoff_z_thresh)
            if airborne:
                self._enter(S.HOVER_SETTLE)
            elif self._t_in() > self.takeoff_timeout:
                rospy.logerr("waypoint_follower: takeoff timeout, continuing")
                self._enter(S.HOVER_SETTLE)
            return

        if self.state == S.HOVER_SETTLE:
            if self.takeoff_pose is None:
                self.takeoff_pose = self._copy_pose(self.cur_pose)
                p = self.takeoff_pose.position
                rospy.loginfo("waypoint_follower: takeoff snapshot "
                              "(%.2f,%.2f,%.2f)  — fixing altitude here",
                              p.x, p.y, p.z)
            self._publish_zero()
            if self._t_in() > self.hover_settle_sec:
                self._enter(S.WAIT_PATH)
            return

        if self.state == S.WAIT_PATH:
            self._publish_zero()
            if self.path_xy and self.wp_idx < len(self.path_xy):
                self._enter(S.YAW_ALIGN)
            return

        # ── YAW_ALIGN: pure wz, vx forced to 0 ─────────────────
        if self.state == S.YAW_ALIGN:
            # Handshake: stop, request TURNING, wait for confirmation
            # before driving any wz. Once confirmed, _ensure_mode is a
            # ~free no-op so the alignment logic runs every tick.
            if not self._ensure_mode(DemoMode.TURNING):
                return
            tx, ty = self.path_xy[self.wp_idx]
            cx, cy = self.cur_pose.position.x, self.cur_pose.position.y
            if math.hypot(tx - cx, ty - cy) < self.pos_radius:
                self._set_freeze(False)
                self._publish_zero()
                self.wp_idx += 1
                if self.wp_idx >= len(self.path_xy):
                    self._enter(S.DONE)
                else:
                    self._enter(S.YAW_ALIGN)
                return

            yaw_des = math.atan2(ty - cy, tx - cx)
            yaw_cur = quat_yaw(self.cur_pose.orientation)
            eyaw    = wrap_pi(yaw_des - yaw_cur)

            # Apply yaw-lead: target the heading that's `lead` short of
            # `yaw_des` in the rotation direction. The slewer's natural
            # ramp-down from yaw_rate to 0 carries the actual yaw the
            # remaining `lead` distance, landing on yaw_des. Sign comes
            # from the initial sweep (snapshotted on entry) so a small
            # overshoot mid-rotation doesn't flip it.
            sign = 1.0 if self._yaw_align_initial_eyaw >= 0.0 else -1.0
            eyaw_lead = wrap_pi(eyaw - sign * self._yaw_align_lead)

            # Exit when the LEAD-ADJUSTED error is small AND wz has
            # settled. The remaining inertia closes the actual gap.
            if (abs(eyaw_lead) < self.yaw_radius
                    and abs(self.last_wz) < self.yaw_settle):
                self._set_freeze(False)
                self._publish_zero()
                self._enter(S.ADVANCE)
                return

            self._set_freeze(self.freeze_during_yaw)
            wz_now = self.last_wz
            brake_d = ((wz_now * wz_now) / (2.0 * self.yaw_accel_limit)
                       + abs(wz_now) / self.ctrl_rate_hz)
            # Drive toward the LEAD-ADJUSTED target.
            same_sign = (wz_now * eyaw_lead) >= 0.0
            if same_sign and abs(eyaw_lead) <= brake_d:
                wz_target = 0.0
            else:
                wz_target = math.copysign(self.yaw_rate, eyaw_lead)
            self._publish_twist(0.0, wz_target)
            return

        # ── ADVANCE: pure vx, wz forced to 0 ────────────────────
        if self.state == S.ADVANCE:
            # Handshake: stop, request FLY_STRAIGHT, wait for confirm
            # before driving any vx. Once confirmed (the typical case —
            # we start in FLY_STRAIGHT, so the first ADVANCE confirms
            # immediately), _ensure_mode is a near-free no-op.
            if not self._ensure_mode(DemoMode.FLY_STRAIGHT):
                return
            self._set_freeze(False)
            tx, ty = self.path_xy[self.wp_idx]
            cx, cy = self.cur_pose.position.x, self.cur_pose.position.y
            yaw_cur = quat_yaw(self.cur_pose.orientation)

            # Compute distance and bearing to the current target.
            d           = math.hypot(tx - cx, ty - cy)
            yaw_des     = math.atan2(ty - cy, tx - cx)
            bearing_err = wrap_pi(yaw_des - yaw_cur)

            # Capture conditions:
            #   captured = entered the pos_radius — the normal case
            #   passed   = bearing-to-target now exceeds passed_bearing_rad,
            #              i.e. the waypoint is more behind than ahead. This
            #              catches near-misses where the drone slipped past
            #              just outside pos_radius (e.g. 0.41 m miss when
            #              pos_radius=0.35 m). Without this, the drone
            #              flies off forever still trying to capture wp 1.
            captured = (d < self.pos_radius)
            passed   = (abs(bearing_err) > self.passed_bearing_rad)

            if captured or passed:
                if passed and not captured:
                    rospy.loginfo("waypoint_follower: wp %d PASSED  "
                                  "d=%.2fm bearing=%+.0f° — advancing",
                                  self.wp_idx + 1, d,
                                  math.degrees(bearing_err))
                self.wp_idx += 1
                if self.wp_idx >= len(self.path_xy):
                    rospy.loginfo("waypoint_follower: GOAL REACHED")
                    self._enter(S.BRAKE)
                    return
                ntx, nty = self.path_xy[self.wp_idx]
                next_bearing = math.atan2(nty - cy, ntx - cx)
                next_err     = wrap_pi(next_bearing - yaw_cur)
                if abs(next_err) < self.skip_yaw_thresh:
                    rospy.loginfo("waypoint_follower: wp %d→%d  Δ=%+.0f°  glide",
                                  self.wp_idx, self.wp_idx + 1,
                                  math.degrees(next_err))
                    tx, ty = ntx, nty
                else:
                    rospy.loginfo("waypoint_follower: wp %d→%d  Δ=%+.0f°  "
                                  "corner — brake & yaw",
                                  self.wp_idx, self.wp_idx + 1,
                                  math.degrees(next_err))
                    self._enter(S.BRAKE)
                    return

            # Drift check: compare CURRENT yaw against the heading we
            # snapshotted on ADVANCE entry. This measures actual drone
            # drift (wind, disturbance, slewer residue), NOT how the
            # bearing-to-target evolves as we approach an off-axis
            # waypoint. The latter is pure approach geometry — the
            # drone can be flying perfectly straight and the bearing
            # still grows past 23° as we pass alongside.  Path-bend
            # cases are handled by the wp-transition skip_yaw_thresh
            # check above, not here.
            actual_drift = wrap_pi(yaw_cur - self._advance_yaw_at_entry)
            if abs(actual_drift) > self.yaw_drift_thresh:
                rospy.loginfo("waypoint_follower: ADVANCE drift=%+.0f° > "
                              "%.0f° (vs entry heading) — brake & re-align",
                              math.degrees(actual_drift),
                              math.degrees(self.yaw_drift_thresh))
                self._enter(S.BRAKE)
                return

            self._publish_twist(self.vel_x, 0.0)
            return

        # ── BRAKE: vx_target=0, wait for slewer ──
        if self.state == S.BRAKE:
            self._set_freeze(False)
            self._publish_twist(0.0, 0.0)
            stopped = abs(self.last_vx) < self.vx_brake_thresh
            timed_out = self._t_in() > self.brake_timeout_s
            if stopped or timed_out:
                if timed_out and not stopped:
                    rospy.logwarn("waypoint_follower: BRAKE timeout "
                                  "(last_vx=%.3f) — proceeding anyway",
                                  self.last_vx)
                if self.wp_idx >= len(self.path_xy):
                    self._enter(S.DONE)
                else:
                    self._enter(S.YAW_ALIGN)
            return

        if self.state == S.DONE:
            self._set_freeze(False)
            self._publish_zero()
            return

    @staticmethod
    def _copy_pose(p):
        out = Pose()
        out.position.x = p.position.x
        out.position.y = p.position.y
        out.position.z = p.position.z
        out.orientation.x = p.orientation.x
        out.orientation.y = p.orientation.y
        out.orientation.z = p.orientation.z
        out.orientation.w = p.orientation.w
        return out

    # ─── 1 Hz status ───────────────────────────────────────────
    def _status(self, _):
        if self.cur_pose is None:
            rospy.loginfo("[%-12s] no /gt_pose yet (subscribed to %s)",
                          self.state, self.t_pose); return
        p = self.cur_pose.position
        yaw = math.degrees(quat_yaw(self.cur_pose.orientation))
        # Status line shows what we publish: vy and vz are always 0.
        extra = ""
        if self.state == S.TAKING_OFF:
            extra = "  takeoff_pubs=%d  z=%.2f→%.2f" % (
                    self.takeoff_count, p.z, self.takeoff_z)
        elif self.state == S.WAIT_PATH:
            extra = "  (no path yet on %s)" % self.t_path
        elif self.state == S.BRAKE:
            extra = "  vx=%.3f → 0  (thresh=%.2f)" % (
                    self.last_vx, self.vx_brake_thresh)
        elif self.state in (S.YAW_ALIGN, S.ADVANCE) and self.path_xy:
            tx, ty = self.path_xy[self.wp_idx]
            d  = math.hypot(tx - p.x, ty - p.y)
            ey = math.degrees(wrap_pi(math.atan2(ty - p.y, tx - p.x)
                              - quat_yaw(self.cur_pose.orientation)))
            extra = ("  wp=%d/%d target=(%.2f,%.2f) d=%.2fm yaw_err=%5.1f°"
                     % (self.wp_idx + 1, len(self.path_xy), tx, ty, d, ey))
        rospy.loginfo("[%-12s] pose=(%.2f,%.2f,%.2f) yaw=%5.1f° | "
                      "cmd: vx=%.2f wz=%.2f%s",
                      self.state, p.x, p.y, p.z, yaw,
                      self.last_vx, self.last_wz, extra)

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


if __name__ == "__main__":
    try:
        WaypointFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass