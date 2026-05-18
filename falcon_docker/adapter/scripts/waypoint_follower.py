#!/usr/bin/env python3
"""
waypoint_follower.py  (v9 — DemoMode handshake + takeoff gating)

PLATFORM INVARIANTS (hard requirements):
  1. vy ≡ 0 in every published Twist.   (no lateral movement)
  2. vz ≡ 0 in every published Twist after takeoff.   (fixed altitude)
  3. vx = 0  OR  wz = 0  in every published Twist.    (never both)
  4. NOTHING is published — no /cmd_vel, no /takeoff, no /sensor_gate
     /freeze, no /xtend/demo_mode_request — while the system DemoMode
     is TAKEOFF (or before any DemoMode has been observed). The drone
     takes off and stabilises without any interference from this node.

vs v8:
  • Takeoff is now owned by the ROS2 system, not this planner. The
    auto_takeoff arg is kept for back-compat but it no longer triggers
    /takeoff publishes from here.
  • Added a hard gate on every publish path. The gate opens only once
    the bridged ROS2 topic /xtend/demo_mode reports anything other
    than TAKEOFF (typically IDLE).
  • Added a strict handshake before every motion-mode change. Before
    the drone physically turns, the node:
        a) commands a hover (zero vx, zero wz),
        b) publishes the requested mode on /xtend/demo_mode_request,
        c) waits for /xtend/demo_mode to report the matching mode,
        d) only then enters the actual motion state (YAW_ALIGN /
           ADVANCE) and starts driving wz / vx.
    The same handshake gates the transition from turning back to
    forward flight (REQ_FLY_STRAIGHT → ADVANCE), and the final
    transition to FINISH on DONE.

State machine:
    TAKEOFF → HOVER_SETTLE → WAIT_PATH →
    REQ_TURNING → YAW_ALIGN → REQ_FLY_STRAIGHT → ADVANCE →
      (BRAKE → REQ_TURNING → YAW_ALIGN → REQ_FLY_STRAIGHT → ADVANCE)*
      → DONE (request FINISH)

The initial S.TAKEOFF mirrors DemoMode.TAKEOFF: the planner is
inert (no publishes, no path computation) until /xtend/demo_mode
explicitly reports IDLE. A stray transient mode received before
IDLE will NOT wake the planner.

All control-loop branches are non-blocking: requests are re-published
at most once per `request_repeat_sec` and confirmation is checked by
the cached current_demo_mode value, never by a sleep/spin.
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
    """Mirrors the ROS2 DemoMode(str, Enum) used by the system state
    machine. Kept as plain strings so equality matches the raw payload
    of std_msgs/String coming over the ros1_bridge."""
    TAKEOFF         = "takeoff"
    IDLE            = "idle"
    FLY_STRAIGHT    = "fly_straight"
    TURNING         = "turning"
    VISUAL_SERVOING = "visual_servoing"
    FINISH          = "finish"


class S:
    TAKEOFF          = "TAKEOFF"
    HOVER_SETTLE     = "HOVER_SETTLE"
    WAIT_PATH        = "WAIT_PATH"
    REQ_TURNING      = "REQ_TURNING"
    YAW_ALIGN        = "YAW_ALIGN"
    REQ_FLY_STRAIGHT = "REQ_FLY_STRAIGHT"
    ADVANCE          = "ADVANCE"
    BRAKE            = "BRAKE"
    DONE             = "DONE"


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

        # Takeoff — owned by the ROS2 system in v9. auto_takeoff is
        # accepted for back-compat but only logged; this node never
        # publishes /takeoff. The drone climbs and stabilises while
        # the system state is DemoMode.TAKEOFF; we just wait silently.
        self.auto_takeoff      = bool (G("~auto_takeoff",      False))
        self.takeoff_z         = float(G("~takeoff_z",         1.0))
        self.takeoff_z_thresh  = float(G("~takeoff_z_thresh",  0.5))
        self.takeoff_timeout   = float(G("~takeoff_timeout",   30.0))
        self.takeoff_retry_sec = float(G("~takeoff_retry_sec", 1.0))
        self.hover_settle_sec  = float(G("~hover_settle_sec",  2.5))

        # DemoMode handshake (bridged via ros1_bridge: see bridge.yaml).
        # demo_mode_topic         : ROS2-owned current state (we read).
        # demo_mode_request_topic : ROS1-owned transition request
        #                           (we publish; system reacts).
        # request_repeat_sec      : while waiting for confirmation we
        #                           re-publish the request at this
        #                           cadence so a brief bridge stutter
        #                           doesn't deadlock the handshake.
        # request_timeout_sec     : log loudly if a request hasn't been
        #                           confirmed in this long. 0 disables.
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

        # State — start in TAKEOFF, mirroring the system DemoMode.
        # The planner stays passive (publishes nothing, plans nothing)
        # until the bridged /xtend/demo_mode reports IDLE. Any other
        # value (None, TAKEOFF, or a stray mode like TURNING received
        # before IDLE) keeps the planner silent.
        self.state         = S.TAKEOFF
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
        # Latched so the most recent request is visible to a late-
        # joining subscriber across the bridge.
        self.demo_req_pub = rospy.Publisher(self.demo_mode_request_topic,
                                            String, queue_size=1, latch=True)

        rospy.Subscriber(self.t_pose,   Pose, self._pose_cb,   queue_size=10)
        rospy.Subscriber(self.t_dstate, Int8, self._dstate_cb, queue_size=10)
        rospy.Subscriber(self.t_path,   Path, self._path_cb,   queue_size=1)
        rospy.Subscriber(self.demo_mode_topic, String,
                         self._demo_mode_cb, queue_size=10)

        rospy.on_shutdown(self._on_shutdown)
        rospy.Timer(rospy.Duration(1.0 / self.ctrl_rate_hz), self._ctrl_loop)
        rospy.Timer(rospy.Duration(1.0 / self.status_hz),    self._status)

        rospy.loginfo("=" * 64)
        rospy.loginfo("waypoint_follower v9 ready  (DemoMode-gated, X+YAW only)")
        rospy.loginfo("  drone_ns = %s", self.drone_ns)
        rospy.loginfo("  ctrl=%dHz  vel_x=%.2f m/s  yaw_rate=%.2f rad/s",
                      int(self.ctrl_rate_hz), self.vel_x, self.yaw_rate)
        rospy.loginfo("  demo_mode  in  = %s   (waiting for IDLE)",
                      self.demo_mode_topic)
        rospy.loginfo("  demo_mode  out = %s   (request repeat=%.2fs)",
                      self.demo_mode_request_topic, self.request_repeat_sec)
        if self.auto_takeoff:
            rospy.logwarn("waypoint_follower: auto_takeoff=true is IGNORED "
                          "in v9 — the ROS2 system owns takeoff. The node "
                          "stays passive until /xtend/demo_mode == IDLE.")
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
        if new_mode == self.current_demo_mode:
            return
        rospy.loginfo("waypoint_follower: DemoMode  %s → %s",
                      self.current_demo_mode, new_mode)
        self.current_demo_mode = new_mode

    # ─── DemoMode handshake helpers ──────────────────────────────
    def _publishing_allowed(self):
        """Hard gate on every outbound publish.

        Returns False until the bridged system state has reported any
        non-TAKEOFF mode at least once. This implements the strict
        "do not interfere with takeoff" requirement: no /cmd_vel, no
        /sensor_gate/freeze, no /takeoff and no /xtend/demo_mode_request
        leaves this node while the system is in TAKEOFF (or before any
        DemoMode message has been received).
        """
        m = self.current_demo_mode
        return m is not None and m != DemoMode.TAKEOFF

    def _request_demo_mode(self, mode):
        """Publish a DemoMode transition request (rate-limited).

        Re-publishing periodically while we wait for confirmation
        survives a brief bridge stutter without deadlocking. The very
        first request for a given target also resets the timeout
        clock used for the staleness warning.
        """
        if not self._publishing_allowed():
            return
        if self.requested_demo_mode != mode:
            self.requested_demo_mode = mode
            self._request_entered_t  = rospy.Time.now()
            self._last_request_pub_t = rospy.Time(0)
            rospy.loginfo("waypoint_follower: DemoMode REQUEST → %s", mode)
        # Once the system has confirmed, stop spamming. The latched
        # publisher means a late subscriber still sees the last value.
        if self._demo_mode_is(mode):
            return
        now = rospy.Time.now()
        if (now - self._last_request_pub_t).to_sec() < self.request_repeat_sec:
            return
        self.demo_req_pub.publish(String(data=mode))
        self._last_request_pub_t = now
        if (self.request_timeout_sec > 0.0 and
                (now - self._request_entered_t).to_sec()
                    > self.request_timeout_sec):
            rospy.logwarn_throttle(2.0,
                "waypoint_follower: DemoMode request '%s' not confirmed "
                "after %.1fs (current=%s) — still waiting, no motion",
                mode, (now - self._request_entered_t).to_sec(),
                self.current_demo_mode)

    def _demo_mode_is(self, mode):
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
        if self.state in (S.REQ_TURNING, S.YAW_ALIGN, S.REQ_FLY_STRAIGHT,
                           S.ADVANCE, S.BRAKE, S.DONE):
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
        # Route the post-new-path decision through the handshake
        # states. Any "go to YAW_ALIGN" becomes "REQ_TURNING first";
        # any "go to ADVANCE" becomes "REQ_FLY_STRAIGHT first".
        # BRAKE still chains via REQ_TURNING after it has stopped.
        if not self.path_xy or self.cur_pose is None:
            self._enter(S.REQ_TURNING); return
        tx, ty = self.path_xy[0]
        cx, cy = self.cur_pose.position.x, self.cur_pose.position.y
        if math.hypot(tx - cx, ty - cy) < 1e-3:
            self._enter(S.REQ_TURNING); return
        bearing = math.atan2(ty - cy, tx - cx)
        yaw_cur = quat_yaw(self.cur_pose.orientation)
        moving  = abs(self.last_vx) > 0.05
        if moving and abs(wrap_pi(bearing - yaw_cur)) < self.skip_yaw_thresh:
            # Already pointing the right way and already moving — no
            # physical mode change, just keep going. We're still in
            # FLY_STRAIGHT, no need to re-handshake.
            self._enter(S.ADVANCE)
        else:
            self._enter(S.BRAKE if moving else S.REQ_TURNING)

    # ─── Helpers ─────────────────────────────────────────────────
    def _enter(self, new):
        # Forward-only mode: skip the turn entirely. Both the request-
        # turn handshake and the physical turn collapse into "request
        # fly_straight then advance".
        if new in (S.REQ_TURNING, S.YAW_ALIGN) and self.forward_only:
            new = S.REQ_FLY_STRAIGHT
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
        # Gated like every other publish: nothing leaves this node
        # while the system is in TAKEOFF.
        if not self._publishing_allowed():
            return
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

        # DemoMode gate: while TAKEOFF (or before any DemoMode is
        # observed), publish absolutely nothing. The state machine
        # keeps ticking but no Twist leaves this node, so the drone
        # is free to take off and stabilise undisturbed.
        if not self._publishing_allowed():
            return

        # Handshake gate: while we're waiting on the system to
        # confirm a requested mode, the only valid Twist is a hover.
        # The control-loop branches do the right thing on their own,
        # but this is a belt-and-braces check: never command vx or
        # wz while the system has not yet entered TURNING /
        # FLY_STRAIGHT respectively.
        if self.state == S.REQ_TURNING and not self._demo_mode_is(
                DemoMode.TURNING):
            vx, wz = 0.0, 0.0
        elif self.state == S.REQ_FLY_STRAIGHT and not self._demo_mode_is(
                DemoMode.FLY_STRAIGHT):
            vx, wz = 0.0, 0.0

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
        # ── TAKEOFF: completely passive. ──
        # Don't publish anything — not even zero Twists, not even
        # /sensor_gate/freeze — and don't plan or compute paths.
        # _publishing_allowed() makes this a hard contract, but we
        # also short-circuit here so no other control-loop logic
        # runs while the system has not yet reached IDLE. We require
        # IDLE specifically (not just "anything but TAKEOFF") so a
        # stray transient mode can't activate the planner early.
        if self.state == S.TAKEOFF:
            if self.current_demo_mode == DemoMode.IDLE:
                rospy.loginfo("waypoint_follower: DemoMode reached IDLE "
                              "— activating planner")
                self._enter(S.HOVER_SETTLE)
            return

        if self.cur_pose is None:
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
                self._enter(S.REQ_TURNING)
            return

        # ── REQ_TURNING: hover, request TURNING, wait for confirm ──
        # The physical turn must not start until the bridged state
        # topic reports DemoMode.TURNING. We publish zeros (which
        # also serves as a fresh brake to kill residual vx) and
        # re-publish the request at request_repeat_sec.
        if self.state == S.REQ_TURNING:
            self._set_freeze(False)
            self._publish_zero()
            self._request_demo_mode(DemoMode.TURNING)
            if self._demo_mode_is(DemoMode.TURNING):
                self._enter(S.YAW_ALIGN)
            return

        # ── YAW_ALIGN: pure wz, vx forced to 0 ─────────────────
        if self.state == S.YAW_ALIGN:
            # Defensive: if the system mode slid out from under us
            # mid-turn (e.g. system commanded a hold), brake and
            # re-handshake on the next tick.
            if not self._demo_mode_is(DemoMode.TURNING):
                rospy.logwarn("waypoint_follower: DemoMode left TURNING "
                              "mid-rotation (now=%s) — re-requesting",
                              self.current_demo_mode)
                self._enter(S.REQ_TURNING)
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
                    # New waypoint, still in TURNING mode — stay here
                    # and re-snapshot the sweep on the next tick. No
                    # handshake needed (we never left TURNING).
                    self._enter(S.YAW_ALIGN)
                    self._snapshot_yaw_lead()
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
                # Rotation complete — handshake back to fly_straight
                # before commanding any forward motion.
                self._enter(S.REQ_FLY_STRAIGHT)
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

        # ── REQ_FLY_STRAIGHT: hover, request FLY_STRAIGHT, wait ──
        if self.state == S.REQ_FLY_STRAIGHT:
            self._set_freeze(False)
            self._publish_zero()
            self._request_demo_mode(DemoMode.FLY_STRAIGHT)
            if self._demo_mode_is(DemoMode.FLY_STRAIGHT):
                self._enter(S.ADVANCE)
            return

        # ── ADVANCE: pure vx, wz forced to 0 ────────────────────
        if self.state == S.ADVANCE:
            # Defensive: same idea as in YAW_ALIGN. If the system
            # mode drifts off FLY_STRAIGHT, stop driving and re-
            # handshake. We brake first so we don't slam into the
            # next state with residual velocity.
            if not self._demo_mode_is(DemoMode.FLY_STRAIGHT):
                rospy.logwarn("waypoint_follower: DemoMode left "
                              "FLY_STRAIGHT mid-advance (now=%s) — "
                              "braking & re-handshaking",
                              self.current_demo_mode)
                self._enter(S.BRAKE)
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
                    # Stopped → handshake into a new turn.
                    self._enter(S.REQ_TURNING)
            return

        if self.state == S.DONE:
            self._set_freeze(False)
            self._publish_zero()
            # Announce completion to the system. Idempotent: once the
            # system confirms FINISH, _request_demo_mode no-ops on
            # repeats so this doesn't spam the bridge.
            self._request_demo_mode(DemoMode.FINISH)
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
        if self.state == S.TAKEOFF:
            rospy.loginfo("[%-16s] passive — demo_mode=%s (waiting for "
                          "IDLE on %s)",
                          self.state, self.current_demo_mode,
                          self.demo_mode_topic)
            return
        if self.cur_pose is None:
            rospy.loginfo("[%-16s] no /gt_pose yet (subscribed to %s)",
                          self.state, self.t_pose); return
        p = self.cur_pose.position
        yaw = math.degrees(quat_yaw(self.cur_pose.orientation))
        # Status line shows what we publish: vy and vz are always 0.
        extra = ""
        if self.state in (S.REQ_TURNING, S.REQ_FLY_STRAIGHT):
            extra = "  requested=%s  current=%s" % (
                    self.requested_demo_mode, self.current_demo_mode)
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
        rospy.loginfo("[%-16s] demo=%s pose=(%.2f,%.2f,%.2f) yaw=%5.1f° | "
                      "cmd: vx=%.2f wz=%.2f%s",
                      self.state, self.current_demo_mode,
                      p.x, p.y, p.z, yaw,
                      self.last_vx, self.last_wz, extra)

    def _on_shutdown(self):
        # On shutdown, only emit a brake burst if we were ever
        # allowed to publish in the first place. If we never left
        # S.TAKEOFF there is nothing to stop.
        try:
            if self._publishing_allowed():
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