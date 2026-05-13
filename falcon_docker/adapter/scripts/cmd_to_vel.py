#!/usr/bin/env python3
"""
cmd_to_vel.py  (v13 — auto-land on shutdown and on exploration complete)

What FALCON actually publishes on /planning/pos_cmd
───────────────────────────────────────────────────
A quadrotor_msgs/PositionCommand sampled from a non-uniform B-spline:
  header.stamp   ← the instant the spline was evaluated by traj_server
  position       ← spline(t_cur)
  velocity       ← spline'(t_cur)
  acceleration   ← spline''(t_cur)
  yaw, yaw_dot   ← yaw spline(t_cur), yaw spline'(t_cur)
traj_server runs at 100 Hz; this controller runs at 50 Hz; ROS network
latency is ~5–15 ms. So the cached pos_cmd is typically 15–35 ms stale
by the time we act on it, and on a curve we're missing the acc term.

Change vs v12 — auto-land
─────────────────────────
The drone now publishes /<drone_ns>/land in two places:
  (a) replan_cb when /planning/replan == 2 (exploration done).
      Previously this just transitioned to DONE and held a hover.
  (b) on_shutdown handler. Covers every exit path — Ctrl-C of a
      manual roslaunch, batch_runner.py's SIGINT, an exception
      anywhere in the stack that propagates to rospy.shutdown.

Both publish via the existing latched self.land_pub, so a late
subscriber (the bridge connecting after we publish) still receives
the message. Idempotent — sjtu_drone's land state machine ignores
subsequent /land while already descending, and most real-drone
firmwares behave the same.

Change vs v11
─────────────
In ACTIVE state, instead of feeding cmd.position / cmd.velocity straight
into the P+FF law, we extrapolate the reference forward to "now":

  dt = (rospy.Time.now() - cmd.header.stamp) + ctrl_lookahead
  ref_p   = cmd.position     + cmd.velocity*dt + 0.5*cmd.acceleration*dt²
  ref_v   = cmd.velocity     + cmd.acceleration*dt
  ref_yaw = cmd.yaw          + cmd.yaw_dot*dt

This re-samples the same B-spline FALCON sampled, just at the right time
(plus an optional small lookahead to compensate for the drone's actuator
response delay). Acceleration becomes feedforward via ref_v. No gain
changes, no saturation changes — just an honest reference.

dt is clamped to [0, max_extrap_dt] so a stalled FALCON can't make the
extrapolation run away.

Other behavior
──────────────
  * Takeoff command is RE-SENT every second while in TAKING_OFF until
    the drone reports flying.
  * MAPPING_SCAN between HOVER_SETTLE and HOVERING: drone slowly yaws
    360° so the depth camera builds out a local map before exploration.
  * Logs the first real FALCON trajectory it receives.
  * Periodic status print so you immediately see if we're waiting on
    /odom_world (== bridge problem).

State machine:
  WAIT_ODOM ──▶ TAKING_OFF ──▶ HOVER_SETTLE ──▶ MAPPING_SCAN ──▶ HOVERING ──▶ ACTIVE
                                                                              │  ▲
                                                                              ▼  │
                                                                    EMERGENCY_HOVER
"""

import math
import rospy
import tf.transformations as tft

from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int8, Int32

from quadrotor_msgs.msg import PositionCommand


# ─────────────────── Helpers ──────────────────────────────────────────────

def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))

def quat_to_yaw(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

def saturate(v, lim):
    if v >  lim: return  lim
    if v < -lim: return -lim
    return v

def saturate_vec(vx, vy, lim):
    mag = math.hypot(vx, vy)
    if mag <= lim or mag < 1e-9: return vx, vy
    s = lim / mag
    return vx * s, vy * s


# ─────────────────── States ───────────────────────────────────────────────

class S:
    WAIT_ODOM       = "WAIT_ODOM"
    TAKING_OFF      = "TAKING_OFF"
    HOVER_SETTLE    = "HOVER_SETTLE"
    MAPPING_SCAN    = "MAPPING_SCAN"
    HOVERING        = "HOVERING"
    ACTIVE          = "ACTIVE"
    EMERGENCY_HOVER = "EMERGENCY_HOVER"
    DONE            = "DONE"


# ─────────────────── Main node ────────────────────────────────────────────

class CmdToVel:
    def __init__(self):
        rospy.init_node("cmd_to_vel")

        self.drone_ns = rospy.get_param("~drone_ns", "/simple_drone")

        # Takeoff orchestration
        self.auto_takeoff      = rospy.get_param("~auto_takeoff", True)
        self.takeoff_z_thresh  = rospy.get_param("~takeoff_z_thresh", 0.5)
        self.takeoff_timeout   = rospy.get_param("~takeoff_timeout", 30.0)
        self.takeoff_retry_sec = rospy.get_param("~takeoff_retry_sec", 1.0)
        self.hover_settle_sec  = rospy.get_param("~hover_settle_sec", 2.5)

        # Mapping scan
        self.mapping_scan_enabled = rospy.get_param("~mapping_scan_enabled", True)
        self.mapping_yaw_rate     = rospy.get_param("~mapping_yaw_rate", 0.4)
        self.mapping_scan_revs    = rospy.get_param("~mapping_scan_revs", 1.0)

        # Watchdog
        self.cmd_timeout_sec = rospy.get_param("~cmd_timeout_sec", 0.5)

        # pos_cmd sanity (rejects FALCON discontinuity glitches)
        self.pos_cmd_jump_threshold = rospy.get_param(
            "~pos_cmd_jump_threshold", 3.0)
        self.rejected_pos_cmd_count = 0

        # Gains and saturations
        self.Kp_xy   = rospy.get_param("~Kp_xy", 1.0)
        self.Kp_z    = rospy.get_param("~Kp_z",  1.5)
        self.Kp_yaw  = rospy.get_param("~Kp_yaw", 1.5)
        self.vel_xy_sat   = rospy.get_param("~vel_xy_sat", 0.4)
        self.vel_z_sat    = rospy.get_param("~vel_z_sat",  0.4)
        self.yaw_rate_sat = rospy.get_param("~yaw_rate_sat", 0.6)
        self.accel_limit       = rospy.get_param("~accel_limit", 0.6)
        self.yaw_accel_limit   = rospy.get_param("~yaw_accel_limit", 1.0)

        # Predictive reference sampling (v12).
        #   ctrl_lookahead   : seconds added on top of header-stamp staleness.
        #                      Compensates for actuator response delay so the
        #                      drone aims slightly *ahead* of "right now".
        #                      Set to 0.0 to only compensate staleness.
        #   max_extrap_dt    : hard cap on the extrapolation horizon. Protects
        #                      against the case where pos_cmd's clock is wrong
        #                      or where FALCON is briefly stalled.
        self.ctrl_lookahead = rospy.get_param("~ctrl_lookahead", 0.05)
        self.max_extrap_dt  = rospy.get_param("~max_extrap_dt",  0.2)

        self.ctrl_rate_hz      = rospy.get_param("~ctrl_rate_hz", 50.0)
        self.odom_gate_rate_hz = rospy.get_param("~odom_gate_rate_hz", 30.0)

        # ── Auto-land (v13) ─────────────────────────────────────────
        # Defaults to True so every exit path lands. Set to False if
        # you have a separate landing supervisor or want the drone to
        # hold its last hover for inspection.
        self.land_on_shutdown = rospy.get_param("~land_on_shutdown", True)
        self.land_on_explore_done = rospy.get_param("~land_on_explore_done", True)

        self.state = S.TAKING_OFF if self.auto_takeoff else S.WAIT_ODOM
        self.state_entered = rospy.Time.now()
        self.cur_odom = None
        self.last_pos_cmd = None
        self.last_pos_cmd_t = rospy.Time(0)        # receipt time (used for staleness watchdog)
        self.last_pos_cmd_eval_t = rospy.Time(0)   # spline-evaluation time (header.stamp; used for predictive sampling)
        self.first_real_traj = False
        self.takeoff_pose = None
        self.last_takeoff_pub = rospy.Time(0)
        self.takeoff_count = 0
        self.drone_state = None
        self.scan_yaw_target = None
        self.last_vx = self.last_vy = self.last_vz = self.last_wz = 0.0

        # Track whether we already issued /land this run so we don't
        # spam it. sjtu_drone tolerates repeats, but extra publishes
        # are noise in the logs and on the bridge.
        self.landed = False

        # Publishers / subscribers
        self.cmd_vel_pub    = rospy.Publisher(self.drone_ns + "/cmd_vel", Twist, queue_size=1)
        self.takeoff_pub    = rospy.Publisher(self.drone_ns + "/takeoff", Empty, queue_size=1, latch=True)
        self.land_pub       = rospy.Publisher(self.drone_ns + "/land",    Empty, queue_size=1, latch=True)
        self.odom_gated_pub = rospy.Publisher("/odom_world_gated", Odometry, queue_size=10)

        rospy.Subscriber("/odom_world", Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.pos_cmd_cb, queue_size=10)
        rospy.Subscriber("/planning/replan", Int32, self.replan_cb, queue_size=10)
        rospy.Subscriber(self.drone_ns + "/state", Int8, self.drone_state_cb, queue_size=10)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo(
            "cmd_to_vel ready  drone=%s  ctrl=%.0fHz  vel_sat=(%.2f,%.2f)  "
            "yaw_rate_sat=%.2f  mapping_scan=%s  lookahead=%.0fms  "
            "land_on_shutdown=%s  land_on_explore_done=%s",
            self.drone_ns, self.ctrl_rate_hz, self.vel_xy_sat, self.vel_z_sat,
            self.yaw_rate_sat, self.mapping_scan_enabled,
            1000.0 * self.ctrl_lookahead,
            self.land_on_shutdown, self.land_on_explore_done)

        rospy.Timer(rospy.Duration(1.0 / self.ctrl_rate_hz), self.ctrl_loop)
        rospy.Timer(rospy.Duration(1.0 / self.odom_gate_rate_hz), self.odom_gate_loop)
        rospy.Timer(rospy.Duration(2.0), self.status_print)

    # ─────────────────── Subscribers ──────────────────────────────────────

    def odom_cb(self, msg):
        if self.cur_odom is None:
            p = msg.pose.pose.position
            rospy.loginfo("cmd_to_vel: first /odom_world received  pose=(%.2f, %.2f, %.2f)",
                          p.x, p.y, p.z)
        self.cur_odom = msg

    def pos_cmd_cb(self, msg):
        # ── Sanity guard ─────────────────────────────────────────────
        # Drop pos_cmd that contains NaN/Inf or that jumps unreasonably
        # far from current pose. Both indicate FALCON published a
        # bad trajectory (negative duration, evaluated out of range).
        # By NOT updating last_pos_cmd_t, the ACTIVE-state watchdog
        # transitions to EMERGENCY_HOVER after cmd_timeout_sec.
        px, py, pz = msg.position.x, msg.position.y, msg.position.z
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)
                and math.isfinite(msg.yaw)):
            self.rejected_pos_cmd_count += 1
            rospy.logerr_throttle(1.0,
                "cmd_to_vel: pos_cmd contains NaN/Inf "
                "(traj_id=%d) — rejecting [reject_count=%d]",
                msg.trajectory_id, self.rejected_pos_cmd_count)
            return

        if self.cur_odom is not None:
            cp = self.cur_odom.pose.pose.position
            d = math.sqrt((px - cp.x) ** 2 +
                          (py - cp.y) ** 2 +
                          (pz - cp.z) ** 2)
            if d > self.pos_cmd_jump_threshold:
                self.rejected_pos_cmd_count += 1
                rospy.logerr_throttle(1.0,
                    "cmd_to_vel: pos_cmd is %.2fm from odom "
                    "(traj_id=%d, ref=(%.2f,%.2f,%.2f), "
                    "drone=(%.2f,%.2f,%.2f)) — rejecting, likely "
                    "negative-duration trajectory glitch "
                    "[reject_count=%d]",
                    d, msg.trajectory_id, px, py, pz,
                    cp.x, cp.y, cp.z, self.rejected_pos_cmd_count)
                return

        # ── Accept ───────────────────────────────────────────────────
        self.last_pos_cmd = msg
        now = rospy.Time.now()
        self.last_pos_cmd_t = now
        # header.stamp = the instant traj_server evaluated the B-spline.
        # If it is missing/zero (shouldn't happen with FALCON's traj_server,
        # but be defensive), fall back to receipt time. dt will then be ~0
        # and the extrapolation collapses to the legacy behavior.
        if msg.header.stamp.to_sec() > 0.0:
            self.last_pos_cmd_eval_t = msg.header.stamp
        else:
            self.last_pos_cmd_eval_t = now
        if msg.trajectory_id >= 1 and not self.first_real_traj:
            rospy.loginfo("cmd_to_vel: first FALCON trajectory  id=%d  "
                          "ref_start=(%.2f, %.2f, %.2f)  yaw=%.2f",
                          msg.trajectory_id, msg.position.x, msg.position.y,
                          msg.position.z, msg.yaw)
            self.first_real_traj = True

    def replan_cb(self, msg):
        if msg.data == 2 and self.state in (S.ACTIVE, S.HOVERING):
            if self.land_on_explore_done:
                rospy.loginfo(
                    "cmd_to_vel: exploration finished → landing → DONE")
                self._publish_land("exploration finished")
            else:
                rospy.loginfo("cmd_to_vel: exploration finished → DONE (hover)")
            self._enter(S.DONE)

    def drone_state_cb(self, msg):
        self.drone_state = msg.data

    # ─────────────────── State helpers ────────────────────────────────────

    def _enter(self, new_state):
        if new_state != self.state:
            rospy.loginfo("cmd_to_vel: %s → %s", self.state, new_state)
            self.state = new_state
            self.state_entered = rospy.Time.now()

    def _t_in_state(self):
        return (rospy.Time.now() - self.state_entered).to_sec()

    def _publish_land(self, reason):
        """
        Publish to <drone_ns>/land exactly once per run lifecycle.
        sjtu_drone's plugin (and most real-drone firmwares) treat a
        single Empty as 'start descent'; the publisher is latched so
        any late subscriber on the bridge still catches it.
        """
        if self.landed:
            return
        try:
            self.land_pub.publish(Empty())
            self.landed = True
            rospy.loginfo("cmd_to_vel: published /land (reason=%s)", reason)
        except Exception as e:
            # Don't propagate — landing is best-effort; we still need
            # to finish whatever shutdown/transition is in progress.
            rospy.logwarn("cmd_to_vel: land publish failed: %s", e)

    def status_print(self, _):
        if self.state == S.WAIT_ODOM:
            rospy.logwarn_throttle(5.0,
                "cmd_to_vel: still waiting for /odom_world — check that "
                "/simple_drone/gt_pose is being relayed by ros1_bridge")
        elif self.state == S.TAKING_OFF:
            z = self.cur_odom.pose.pose.position.z if self.cur_odom else float('nan')
            rospy.loginfo_throttle(2.0,
                "cmd_to_vel: TAKING_OFF  z=%.2f  drone_state=%s  takeoff_pubs=%d",
                z, str(self.drone_state), self.takeoff_count)
        elif self.rejected_pos_cmd_count > 0 and self.state in (S.ACTIVE, S.EMERGENCY_HOVER):
            rospy.logwarn_throttle(5.0,
                "cmd_to_vel: %d pos_cmd rejected so far "
                "(state=%s) — investigate FALCON optimizer health",
                self.rejected_pos_cmd_count, self.state)

    # ─────────────────── Odom gate ────────────────────────────────────────

    def odom_gate_loop(self, _):
        if self.cur_odom is None: return
        if self.state in (S.WAIT_ODOM, S.TAKING_OFF, S.HOVER_SETTLE, S.MAPPING_SCAN):
            return
        out = self.cur_odom
        out.header.stamp = rospy.Time.now()
        self.odom_gated_pub.publish(out)

    # ─────────────────── Control loop ─────────────────────────────────────

    def ctrl_loop(self, _):
        if self.state == S.WAIT_ODOM:
            if self.cur_odom is not None:
                if self.auto_takeoff:
                    self._enter(S.TAKING_OFF)
                else:
                    self.takeoff_pose = self._copy_pose(self.cur_odom.pose.pose)
                    self._enter(S.HOVERING)

        elif self.state == S.TAKING_OFF:
            now = rospy.Time.now()
            if (now - self.last_takeoff_pub).to_sec() > self.takeoff_retry_sec:
                self.takeoff_pub.publish(Empty())
                self.last_takeoff_pub = now
                self.takeoff_count += 1
                rospy.loginfo("cmd_to_vel: published /takeoff (#%d)", self.takeoff_count)
            self._publish_zero_vel()

            z = self.cur_odom.pose.pose.position.z if self.cur_odom else 0.0
            airborne = (self.drone_state == 1) or (z >= self.takeoff_z_thresh)
            if airborne:
                self._enter(S.HOVER_SETTLE)
            elif self._t_in_state() > self.takeoff_timeout:
                rospy.logerr("cmd_to_vel: takeoff timeout after %d /takeoff publishes; "
                             "continuing", self.takeoff_count)
                self._enter(S.HOVER_SETTLE)

        elif self.state == S.HOVER_SETTLE:
            if self.takeoff_pose is None and self.cur_odom is not None:
                self.takeoff_pose = self._copy_pose(self.cur_odom.pose.pose)
                p = self.takeoff_pose.position
                rospy.loginfo("cmd_to_vel: takeoff pose snapshot at (%.2f, %.2f, %.2f)",
                              p.x, p.y, p.z)
            self._hold_at_pose(self.takeoff_pose)
            if self._t_in_state() > self.hover_settle_sec:
                if self.mapping_scan_enabled and self.cur_odom is not None:
                    self.scan_yaw_target = quat_to_yaw(self.cur_odom.pose.pose.orientation)
                    self._enter(S.MAPPING_SCAN)
                else:
                    self._enter(S.HOVERING)

        elif self.state == S.MAPPING_SCAN:
            self._mapping_scan_step()
            scan_duration = (2.0 * math.pi * self.mapping_scan_revs) / max(
                self.mapping_yaw_rate, 1e-3)
            if self._t_in_state() > scan_duration + 1.0:
                rospy.loginfo("cmd_to_vel: mapping scan complete; opening odom gate")
                self._enter(S.HOVERING)

        elif self.state == S.HOVERING:
            self._hold_at_pose(self.takeoff_pose)
            if self.first_real_traj:
                self._enter(S.ACTIVE)

        elif self.state == S.ACTIVE:
            stale = (rospy.Time.now() - self.last_pos_cmd_t).to_sec() > self.cmd_timeout_sec
            if stale:
                rospy.logwarn_throttle(1.0,
                    "cmd_to_vel: PositionCommand stale (%.2fs) → emergency hover",
                    (rospy.Time.now() - self.last_pos_cmd_t).to_sec())
                self._enter(S.EMERGENCY_HOVER)
            else:
                self._run_pid_from_pos_cmd(self.last_pos_cmd)

        elif self.state == S.EMERGENCY_HOVER:
            if self.last_pos_cmd is not None:
                self._hold_at_pose(self._pose_from_pos_cmd(self.last_pos_cmd))
            else:
                self._publish_zero_vel()
            if (rospy.Time.now() - self.last_pos_cmd_t).to_sec() < self.cmd_timeout_sec:
                self._enter(S.ACTIVE)

        elif self.state == S.DONE:
            # DONE keeps publishing zero velocity. If /land was already
            # issued, the drone's own landing state machine will be
            # overriding cmd_vel — these zeros are harmless. If the
            # operator disabled land_on_explore_done, the zeros hold
            # a hover at the position where DONE was entered.
            self._publish_zero_vel()

        if self.drone_state == 0 and self.state not in (S.WAIT_ODOM, S.DONE):
            rospy.logwarn("cmd_to_vel: drone reports landed externally; resetting")
            self._publish_zero_vel()
            self.takeoff_count = 0
            self.takeoff_pose = None
            self.first_real_traj = False
            self.landed = False     # next run will need its own land()
            self._enter(S.WAIT_ODOM)

    # ─────────────────── Mapping scan ─────────────────────────────────────

    def _mapping_scan_step(self):
        if self.cur_odom is None or self.takeoff_pose is None:
            self._publish_zero_vel()
            return

        dt = 1.0 / self.ctrl_rate_hz
        self.scan_yaw_target = wrap_pi(self.scan_yaw_target +
                                        self.mapping_yaw_rate * dt)

        cur = self.cur_odom.pose.pose
        yaw_cur = quat_to_yaw(cur.orientation)

        ex = self.takeoff_pose.position.x - cur.position.x
        ey = self.takeoff_pose.position.y - cur.position.y
        ez = self.takeoff_pose.position.z - cur.position.z

        vx_w = self.Kp_xy * ex
        vy_w = self.Kp_xy * ey
        vz_w = self.Kp_z  * ez

        eyaw = wrap_pi(self.scan_yaw_target - yaw_cur)
        wz   = self.mapping_yaw_rate + self.Kp_yaw * eyaw

        self._project_and_publish(vx_w, vy_w, vz_w, wz, yaw_cur)

    # ─────────────────── Control primitives ──────────────────────────────

    def _publish_zero_vel(self):
        self._publish_twist(0.0, 0.0, 0.0, 0.0)

    def _hold_at_pose(self, pose):
        if self.cur_odom is None or pose is None:
            self._publish_zero_vel()
            return
        cur = self.cur_odom.pose.pose
        yaw_cur = quat_to_yaw(cur.orientation)
        yaw_des = quat_to_yaw(pose.orientation)
        ex = pose.position.x - cur.position.x
        ey = pose.position.y - cur.position.y
        ez = pose.position.z - cur.position.z
        eyaw = wrap_pi(yaw_des - yaw_cur)
        vx_w = self.Kp_xy * ex
        vy_w = self.Kp_xy * ey
        vz_w = self.Kp_z  * ez
        wz   = self.Kp_yaw * eyaw
        self._project_and_publish(vx_w, vy_w, vz_w, wz, yaw_cur)

    def _run_pid_from_pos_cmd(self, cmd):
        """
        FALCON publishes pos_cmd by sampling its B-spline at time
        cmd.header.stamp. By the time we get here, the spline has moved
        on by  dt = (now - header.stamp) + ctrl_lookahead.  Use the same
        kinematic state the message carries to extrapolate the reference
        forward by dt (a Taylor expansion of the spline using its own
        velocity and acceleration). This:
          (a) corrects the "where on the track am I supposed to be NOW"
              question instead of using the stale published point,
          (b) folds cmd.acceleration into the velocity feedforward, which
              is otherwise unused.

        dt is clamped to [0, max_extrap_dt] so we can't run away if
        FALCON briefly stalls or clocks disagree.
        """
        if self.cur_odom is None or cmd is None:
            self._publish_zero_vel()
            return

        # Predictive sampling horizon
        dt = (rospy.Time.now() - self.last_pos_cmd_eval_t).to_sec() + self.ctrl_lookahead
        if dt < 0.0:
            dt = 0.0
        elif dt > self.max_extrap_dt:
            dt = self.max_extrap_dt

        # Re-sample the spline at "now" (Taylor: x + v*dt + 0.5*a*dt^2)
        half_dt2 = 0.5 * dt * dt
        ref_px = cmd.position.x + cmd.velocity.x * dt + cmd.acceleration.x * half_dt2
        ref_py = cmd.position.y + cmd.velocity.y * dt + cmd.acceleration.y * half_dt2
        ref_pz = cmd.position.z + cmd.velocity.z * dt + cmd.acceleration.z * half_dt2
        ref_vx = cmd.velocity.x + cmd.acceleration.x * dt
        ref_vy = cmd.velocity.y + cmd.acceleration.y * dt
        ref_vz = cmd.velocity.z + cmd.acceleration.z * dt
        ref_yaw     = wrap_pi(cmd.yaw + cmd.yaw_dot * dt)
        ref_yaw_dot = cmd.yaw_dot

        cur = self.cur_odom.pose.pose
        yaw_cur = quat_to_yaw(cur.orientation)
        ex   = ref_px - cur.position.x
        ey   = ref_py - cur.position.y
        ez   = ref_pz - cur.position.z
        eyaw = wrap_pi(ref_yaw - yaw_cur)

        vx_w = ref_vx + self.Kp_xy * ex
        vy_w = ref_vy + self.Kp_xy * ey
        vz_w = ref_vz + self.Kp_z  * ez
        wz   = ref_yaw_dot + self.Kp_yaw * eyaw
        self._project_and_publish(vx_w, vy_w, vz_w, wz, yaw_cur)

    def _project_and_publish(self, vx_w, vy_w, vz_w, wz, yaw_cur):
        vx_w, vy_w = saturate_vec(vx_w, vy_w, self.vel_xy_sat)
        vz_w = saturate(vz_w, self.vel_z_sat)
        wz   = saturate(wz, self.yaw_rate_sat)
        c, s = math.cos(yaw_cur), math.sin(yaw_cur)
        vx_b =  c * vx_w + s * vy_w
        vy_b = -s * vx_w + c * vy_w
        self._publish_twist(vx_b, vy_b, vz_w, wz)

    def _publish_twist(self, vx, vy, vz, wz):
        dt = 1.0 / self.ctrl_rate_hz
        max_dv  = self.accel_limit     * dt
        max_dwz = self.yaw_accel_limit * dt
        vx = self._slew(vx, self.last_vx, max_dv)
        vy = self._slew(vy, self.last_vy, max_dv)
        vz = self._slew(vz, self.last_vz, max_dv)
        wz = self._slew(wz, self.last_wz, max_dwz)
        msg = Twist()
        msg.linear.x = vx; msg.linear.y = vy; msg.linear.z = vz
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)
        self.last_vx, self.last_vy, self.last_vz, self.last_wz = vx, vy, vz, wz

    @staticmethod
    def _slew(target, current, max_step):
        delta = target - current
        if delta >  max_step: return current + max_step
        if delta < -max_step: return current - max_step
        return target

    @staticmethod
    def _copy_pose(pose):
        out = Pose()
        out.position = Point(pose.position.x, pose.position.y, pose.position.z)
        out.orientation = Quaternion(pose.orientation.x, pose.orientation.y,
                                     pose.orientation.z, pose.orientation.w)
        return out

    @staticmethod
    def _pose_from_pos_cmd(cmd):
        p = Pose()
        p.position = Point(cmd.position.x, cmd.position.y, cmd.position.z)
        q = tft.quaternion_from_euler(0.0, 0.0, cmd.yaw)
        p.orientation = Quaternion(q[0], q[1], q[2], q[3])
        return p

    def on_shutdown(self):
        # Order matters here. Publish /land FIRST while the ROS master
        # is still routing messages — this is our last chance to tell
        # the drone to descend. THEN drain cmd_vel with zeros so we
        # don't leave a stale velocity command lingering on the bridge.
        # The drone's land state machine ignores cmd_vel during descent,
        # so the zeros are harmless overlap.
        if self.land_on_shutdown:
            self._publish_land("shutdown")
        try:
            for _ in range(3):
                self._publish_zero_vel()
                rospy.sleep(0.02)
        except Exception:
            pass


if __name__ == "__main__":
    try:
        CmdToVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass