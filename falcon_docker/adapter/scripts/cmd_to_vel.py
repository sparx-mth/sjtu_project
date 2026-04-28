#!/usr/bin/env python3
"""
cmd_to_vel.py
  * Takeoff command is RE-SENT every second while in TAKING_OFF until
    the drone reports flying. Handles the case where the bridge wasn't
    ready when the first /takeoff was published.
  * NEW state MAPPING_SCAN between HOVER_SETTLE and HOVERING: the drone
    slowly yaws 360° while hovering, so the depth camera builds out a
    full local map before exploration begins. Prevents FALCON from
    planning into mostly-unknown space.
  * Logs the moment FALCON sends its first real trajectory, with its
    starting xyz, so you can compare against drone position to spot a
    z-bias problem if the gating logic ever fails again.
  * Periodic status print so you immediately see if cmd_to_vel is
    waiting on /odom_world (== bridge problem).

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

        self.ctrl_rate_hz      = rospy.get_param("~ctrl_rate_hz", 50.0)
        self.odom_gate_rate_hz = rospy.get_param("~odom_gate_rate_hz", 30.0)

        # Internal state
        self.state = S.WAIT_ODOM
        self.state_entered = rospy.Time.now()
        self.cur_odom = None
        self.last_pos_cmd = None
        self.last_pos_cmd_t = rospy.Time(0)
        self.first_real_traj = False
        self.takeoff_pose = None
        self.last_takeoff_pub = rospy.Time(0)
        self.takeoff_count = 0
        self.drone_state = None
        self.scan_yaw_target = None
        self.last_vx = self.last_vy = self.last_vz = self.last_wz = 0.0

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
            "yaw_rate_sat=%.2f  mapping_scan=%s",
            self.drone_ns, self.ctrl_rate_hz, self.vel_xy_sat, self.vel_z_sat,
            self.yaw_rate_sat, self.mapping_scan_enabled)

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
        self.last_pos_cmd_t = rospy.Time.now()
        if msg.trajectory_id >= 1 and not self.first_real_traj:
            rospy.loginfo("cmd_to_vel: first FALCON trajectory  id=%d  "
                          "ref_start=(%.2f, %.2f, %.2f)  yaw=%.2f",
                          msg.trajectory_id, msg.position.x, msg.position.y,
                          msg.position.z, msg.yaw)
            self.first_real_traj = True

    def replan_cb(self, msg):
        if msg.data == 2 and self.state in (S.ACTIVE, S.HOVERING):
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
            self._publish_zero_vel()

        if self.drone_state == 0 and self.state not in (S.WAIT_ODOM, S.DONE):
            rospy.logwarn("cmd_to_vel: drone reports landed externally; resetting")
            self._publish_zero_vel()
            self.takeoff_count = 0
            self.takeoff_pose = None
            self.first_real_traj = False
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
        if self.cur_odom is None or cmd is None:
            self._publish_zero_vel()
            return
        cur = self.cur_odom.pose.pose
        yaw_cur = quat_to_yaw(cur.orientation)
        ex = cmd.position.x - cur.position.x
        ey = cmd.position.y - cur.position.y
        ez = cmd.position.z - cur.position.z
        eyaw = wrap_pi(cmd.yaw - yaw_cur)
        vx_w = cmd.velocity.x + self.Kp_xy * ex
        vy_w = cmd.velocity.y + self.Kp_xy * ey
        vz_w = cmd.velocity.z + self.Kp_z  * ez
        wz   = cmd.yaw_dot    + self.Kp_yaw * eyaw
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