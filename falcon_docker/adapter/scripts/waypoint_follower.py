#!/usr/bin/env python3
"""
waypoint_follower.py  (v4 — forked from cmd_to_vel.py)

Same takeoff/control infrastructure as the working cmd_to_vel.py:
  • 50 Hz control loop
  • _publish_twist with slew limiting on vx/vy/vz/wz every tick
    (the thing sjtu_drone actually needs to actuate)
  • No /posctrl publishes (matches cmd_to_vel; drone keeps its default)
  • cmd_vel published every tick of every state — no gaps

What's replaced: the FALCON-trajectory ACTIVE state. Instead, after
takeoff and hover-settle we go to WAIT_PATH and consume /path/waypoints
with a YAW-then-X state machine:

  WAIT_POSE → TAKING_OFF → HOVER_SETTLE → WAIT_PATH → YAW_ALIGN ⇄ ADVANCE → DONE

  YAW_ALIGN:  vx=vy=vz=0, wz=±yaw_rate until |yaw_err| < yaw_radius.
              freezes the sensor_gate so FALCON's voxels don't update
              from rotating depth frames.
  ADVANCE:    vx_b=vel_x (body forward), wz=0, optional vz alt-hold,
              until hypot(dx,dy) < pos_radius. unfreezes the gate.
              if yaw drifts >2× threshold, falls back to YAW_ALIGN.

Body-frame command convention (matches sjtu_drone): linear.x is body
forward. We don't do a world→body projection because the user's spec
forbids vy_b: we drive purely along the drone's heading, and YAW_ALIGN
is responsible for putting that heading on the next waypoint first.
"""
import math
import rospy
import tf.transformations as tft

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool, Int8


def wrap_pi(a):    return math.atan2(math.sin(a), math.cos(a))
def quat_yaw(q):   return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
def saturate(v, lim):
    if v >  lim: return  lim
    if v < -lim: return -lim
    return v


class S:
    WAIT_POSE    = "WAIT_POSE"
    TAKING_OFF   = "TAKING_OFF"
    HOVER_SETTLE = "HOVER_SETTLE"
    WAIT_PATH    = "WAIT_PATH"
    YAW_ALIGN    = "YAW_ALIGN"
    ADVANCE      = "ADVANCE"
    DONE         = "DONE"


class WaypointFollower:
    def __init__(self):
        rospy.init_node("waypoint_follower")
        G = rospy.get_param

        self.drone_ns = G("~drone_ns", "/simple_drone")

        # Speeds
        self.vel_x    = float(G("~vel_x",    0.4))   # m/s, body-x
        self.yaw_rate = float(G("~yaw_rate", 0.7))   # rad/s

        # Acquisition radii
        self.pos_radius = float(G("~pos_acquisition_radius", 0.35))
        self.yaw_radius = float(G("~yaw_acquisition_radius", 0.10))

        # Takeoff / hover (mirrors cmd_to_vel)
        self.auto_takeoff      = bool (G("~auto_takeoff",      True))
        self.takeoff_z_thresh  = float(G("~takeoff_z_thresh",  0.5))
        self.takeoff_timeout   = float(G("~takeoff_timeout",   30.0))
        self.takeoff_retry_sec = float(G("~takeoff_retry_sec", 1.0))
        self.hover_settle_sec  = float(G("~hover_settle_sec",  2.5))

        # Slew + saturations (same defaults as cmd_to_vel's working values)
        self.vel_xy_sat       = float(G("~vel_xy_sat",       1.25))
        self.vel_z_sat        = float(G("~vel_z_sat",        1.25))
        self.yaw_rate_sat     = float(G("~yaw_rate_sat",     2.4))
        self.accel_limit      = float(G("~accel_limit",      1.5))
        self.yaw_accel_limit  = float(G("~yaw_accel_limit",  2.0))

        # Behaviour
        self.freeze_during_yaw = bool (G("~freeze_during_yaw", True))
        self.maintain_altitude = bool (G("~maintain_altitude", True))
        self.cruise_z          = float(G("~cruise_z",          1.0))
        self.alt_kp            = float(G("~alt_kp",            0.6))
        self.skip_initial_eps  = float(G("~skip_initial_eps",  0.5))
        self.ctrl_rate_hz      = float(G("~ctrl_rate_hz",      50.0))
        self.status_hz         = float(G("~status_hz",         1.0))

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
        # Last commanded values (for slew memory across ticks)
        self.last_vx = self.last_vy = self.last_vz = self.last_wz = 0.0

        # Topics (resolved once for clean logs)
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

        rospy.Subscriber(self.t_pose,   Pose, self._pose_cb,   queue_size=10)
        rospy.Subscriber(self.t_dstate, Int8, self._dstate_cb, queue_size=10)
        rospy.Subscriber(self.t_path,   Path, self._path_cb,   queue_size=1)

        rospy.on_shutdown(self._on_shutdown)
        rospy.Timer(rospy.Duration(1.0 / self.ctrl_rate_hz), self._ctrl_loop)
        rospy.Timer(rospy.Duration(1.0 / self.status_hz),    self._status)

        rospy.loginfo("=" * 64)
        rospy.loginfo("waypoint_follower v4 ready (cmd_to_vel-style infra)")
        rospy.loginfo("  drone_ns = %s",  self.drone_ns)
        rospy.loginfo("  pose in  = %s",  self.t_pose)
        rospy.loginfo("  cmd_vel  = %s",  self.t_cmd_vel)
        rospy.loginfo("  takeoff  = %s",  self.t_takeoff)
        rospy.loginfo("  path in  = %s",  self.t_path)
        rospy.loginfo("  ctrl=%dHz  vel_x=%.2f  yaw_rate=%.2f  "
                      "pos_rad=%.2f  yaw_rad=%.2f",
                      int(self.ctrl_rate_hz), self.vel_x, self.yaw_rate,
                      self.pos_radius, self.yaw_radius)
        rospy.loginfo("  slew: accel=%.2fm/s²  yaw_accel=%.2frad/s²  "
                      "sat: xy=%.2f z=%.2f wz=%.2f",
                      self.accel_limit, self.yaw_accel_limit,
                      self.vel_xy_sat, self.vel_z_sat, self.yaw_rate_sat)
        rospy.loginfo("  cruise_z=%.2f  freeze_yaw=%s  maintain_alt=%s",
                      self.cruise_z, self.freeze_during_yaw,
                      self.maintain_altitude)
        rospy.loginfo("=" * 64)

    # ─── Callbacks ───────────────────────────────────────────────
    def _pose_cb(self, msg):
        if self.cur_pose is None:
            rospy.loginfo("waypoint_follower: first /gt_pose  pose=(%.2f,%.2f,%.2f)",
                          msg.position.x, msg.position.y, msg.position.z)
        self.cur_pose = msg

    def _dstate_cb(self, msg): self.drone_state = msg.data

    def _path_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            rospy.logwarn("waypoint_follower: empty path"); return
        if self.cur_pose is not None and self.skip_initial_eps > 0:
            cx, cy = self.cur_pose.position.x, self.cur_pose.position.y
            while len(pts) > 1 and math.hypot(pts[0][0]-cx,
                                              pts[0][1]-cy) < self.skip_initial_eps:
                pts.pop(0)
        self.path_xy = pts
        self.wp_idx  = 0
        rospy.loginfo("waypoint_follower: NEW PATH  %d waypoints  "
                      "first=(%.2f,%.2f)  last=(%.2f,%.2f)",
                      len(pts), pts[0][0], pts[0][1], pts[-1][0], pts[-1][1])
        # Mid-flight retargeting: if we're past WAIT_PATH (currently
        # flying or finished), restart path-following from waypoint 0 of
        # the new path. The DONE case is the important one — without
        # this, a click after reaching a goal would update the plan but
        # the follower would stay parked. _enter is a no-op when state
        # already equals YAW_ALIGN, so this is safe to call always.
        if self.state in (S.YAW_ALIGN, S.ADVANCE, S.DONE):
            self._enter(S.YAW_ALIGN)

    # ─── Helpers ─────────────────────────────────────────────────
    def _enter(self, new):
        if new != self.state:
            rospy.loginfo("waypoint_follower: %s → %s", self.state, new)
            self.state   = new
            self.t_state = rospy.Time.now()

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

    def _publish_twist(self, vx, vy, vz, wz):
        """
        EXACT pattern from cmd_to_vel.py — slew + saturate, then publish.
        Called every tick; never skipped. This is what makes sjtu_drone
        actuate cleanly.
        """
        vx = saturate(vx, self.vel_xy_sat)
        vy = saturate(vy, self.vel_xy_sat)
        vz = saturate(vz, self.vel_z_sat)
        wz = saturate(wz, self.yaw_rate_sat)
        dt = 1.0 / self.ctrl_rate_hz
        max_dv  = self.accel_limit     * dt
        max_dwz = self.yaw_accel_limit * dt
        vx = self._slew(vx, self.last_vx, max_dv)
        vy = self._slew(vy, self.last_vy, max_dv)
        vz = self._slew(vz, self.last_vz, max_dv)
        wz = self._slew(wz, self.last_wz, max_dwz)
        m = Twist()
        m.linear.x = vx; m.linear.y = vy; m.linear.z = vz
        m.angular.z = wz
        self.cmd_vel_pub.publish(m)
        self.last_vx, self.last_vy, self.last_vz, self.last_wz = vx, vy, vz, wz

    def _publish_zero(self):
        self._publish_twist(0.0, 0.0, 0.0, 0.0)

    # ─── Control loop ────────────────────────────────────────────
    def _ctrl_loop(self, _):
        # WAIT_POSE — publish nothing (publishers may not be wired yet)
        if self.state == S.WAIT_POSE:
            if self.cur_pose is not None:
                self._enter(S.TAKING_OFF if self.auto_takeoff
                            else S.HOVER_SETTLE)
            return

        if self.cur_pose is None:
            return  # paranoia

        # TAKING_OFF — re-publish /takeoff every second; pump zeros
        if self.state == S.TAKING_OFF:
            now = rospy.Time.now()
            if (now - self.last_takeoff).to_sec() > self.takeoff_retry_sec:
                self.takeoff_pub.publish(Empty())
                self.last_takeoff   = now
                self.takeoff_count += 1
                rospy.loginfo("waypoint_follower: published /takeoff (#%d)",
                              self.takeoff_count)
            self._publish_zero()
            airborne = (self.drone_state == 1
                        or self.cur_pose.position.z >= self.takeoff_z_thresh)
            if airborne:
                self._enter(S.HOVER_SETTLE)
            elif self._t_in() > self.takeoff_timeout:
                rospy.logerr("waypoint_follower: takeoff timeout, continuing")
                self._enter(S.HOVER_SETTLE)
            return

        # HOVER_SETTLE — snapshot takeoff pose, hold zeros for `hover_settle_sec`
        if self.state == S.HOVER_SETTLE:
            if self.takeoff_pose is None:
                self.takeoff_pose = self._copy_pose(self.cur_pose)
                p = self.takeoff_pose.position
                rospy.loginfo("waypoint_follower: takeoff snapshot "
                              "(%.2f,%.2f,%.2f)", p.x, p.y, p.z)
            self._publish_zero()  # let the drone settle on its own AP
            if self._t_in() > self.hover_settle_sec:
                self._enter(S.WAIT_PATH)
            return

        # WAIT_PATH — hold zero until /path/waypoints arrives
        if self.state == S.WAIT_PATH:
            self._publish_zero()
            if self.path_xy and self.wp_idx < len(self.path_xy):
                self._enter(S.YAW_ALIGN)
            return

        # YAW_ALIGN — only wz; freeze the gate so voxels don't update
        if self.state == S.YAW_ALIGN:
            tx, ty = self.path_xy[self.wp_idx]
            cx, cy = self.cur_pose.position.x, self.cur_pose.position.y
            if math.hypot(tx - cx, ty - cy) < self.pos_radius:
                self._set_freeze(False)
                self._publish_zero()  # always publish, even on transition
                self._next_waypoint()
                return
            yaw_des = math.atan2(ty - cy, tx - cx)
            yaw_cur = quat_yaw(self.cur_pose.orientation)
            eyaw    = wrap_pi(yaw_des - yaw_cur)
            if abs(eyaw) < self.yaw_radius:
                self._set_freeze(False)
                self._publish_zero()
                self._enter(S.ADVANCE)
                return
            self._set_freeze(self.freeze_during_yaw)
            # Trapezoidal yaw profile: command the largest |wz| from which
            # we can still decelerate to zero by the time eyaw=0, capped at
            # yaw_rate. This eliminates overshoot:
            #   v² = 2·a·d  →  v_max = √(2·yaw_accel_limit·|eyaw|)
            # Below the crossover |eyaw| = yaw_rate²/(2·yaw_accel_limit) the
            # profile linearly decelerates; above it, we run at full speed.
            stop_wz = math.sqrt(2.0 * self.yaw_accel_limit * abs(eyaw))
            wz_mag  = min(self.yaw_rate, stop_wz)
            wz      = wz_mag if eyaw > 0 else -wz_mag
            self._publish_twist(0.0, 0.0, 0.0, wz)
            return

        # ADVANCE — only vx_b (and optional vz alt-hold)
        if self.state == S.ADVANCE:
            self._set_freeze(False)
            tx, ty = self.path_xy[self.wp_idx]
            cx, cy = self.cur_pose.position.x, self.cur_pose.position.y
            if math.hypot(tx - cx, ty - cy) < self.pos_radius:
                self._publish_zero()
                self._next_waypoint()
                return
            yaw_des = math.atan2(ty - cy, tx - cx)
            yaw_cur = quat_yaw(self.cur_pose.orientation)
            if abs(wrap_pi(yaw_des - yaw_cur)) > 2.0 * self.yaw_radius:
                rospy.loginfo("waypoint_follower: yaw drift in ADVANCE → re-align")
                self._publish_zero()
                self._enter(S.YAW_ALIGN)
                return
            vz = 0.0
            if self.maintain_altitude:
                vz = saturate(self.alt_kp *
                              (self.cruise_z - self.cur_pose.position.z),
                              self.vel_z_sat)
            self._publish_twist(self.vel_x, 0.0, vz, 0.0)
            return

        # DONE — hold last waypoint with zeros
        if self.state == S.DONE:
            self._set_freeze(False)
            self._publish_zero()
            return

    def _next_waypoint(self):
        self.wp_idx += 1
        if self.wp_idx >= len(self.path_xy):
            rospy.loginfo("waypoint_follower: GOAL REACHED")
            self._enter(S.DONE)
        else:
            self._enter(S.YAW_ALIGN)

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

    # ─── 1 Hz status (every state) ──────────────────────────────
    def _status(self, _):
        if self.cur_pose is None:
            rospy.loginfo("[%-12s] no /gt_pose yet (subscribed to %s)",
                          self.state, self.t_pose); return
        p = self.cur_pose.position
        yaw = math.degrees(quat_yaw(self.cur_pose.orientation))
        cmd = (self.last_vx, self.last_vy, self.last_vz, self.last_wz)
        extra = ""
        if self.state == S.TAKING_OFF:
            extra = "  takeoff_pubs=%d  z=%.2f" % (self.takeoff_count, p.z)
        elif self.state == S.WAIT_PATH:
            extra = "  (no path yet on %s)" % self.t_path
        elif self.state in (S.YAW_ALIGN, S.ADVANCE) and self.path_xy:
            tx, ty = self.path_xy[self.wp_idx]
            d  = math.hypot(tx - p.x, ty - p.y)
            ey = math.degrees(wrap_pi(math.atan2(ty - p.y, tx - p.x)
                              - quat_yaw(self.cur_pose.orientation)))
            extra = ("  wp=%d/%d target=(%.2f,%.2f) d=%.2fm yaw_err=%5.1f°"
                     % (self.wp_idx + 1, len(self.path_xy), tx, ty, d, ey))
        rospy.loginfo("[%-12s] pose=(%.2f,%.2f,%.2f) yaw=%5.1f° | "
                      "cmd: vx=%.2f vy=%.2f vz=%.2f wz=%.2f%s",
                      self.state, p.x, p.y, p.z, yaw,
                      cmd[0], cmd[1], cmd[2], cmd[3], extra)

    def _on_shutdown(self):
        try:
            for _ in range(5):
                self._publish_zero()
                rospy.sleep(0.02)
        except Exception:
            pass


if __name__ == "__main__":
    try:
        WaypointFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass