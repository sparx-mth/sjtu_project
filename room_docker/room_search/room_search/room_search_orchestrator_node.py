#!/usr/bin/env python3
"""
room_search_orchestrator_node.py — go to a room, rotate to find a target,
                                   visually close in from the RGB image
                                   alone, land.

State machine
-------------
  WAIT_INIT          have we seen pose yet? if so → NAV_TO_ROOM
  NAV_TO_ROOM        publish (room_center_x, room_center_y) to /waypoint_nav/goal,
                     watch drone pose, wait until within nav_arrival_radius_m.
                     Re-publish the goal periodically in case the path planner
                     hadn't started up yet on first publish.
                     → ROTATE_AND_SEARCH
  ROTATE_AND_SEARCH  take over /cmd_vel (publish external_ctrl=True so the
                     ROS1 waypoint_follower stops emitting zeros). Spin in
                     place at rotation_rate_rad_s while watching
                     /perception/detections for the target class.
                     - hit  → VISUAL_APPROACH
                     - timeout after max_rotation_revs full revolutions
                       → GIVE_UP
  VISUAL_APPROACH    KEEP external_ctrl=True. Closed-loop on the RGB
                     bounding box only — no depth, no localisation.
                     PLATFORM INVARIANT: every published Twist has
                       linear.y = linear.z = 0   (no roll, no climb)
                       (linear.x = 0)  XOR  (angular.z = 0)
                     i.e. either pure-yaw OR pure-forward, never both.
                     The loop alternates two sub-modes with hysteresis:
                       * YAW:     publishing (0, wz)  until |x_off| drops
                                  below visual_yaw_deadband_exit
                       * ADVANCE: publishing (vx, 0)  until |x_off| rises
                                  above visual_yaw_deadband_enter
                     vx is ramped down with bbox area (1/d² proxy for
                     distance). A single (0,0) brake tick is emitted on
                     every mode switch.
                     Exits when bbox area fraction >= visual_land_area_frac.
                     → LAND
  LAND               publish a short burst of /<drone_ns>/land (Empty).
                     The sjtu_drone landing controller owns the descent.
                     → DONE
  DONE               idle.
  GIVE_UP            idle, log a warning.

Why RGB-only (no depth)
-----------------------
Depth sensors fail in glare, on glass, on dark/low-texture surfaces,
and at very close range — all common when "landing on" a desk object.
The bbox itself is a clean, monotonic proxy for proximity: at distance
d, the bbox area fraction grows ~ 1/d² (a flat object viewed
perpendicularly through a pinhole). We don't need an absolute distance
to know we're "as close as we can get" — we just need a threshold on
how much of the image the target is filling. The orchestrator picks the
highest-confidence detection matching the target class out of the
/perception/detections stream (which is the YOLO detector running on
the RGB camera topic) and drives the loop on bbox geometry alone.

What we expect on the wire
--------------------------
  IN  (ROS2 native, from perception_docker/semantic_mapper):
    /target_seen           std_msgs/Bool                  TRANSIENT_LOCAL
    /target_seen/info      std_msgs/String                TRANSIENT_LOCAL (JSON)
    /perception/detections vision_msgs/Detection2DArray   sensor QoS
    /perception/objects    std_msgs/String                TRANSIENT_LOCAL (JSON)
                                                          (diagnostic only)

  IN  (must be bridged from ROS1):
    pose_topic            nav_msgs/Odometry (default /odom_world)

  OUT (must be bridged to ROS1):
    /waypoint_nav/goal                   geometry_msgs/Point      (NAV_TO_ROOM)
    /<drone_ns>/cmd_vel                  geometry_msgs/Twist      (rotation + visual approach)
    /<drone_ns>/land                     std_msgs/Empty           (LAND)
    /waypoint_follower/external_ctrl     std_msgs/Bool (latched)  (toggled by state machine)
"""

import json
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray


class S:
    WAIT_INIT         = "WAIT_INIT"
    NAV_TO_ROOM       = "NAV_TO_ROOM"
    ROTATE_AND_SEARCH = "ROTATE_AND_SEARCH"
    VISUAL_APPROACH   = "VISUAL_APPROACH"
    LAND              = "LAND"
    DONE              = "DONE"
    GIVE_UP           = "GIVE_UP"


def _now_s(node: Node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


def _saturate(v: float, lim: float) -> float:
    if v >  lim: return  lim
    if v < -lim: return -lim
    return v


def _clamp01(v: float) -> float:
    if v < 0.0: return 0.0
    if v > 1.0: return 1.0
    return v


class RoomSearchOrchestrator(Node):
    def __init__(self):
        super().__init__("room_search_orchestrator")

        P = self.declare_parameter

        # ── Mission ───────────────────────────────────────────────
        P("room_center_x",        4.0)
        P("room_center_y",        5.0)
        P("target_object",        "keyboard")
        P("drone_ns",             "/simple_drone")

        # ── Topic names ──────────────────────────────────────────
        P("pose_topic",           "/odom_world")
        P("pose_type",            "odometry")  # odometry | pose_stamped | pose
        P("detections_topic",     "/perception/detections")

        # ── Phase radii / timings ────────────────────────────────
        P("nav_arrival_radius_m",   0.50)
        P("nav_goal_republish_s",   3.0)

        P("rotation_rate_rad_s",    0.5)
        P("rotation_ctrl_hz",      20.0)
        P("max_rotation_revs",      2.0)

        # ── Visual servoing on the RGB bbox only ─────────────────
        # RGB image dimensions used by the detector. The bbox centre is
        # in RGB pixels; we normalise by RGB width/height. Defaults are
        # the sjtu_drone front camera (640x360). For a real camera,
        # override at launch (or wire up CameraInfo later).
        P("rgb_image_width",        640)
        P("rgb_image_height",       360)

        # Yaw P-gain on normalised x-offset ∈ [-1, +1].
        # wz = -kp * x_off (negative because ROS body-frame +z yaws CCW,
        # which shifts the image content LEFTWARDS — so target right of
        # centre needs wz < 0 to recentre).
        P("visual_kp_yaw",          0.9)
        P("visual_max_yaw_rate",    0.6)   # rad/s saturation
        # Hysteresis on the YAW ↔ ADVANCE switch. While in ADVANCE we
        # stay until |x_off| > deadband_enter; once in YAW we stay until
        # |x_off| < deadband_exit. exit < enter prevents flapping near
        # the threshold, which would oscillate the underlying flight
        # controller and (on a platform that only accepts one of the two
        # commands at a time) introduce dead time on every flip.
        P("visual_yaw_deadband_enter", 0.20)  # ~±10% off centre
        P("visual_yaw_deadband_exit",  0.08)  # tighter exit

        # Bbox-area-fraction based forward velocity.
        # bbox_area_frac = (bbox_w * bbox_h) / (rgb_W * rgb_H). At
        # distance d for a flat target viewed perpendicularly, this
        # grows ~ 1/d², so larger area_frac = closer.
        #   area_frac < slowdown_start  → vx = vx_max
        #   slowdown_start ≤ area_frac < land_area_frac
        #                                → vx = vx_max * (land - area) /
        #                                                 (land - slowdown)
        #   area_frac ≥ land_area_frac  → → LAND
        # Defaults are calibrated for a keyboard-sized target on the
        # sjtu_drone front camera: ~12% of the image roughly corresponds
        # to ~25–30 cm range. Tune with rosparam after a test run.
        P("visual_vx_max",                 0.20)
        P("visual_slowdown_area_frac",     0.03)
        P("visual_land_area_frac",         0.12)

        # Lost-target handling.
        P("visual_lost_hover_s",     0.6)
        P("visual_giveup_s",        15.0)
        # Hard fallback: if VISUAL_APPROACH lingers past this, LAND
        # anyway (small target / wide FOV / bad lighting can mean the
        # area threshold is never reached).
        P("visual_approach_timeout_s", 90.0)

        P("visual_ctrl_hz",         15.0)  # visual servo loop rate

        # ── Land burst ───────────────────────────────────────────
        P("land_burst_count",        10)
        P("land_burst_hz",          10.0)

        P("tick_hz",                 5.0)

        g = lambda n: self.get_parameter(n).value
        self.room_xy: Tuple[float, float] = (
            float(g("room_center_x")), float(g("room_center_y")))
        self.target_label  = str(g("target_object")).strip().lower()
        self.drone_ns      = str(g("drone_ns"))
        self.pose_topic    = str(g("pose_topic"))
        self.pose_type     = str(g("pose_type")).lower()
        self.det_topic     = str(g("detections_topic"))

        self.nav_arr_r     = float(g("nav_arrival_radius_m"))
        self.nav_republ_s  = float(g("nav_goal_republish_s"))

        self.rot_rate      = float(g("rotation_rate_rad_s"))
        self.rot_hz        = float(g("rotation_ctrl_hz"))
        self.max_revs      = float(g("max_rotation_revs"))

        self.rgb_W         = int(g("rgb_image_width"))
        self.rgb_H         = int(g("rgb_image_height"))

        self.kp_yaw         = float(g("visual_kp_yaw"))
        self.max_wz         = float(g("visual_max_yaw_rate"))
        self.yaw_enter      = float(g("visual_yaw_deadband_enter"))
        self.yaw_exit       = float(g("visual_yaw_deadband_exit"))
        if self.yaw_exit > self.yaw_enter:
            self.get_logger().warn(
                f"visual_yaw_deadband_exit ({self.yaw_exit:.2f}) > "
                f"visual_yaw_deadband_enter ({self.yaw_enter:.2f}); "
                "swapping to maintain hysteresis")
            self.yaw_enter, self.yaw_exit = self.yaw_exit, self.yaw_enter
        self.vx_max        = float(g("visual_vx_max"))
        self.area_slow     = float(g("visual_slowdown_area_frac"))
        self.area_land     = float(g("visual_land_area_frac"))
        self.lost_hover_s  = float(g("visual_lost_hover_s"))
        self.giveup_s      = float(g("visual_giveup_s"))
        self.appr_to_s     = float(g("visual_approach_timeout_s"))
        self.visual_hz     = float(g("visual_ctrl_hz"))

        self.land_n        = int(g("land_burst_count"))
        self.land_hz       = float(g("land_burst_hz"))
        self.tick_hz       = float(g("tick_hz"))

        # ── State ────────────────────────────────────────────────
        self.state         = S.WAIT_INIT
        self.t_state       = _now_s(self)
        self.cur_xy: Optional[Tuple[float, float]] = None
        # World XY of the matched object, kept for diagnostics only —
        # the visual approach loop does not consume it.
        self.target_xy: Optional[Tuple[float, float]] = None
        self.target_oid: Optional[int] = None
        self.target_seen   = False

        self.last_nav_pub  = 0.0
        self.rot_yaw_acc   = 0.0    # cumulative rotation, rad (for max_revs)

        # Latest matched detection bbox (in RGB pixel coords) + arrival time.
        # (cx, cy, w, h, score) in RGB pixel coords.
        self.last_det: Optional[Tuple[float, float, float, float, float]] = None
        self.last_det_t: float = 0.0
        self.last_visual_acquired_t: float = 0.0  # last time we had a det

        # VISUAL_APPROACH sub-state. "YAW" = publishing (0, wz);
        # "ADVANCE" = publishing (vx, 0). Set on entry to VISUAL_APPROACH.
        self.visual_mode: str = "YAW"

        self._land_pubs_left = 0
        self._land_timer = None

        # ── QoS ──────────────────────────────────────────────────
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=5)

        # ── Publishers (bridged to ROS1) ─────────────────────────
        self.pub_goal = self.create_publisher(
            Point, "/waypoint_nav/goal", 1)
        self.pub_cmd  = self.create_publisher(
            Twist, self.drone_ns + "/cmd_vel", 1)
        self.pub_land = self.create_publisher(
            Empty, self.drone_ns + "/land", 1)
        self.pub_ext  = self.create_publisher(
            Bool, "/waypoint_follower/external_ctrl", latched)

        self.pub_ext.publish(Bool(data=False))
        self._ext_state = False

        # ── Subscribers ──────────────────────────────────────────
        if self.pose_type == "odometry":
            self.create_subscription(
                Odometry, self.pose_topic, self._odom_cb, sensor)
        elif self.pose_type == "pose_stamped":
            self.create_subscription(
                PoseStamped, self.pose_topic, self._ps_cb, sensor)
        elif self.pose_type == "pose":
            self.create_subscription(
                Pose, self.pose_topic, self._pose_cb, sensor)
        else:
            self.get_logger().fatal(
                f"pose_type={self.pose_type!r} not in "
                "{'odometry','pose_stamped','pose'}")
            raise RuntimeError("bad pose_type")

        # Target signals — gate ROTATE → VISUAL transition.
        self.create_subscription(
            Bool,   "/target_seen",        self._seen_cb,    latched)
        self.create_subscription(
            String, "/target_seen/info",   self._info_cb,    latched)
        # Diagnostic only — bbox area drives the visual loop.
        self.create_subscription(
            String, "/perception/objects", self._objects_cb, latched)

        # The one stream the visual loop actually depends on.
        self.create_subscription(
            Detection2DArray, self.det_topic, self._det_cb, sensor)

        # ── Timers ───────────────────────────────────────────────
        self.create_timer(1.0 / self.tick_hz, self._tick)
        self.create_timer(1.0 / self.rot_hz, self._rotate_tick)
        self.create_timer(1.0 / self.visual_hz, self._visual_tick)
        self.create_timer(5.0, self._hb)

        self.get_logger().info("=" * 64)
        self.get_logger().info(
            "room_search_orchestrator ready  (RGB-only visual close-in)")
        self.get_logger().info(
            f"  room_center = ({self.room_xy[0]:.2f}, {self.room_xy[1]:.2f}) m")
        self.get_logger().info(f"  target_object = {self.target_label!r}")
        self.get_logger().info(
            f"  drone_ns = {self.drone_ns!r}   "
            f"pose = {self.pose_topic} ({self.pose_type})")
        self.get_logger().info(
            f"  detections = {self.det_topic}   "
            f"rgb_image = {self.rgb_W}x{self.rgb_H}")
        self.get_logger().info(
            f"  nav_arrival_radius={self.nav_arr_r:.2f}m  "
            f"rot_rate={self.rot_rate:.2f}rad/s  max_revs={self.max_revs:.1f}")
        self.get_logger().info(
            f"  visual: vx_max={self.vx_max:.2f}m/s  kp_yaw={self.kp_yaw:.2f}  "
            f"yaw_deadband enter/exit={self.yaw_enter:.2f}/{self.yaw_exit:.2f}  "
            f"slowdown@area={self.area_slow:.3f}  "
            f"land@area={self.area_land:.3f}")
        self.get_logger().info(
            "  platform invariant: every Twist has vy=vz=0 AND "
            "(vx=0 XOR wz=0)")
        self.get_logger().info("=" * 64)

    # ─── State helpers ──────────────────────────────────────────────
    def _enter(self, new: str):
        if new == self.state:
            return
        self.get_logger().info(
            f"room_search: {self.state} -> {new}")
        self.state = new
        self.t_state = _now_s(self)
        if new == S.NAV_TO_ROOM:
            self._publish_nav_goal(self.room_xy)
            self.last_nav_pub = _now_s(self)
        elif new == S.ROTATE_AND_SEARCH:
            self._set_external_ctrl(True)
            self.rot_yaw_acc = 0.0
        elif new == S.VISUAL_APPROACH:
            # KEEP external_ctrl=True. Closed loop on the RGB bbox all
            # the way to touchdown — no /waypoint_nav/goal, no A*, no
            # depth, no localisation involvement.
            self._set_external_ctrl(True)
            self.last_visual_acquired_t = _now_s(self)
            # Start in YAW: we just stopped a rotation, so the bbox is
            # probably off-axis. The first tick will re-evaluate and may
            # immediately switch to ADVANCE if x_off is already small.
            self.visual_mode = "YAW"
        elif new == S.LAND:
            self._set_external_ctrl(True)
            self._publish_cmd(0.0, 0.0)
            self._land_pubs_left = max(1, int(self.land_n))
            period = 1.0 / max(self.land_hz, 1.0)
            self._land_timer = self.create_timer(period, self._land_tick)
        elif new in (S.DONE, S.GIVE_UP):
            self._publish_cmd(0.0, 0.0)
            self._set_external_ctrl(False)

    def _t_in(self) -> float:
        return _now_s(self) - self.t_state

    # ─── Outgoing primitives ────────────────────────────────────────
    def _publish_nav_goal(self, xy: Tuple[float, float]):
        m = Point()
        m.x = float(xy[0]); m.y = float(xy[1]); m.z = 0.0
        self.pub_goal.publish(m)
        self.get_logger().info(
            f"room_search: nav goal -> ({xy[0]:.2f}, {xy[1]:.2f})")

    def _set_external_ctrl(self, want: bool):
        if want == self._ext_state:
            return
        self.pub_ext.publish(Bool(data=bool(want)))
        self._ext_state = bool(want)
        self.get_logger().info(
            f"room_search: external_ctrl -> {want}")

    def _publish_cmd(self, vx: float, wz: float):
        """Publish one Twist with the platform invariants enforced:
            linear.y = linear.z = 0       (no lateral, no climb)
            linear.x = 0  OR  angular.z = 0   (never both — real-drone
                                               flight controller can't
                                               accept yaw + forward in
                                               the same command).
        Any caller that asks for both gets a warning and wz is zeroed.
        """
        if abs(vx) > 1e-6 and abs(wz) > 1e-6:
            self.get_logger().error(
                f"INVARIANT VIOLATION  vx={vx:.3f}  wz={wz:.3f}  "
                f"(state={self.state}); zeroing wz",
                throttle_duration_sec=1.0)
            wz = 0.0
        m = Twist()
        m.linear.x  = float(vx)
        m.linear.y  = 0.0
        m.linear.z  = 0.0
        m.angular.x = 0.0
        m.angular.y = 0.0
        m.angular.z = float(wz)
        self.pub_cmd.publish(m)

    # ─── Pose callbacks ─────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.cur_xy = (float(p.x), float(p.y))

    def _ps_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.cur_xy = (float(p.x), float(p.y))

    def _pose_cb(self, msg: Pose):
        self.cur_xy = (float(msg.position.x), float(msg.position.y))

    # ─── Target signal callbacks (ROTATE → VISUAL gating) ───────────
    def _seen_cb(self, msg: Bool):
        if bool(msg.data) and not self.target_seen:
            self.get_logger().info(
                "room_search: /target_seen=True received")
        self.target_seen = bool(msg.data) or self.target_seen

    def _info_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        # Use the matched class for detection filtering (target_watcher
        # already did the fuzzy match — respect its verdict so e.g.
        # target='couch' / matched='sofa' filters on 'sofa').
        cname = str(d.get("matched_class", "")).strip().lower()
        if cname:
            if cname != self.target_label:
                self.get_logger().info(
                    f"room_search: matched_class={cname!r} differs from "
                    f"target_object={self.target_label!r}; using "
                    f"matched_class for visual filtering")
            self.target_label = cname
        oid = int(d.get("object_id", -1))
        if oid >= 0:
            self.target_oid = oid
        xy = d.get("xy")
        if (isinstance(xy, (list, tuple)) and len(xy) >= 2
                and self.target_xy is None):
            self.target_xy = (float(xy[0]), float(xy[1]))
            self.get_logger().info(
                f"room_search: target world XY (diagnostic) = "
                f"({self.target_xy[0]:.2f}, {self.target_xy[1]:.2f})")

    def _objects_cb(self, msg: String):
        # Diagnostic only; visual loop is RGB-only.
        pass

    # ─── Detections (the only stream the visual loop needs) ─────────
    def _det_cb(self, msg: Detection2DArray):
        best = None
        best_score = -1.0
        t = self.target_label
        for d in msg.detections:
            for r in d.results:
                c = str(r.hypothesis.class_id).strip().lower()
                if not c:
                    continue
                if c == t or t in c or c in t:
                    s = float(r.hypothesis.score)
                    if s > best_score:
                        best = (
                            float(d.bbox.center.position.x),
                            float(d.bbox.center.position.y),
                            float(d.bbox.size_x),
                            float(d.bbox.size_y),
                            s,
                        )
                        best_score = s
                    break  # first hypothesis per detection wins
        if best is not None:
            self.last_det = best
            self.last_det_t = _now_s(self)

    # ─── Bbox metrics ───────────────────────────────────────────────
    def _area_frac(self, det) -> float:
        _cx, _cy, w, h, _s = det
        return (w * h) / max(1.0, float(self.rgb_W * self.rgb_H))

    def _x_off(self, det) -> float:
        cx, _cy, _w, _h, _s = det
        return _saturate((cx - 0.5 * self.rgb_W)
                         / max(1e-6, 0.5 * self.rgb_W), 1.0)

    # ─── Rotation tick ──────────────────────────────────────────────
    def _rotate_tick(self):
        if self.state != S.ROTATE_AND_SEARCH:
            return
        dt = 1.0 / max(self.rot_hz, 1.0)
        self.rot_yaw_acc += abs(self.rot_rate) * dt
        self._publish_cmd(0.0, self.rot_rate)

    # ─── Visual servo tick (RGB-only close-in) ──────────────────────
    def _visual_tick(self):
        if self.state != S.VISUAL_APPROACH:
            return
        now = _now_s(self)
        det = self.last_det
        det_age = now - self.last_det_t if self.last_det is not None else 1e9

        # Lost: no fresh matching detection in the recent past.
        if det is None or det_age > self.lost_hover_s:
            self._publish_cmd(0.0, 0.0)
            lost_for = now - self.last_visual_acquired_t
            self.get_logger().warn(
                f"room_search: visual target lost  age={det_age:.1f}s  "
                f"lost_for={lost_for:.1f}s",
                throttle_duration_sec=1.0)
            if lost_for > self.giveup_s:
                self._enter(S.GIVE_UP)
            return

        self.last_visual_acquired_t = now

        area = self._area_frac(det)
        x_off = self._x_off(det)

        # Terminal: bbox is large enough → we're as close as RGB-only
        # control can determine. LAND.
        if area >= self.area_land:
            self.get_logger().info(
                f"room_search: bbox area_frac={area:.3f} >= "
                f"land_area_frac={self.area_land:.3f}  -> LAND")
            self._enter(S.LAND)
            return

        # Safety: stayed in approach too long without ever filling the
        # frame. Land anyway — better than hovering forever.
        if self._t_in() > self.appr_to_s:
            self.get_logger().warn(
                f"room_search: approach timed out after "
                f"{self.appr_to_s:.0f}s (max area_frac={area:.3f}); "
                f"landing where we are")
            self._enter(S.LAND)
            return

        # ── Sub-mode selection with hysteresis ──────────────────
        # The real drone refuses vx+wz in the same Twist. We emit
        # either pure-yaw OR pure-forward and switch between them
        # using a Schmitt trigger on |x_off|:
        #   in YAW:     stay until |x_off| < yaw_exit, then ADVANCE
        #   in ADVANCE: stay until |x_off| > yaw_enter, then YAW
        # On every mode switch we emit one (0, 0) brake tick before
        # the new mode's command takes effect on the wire — gives the
        # platform's PID a beat to settle the previous axis.
        prev = self.visual_mode
        ax = abs(x_off)
        if prev == "YAW" and ax < self.yaw_exit:
            self.visual_mode = "ADVANCE"
        elif prev == "ADVANCE" and ax > self.yaw_enter:
            self.visual_mode = "YAW"

        if self.visual_mode != prev:
            self.get_logger().info(
                f"room_search: visual sub-mode {prev} -> "
                f"{self.visual_mode}  (|x_off|={ax:.2f}, area={area:.3f})")
            # Brake tick: pure zeros (satisfies the invariant trivially).
            self._publish_cmd(0.0, 0.0)
            return

        if self.visual_mode == "YAW":
            # Pure-yaw command. Sign rationale: +angular.z yaws the body
            # CCW, which shifts the camera content LEFTWARDS. Target
            # right of centre (x_off > 0) → yaw RIGHT, wz < 0. Hence the
            # minus on kp_yaw.
            wz = _saturate(-self.kp_yaw * x_off, self.max_wz)
            self._publish_cmd(0.0, wz)
            return

        # ADVANCE: pure-forward command, ramped by bbox area.
        if area < self.area_slow:
            vx = self.vx_max
        else:
            span = max(1e-6, self.area_land - self.area_slow)
            vx = self.vx_max * _clamp01(
                (self.area_land - area) / span)
        self._publish_cmd(vx, 0.0)

    # ─── Land burst ─────────────────────────────────────────────────
    def _land_tick(self):
        if self._land_pubs_left <= 0:
            try:
                if self._land_timer is not None:
                    self._land_timer.cancel()
                    self._land_timer = None
            except Exception:
                pass
            self._enter(S.DONE)
            return
        self.pub_land.publish(Empty())
        self._land_pubs_left -= 1

    # ─── Main tick (state transitions) ──────────────────────────────
    def _tick(self):
        if self.state == S.WAIT_INIT:
            if self.cur_xy is not None:
                self._enter(S.NAV_TO_ROOM)
            return

        if self.state == S.NAV_TO_ROOM:
            if self.cur_xy is None:
                return
            d = math.hypot(self.cur_xy[0] - self.room_xy[0],
                           self.cur_xy[1] - self.room_xy[1])
            if d < self.nav_arr_r:
                self.get_logger().info(
                    f"room_search: arrived at room center  d={d:.2f}m "
                    f"< {self.nav_arr_r:.2f}m")
                self._enter(S.ROTATE_AND_SEARCH)
                return
            if (_now_s(self) - self.last_nav_pub) > self.nav_republ_s:
                self._publish_nav_goal(self.room_xy)
                self.last_nav_pub = _now_s(self)
            return

        if self.state == S.ROTATE_AND_SEARCH:
            # Transition is gated by:
            #   * target_watcher fired /target_seen=True (semantic confirm)
            #   * AND we have a fresh matching detection in
            #     /perception/detections (visual lock at this instant)
            now = _now_s(self)
            det_fresh = (self.last_det is not None
                         and (now - self.last_det_t) < self.lost_hover_s)
            if self.target_seen and det_fresh:
                self._publish_cmd(0.0, 0.0)  # brake yaw
                self._enter(S.VISUAL_APPROACH)
                return
            if self.target_seen and not det_fresh:
                self.get_logger().info(
                    "room_search: /target_seen latched but no fresh "
                    "detection — continuing to spin",
                    throttle_duration_sec=2.0)
            if self.rot_yaw_acc > (2.0 * math.pi * self.max_revs):
                self.get_logger().warn(
                    f"room_search: rotated ~{self.rot_yaw_acc:.1f} rad "
                    f"(>={self.max_revs:.1f} revs) without locking onto "
                    f"{self.target_label!r} — giving up")
                self._publish_cmd(0.0, 0.0)
                self._enter(S.GIVE_UP)
            return

        # VISUAL_APPROACH / LAND are driven by their own timers.

    # ─── Heartbeat ──────────────────────────────────────────────────
    def _hb(self):
        bits = [f"state={self.state}",
                f"t_in={self._t_in():.1f}s",
                f"target_seen={self.target_seen}"]
        if self.cur_xy is not None:
            bits.append(f"pose=({self.cur_xy[0]:.2f},{self.cur_xy[1]:.2f})")
        else:
            bits.append("pose=None")
        if self.last_det is not None:
            age = _now_s(self) - self.last_det_t
            cx, cy, w, h, s = self.last_det
            area = self._area_frac(self.last_det)
            x_off = self._x_off(self.last_det)
            bits.append(
                f"det(age={age:.1f}s  bbox_c=({cx:.0f},{cy:.0f}) "
                f"area={area:.3f}  x_off={x_off:+.2f}  score={s:.2f})")
        else:
            bits.append("det=None")
        if self.state == S.ROTATE_AND_SEARCH:
            bits.append(f"yaw_acc={self.rot_yaw_acc:.2f}rad")
        if self.state == S.VISUAL_APPROACH:
            bits.append(f"sub={self.visual_mode}")
        self.get_logger().info("room_search hb  " + "  ".join(bits))


def main():
    rclpy.init()
    node = RoomSearchOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_ext.publish(Bool(data=False))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
