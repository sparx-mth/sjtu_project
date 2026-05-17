#!/usr/bin/env python3
"""
room_search_orchestrator_node.py — go to a room, rotate to find a target,
                                   close in by tracking the bbox with
                                   sparse optical flow on the RGB stream,
                                   land.

State machine
-------------
  WAIT_INIT          have we seen pose yet? if so → NAV_TO_ROOM
  NAV_TO_ROOM        publish (room_center_x, room_center_y) to /waypoint_nav/goal,
                     watch drone pose, wait until within nav_arrival_radius_m.
                     → ROTATE_AND_SEARCH
  ROTATE_AND_SEARCH  take over /cmd_vel (external_ctrl=True), spin at
                     rotation_rate_rad_s while watching the YOLO detector
                     (/perception/detections) for the target class. When a
                     matching detection arrives, capture its bbox AND seed
                     the optical-flow tracker on the corresponding RGB
                     frame.
                     → VISUAL_APPROACH
                     (GIVE_UP after max_rotation_revs full revolutions)
  VISUAL_APPROACH    KEEP external_ctrl=True. Closed-loop on the RGB
                     stream alone — no depth, no localisation, and (after
                     the first detection) no YOLO in the inner loop:
                       * sparse Lucas-Kanade optical flow on the corners
                         seeded inside the YOLO bbox propagates the bbox
                         frame-to-frame at camera rate
                       * bounding rect of the tracked corners IS the new
                         bbox — auto-scales as the drone closes in
                       * controller is the same as before: pure-yaw OR
                         pure-forward (XOR), hysteresis on |x_off|
                       * tracker loss (too few matched corners) → hold
                         (and wait for any fresh YOLO match to re-seed)
                       * fresh YOLO match while in VISUAL_APPROACH (and
                         track_re_seed_on_detection=true) → re-seed the
                         tracker on the new bbox; corrects any drift
                     Exits when bbox area frac >= visual_land_area_frac.
                     → LAND
  LAND               publish a short burst of /<drone_ns>/land (Empty).
                     The sjtu_drone landing controller owns the descent.
                     → DONE
  DONE               idle.
  GIVE_UP            idle, log a warning.

Why detect-once / track-many
----------------------------
Running YOLO-World at 4 Hz on a Jetson AGX while perception, control,
ROS2, and the drone hardware loop all share the same SoC is wasteful
once we already have a bbox. Sparse Lucas-Kanade is among the
lightest robust trackers available (no neural inference, no contrib
modules, ~2-3 ms per frame at 640x360 on Jetson AGX Orin CPU), runs
at 200+ Hz, and handles scale implicitly because the bounding rect of
the tracked corners grows as the drone approaches the target. We
re-seed the tracker on every fresh YOLO detection so any drift is
bounded by the YOLO inter-arrival time (set with yolo_min_dt at
launch — 1.0 s default on Jetson is plenty).

Platform invariant
------------------
Every published Twist has linear.y = linear.z = 0 AND
(linear.x = 0  XOR  angular.z = 0). The real drone refuses Twists
that mix forward and yaw, so the visual loop alternates pure-yaw and
pure-forward sub-modes with hysteresis.

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
    rgb_topic             sensor_msgs/Image (default /simple_drone/front/image_raw)

  OUT (must be bridged to ROS1):
    /waypoint_nav/goal                   geometry_msgs/Point      (NAV_TO_ROOM)
    /<drone_ns>/cmd_vel                  geometry_msgs/Twist      (rotation + visual approach)
    /<drone_ns>/land                     std_msgs/Empty           (LAND)
    /waypoint_follower/external_ctrl     std_msgs/Bool (latched)  (toggled by state machine)
"""

import json
import math
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


# Optional heavy imports — done at module load so import errors surface
# at startup, not on the first frame.
import cv2
from cv_bridge import CvBridge


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
        P("rgb_topic",            "/simple_drone/front/image_raw")

        # ── Phase radii / timings ────────────────────────────────
        P("nav_arrival_radius_m",   0.50)
        P("nav_goal_republish_s",   3.0)

        P("rotation_rate_rad_s",    0.5)
        P("rotation_ctrl_hz",      20.0)
        P("max_rotation_revs",      2.0)

        # ── Visual servoing ──────────────────────────────────────
        P("rgb_image_width",        640)
        P("rgb_image_height",       360)

        P("visual_kp_yaw",                  0.9)
        P("visual_max_yaw_rate",            0.6)
        P("visual_yaw_deadband_enter",      0.20)
        P("visual_yaw_deadband_exit",       0.08)
        P("visual_vx_max",                  0.20)
        P("visual_slowdown_area_frac",      0.03)
        P("visual_land_area_frac",          0.12)
        P("visual_lost_hover_s",            0.6)
        P("visual_giveup_s",               15.0)
        P("visual_approach_timeout_s",     90.0)
        P("visual_ctrl_hz",                20.0)

        # ── Sparse-LK tracker ────────────────────────────────────
        # Seed corners from the YOLO bbox with Shi-Tomasi; propagate
        # frame-to-frame with calcOpticalFlowPyrLK; the bbox each tick
        # is the bounding rect of the still-matched corners (so scale
        # is implicit). Defaults sized for the sjtu_drone front camera
        # (640x360) and tuned conservatively — see notes below.
        P("track_max_corners",       80)
        P("track_corner_quality",     0.05)
        P("track_corner_min_dist",    5.0)
        P("track_lk_win",            21)    # winSize for LK
        P("track_lk_levels",          3)    # pyramid levels
        P("track_min_matches",        8)    # below this → tracker lost
        # When True, every fresh YOLO match while in VISUAL_APPROACH
        # re-seeds the tracker on the new bbox + the closest cached
        # frame. Keep this on for any non-trivial run; it bounds drift
        # to the YOLO inter-arrival time. Turn off only if YOLO is
        # completely disabled after the first hit.
        P("track_re_seed_on_detection", True)
        # Recent-frame buffer for stamp-matching incoming detections.
        # 30 frames is ~1 s at 30 Hz; covers typical YOLO publish latency.
        P("track_frame_buffer_len",  30)
        # Margin (fraction of bbox W/H) added when extracting the ROI
        # for seeding — gives Shi-Tomasi a bit more context to find
        # strong corners on the object's edges.
        P("track_seed_roi_margin",    0.10)

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
        self.rgb_topic     = str(g("rgb_topic"))

        self.nav_arr_r     = float(g("nav_arrival_radius_m"))
        self.nav_republ_s  = float(g("nav_goal_republish_s"))

        self.rot_rate      = float(g("rotation_rate_rad_s"))
        self.rot_hz        = float(g("rotation_ctrl_hz"))
        self.max_revs      = float(g("max_rotation_revs"))

        self.rgb_W         = int(g("rgb_image_width"))
        self.rgb_H         = int(g("rgb_image_height"))

        self.kp_yaw        = float(g("visual_kp_yaw"))
        self.max_wz        = float(g("visual_max_yaw_rate"))
        self.yaw_enter     = float(g("visual_yaw_deadband_enter"))
        self.yaw_exit      = float(g("visual_yaw_deadband_exit"))
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

        self.trk_n_corners = int  (g("track_max_corners"))
        self.trk_qlevel    = float(g("track_corner_quality"))
        self.trk_min_dist  = float(g("track_corner_min_dist"))
        self.trk_lk_win    = int  (g("track_lk_win"))
        self.trk_lk_levels = int  (g("track_lk_levels"))
        self.trk_min_match = int  (g("track_min_matches"))
        self.trk_re_seed   = bool (g("track_re_seed_on_detection"))
        self.trk_buf_len   = int  (g("track_frame_buffer_len"))
        self.trk_margin    = float(g("track_seed_roi_margin"))

        self.land_n        = int(g("land_burst_count"))
        self.land_hz       = float(g("land_burst_hz"))
        self.tick_hz       = float(g("tick_hz"))

        # ── State ────────────────────────────────────────────────
        self.state         = S.WAIT_INIT
        self.t_state       = _now_s(self)
        self.cur_xy: Optional[Tuple[float, float]] = None
        self.target_xy: Optional[Tuple[float, float]] = None
        self.target_oid: Optional[int] = None
        self.target_seen   = False

        self.last_nav_pub  = 0.0
        self.rot_yaw_acc   = 0.0    # cumulative rotation, rad (for max_revs)

        # Latest matched YOLO detection bbox (RGB pixels) + arrival time.
        self.last_det: Optional[Tuple[float, float, float, float, float]] = None
        self.last_det_t: float = 0.0
        self.last_det_stamp: float = 0.0   # original header stamp from YOLO

        # ── LK tracker state ─────────────────────────────────────
        self.bridge = CvBridge()
        # Ring buffer of (header_stamp_s, gray_frame) for stamp-matching.
        self._frame_buf: Deque[Tuple[float, np.ndarray]] = deque(
            maxlen=max(2, self.trk_buf_len))
        # Tracker state
        self.tracker_valid = False
        self.prev_gray: Optional[np.ndarray] = None
        self.prev_pts:  Optional[np.ndarray] = None   # Nx1x2 float32
        self.tracked_bbox: Optional[Tuple[float, float, float, float]] = None
        self.last_track_t: float = 0.0
        self.last_visual_acquired_t: float = 0.0  # last time we had a valid track

        # VISUAL_APPROACH sub-state. "YAW" / "ADVANCE".
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

        self.create_subscription(
            Bool,   "/target_seen",        self._seen_cb,    latched)
        self.create_subscription(
            String, "/target_seen/info",   self._info_cb,    latched)
        self.create_subscription(
            String, "/perception/objects", self._objects_cb, latched)

        # YOLO detections (seeds + re-seeds the LK tracker).
        self.create_subscription(
            Detection2DArray, self.det_topic, self._det_cb, sensor)
        # RGB stream — the only input the inner loop needs once seeded.
        self.create_subscription(
            Image, self.rgb_topic, self._rgb_cb, sensor)

        # ── Timers ───────────────────────────────────────────────
        self.create_timer(1.0 / self.tick_hz, self._tick)
        self.create_timer(1.0 / self.rot_hz, self._rotate_tick)
        self.create_timer(1.0 / self.visual_hz, self._visual_tick)
        self.create_timer(5.0, self._hb)

        self.get_logger().info("=" * 64)
        self.get_logger().info(
            "room_search_orchestrator ready  (RGB + LK tracker close-in)")
        self.get_logger().info(
            f"  room_center = ({self.room_xy[0]:.2f}, {self.room_xy[1]:.2f}) m")
        self.get_logger().info(f"  target_object = {self.target_label!r}")
        self.get_logger().info(
            f"  drone_ns = {self.drone_ns!r}   "
            f"pose = {self.pose_topic} ({self.pose_type})")
        self.get_logger().info(
            f"  detections = {self.det_topic}   rgb = {self.rgb_topic}   "
            f"rgb_size = {self.rgb_W}x{self.rgb_H}")
        self.get_logger().info(
            f"  visual: vx_max={self.vx_max:.2f}m/s  kp_yaw={self.kp_yaw:.2f}  "
            f"yaw_deadband enter/exit={self.yaw_enter:.2f}/{self.yaw_exit:.2f}  "
            f"slowdown@area={self.area_slow:.3f}  "
            f"land@area={self.area_land:.3f}")
        self.get_logger().info(
            f"  LK tracker: max_corners={self.trk_n_corners}  "
            f"min_matches={self.trk_min_match}  "
            f"lk_win={self.trk_lk_win}  lvls={self.trk_lk_levels}  "
            f"re_seed_on_yolo={self.trk_re_seed}")
        self.get_logger().info(
            "  platform invariant: every Twist has vy=vz=0 AND "
            "(vx=0 XOR wz=0)")
        self.get_logger().info("=" * 64)

    # ─── State helpers ──────────────────────────────────────────────
    def _enter(self, new: str):
        if new == self.state:
            return
        self.get_logger().info(f"room_search: {self.state} -> {new}")
        self.state = new
        self.t_state = _now_s(self)
        if new == S.NAV_TO_ROOM:
            self._publish_nav_goal(self.room_xy)
            self.last_nav_pub = _now_s(self)
        elif new == S.ROTATE_AND_SEARCH:
            self._set_external_ctrl(True)
            self.rot_yaw_acc = 0.0
        elif new == S.VISUAL_APPROACH:
            self._set_external_ctrl(True)
            self.last_visual_acquired_t = _now_s(self)
            # Start in YAW: we just stopped a rotation, so the bbox is
            # probably off-axis. The first visual tick will re-evaluate.
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
            self.tracker_valid = False

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
        self.get_logger().info(f"room_search: external_ctrl -> {want}")

    def _publish_cmd(self, vx: float, wz: float):
        """One Twist with the platform invariants hardwired:
            linear.y = linear.z = 0
            linear.x = 0  XOR  angular.z = 0
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

    # ─── Target signal callbacks ────────────────────────────────────
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
        # Diagnostic only; visual loop is image-only.
        pass

    # ─── YOLO detections: seed / re-seed the tracker ────────────────
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
        if best is None:
            return

        now = _now_s(self)
        self.last_det = best
        self.last_det_t = now
        try:
            self.last_det_stamp = (msg.header.stamp.sec
                                   + msg.header.stamp.nanosec * 1e-9)
        except Exception:
            self.last_det_stamp = now

        # In ROTATE_AND_SEARCH the main tick uses last_det to decide
        # whether to transition; the actual seed happens then. In
        # VISUAL_APPROACH, if re-seed is on, re-seed now to correct any
        # tracker drift against the YOLO ground truth.
        if (self.state == S.VISUAL_APPROACH
                and self.trk_re_seed
                and self._frame_buf):
            frame = self._closest_frame(self.last_det_stamp)
            if frame is not None:
                ok = self._seed_tracker(best, frame)
                if ok:
                    self.get_logger().info(
                        f"room_search: tracker re-seeded from YOLO  "
                        f"area_frac={(best[2]*best[3])/(self.rgb_W*self.rgb_H):.3f}",
                        throttle_duration_sec=2.0)

    # ─── RGB stream: cache + advance the LK tracker ─────────────────
    def _rgb_cb(self, msg: Image):
        # Convert + grayscale once. Keep a small ring buffer so the
        # detection callback can pick the frame closest to YOLO's stamp.
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(
                f"cv_bridge decode failed: {e}",
                throttle_duration_sec=5.0)
            return
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception:
            stamp = _now_s(self)
        self._frame_buf.append((stamp, gray))

        # Update tracker if it's valid. Cheap — sparse LK at the chosen
        # window/level settings is ~2-3 ms at 640x360 on a Jetson AGX
        # Orin CPU.
        if not self.tracker_valid or self.prev_gray is None \
                or self.prev_pts is None:
            return

        new_pts, status, _err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None,
            winSize=(self.trk_lk_win, self.trk_lk_win),
            maxLevel=self.trk_lk_levels,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                      20, 0.03))
        if new_pts is None or status is None:
            self.tracker_valid = False
            return

        flat = status.flatten() == 1
        # Also reject points that left the image plane.
        if new_pts.ndim == 3:
            xy = new_pts.reshape(-1, 2)
        else:
            xy = new_pts
        in_bounds = ((xy[:, 0] >= 0) & (xy[:, 0] < gray.shape[1])
                     & (xy[:, 1] >= 0) & (xy[:, 1] < gray.shape[0]))
        ok = flat & in_bounds
        if int(ok.sum()) < self.trk_min_match:
            self.tracker_valid = False
            self.get_logger().info(
                "room_search: LK tracker lost  "
                f"matches={int(ok.sum())}<{self.trk_min_match}",
                throttle_duration_sec=1.0)
            return

        matched = xy[ok].astype(np.float32)
        x_min, y_min = float(matched[:, 0].min()), float(matched[:, 1].min())
        x_max, y_max = float(matched[:, 0].max()), float(matched[:, 1].max())
        cx = 0.5 * (x_min + x_max)
        cy = 0.5 * (y_min + y_max)
        w  = max(1.0, x_max - x_min)
        h  = max(1.0, y_max - y_min)
        self.tracked_bbox = (cx, cy, w, h)
        self.last_track_t = _now_s(self)
        # Roll the LK state forward.
        self.prev_gray = gray
        self.prev_pts = matched.reshape(-1, 1, 2)

    # ─── Tracker helpers ────────────────────────────────────────────
    def _closest_frame(self, stamp: float) -> Optional[np.ndarray]:
        if not self._frame_buf:
            return None
        # Linear scan is fine — buf is small.
        best = min(self._frame_buf, key=lambda kv: abs(kv[0] - stamp))
        return best[1]

    def _seed_tracker(self, det, gray: np.ndarray) -> bool:
        """Initialise the LK tracker from a bbox in `det` and a gray frame.
        Picks Shi-Tomasi corners inside a slightly-padded ROI so the
        seed includes the object's edges (where the strongest corners
        usually live). Returns True on success."""
        cx, cy, w, h, _s = det
        margin = self.trk_margin
        Hf, Wf = gray.shape[:2]
        x0 = int(max(0, cx - 0.5 * w * (1.0 + 2 * margin)))
        y0 = int(max(0, cy - 0.5 * h * (1.0 + 2 * margin)))
        x1 = int(min(Wf, cx + 0.5 * w * (1.0 + 2 * margin)))
        y1 = int(min(Hf, cy + 0.5 * h * (1.0 + 2 * margin)))
        if x1 - x0 < 4 or y1 - y0 < 4:
            self.tracker_valid = False
            return False
        # Build a mask so corners are picked only inside the original
        # bbox (margin is just for context).
        mask = np.zeros_like(gray, dtype=np.uint8)
        mx0 = int(max(0, cx - 0.5 * w))
        my0 = int(max(0, cy - 0.5 * h))
        mx1 = int(min(Wf, cx + 0.5 * w))
        my1 = int(min(Hf, cy + 0.5 * h))
        if mx1 - mx0 < 2 or my1 - my0 < 2:
            self.tracker_valid = False
            return False
        mask[my0:my1, mx0:mx1] = 255

        pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.trk_n_corners,
            qualityLevel=self.trk_qlevel,
            minDistance=self.trk_min_dist,
            mask=mask,
            blockSize=7)
        if pts is None or len(pts) < self.trk_min_match:
            self.get_logger().warn(
                "room_search: tracker seed failed — "
                f"only {0 if pts is None else len(pts)} corners "
                f"(<{self.trk_min_match})",
                throttle_duration_sec=2.0)
            self.tracker_valid = False
            return False

        self.prev_gray = gray.copy()
        self.prev_pts  = pts.astype(np.float32)
        self.tracked_bbox = (float(cx), float(cy),
                             float(max(1.0, w)), float(max(1.0, h)))
        self.tracker_valid = True
        self.last_track_t  = _now_s(self)
        self.last_visual_acquired_t = self.last_track_t
        return True

    # ─── Bbox metrics ───────────────────────────────────────────────
    def _area_frac(self, bbox: Tuple[float, float, float, float]) -> float:
        _cx, _cy, w, h = bbox
        return (w * h) / max(1.0, float(self.rgb_W * self.rgb_H))

    def _x_off(self, bbox: Tuple[float, float, float, float]) -> float:
        cx, _cy, _w, _h = bbox
        return _saturate((cx - 0.5 * self.rgb_W)
                         / max(1e-6, 0.5 * self.rgb_W), 1.0)

    # ─── Rotation tick ──────────────────────────────────────────────
    def _rotate_tick(self):
        if self.state != S.ROTATE_AND_SEARCH:
            return
        dt = 1.0 / max(self.rot_hz, 1.0)
        self.rot_yaw_acc += abs(self.rot_rate) * dt
        self._publish_cmd(0.0, self.rot_rate)

    # ─── Visual servo tick (uses tracked_bbox, not last_det) ────────
    def _visual_tick(self):
        if self.state != S.VISUAL_APPROACH:
            return
        now = _now_s(self)

        if not self.tracker_valid or self.tracked_bbox is None:
            # Hold while we wait for a YOLO re-seed.
            self._publish_cmd(0.0, 0.0)
            lost_for = now - self.last_visual_acquired_t
            self.get_logger().warn(
                f"room_search: tracker lost  lost_for={lost_for:.1f}s",
                throttle_duration_sec=1.0)
            if lost_for > self.giveup_s:
                self._enter(S.GIVE_UP)
            return

        # Track-staleness check (camera dropout, not LK failure).
        track_age = now - self.last_track_t
        if track_age > self.lost_hover_s:
            self._publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                f"room_search: tracked_bbox stale  age={track_age:.1f}s "
                f"(RGB stream silent?)",
                throttle_duration_sec=1.0)
            if track_age > self.giveup_s:
                self._enter(S.GIVE_UP)
            return

        self.last_visual_acquired_t = now

        bbox = self.tracked_bbox
        area  = self._area_frac(bbox)
        x_off = self._x_off(bbox)

        # Terminal: bbox is large enough → as close as RGB-only control
        # can determine. LAND.
        if area >= self.area_land:
            self.get_logger().info(
                f"room_search: bbox area_frac={area:.3f} >= "
                f"land_area_frac={self.area_land:.3f}  -> LAND")
            self._enter(S.LAND)
            return

        # Safety: stayed in approach too long. Land anyway.
        if self._t_in() > self.appr_to_s:
            self.get_logger().warn(
                f"room_search: approach timed out after "
                f"{self.appr_to_s:.0f}s (max area_frac={area:.3f}); "
                f"landing where we are")
            self._enter(S.LAND)
            return

        # Sub-mode selection with hysteresis on |x_off|.
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
            # Brake tick on mode switch.
            self._publish_cmd(0.0, 0.0)
            return

        if self.visual_mode == "YAW":
            wz = _saturate(-self.kp_yaw * x_off, self.max_wz)
            self._publish_cmd(0.0, wz)
            return

        # ADVANCE: pure-forward, ramped by bbox area.
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
            #   * AND we have a fresh matching detection
            #   * AND we successfully seeded the LK tracker on a recent
            #     RGB frame near the detection's stamp
            now = _now_s(self)
            det_fresh = (self.last_det is not None
                         and (now - self.last_det_t) < self.lost_hover_s)
            if self.target_seen and det_fresh:
                frame = self._closest_frame(self.last_det_stamp)
                if frame is None:
                    self.get_logger().info(
                        "room_search: detection in hand but no RGB "
                        "frame buffered yet — waiting one tick",
                        throttle_duration_sec=2.0)
                else:
                    self._publish_cmd(0.0, 0.0)  # brake yaw
                    ok = self._seed_tracker(self.last_det, frame)
                    if ok:
                        self.get_logger().info(
                            f"room_search: LK tracker seeded with "
                            f"{len(self.prev_pts)} corners")
                        self._enter(S.VISUAL_APPROACH)
                        return
                    # else: seed failed; keep rotating, retry next det
            elif self.target_seen and not det_fresh:
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
        bits.append(f"rgb_buf={len(self._frame_buf)}")
        if self.last_det is not None:
            age = _now_s(self) - self.last_det_t
            bits.append(f"yolo(age={age:.1f}s score={self.last_det[4]:.2f})")
        else:
            bits.append("yolo=None")
        if self.tracker_valid and self.tracked_bbox is not None:
            cx, cy, w, h = self.tracked_bbox
            n = 0 if self.prev_pts is None else len(self.prev_pts)
            area = self._area_frac(self.tracked_bbox)
            x_off = self._x_off(self.tracked_bbox)
            bits.append(
                f"trk(n={n}  bbox_c=({cx:.0f},{cy:.0f}) "
                f"area={area:.3f}  x_off={x_off:+.2f})")
        else:
            bits.append("trk=invalid")
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
