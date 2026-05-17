#!/usr/bin/env python3
"""
room_search_orchestrator_node.py — go to a room, rotate to find a target,
                                   visually close in on it from the depth
                                   image, land.

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
  VISUAL_APPROACH    KEEP external_ctrl=True (orchestrator owns /cmd_vel
                     all the way to touchdown). Closed-loop on the depth
                     image only — no world-frame nav goal, no A*, no
                     localisation:
                       * yaw to keep the target bbox centred horizontally
                         in the RGB image,
                       * advance forward at a velocity that ramps down
                         with the depth sampled at the target bbox
                         centre,
                       * if no matching detection for visual_lost_timeout_s
                         seconds, hold (and eventually GIVE_UP).
                     Exits when sampled depth < visual_land_depth_m.
                     → LAND
  LAND               publish a short burst of /<drone_ns>/land (Empty).
                     The sjtu_drone landing controller owns the descent.
                     → DONE
  DONE               idle.
  GIVE_UP            idle, log a warning.

Why depth-only after detection (no localisation)
------------------------------------------------
World-frame XY from object_mapper depends on the camera pose, depth
quality, and projection at the moment of detection — three independent
sources of metric error. For "land exactly on the keyboard", a closed
visual loop on the live image is more direct: the bbox centre IS the
target, the depth at that pixel IS the distance, and the controller
shrinks both errors to zero without ever materialising a world XY.

What we expect on the wire
--------------------------
  IN  (ROS2 native, from perception_docker/semantic_mapper):
    /target_seen          std_msgs/Bool          TRANSIENT_LOCAL
    /target_seen/info     std_msgs/String        TRANSIENT_LOCAL (JSON)
    /perception/detections vision_msgs/Detection2DArray   sensor QoS
    /perception/objects   std_msgs/String        TRANSIENT_LOCAL (JSON)
                                                 (only used for diagnostics)

  IN  (must be bridged from ROS1):
    pose_topic            nav_msgs/Odometry (default /odom_world)
    /map_ros/depth        sensor_msgs/Image (32FC1)
                          - bridged for perception_docker already

  OUT (must be bridged to ROS1):
    /waypoint_nav/goal                   geometry_msgs/Point      (NAV_TO_ROOM)
    /<drone_ns>/cmd_vel                  geometry_msgs/Twist      (rotation + visual approach)
    /<drone_ns>/land                     std_msgs/Empty           (LAND)
    /waypoint_follower/external_ctrl     std_msgs/Bool (latched)  (toggled by state machine)
"""

import json
import math
from typing import Optional, Tuple

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


class RoomSearchOrchestrator(Node):
    def __init__(self):
        super().__init__("room_search_orchestrator")

        P = self.declare_parameter

        # ── Mission ───────────────────────────────────────────────
        P("room_center_x",        4.0)
        P("room_center_y",        5.0)
        # Target object label. Must match (case-insensitive) the YOLO
        # class string that target_watcher confirmed. Substring is also
        # accepted (e.g. 'keyboard' matches detected 'keyboard ').
        P("target_object",        "keyboard")
        P("drone_ns",             "/simple_drone")

        # ── Topic names (override per environment) ────────────────
        P("pose_topic",           "/odom_world")
        P("pose_type",            "odometry")    # odometry | pose_stamped | pose
        P("detections_topic",     "/perception/detections")
        P("depth_topic",          "/map_ros/depth")

        # ── Phase radii / timings ────────────────────────────────
        P("nav_arrival_radius_m",   0.50)
        P("nav_goal_republish_s",   3.0)

        P("rotation_rate_rad_s",    0.5)
        P("rotation_ctrl_hz",      20.0)
        P("max_rotation_revs",      2.0)

        # ── Visual servoing (NEW) ────────────────────────────────
        # RGB image dimensions used by the detector. The bbox centre is
        # in RGB pixels; we normalise by RGB width/height. Defaults are
        # the sjtu_drone front camera (640x360). For a real camera,
        # override at launch.
        P("rgb_image_width",        640)
        P("rgb_image_height",       360)

        # Yaw proportional gain — applied to x_offset_normalised ∈ [-1,+1].
        # wz = -kp * x_off (negate because positive yaw turns the camera
        # frame LEFT in the image; if the target is to the RIGHT of
        # centre we want to yaw RIGHT, i.e. negative angular.z under the
        # standard ROS body frame convention).
        P("visual_kp_yaw",          0.9)
        P("visual_max_yaw_rate",    0.6)    # rad/s saturation
        # If |x_offset_normalised| exceeds this, set vx=0 and yaw only.
        # Prevents flying past a target that's far off-axis.
        P("visual_yaw_deadband",    0.20)   # 0.20 ≈ ±10% off centre

        # Forward velocity ramp. vx = vx_max * clamp((depth - land) /
        # (slowdown_start - land), 0, 1). vx_max ≈ 0.2 m/s is comfortable
        # for sjtu_drone defaults.
        P("visual_vx_max",          0.20)
        P("visual_slowdown_start_m", 1.50)  # at this depth, start ramping vx
        P("visual_land_depth_m",    0.45)   # at this depth, transition to LAND

        # Depth sampling — robust percentile in a centred patch (mirrors
        # object_mapper_node's approach, intentionally).
        P("visual_depth_patch_pct",  0.50)  # patch is 50% of bbox dims
        P("visual_depth_percentile", 30.0)
        P("visual_depth_min_m",      0.30)
        P("visual_depth_max_m",      8.00)
        P("visual_depth_min_valid_px", 20)

        # Lost-target handling.
        # If no matching detection arrives within this window, hold a
        # zero cmd. If we stay lost for visual_giveup_s, GIVE_UP.
        P("visual_lost_hover_s",     0.6)
        P("visual_giveup_s",        15.0)

        P("visual_ctrl_hz",         15.0)   # visual servo loop rate

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
        self.depth_topic   = str(g("depth_topic"))

        self.nav_arr_r     = float(g("nav_arrival_radius_m"))
        self.nav_republ_s  = float(g("nav_goal_republish_s"))

        self.rot_rate      = float(g("rotation_rate_rad_s"))
        self.rot_hz        = float(g("rotation_ctrl_hz"))
        self.max_revs      = float(g("max_rotation_revs"))

        self.rgb_W         = int(g("rgb_image_width"))
        self.rgb_H         = int(g("rgb_image_height"))

        self.kp_yaw        = float(g("visual_kp_yaw"))
        self.max_wz        = float(g("visual_max_yaw_rate"))
        self.yaw_deadband  = float(g("visual_yaw_deadband"))
        self.vx_max        = float(g("visual_vx_max"))
        self.slowdown_d    = float(g("visual_slowdown_start_m"))
        self.land_d        = float(g("visual_land_depth_m"))
        self.depth_patch   = float(g("visual_depth_patch_pct"))
        self.depth_pctl    = float(g("visual_depth_percentile"))
        self.depth_min_m   = float(g("visual_depth_min_m"))
        self.depth_max_m   = float(g("visual_depth_max_m"))
        self.depth_min_n   = int  (g("visual_depth_min_valid_px"))
        self.lost_hover_s  = float(g("visual_lost_hover_s"))
        self.giveup_s      = float(g("visual_giveup_s"))
        self.visual_hz     = float(g("visual_ctrl_hz"))

        self.land_n        = int(g("land_burst_count"))
        self.land_hz       = float(g("land_burst_hz"))
        self.tick_hz       = float(g("tick_hz"))

        # ── State ────────────────────────────────────────────────
        self.state         = S.WAIT_INIT
        self.t_state       = _now_s(self)
        self.cur_xy: Optional[Tuple[float, float]] = None
        # World XY of the matched object (best-effort; kept for logging
        # only, not used as a nav target in the visual approach).
        self.target_xy: Optional[Tuple[float, float]] = None
        self.target_oid: Optional[int] = None
        self.target_seen   = False

        self.last_nav_pub  = 0.0
        self.rot_yaw_acc   = 0.0    # cumulative rotation, rad (for max_revs)

        # Visual servoing buffers.
        self.depth_img: Optional[np.ndarray] = None   # HxW float32, metres
        self.depth_shape: Optional[Tuple[int, int]] = None  # (H, W)
        # Latest matched detection (bbox in RGB pixels) + arrival time.
        # (cx, cy, w, h, score) in RGB pixel coords.
        self.last_det: Optional[Tuple[float, float, float, float, float]] = None
        self.last_det_t: float = 0.0
        self.last_visual_acquired_t: float = 0.0  # last time we had det+depth

        self._land_pubs_left = 0
        self._land_timer = None

        # ── QoS ──────────────────────────────────────────────────
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=5)

        # ── Publishers (most are bridged to ROS1) ────────────────
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

        # Target signals from target_watcher_node (used to gate the
        # state-machine transition out of ROTATE_AND_SEARCH).
        self.create_subscription(
            Bool,   "/target_seen",        self._seen_cb,   latched)
        self.create_subscription(
            String, "/target_seen/info",   self._info_cb,   latched)
        # Live objects list — diagnostic only in this mode (XY refines
        # would be ignored by the visual servo loop).
        self.create_subscription(
            String, "/perception/objects", self._objects_cb, latched)

        # Visual inputs.
        self.create_subscription(
            Detection2DArray, self.det_topic, self._det_cb, sensor)
        self.create_subscription(
            Image, self.depth_topic, self._depth_cb, sensor)

        # ── Timers ───────────────────────────────────────────────
        self.create_timer(1.0 / self.tick_hz, self._tick)
        # Rotation control runs continuously; only emits when state matches.
        self.create_timer(1.0 / self.rot_hz, self._rotate_tick)
        # Visual servo loop, same pattern.
        self.create_timer(1.0 / self.visual_hz, self._visual_tick)

        self.create_timer(5.0, self._hb)

        self.get_logger().info("=" * 64)
        self.get_logger().info("room_search_orchestrator ready  (visual close-in)")
        self.get_logger().info(
            f"  room_center = ({self.room_xy[0]:.2f}, {self.room_xy[1]:.2f}) m")
        self.get_logger().info(f"  target_object = {self.target_label!r}")
        self.get_logger().info(
            f"  drone_ns = {self.drone_ns!r}   "
            f"pose = {self.pose_topic} ({self.pose_type})")
        self.get_logger().info(
            f"  detections = {self.det_topic}   depth = {self.depth_topic}")
        self.get_logger().info(
            f"  rgb_image = {self.rgb_W}x{self.rgb_H}")
        self.get_logger().info(
            f"  nav_arrival_radius={self.nav_arr_r:.2f}m  "
            f"rot_rate={self.rot_rate:.2f}rad/s  max_revs={self.max_revs:.1f}")
        self.get_logger().info(
            f"  visual: vx_max={self.vx_max:.2f}m/s  kp_yaw={self.kp_yaw:.2f}  "
            f"yaw_deadband={self.yaw_deadband:.2f}  "
            f"slowdown@{self.slowdown_d:.2f}m  land@{self.land_d:.2f}m")
        self.get_logger().info("=" * 64)

    # ─── State helpers ──────────────────────────────────────────────
    def _enter(self, new: str):
        if new == self.state:
            return
        self.get_logger().info(
            f"room_search: {self.state} -> {new}")
        self.state = new
        self.t_state = _now_s(self)
        # Per-state entry hooks.
        if new == S.NAV_TO_ROOM:
            self._publish_nav_goal(self.room_xy)
            self.last_nav_pub = _now_s(self)
        elif new == S.ROTATE_AND_SEARCH:
            self._set_external_ctrl(True)
            self.rot_yaw_acc = 0.0
        elif new == S.VISUAL_APPROACH:
            # KEEP external_ctrl=True. Closed-loop on the depth image
            # from here all the way to LAND — no /waypoint_nav/goal, no
            # A*, no localisation involvement.
            self._set_external_ctrl(True)
            self.last_visual_acquired_t = _now_s(self)
        elif new == S.LAND:
            # Stop any motion and start the land burst. We keep
            # external_ctrl=True briefly so waypoint_follower doesn't
            # publish a non-zero twist while sjtu_drone's landing
            # controller is taking over.
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

    # ─── Target signal callbacks (just for ROTATE→VISUAL gating) ────
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
        # Pick up the matched class so we can filter detections more
        # precisely than substring-on-target. target_watcher already did
        # the fuzzy match; respect its verdict.
        cname = str(d.get("matched_class", "")).strip().lower()
        if cname:
            # If matched class differs from the user-typed target_object
            # (e.g. target='couch', matched='sofa'), prefer the matched
            # class for detection filtering.
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
        # Kept for future use / diagnostics. The visual servo loop does
        # NOT consume world XY — it closes the loop on the image only.
        pass

    # ─── Visual inputs ──────────────────────────────────────────────
    def _depth_cb(self, msg: Image):
        if msg.encoding != "32FC1":
            self.get_logger().warn(
                f"depth encoding {msg.encoding!r} unsupported; need 32FC1",
                throttle_duration_sec=10.0)
            return
        try:
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width)
        except Exception as e:
            self.get_logger().warn(f"depth decode failed: {e}",
                                   throttle_duration_sec=5.0)
            return
        # Hold the raw view; downstream copies the patch as needed.
        self.depth_img   = arr
        self.depth_shape = (int(msg.height), int(msg.width))

    def _det_cb(self, msg: Detection2DArray):
        # Pick the highest-confidence detection whose class matches the
        # target label (exact or substring, case-insensitive). Doors/
        # walls are vocabulary-only artefacts we never want to chase.
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

    # ─── Depth sampling at the target bbox ──────────────────────────
    def _depth_at_bbox(self, det) -> Optional[float]:
        """Robust depth at the centre of a detection bbox.

        det = (cx_r, cy_r, w_r, h_r, score) in RGB pixel coords.
        We assume the RGB and depth images cover roughly the same FOV
        (true for the sjtu_drone front pair). The bbox centre is scaled
        from RGB into depth pixel coords by the (W_d/W_r, H_d/H_r)
        ratio. A centred patch covering visual_depth_patch_pct of the
        bbox in depth pixels is sampled and the visual_depth_percentile
        percentile of in-range values is returned. None when the patch
        carries fewer than visual_depth_min_valid_px valid samples.
        """
        if self.depth_img is None or self.depth_shape is None:
            return None
        cx_r, cy_r, w_r, h_r, _ = det
        Hd, Wd = self.depth_shape
        # Project RGB bbox centre into depth coords by simple proportional
        # scaling (good enough for cameras with similar FOV; replace with
        # full fx/fy intrinsics scaling if your RGB and depth FOV differ).
        sx = Wd / max(self.rgb_W, 1)
        sy = Hd / max(self.rgb_H, 1)
        u = cx_r * sx
        v = cy_r * sy
        hw = 0.5 * w_r * sx * max(0.05, min(1.0, self.depth_patch))
        hh = 0.5 * h_r * sy * max(0.05, min(1.0, self.depth_patch))
        x0 = int(max(0, u - hw)); x1 = int(min(Wd, u + hw))
        y0 = int(max(0, v - hh)); y1 = int(min(Hd, v + hh))
        if x1 <= x0 or y1 <= y0:
            return None
        patch = self.depth_img[y0:y1, x0:x1]
        valid = np.isfinite(patch) \
                & (patch >= self.depth_min_m) \
                & (patch <= self.depth_max_m)
        if int(valid.sum()) < self.depth_min_n:
            return None
        return float(np.percentile(patch[valid], self.depth_pctl))

    # ─── Rotation tick ──────────────────────────────────────────────
    def _rotate_tick(self):
        if self.state != S.ROTATE_AND_SEARCH:
            return
        dt = 1.0 / max(self.rot_hz, 1.0)
        self.rot_yaw_acc += abs(self.rot_rate) * dt
        self._publish_cmd(0.0, self.rot_rate)

    # ─── Visual servo tick (depth-image-only close-in) ──────────────
    def _visual_tick(self):
        if self.state != S.VISUAL_APPROACH:
            return
        now = _now_s(self)
        det = self.last_det
        det_age = now - self.last_det_t if self.last_det is not None else 1e9

        # No fresh detection in the recent past — hold position. After
        # visual_giveup_s of staying lost, give up.
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

        depth = self._depth_at_bbox(det)
        if depth is None:
            self._publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                "room_search: no valid depth at target bbox; holding",
                throttle_duration_sec=1.0)
            return

        # We have a fresh detection AND a valid depth — visual lock is good.
        self.last_visual_acquired_t = now

        # Terminal: close enough, transition to LAND.
        if depth <= self.land_d:
            self.get_logger().info(
                f"room_search: depth={depth:.2f}m <= land_depth="
                f"{self.land_d:.2f}m  -> LAND")
            self._enter(S.LAND)
            return

        # Yaw: drive bbox centre toward image centre.
        cx, cy, w, h, score = det
        x_off = (cx - 0.5 * self.rgb_W) / max(1e-6, 0.5 * self.rgb_W)
        x_off = max(-1.0, min(1.0, x_off))
        # ROS body frame: +angular.z yaws CCW. With a forward-facing
        # camera, that shifts image content LEFTWARDS. Target right of
        # centre (x_off > 0) → yaw RIGHT, i.e. wz < 0. Hence the minus.
        wz = _saturate(-self.kp_yaw * x_off, self.max_wz)

        # Forward: ramp vx down from vx_max at slowdown_start_m to 0 at
        # land_depth_m. Block forward motion if the target is too far
        # off-axis (yaw to centre it first).
        if abs(x_off) > self.yaw_deadband:
            vx = 0.0
        else:
            span = max(1e-3, self.slowdown_d - self.land_d)
            frac = max(0.0, min(1.0, (depth - self.land_d) / span))
            vx = self.vx_max * frac

        self._publish_cmd(vx, wz)

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

    # ─── Main tick ──────────────────────────────────────────────────
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
            #   * AND we have a fresh detection in /perception/detections
            #     matching the target label (visual lock at this instant)
            now = _now_s(self)
            det_fresh = (self.last_det is not None
                         and (now - self.last_det_t) < self.lost_hover_s)
            if self.target_seen and det_fresh:
                self._publish_cmd(0.0, 0.0)  # brake yaw
                self._enter(S.VISUAL_APPROACH)
                return
            if self.target_seen and not det_fresh:
                # Target was claimed but we don't see it in the current
                # frame — keep rotating, it'll re-enter the FOV soon.
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

        # VISUAL_APPROACH / LAND are driven by their own timers; the
        # main tick only kicks state transitions on entry/exit.

    # ─── Heartbeat ──────────────────────────────────────────────────
    def _hb(self):
        bits = [f"state={self.state}",
                f"t_in={self._t_in():.1f}s",
                f"target_seen={self.target_seen}"]
        if self.cur_xy is not None:
            bits.append(f"pose=({self.cur_xy[0]:.2f},{self.cur_xy[1]:.2f})")
        else:
            bits.append("pose=None")
        bits.append(f"depth={'y' if self.depth_img is not None else 'NO'}")
        if self.last_det is not None:
            age = _now_s(self) - self.last_det_t
            cx, cy, w, h, s = self.last_det
            bits.append(f"det(age={age:.1f}s "
                        f"bbox_c=({cx:.0f},{cy:.0f}) score={s:.2f})")
        else:
            bits.append("det=None")
        if self.state == S.ROTATE_AND_SEARCH:
            bits.append(f"yaw_acc={self.rot_yaw_acc:.2f}rad")
        if self.state == S.VISUAL_APPROACH and self.last_det is not None:
            d = self._depth_at_bbox(self.last_det)
            bits.append(f"depth_at_target={d:.2f}m" if d is not None
                        else "depth_at_target=None")
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
