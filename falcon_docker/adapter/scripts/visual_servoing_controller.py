#!/usr/bin/env python3
"""
visual_servoing_controller.py — the VISUAL_SERVOING orchestrator.

WHAT PUTS US IN THE MODE is out of scope (external state machine over
/xtend/demo_mode). This node implements WHAT HAPPENS once the system
reports `visual_servoing`:

  1. Take the A* path captured *before* the switch (pose still good).
  2. Pick the furthest waypoint in the camera FOV and not behind a
     known wall            (nav_geom.furthest_visible_waypoint).
  3. Convert it to a body-frame POINTGOAL and lock the pixel patch we
     are aiming at + its depth (pixel_goal_tracker), so the goal can be
     refreshed from VISION, not from drifting localization.

Then TWO loops run concurrently:

  • inference thread (~infer_hz): snapshot telemetry as the NEW ZERO →
    refresh pointgoal from the pixel tracker → ask NavDP for 24
    body-frame points → atomically install (points, snapshot).

  • control loop (~ctrl_hz, faster): read current telemetry, express it
    in the inference-LOCAL frame as the delta since the snapshot, run
    TrajectoryTracker (turn-then-fly, same algorithm as
    waypoint_follower) and publish /cmd_vel.

Re-zeroing is explicit: when a new inference installs its points it
also installs the telemetry snapshot taken at request time, and the
control loop measures the drone relative to THAT snapshot. Absolute
localization drift is dropped every inference; only the short
within-window relative motion (locally accurate) is used for tracking.

We own /cmd_vel while in visual_servoing. waypoint_follower goes
passive in this mode (one-line guard in its _ctrl_loop — see README).
"""
import math
import os
import sys
import threading

import numpy as np
import rospy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_geom                                       # noqa: E402
from navdp_client import NavDPClient, NavDPError      # noqa: E402
from pixel_goal_tracker import PixelGoalTracker       # noqa: E402
from trajectory_tracker import TrajectoryTracker      # noqa: E402

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import String

VISUAL_SERVOING = "visual_servoing"


def _decode_rgb(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height, msg.width, -1)
    if (msg.encoding or "rgb8").lower().startswith("bgr"):
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img[:, :, :3])


def _decode_depth(msg):
    if msg.encoding in ("16UC1", "16uc1"):
        d = np.frombuffer(msg.data, dtype=np.uint16).astype(np.float32) * 1e-3
    else:                                              # 32FC1 default
        d = np.frombuffer(msg.data, dtype=np.float32)
    return np.ascontiguousarray(d.reshape(msg.height, msg.width))


def _saturate(v, lim):
    return lim if v > lim else (-lim if v < -lim else v)


class VisualServoingController:
    def __init__(self):
        rospy.init_node("visual_servoing_controller")
        G = rospy.get_param

        self.drone_ns = G("~drone_ns", "/simple_drone")
        self.rgb_topic = G("~rgb_topic", "/camera/rgb/image_raw")
        self.depth_topic = G("~depth_topic",
                             self.drone_ns + "/front_depth/depth/image_raw")
        self.astar_topic = G("~astar_path_topic", "/path/waypoints")
        self.bev_topic = G("~bev_topic", "/falcon/bev_2d")
        self.demo_mode_topic = G("~demo_mode_topic", "/xtend/demo_mode")

        self.infer_hz = float(G("~infer_hz", 2.0))
        self.ctrl_hz = float(G("~ctrl_hz", 8.0))
        self.max_range = float(G("~goal_max_range_m", 8.0))
        self.min_range = float(G("~goal_min_range_m", 0.30))

        # Slew / saturation (mirror waypoint_follower so motion feels
        # identical and the platform invariant holds).
        self.vel_xy_sat = float(G("~vel_xy_sat", 1.25))
        self.yaw_rate_sat = float(G("~yaw_rate_sat", 2.4))
        self.accel_limit = float(G("~accel_limit", 1.5))
        self.yaw_accel_limit = float(G("~yaw_accel_limit", 3.5))

        P = "/uav_model/sensing_parameters"
        fx = float(G(P + "/camera_intrinsics/fx", 320.0))
        fy = float(G(P + "/camera_intrinsics/fy", 320.0))
        cx = float(G(P + "/camera_intrinsics/cx", 320.0))
        cy = float(G(P + "/camera_intrinsics/cy", 240.0))
        iw = float(G(P + "/image_width", 640.0))
        self.K = (fx, fy, cx, cy)
        self.half_fov = math.atan2(iw * 0.5, fx)

        self.T_b_c = nav_geom.make_T_b_c(
            float(G("~cam_offset_x", 0.2)),
            float(G("~cam_offset_y", 0.0)),
            float(G("~cam_offset_z", 0.0)))

        self.navdp = NavDPClient(
            G("~navdp_url", "http://127.0.0.1:8888/pointgoal"),
            timeout_s=float(G("~navdp_timeout_s", 2.0)),
            logger=rospy.loginfo)
        self.pix = PixelGoalTracker(
            patch=int(G("~pix_patch", 21)),
            search=int(G("~pix_search", 80)),
            min_score=float(G("~pix_min_score", 0.45)),
            logger=rospy.loginfo)
        self.tracker = TrajectoryTracker(
            pos_radius=float(G("~pos_radius", 0.35)),
            yaw_settle=float(G("~yaw_settle", 0.05)),
            vel_x=float(G("~vel_x", 0.3)),
            yaw_rate=float(G("~yaw_rate", 0.7)),
            yaw_kp=float(G("~yaw_kp", 1.8)),
            skip_yaw_thresh=float(G("~skip_yaw_thresh", 0.25)),
            passed_bearing_deg=float(G("~passed_bearing_deg", 100.0)),
            yaw_realign_thresh=float(G("~yaw_realign_thresh", 0.40)),
            logger=rospy.loginfo)

        # Shared state
        self._lock = threading.Lock()
        self._pose = None
        self._rgb = None
        self._depth = None
        self._grid = None
        self._astar_xy = []
        self._mode = None
        self._engaged = False
        self._goal_world = None
        self._goal_lost_logged = False
        # The re-zeroable pairing: trajectory points + the telemetry
        # pose that is their (0,0,0). Installed atomically each inference.
        self._traj_snapshot = None        # (x0, y0, yaw0)
        self._have_traj = False
        self.last_vx = 0.0
        self.last_wz = 0.0

        self.cmd_pub = rospy.Publisher(self.drone_ns + "/cmd_vel", Twist,
                                       queue_size=1)

        rospy.Subscriber(self.demo_mode_topic, String, self._mode_cb,
                          queue_size=5)
        rospy.Subscriber(self.drone_ns + "/gt_pose", Pose, self._pose_cb,
                          queue_size=10)
        rospy.Subscriber(self.rgb_topic, Image, self._rgb_cb, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self._depth_cb,
                          queue_size=1)
        rospy.Subscriber(self.bev_topic, OccupancyGrid, self._grid_cb,
                          queue_size=1)
        rospy.Subscriber(self.astar_topic, Path, self._astar_cb,
                          queue_size=1)

        self._infer_thread = threading.Thread(target=self._infer_loop)
        self._infer_thread.daemon = True
        self._infer_thread.start()
        rospy.Timer(rospy.Duration(1.0 / self.ctrl_hz), self._ctrl_loop)

        rospy.loginfo("=" * 64)
        rospy.loginfo("visual_servoing_controller ready")
        rospy.loginfo("  rgb=%s  depth=%s", self.rgb_topic, self.depth_topic)
        rospy.loginfo("  infer@%.1fHz  ctrl@%.1fHz  half_fov=%.0f deg",
                      self.infer_hz, self.ctrl_hz,
                      math.degrees(self.half_fov))
        rospy.loginfo("  cmd_vel out = %s/cmd_vel", self.drone_ns)
        rospy.loginfo("  navdp = %s", self.navdp.url)
        rospy.loginfo("=" * 64)

    # ── subscribers ─────────────────────────────────────────────────
    def _mode_cb(self, m):
        with self._lock:
            self._mode = (m.data or "").strip().lower()

    def _pose_cb(self, m):
        with self._lock:
            self._pose = m

    def _rgb_cb(self, m):
        try:
            img = _decode_rgb(m)
        except Exception as e:                          # noqa: BLE001
            rospy.logwarn_throttle(5.0, "vs: rgb decode: %s", e)
            return
        with self._lock:
            self._rgb = img

    def _depth_cb(self, m):
        try:
            d = _decode_depth(m)
        except Exception as e:                          # noqa: BLE001
            rospy.logwarn_throttle(5.0, "vs: depth decode: %s", e)
            return
        with self._lock:
            self._depth = d

    def _grid_cb(self, m):
        g = nav_geom.OccGrid(m.info.width, m.info.height,
                             m.info.resolution,
                             m.info.origin.position.x,
                             m.info.origin.position.y, m.data)
        with self._lock:
            self._grid = g

    def _astar_cb(self, m):
        with self._lock:
            if self._engaged:           # we don't republish, but stay safe
                return
            self._astar_xy = [(p.pose.position.x, p.pose.position.y)
                              for p in m.poses]

    # ── helpers ─────────────────────────────────────────────────────
    @staticmethod
    def _xyyaw(pose):
        o = pose.orientation
        return (pose.position.x, pose.position.y,
                nav_geom.yaw_from_quat(o.x, o.y, o.z, o.w))

    def _snapshot(self):
        with self._lock:
            return (self._pose, self._rgb, self._depth, self._grid,
                    list(self._astar_xy), self._mode)

    def _engage(self, pose, rgb, depth, grid, astar):
        if not astar:
            rospy.logwarn_throttle(2.0, "vs: no A* path captured")
            return False
        ox, oy, oyaw = self._xyyaw(pose)
        pick = nav_geom.furthest_visible_waypoint(
            astar, (ox, oy, oyaw), grid,
            self.half_fov, self.max_range, self.min_range)
        if pick is None:
            rospy.logwarn_throttle(2.0, "vs: no A* waypoint visible")
            return False
        idx, (wx, wy), (gx, gy) = pick
        self._goal_world = (wx, wy)
        ok = self.pix.lock(rgb, depth, (gx, gy, 0.0), self.K, self.T_b_c)
        self.tracker.set_trajectory([])     # nothing to track until 1st infer
        self._have_traj = False
        self._engaged = True
        self._goal_lost_logged = False
        rospy.loginfo("vs: ENGAGE  wp#%d world=(%.2f,%.2f) body=(%.2f,%.2f) "
                      "pixel_lock=%s", idx, wx, wy, gx, gy,
                      "ok" if ok else "FAILED")
        return True

    def _pointgoal(self, pose, rgb, depth):
        g = self.pix.update(rgb, depth, self.K, self.T_b_c)
        if g is not None:
            self._goal_lost_logged = False
            return g, "vision"
        if self._goal_world is not None:
            ox, oy, oyaw = self._xyyaw(pose)
            gx, gy = nav_geom.world_to_body(
                self._goal_world[0], self._goal_world[1], ox, oy, oyaw)
            if not self._goal_lost_logged:
                rospy.logwarn("vs: pixel lock lost — localization "
                              "pointgoal fallback (drift-prone)")
                self._goal_lost_logged = True
            return (gx, gy), "localization"
        return None, "none"

    # ── inference thread: produce points + reset the zero ───────────
    def _infer_loop(self):
        rate = rospy.Rate(self.infer_hz)
        while not rospy.is_shutdown():
            pose, rgb, depth, grid, astar, mode = self._snapshot()
            if mode != VISUAL_SERVOING:
                rate.sleep()
                continue
            if pose is None or rgb is None or depth is None:
                rospy.logwarn_throttle(2.0, "vs: waiting pose/rgb/depth")
                rate.sleep()
                continue
            if not self._engaged and not self._engage(
                    pose, rgb, depth, grid, astar):
                rate.sleep()
                continue

            # THE NEW ZERO: telemetry at request time, paired with the
            # trajectory NavDP returns for the observation taken now.
            snap = self._xyyaw(pose)
            pg, src = self._pointgoal(pose, rgb, depth)
            if pg is None:
                rate.sleep()
                continue
            try:
                traj = self.navdp.infer(pg, rgb, depth)
            except NavDPError as e:
                rospy.logwarn_throttle(2.0, "vs: %s — holding", e)
                rate.sleep()
                continue

            with self._lock:
                self.tracker.set_trajectory(traj)
                self._traj_snapshot = snap
                self._have_traj = True
            rospy.loginfo_throttle(
                1.0, "vs: NEW ZERO=(%.2f,%.2f,%.0fdeg) pointgoal(%s)="
                "(%.2f,%.2f)  %d pts -> tracker",
                snap[0], snap[1], math.degrees(snap[2]), src,
                pg[0], pg[1], len(traj))
            rate.sleep()

    # ── control loop: track the points in the local frame ──────────
    def _ctrl_loop(self, _evt):
        with self._lock:
            pose = self._pose
            mode = self._mode
            snap = self._traj_snapshot
            have = self._have_traj

        if mode != VISUAL_SERVOING:
            if self._engaged:
                rospy.loginfo("vs: left visual_servoing — disengaging")
                self._engaged = False
                self._have_traj = False
                self._goal_world = None
            return

        if pose is None or not have or snap is None:
            self._publish(0.0, 0.0)            # hold until first inference
            return

        # Drone pose in the INFERENCE-LOCAL frame = telemetry delta
        # since the snapshot. This is the only place telemetry is used,
        # and only as a *relative* measure over one short window.
        x, y, yaw = self._xyyaw(pose)
        sx, sy, syaw = snap
        lx, ly = nav_geom.world_to_body(x, y, sx, sy, syaw)
        lyaw = nav_geom.wrap_pi(yaw - syaw)

        vx, wz = self.tracker.step(lx, ly, lyaw)
        self._publish(vx, wz)

    # ── safe publish (invariants + slew, like waypoint_follower) ────
    def _publish(self, vx, wz):
        if abs(vx) > 1e-6 and abs(wz) > 1e-6:          # invariant guard
            wz = 0.0
        vx = _saturate(vx, self.vel_xy_sat)
        wz = _saturate(wz, self.yaw_rate_sat)
        dt = 1.0 / self.ctrl_hz
        vx = self._slew(vx, self.last_vx, self.accel_limit * dt)
        wz = self._slew(wz, self.last_wz, self.yaw_accel_limit * dt)
        m = Twist()
        m.linear.x = vx
        m.linear.y = 0.0          # HARDWIRED
        m.linear.z = 0.0          # HARDWIRED
        m.angular.z = wz
        self.cmd_pub.publish(m)
        self.last_vx, self.last_wz = vx, wz

    @staticmethod
    def _slew(target, cur, max_step):
        d = target - cur
        if d > max_step:
            return cur + max_step
        if d < -max_step:
            return cur - max_step
        return target


if __name__ == "__main__":
    try:
        VisualServoingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass