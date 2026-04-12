#!/usr/bin/env python3
"""
internvla_n1_drone_live.py
==========================
Live InternVLA-N1 (dual system) → drone control, NavDP-style.

The server is assumed to ALWAYS return a trajectory (see patch notes).
Every response is treated as a NavDP-style body-frame trajectory and
drawn / followed accordingly.

Visualisation
-------------
  • Live RGB with trajectory + last S2 pixel-goal overlaid EVERY FRAME
    (cached body-frame data is reprojected each tick so the user sees
    the plan continuously, not only at replan time). The pixel goal
    PERSISTS between replans — it's only updated when S2 actually
    grounds a new one, never cleared.
  • 2-D occupancy map with original (red) + corrected (green) paths.
  • Correction-debug panel from TrajectorySafetyCorrector.

Controls
--------
  SPACE   force an immediate replan
  P       pause / resume auto-replanning
  S       stop & clear trajectory
  G       clear the cached pixel goal (debug)
  Q/ESC   quit
"""

import argparse, io, json, math, os, time, threading, base64
import cv2, numpy as np, requests, yaml
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from trajectory_safety_corrector import TrajectorySafetyCorrector

# ── camera intrinsics (same as navdp_drone_live) ──
FX, FY, CX, CY = 320.0, 320.0, 320.5, 240.5
INTRINSIC = [[FX, 0, CX], [0, FY, CY], [0, 0, 1]]


# =====================================================================
# Projection helpers (module-level so the live-loop can use them freely)
# =====================================================================
def _to_pixel(x_fwd: float, y_left: float, img_w: int, img_h: int,
              cam_height: float):
    """Body-frame waypoint (fwd, left) → image pixel on the ground plane.
    Clamps near-field points to the image bottom instead of dropping them,
    so a short-range trajectory still renders something visible."""
    if x_fwd < 0.02:
        return None
    cam_x = int(FX * (-y_left) / x_fwd + CX)
    cam_y = int(FY * cam_height / x_fwd + CY)
    # clamp instead of drop — keeps near points attached to the origin
    cam_x = max(0, min(img_w - 1, cam_x))
    cam_y = max(0, min(img_h - 1, cam_y))
    return (cam_x, cam_y)


def _critic_color(value: float):
    norm = np.clip(-value * 0.1, 0.0, 1.0)
    lut = np.array([[[int(norm * 255)]]], dtype=np.uint8)
    bgr = cv2.applyColorMap(lut, cv2.COLORMAP_JET)
    return tuple(int(c) for c in bgr[0, 0])


def draw_overlay(img_bgr: np.ndarray,
                 best_traj: np.ndarray | None,
                 all_traj:  np.ndarray | None,
                 all_vals:  np.ndarray | None,
                 pixel_goal: tuple[int, int] | None,
                 cam_height: float,
                 label: str = "") -> np.ndarray:
    """
    Draw trajectory (candidates + best) and persistent pixel-goal on
    a live BGR frame. Works in place on a copy.
    """
    vis = img_bgr.copy()
    h, w = vis.shape[:2]
    origin = (int(CX), h - 1)

    # candidates
    if all_traj is not None and all_vals is not None and len(all_traj) > 1:
        for traj, val in zip(all_traj, all_vals):
            colour = _critic_color(float(val))
            pts = [_to_pixel(float(wp[0]), float(wp[1]), w, h, cam_height) for wp in traj]
            first = next((p for p in pts if p is not None), None)
            if first:
                cv2.line(vis, origin, first, colour, 2)
            for i in range(len(pts) - 1):
                if pts[i] and pts[i + 1]:
                    cv2.line(vis, pts[i], pts[i + 1], colour, 2)

    # best trajectory — thick white outline + green fill
    if best_traj is not None:
        best_pts = [_to_pixel(float(wp[0]), float(wp[1]), w, h, cam_height) for wp in best_traj]
        first_best = next((p for p in best_pts if p is not None), None)
        if first_best:
            cv2.line(vis, origin, first_best, (255, 255, 255), 4)
            cv2.line(vis, origin, first_best, (0, 255, 0), 2)
        for i in range(len(best_pts) - 1):
            if best_pts[i] and best_pts[i + 1]:
                cv2.line(vis, best_pts[i], best_pts[i + 1], (255, 255, 255), 4)
                cv2.line(vis, best_pts[i], best_pts[i + 1], (0, 255, 0), 2)
        # waypoint dots
        for p in best_pts:
            if p is not None:
                cv2.circle(vis, p, 3, (0, 255, 0), -1)

    # persistent pixel-goal crosshair (yellow)
    if pixel_goal is not None:
        u, v = int(pixel_goal[0]), int(pixel_goal[1])
        if 0 <= u < w and 0 <= v < h:
            cv2.drawMarker(vis, (u, v), (0, 255, 255),
                           cv2.MARKER_CROSS, markerSize=30, thickness=3)
            cv2.circle(vis, (u, v), 18, (0, 255, 255), 2)
            cv2.putText(vis, "S2 goal", (u + 22, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    if label:
        cv2.putText(vis, label, (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return vis


def draw_bev_inset(img_bgr: np.ndarray,
                   best_traj: np.ndarray | None,
                   all_traj:  np.ndarray | None,
                   all_vals:  np.ndarray | None,
                   size: int = 200,
                   range_m: float = 4.0,
                   margin: int = 10) -> np.ndarray:
    """
    Draw a small top-down (BEV) HUD in the top-right corner showing the
    body-frame trajectory. Unlike the ground-plane projection, this
    always renders regardless of camera geometry / drone altitude.

    +fwd is up, +left is left. Drone sits at bottom-center.
    """
    h_img, w_img = img_bgr.shape[:2]
    x0 = w_img - size - margin
    y0 = margin
    x1 = x0 + size
    y1 = y0 + size

    # translucent dark background
    overlay = img_bgr.copy()
    cv2.rectangle(overlay, (x0, y0), (x1, y1), (20, 20, 20), -1)
    img_bgr = cv2.addWeighted(overlay, 0.55, img_bgr, 0.45, 0)
    cv2.rectangle(img_bgr, (x0, y0), (x1, y1), (90, 90, 90), 1)

    cx = x0 + size // 2
    cy = y1 - 20                 # drone at bottom-center
    px_per_m = (size - 30) / range_m

    # range rings + axes
    for r_m in range(1, int(range_m) + 1):
        cv2.circle(img_bgr, (cx, cy), int(r_m * px_per_m), (60, 60, 60), 1)
    cv2.line(img_bgr, (cx, y0 + 4), (cx, cy), (60, 60, 60), 1)
    cv2.line(img_bgr, (x0 + 4, cy), (x1 - 4, cy), (60, 60, 60), 1)

    def to_px(fwd, left):
        return (int(cx - left * px_per_m),
                int(cy - fwd  * px_per_m))

    # candidates
    if all_traj is not None and all_vals is not None and len(all_traj) > 1:
        for traj, val in zip(all_traj, all_vals):
            colour = _critic_color(float(val))
            pts = [to_px(float(wp[0]), float(wp[1])) for wp in traj]
            for i in range(len(pts) - 1):
                cv2.line(img_bgr, pts[i], pts[i + 1], colour, 1)

    # best trajectory
    if best_traj is not None and len(best_traj) > 0:
        pts = [to_px(float(wp[0]), float(wp[1])) for wp in best_traj]
        for i in range(len(pts) - 1):
            cv2.line(img_bgr, pts[i], pts[i + 1], (0, 255, 0), 2)
        for p in pts:
            cv2.circle(img_bgr, p, 2, (0, 255, 0), -1)
        # endpoint marker
        cv2.circle(img_bgr, pts[-1], 5, (0, 0, 255), 2)
        # endpoint text
        fwd_end = float(best_traj[-1, 0])
        left_end = float(best_traj[-1, 1])
        cv2.putText(img_bgr,
                    f"end ({fwd_end:+.2f},{left_end:+.2f})m",
                    (x0 + 4, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

    # drone marker
    cv2.drawMarker(img_bgr, (cx, cy), (255, 255, 0),
                   cv2.MARKER_DIAMOND, markerSize=10, thickness=2)

    # labels
    cv2.putText(img_bgr, "BEV", (x0 + 4, y0 + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    cv2.putText(img_bgr, f"{int(range_m)}m", (x1 - 26, y0 + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 140, 140), 1)
    return img_bgr


# =====================================================================
# Node
# =====================================================================
class InternVLAN1DroneController(Node):
    def __init__(self, server_url: str, instruction: str,
                 map_yaml: str | None,
                 rgb_topic: str, depth_topic: str,
                 n_iterations: int = 5,
                 correction_gain: float = 0.6,
                 corrector_max_corr: float = 0.25,
                 depth_scale: float = 1.73):
        super().__init__("internvla_n1_drone_controller")
        self.server_url = server_url.rstrip("/")
        self.instruction = instruction

        # shared state
        self.drone_pose = None
        self.rgb_frame: np.ndarray | None = None
        self.depth_frame: np.ndarray | None = None

        # trajectory state (world frame, for pure pursuit)
        self.world_wps: list[tuple[float, float]] | None = None
        self.world_wps_original: list[tuple[float, float]] | None = None
        self.wp_idx = 0
        self.following = False

        # cached body-frame data for LIVE overlay — these persist so the
        # camera panel keeps showing the plan between replans
        self.last_best_traj: np.ndarray | None = None
        self.last_all_traj:  np.ndarray | None = None
        self.last_all_vals:  np.ndarray | None = None
        self.last_pixel_goal: tuple[int, int] | None = None   # never cleared by replan
        self.pixel_goal_frame: np.ndarray | None = None       # BGR frame at the moment pg was last set
        self.pixel_goal_t: float = 0.0                         # wall time when pg was last set
        self.last_source: str = "none"

        # buffer for the most recent frame sent to the server (so we can
        # stash it as pixel_goal_frame if this plan grounds a new goal)
        self._last_rgb_snap_bgr: np.ndarray | None = None

        # viz extras
        self.corrector_vis: np.ndarray | None = None
        self.last_rtt: float = 0.0
        self.last_plan_t: float = 0.0

        # policy session
        self._policy_init = True
        self._http_idx = -1

        # 2-D map (optional)
        self.map_img = self.map_res = self.map_origin = None
        if map_yaml and os.path.exists(map_yaml):
            self._load_map(map_yaml)

        # ROS
        cb = ReentrantCallbackGroup()
        self.create_subscription(Pose,  "/simple_drone/gt_pose",  self._pose_cb, 10, callback_group=cb)
        self.create_subscription(Image, rgb_topic,                self._rgb_cb,   5, callback_group=cb)
        self.create_subscription(Image, depth_topic,              self._depth_cb, 5, callback_group=cb)
        self.cmd_pub = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)

        # safety corrector
        self.corrector = TrajectorySafetyCorrector(
            n_iterations=n_iterations,
            correction_gain=correction_gain,
            max_correction_m=corrector_max_corr,
            depth_scale=depth_scale,
        )
        qos_latched = QoSProfile(depth=1,
                                 reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map_local',
                                 self._map_cb, qos_latched, callback_group=cb)
        self.create_subscription(Image, '/potential_field/u_rep',
                                 self._u_rep_cb, qos_latched, callback_group=cb)

        # control parameters (identical to navdp_drone_live)
        self.cruise_speed = 0.35
        self.wp_tolerance = 0.25
        self.cruise_alt = 1.5
        self.alt_kp = 1.2
        self.max_vz = 0.3
        self.yaw_kp = 0.6
        self.max_yaw_rate = 0.4

    # ─── ROS callbacks ───────────────────────────────────────────
    def _pose_cb(self, msg: Pose):
        q = msg.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        self.drone_pose = (msg.position.x, msg.position.y, msg.position.z, yaw)

    def _rgb_cb(self, msg: Image):
        if msg.encoding == "rgb8":
            self.rgb_frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
        elif msg.encoding == "bgr8":
            bgr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            self.rgb_frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).copy()

    def _depth_cb(self, msg: Image):
        if msg.encoding == "32FC1":
            self.depth_frame = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
        elif msg.encoding == "16UC1":
            raw = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
            self.depth_frame = raw.astype(np.float32) / 1000.0

    def _map_cb(self, msg: OccupancyGrid):
        self.corrector.update_grid_metadata(msg)

    def _u_rep_cb(self, msg: Image):
        u_rep = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
        self.corrector.update_u_rep(u_rep)

    # ─── map helpers (same as navdp_drone_live) ──────────────────
    def _load_map(self, yaml_path: str):
        with open(yaml_path) as f:
            info = yaml.safe_load(f)
        self.map_res = float(info["resolution"])
        self.map_origin = info["origin"]
        img_path = info["image"]
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)
        raw = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        if raw is not None:
            self.map_img = raw
            self.get_logger().info(f"Map loaded: {img_path} {raw.shape[1]}×{raw.shape[0]} res={self.map_res}")

    def _world_to_map_cv(self, x: float, y: float) -> tuple[int, int]:
        ox, oy = self.map_origin[0], self.map_origin[1]
        col = (x - ox) / self.map_res
        row = self.map_img.shape[0] - (y - oy) / self.map_res
        return int(round(col)), int(round(row))

    # ─── HTTP ────────────────────────────────────────────────────
    def plan_once(self) -> dict | None:
        if self.rgb_frame is None or self.depth_frame is None:
            return None
        rgb_snap   = self.rgb_frame.copy()
        depth_snap = self.depth_frame.copy()
        # cache as BGR so we can freeze it for the pixel-goal panel
        self._last_rgb_snap_bgr = cv2.cvtColor(rgb_snap, cv2.COLOR_RGB2BGR)

        rgb_buf = io.BytesIO()
        PILImage.fromarray(rgb_snap, "RGB").save(rgb_buf, format="JPEG")
        rgb_buf.seek(0)

        d_u16 = np.clip(depth_snap * 10000.0, 0, 65535).astype(np.uint16)
        d_buf = io.BytesIO()
        PILImage.fromarray(d_u16).save(d_buf, format="PNG")
        d_buf.seek(0)

        files = {"image": ("rgb.jpg", rgb_buf, "image/jpeg"),
                 "depth": ("depth.png", d_buf, "image/png")}
        payload = {"json": json.dumps({
            "reset": self._policy_init,
            "idx": self._http_idx,
            "instruction": self.instruction,
        })}
        t0 = time.time()
        try:
            r = requests.post(f"{self.server_url}/eval_dual",
                              files=files, data=payload, timeout=100)
        except requests.RequestException as e:
            self.get_logger().error(f"http error: {e}")
            return None
        self.last_rtt = time.time() - t0
        self._policy_init = False
        self._http_idx += 1
        if r.status_code != 200:
            self.get_logger().error(f"server {r.status_code}: {r.text[:200]}")
            return None
        try:
            return r.json()
        except Exception:
            self.get_logger().error(f"bad json: {r.text[:200]}")
            return None

    # ─── trajectory → world (identical to navdp_drone_live) ──────
    @staticmethod
    def traj_to_world(traj: np.ndarray, ref_x: float, ref_y: float, ref_yaw: float):
        c, s = math.cos(ref_yaw), math.sin(ref_yaw)
        wps = []
        for wp in traj:
            fwd, left = float(wp[0]), float(wp[1])
            wx = ref_x + fwd * c - left * s
            wy = ref_y + fwd * s + left * c
            wps.append((wx, wy))
        return wps

    # ─── drone motion (identical to navdp_drone_live) ────────────
    def stop(self):
        t = Twist()
        for _ in range(5):
            self.cmd_pub.publish(t)
            time.sleep(0.02)
        self.following = False

    def control_step(self):
        if not self.following or self.world_wps is None or self.drone_pose is None:
            return
        px, py, pz, yaw = self.drone_pose

        while self.wp_idx < len(self.world_wps):
            wx, wy = self.world_wps[self.wp_idx]
            if math.hypot(wx - px, wy - py) < self.wp_tolerance:
                self.wp_idx += 1
            else:
                break

        if self.wp_idx >= len(self.world_wps):
            self.get_logger().info("Trajectory complete.")
            self.stop()
            return

        look_idx = min(self.wp_idx + 3, len(self.world_wps) - 1)
        tx, ty = self.world_wps[look_idx]
        dx, dy = tx - px, ty - py
        dist = math.hypot(dx, dy)
        if dist < 0.03:
            return

        fx, fy = self.world_wps[-1]
        d_goal = math.hypot(fx - px, fy - py)
        speed = max(0.08, self.cruise_speed * (0.3 + 0.7 * min(d_goal, 1.0)))

        vx_w = dx / dist * speed
        vy_w = dy / dist * speed
        vz = max(-self.max_vz, min(self.alt_kp * (self.cruise_alt - pz), self.max_vz))

        des_yaw = math.atan2(dy, dx)
        yerr = des_yaw - yaw
        while yerr >  math.pi: yerr -= 2 * math.pi
        while yerr < -math.pi: yerr += 2 * math.pi
        yr = max(-self.max_yaw_rate, min(self.yaw_kp * yerr, self.max_yaw_rate))

        c, s = math.cos(yaw), math.sin(yaw)
        tw = Twist()
        tw.linear.x  =  vx_w * c + vy_w * s
        tw.linear.y  = -vx_w * s + vy_w * c
        tw.linear.z  = vz
        tw.angular.z = yr
        self.cmd_pub.publish(tw)

    # ─── map visualisation (identical to navdp_drone_live) ───────
    def render_map(self) -> np.ndarray | None:
        if self.map_img is None:
            return None
        vis = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR) \
              if self.map_img.ndim == 2 else self.map_img.copy()

        if self.world_wps_original:
            pts = [self._world_to_map_cv(x, y) for x, y in self.world_wps_original]
            for i in range(len(pts) - 1):
                cv2.line(vis, pts[i], pts[i + 1], (0, 0, 255), 2)
            for p in pts:
                cv2.circle(vis, p, 2, (0, 0, 200), -1)

        if self.world_wps:
            pts = [self._world_to_map_cv(x, y) for x, y in self.world_wps]
            for i in range(len(pts) - 1):
                cv2.line(vis, pts[i], pts[i + 1], (0, 255, 0), 2)
            for p in pts:
                cv2.circle(vis, p, 3, (0, 200, 0), -1)
            cv2.circle(vis, pts[-1], 8, (0, 0, 255), 2)
            cv2.circle(vis, pts[0],  8, (255, 0, 0), 2)

        cv2.putText(vis, "Original",  (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        cv2.putText(vis, "Corrected", (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        if self.drone_pose:
            dc, dr = self._world_to_map_cv(self.drone_pose[0], self.drone_pose[1])
            cv2.circle(vis, (dc, dr), 6, (0, 0, 255), -1)
            yaw = self.drone_pose[3]; a = 20
            cv2.arrowedLine(vis, (dc, dr),
                            (dc + int(a * math.cos(yaw)),
                             dr - int(a * math.sin(yaw))),
                            (0, 0, 255), 2, tipLength=0.4)
        return vis

    # ─── one full plan→correct→follow cycle ──────────────────────
    def replan_and_follow(self):
        result = self.plan_once()
        if result is None or self.drone_pose is None:
            return False

        best_traj = np.asarray(result["trajectory"], dtype=np.float32)
        all_traj  = np.asarray(result.get("all_trajectory", [best_traj]), dtype=np.float32)
        all_vals  = np.asarray(result.get("all_values",      [0.0]),       dtype=np.float32)

        # Cache for live overlay. These stay sticky until the next plan.
        self.last_best_traj = best_traj
        self.last_all_traj  = all_traj
        self.last_all_vals  = all_vals
        self.last_source    = result.get("source", "trajectory")

        # Pixel goal is PERSISTENT — only update when the server gave us a
        # new one; never reset to None just because this response lacked it.
        # When it IS updated, freeze the frame that was used for inference
        # so the user can see what S2 was looking at when it grounded.
        pg = result.get("pixel_goal")
        if pg is not None:
            self.last_pixel_goal = (int(pg[0]), int(pg[1]))
            if self._last_rgb_snap_bgr is not None:
                self.pixel_goal_frame = self._last_rgb_snap_bgr.copy()
                self.pixel_goal_t = time.time()

        # potential-field correction
        corrected = self.corrector.correct(best_traj, verbose=False)
        max_shift = float(np.max(np.abs(corrected[:, :2] - best_traj[:, :2])))
        self.corrector_vis = self.corrector.visualize_corrections(best_traj, corrected)

        # body → world
        ref = self.drone_pose
        self.world_wps_original = self.traj_to_world(best_traj, ref[0], ref[1], ref[3])
        self.world_wps          = self.traj_to_world(corrected, ref[0], ref[1], ref[3])
        self.wp_idx = 0
        self.following = True
        self.last_plan_t = time.time()
        self.get_logger().info(
            f"[{self.last_source}] traj N={len(corrected)}  "
            f"end=({corrected[-1,0]:+.2f},{corrected[-1,1]:+.2f})  "
            f"corr_max={max_shift:.2f}m  rtt={self.last_rtt:.2f}s  "
            f"pg={self.last_pixel_goal}")
        return True


# =====================================================================
# main loop
# =====================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",        type=int, default=5801)
    ap.add_argument("--host",        type=str, default="127.0.0.1")
    ap.add_argument("--instruction", type=str, required=True,
                    help="Natural-language task for System-2")
    ap.add_argument("--map_yaml",    type=str, default="")
    ap.add_argument("--rgb_topic",   type=str, default="/simple_drone/front/image_raw")
    ap.add_argument("--depth_topic", type=str, default="/simple_drone/front_depth/depth/image_raw")
    ap.add_argument("--alt",         type=float, default=1.5)
    ap.add_argument("--replan_hz",   type=float, default=2.0)
    ap.add_argument("--iters",       type=int,   default=5)
    ap.add_argument("--corr_gain",   type=float, default=0.6)
    ap.add_argument("--corr_max",    type=float, default=0.25)
    ap.add_argument("--depth_scale", type=float, default=1.73)
    args = ap.parse_args()

    rclpy.init()
    node = InternVLAN1DroneController(
        server_url=f"http://{args.host}:{args.port}",
        instruction=args.instruction,
        map_yaml=args.map_yaml or None,
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        n_iterations=args.iters,
        correction_gain=args.corr_gain,
        corrector_max_corr=args.corr_max,
        depth_scale=args.depth_scale,
    )
    node.cruise_alt = args.alt

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    print("Waiting for camera + pose ...")
    while node.rgb_frame is None or node.depth_frame is None or node.drone_pose is None:
        time.sleep(0.1)
    print(f"Ready. instruction='{args.instruction}'  replanning @ {args.replan_hz} Hz")

    cv2.namedWindow("InternVLA-N1 Live", cv2.WINDOW_NORMAL)
    if node.map_img is not None:
        cv2.namedWindow("InternVLA-N1 Map", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Correction Debug", cv2.WINDOW_NORMAL)

    replan_period = 1.0 / max(args.replan_hz, 0.1)
    last_replan = 0.0
    paused = False
    last_ctrl = time.time()
    ctrl_dt = 1.0 / 30.0

    try:
        while rclpy.ok():
            now = time.time()

            # 30 Hz control tick
            if now - last_ctrl >= ctrl_dt:
                node.control_step()
                last_ctrl = now

            # auto-replan
            if (not paused) and (now - last_replan >= replan_period):
                node.replan_and_follow()
                last_replan = now

            rgb = node.rgb_frame
            if rgb is None:
                time.sleep(0.01); continue
            live = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            h, w = live.shape[:2]

            # ── LIVE trajectory + pixel-goal overlay (redrawn every frame
            #    from cached body-frame data — persistent pixel goal) ──
            label = f"InternVLA-N1  [{node.last_source}]"
            live = draw_overlay(
                live,
                best_traj=node.last_best_traj,
                all_traj=node.last_all_traj,
                all_vals=node.last_all_vals,
                pixel_goal=node.last_pixel_goal,
                cam_height=node.cruise_alt,
                label=label,
            )
            # Always-visible BEV inset (works even when trajectories are
            # short-range, zero, or the ground-plane projection misses).
            live = draw_bev_inset(
                live,
                best_traj=node.last_best_traj,
                all_traj=node.last_all_traj,
                all_vals=node.last_all_vals,
            )

            # status bar
            cv2.rectangle(live, (0, h - 28), (w, h), (0, 0, 0), -1)
            status = "FOLLOWING" if node.following else "IDLE"
            if node.following and node.world_wps:
                status += f"  wp {node.wp_idx}/{len(node.world_wps)}"
            if paused:
                status = "PAUSED  " + status
            age = f"plan_age={now - node.last_plan_t:.1f}s" if node.last_plan_t else "no plan"
            pg_str = f"pg={node.last_pixel_goal}" if node.last_pixel_goal else "pg=-"
            cv2.putText(live,
                        f"{status}  |  {age}  |  {pg_str}  |  "
                        f"SPACE=replan P=pause S=stop G=clr-goal Q=quit",
                        (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (200, 200, 200), 1)

            # ── right panel: frozen frame at pixel-goal grounding time ──
            if node.pixel_goal_frame is not None and node.last_pixel_goal is not None:
                frozen = node.pixel_goal_frame.copy()
                u, v = node.last_pixel_goal
                fh, fw = frozen.shape[:2]
                if 0 <= u < fw and 0 <= v < fh:
                    cv2.drawMarker(frozen, (u, v), (0, 255, 255),
                                   cv2.MARKER_CROSS, markerSize=30, thickness=3)
                    cv2.circle(frozen, (u, v), 18, (0, 255, 255), 2)
                    cv2.putText(frozen, "S2 goal", (u + 22, v - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                pg_age = now - node.pixel_goal_t if node.pixel_goal_t else 0.0
                cv2.putText(frozen,
                            f"Grounded frame  (age {pg_age:.1f}s)",
                            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2)
                # match heights so hstack works
                if frozen.shape[0] != h:
                    scale = h / frozen.shape[0]
                    frozen = cv2.resize(frozen,
                                        (int(frozen.shape[1] * scale), h))
            else:
                frozen = np.zeros_like(live)
                cv2.putText(frozen, "waiting for S2 to ground a pixel ...",
                            (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (120, 120, 120), 1)

            composite = np.hstack([live, frozen])
            cv2.imshow("InternVLA-N1 Live", composite)

            # ── map ──
            map_vis = node.render_map()
            if map_vis is not None:
                cv2.imshow("InternVLA-N1 Map", map_vis)

            # ── correction debug ──
            if node.corrector_vis is not None:
                cv2.imshow("Correction Debug", node.corrector_vis)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord(" "):
                last_replan = 0.0
            elif key == ord("p"):
                paused = not paused
                print("paused" if paused else "resumed")
            elif key == ord("s"):
                node.stop()
                node.world_wps = node.world_wps_original = None
            elif key == ord("g"):
                node.last_pixel_goal = None
                node.pixel_goal_frame = None
                node.pixel_goal_t = 0.0
                print("cleared pixel goal")

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        cv2.destroyAllWindows()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()