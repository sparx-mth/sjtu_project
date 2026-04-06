#!/usr/bin/env python3
"""
navdp_drone_live.py
===================
Live NavDP → drone control integration.

Opens a camera window, lets you click a pixel goal, sends it to the NavDP
HTTP server, receives a trajectory, converts it to world coordinates, and
flies the drone along it using pure-pursuit.  Optionally overlays the
trajectory on a 2-D occupancy map.

Run OUTSIDE Docker on the host (same machine as NavDP server).
Requires ROS 2 Humble + CycloneDDS to reach the topics inside the container.

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    source /opt/ros/humble/setup.bash
    python navdp_drone_live.py --port 8888 --map_yaml <path_to_yaml>

Controls
--------
  LEFT-CLICK   set pixel goal (yellow dot)
  ENTER        confirm & send to NavDP → fly trajectory
  ESC / q      quit
"""

import argparse, io, json, math, os, sys, time, threading, base64
import cv2, numpy as np, requests, yaml
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Image

# ── camera intrinsics (from /simple_drone/front_depth/camera_info) ──
FX, FY, CX, CY = 320.0, 320.0, 320.5, 240.5
INTRINSIC = [[FX, 0, CX], [0, FY, CY], [0, 0, 1]]


# ====================================================================
# ROS 2 node
# ====================================================================
class NavDPDroneController(Node):

    def __init__(self, navdp_port: int, map_yaml: str | None,
                 rgb_topic: str, depth_topic: str):
        super().__init__("navdp_drone_controller")
        self.navdp_url = f"http://127.0.0.1:{navdp_port}"

        # ── shared state (updated by ROS callbacks) ──────────────
        self.drone_pose = None          # (x, y, z, yaw)
        self.rgb_frame: np.ndarray | None = None
        self.depth_frame: np.ndarray | None = None

        # ── trajectory state ─────────────────────────────────────
        self.world_wps: list[tuple[float, float]] | None = None
        self.wp_idx = 0
        self.following = False

        # ── click state ──────────────────────────────────────────
        self.click_pos: tuple[int, int] | None = None

        # ── NavDP visualisation (side-by-side panel) ─────────────
        self.inference_frame: np.ndarray | None = None   # RGB sent to model
        self.navdp_vis: np.ndarray | None = None         # BGR with trajectories

        # ── 2-D map (optional) ───────────────────────────────────
        self.map_img = self.map_res = self.map_origin = None
        if map_yaml and os.path.exists(map_yaml):
            self._load_map(map_yaml)

        # ── ROS interfaces ───────────────────────────────────────
        cb = ReentrantCallbackGroup()
        self.create_subscription(
            Pose, "/simple_drone/gt_pose", self._pose_cb, 10, callback_group=cb)
        self.create_subscription(
            Image, rgb_topic, self._rgb_cb, 5, callback_group=cb)
        self.create_subscription(
            Image, depth_topic, self._depth_cb, 5, callback_group=cb)
        self.cmd_pub = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)

        # ── control parameters ───────────────────────────────────
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
            self.rgb_frame = np.frombuffer(
                msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
        elif msg.encoding == "bgr8":
            bgr = np.frombuffer(
                msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            self.rgb_frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).copy()

    def _depth_cb(self, msg: Image):
        if msg.encoding == "32FC1":
            self.depth_frame = np.frombuffer(
                msg.data, np.float32).reshape(msg.height, msg.width).copy()
        elif msg.encoding == "16UC1":
            raw = np.frombuffer(
                msg.data, np.uint16).reshape(msg.height, msg.width)
            self.depth_frame = raw.astype(np.float32) / 1000.0

    # ─── map helpers ─────────────────────────────────────────────
    def _load_map(self, yaml_path: str):
        with open(yaml_path) as f:
            info = yaml.safe_load(f)
        self.map_res = float(info["resolution"])
        self.map_origin = info["origin"]  # [ox, oy, yaw]
        img_path = info["image"]
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)
        raw = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        if raw is not None:
            self.map_img = raw  # keep original orientation for cv2
            self.get_logger().info(f"Map loaded: {img_path}  "
                                   f"{raw.shape[1]}×{raw.shape[0]}  "
                                   f"res={self.map_res}")

    def _world_to_map_cv(self, x: float, y: float) -> tuple[int, int]:
        """World → OpenCV pixel (col, row) on the original PGM image."""
        ox, oy = self.map_origin[0], self.map_origin[1]
        col = (x - ox) / self.map_res
        row = self.map_img.shape[0] - (y - oy) / self.map_res
        return int(round(col)), int(round(row))

    # ─── pixel → pointgoal (same logic as navigate_stream.py) ───
    @staticmethod
    def pixel_to_pointgoal(px: int, py: int, depth: np.ndarray):
        h, w = depth.shape
        patch = depth[max(0,py-10):min(h,py+10), max(0,px-10):min(w,px+10)]
        valid = patch[(patch > 0.1) & (patch < 50.0)]
        d = float(np.median(valid)) if len(valid) > 0 else 3.0
        gx = float(np.clip(d, 0.1, 10.0))
        gy = float(np.clip(-(px - CX) * d / FX, -10.0, 10.0))
        return gx, gy, d

    # ─── NavDP HTTP calls ────────────────────────────────────────
    def navdp_reset(self):
        try:
            r = requests.post(f"{self.navdp_url}/navigator_reset",
                              json={"intrinsic": INTRINSIC,
                                    "stop_threshold": -999,
                                    "batch_size": 1}, timeout=30)
            ok = r.status_code == 200
            self.get_logger().info(f"NavDP reset: {'OK' if ok else 'FAIL'}")
            return ok
        except Exception as e:
            self.get_logger().error(f"NavDP reset error: {e}")
            return False

    def navdp_pointgoal(self, rgb: np.ndarray, depth: np.ndarray,
                        gx: float, gy: float,
                        click_px: int = -1, click_py: int = -1) -> dict | None:
        rgb_buf = io.BytesIO()
        PILImage.fromarray(rgb, "RGB").save(rgb_buf, format="PNG")
        rgb_buf.seek(0)

        d_clip = np.clip(depth, 0.0, 10.0)
        d_int = (d_clip * 10000).astype(np.uint16)
        d_buf = io.BytesIO()
        PILImage.fromarray(d_int, mode="I;16").save(d_buf, format="PNG")
        d_buf.seek(0)

        files = {"image": ("rgb.png", rgb_buf, "image/png"),
                 "depth": ("depth.png", d_buf, "image/png")}
        data  = {"goal_data": json.dumps({"goal_x": [gx], "goal_y": [gy],
                                          "click_px": click_px, "click_py": click_py})}
        try:
            r = requests.post(f"{self.navdp_url}/pointgoal_step",
                              files=files, data=data, timeout=30)
            return r.json() if r.status_code == 200 else None
        except Exception as e:
            self.get_logger().error(f"NavDP step error: {e}")
            return None

    # ─── body-frame trajectory → world waypoints ────────────────
    @staticmethod
    def traj_to_world(traj: np.ndarray,
                      ref_x: float, ref_y: float, ref_yaw: float):
        """
        traj: (24, 3)  cumulative (forward, left, yaw) in body frame.
        Returns list of (world_x, world_y).
        """
        c, s = math.cos(ref_yaw), math.sin(ref_yaw)
        wps = []
        for wp in traj:
            fwd, left = float(wp[0]), float(wp[1])
            wx = ref_x + fwd * c - left * s
            wy = ref_y + fwd * s + left * c
            wps.append((wx, wy))
        return wps

    # ─── project trajectories onto image (mirrors policy_agent) ──
    @staticmethod
    def project_trajectories(rgb_bgr: np.ndarray,
                             all_traj: np.ndarray,
                             all_vals: np.ndarray,
                             best_traj: np.ndarray,
                             click_px: tuple[int, int] | None = None,
                             cam_height: float = 1.5):
        """
        Draw all candidate trajectories on the input image, colour-coded
        by critic score (jet colourmap, same as policy_agent.project_trajectory).

        all_traj : (16, 24, 3)  — all candidate trajectories
        all_vals : (16,)        — critic values
        best_traj: (24, 3)      — the chosen trajectory
        cam_height: camera height above ground plane (metres)
        """
        vis = rgb_bgr.copy()
        h = vis.shape[0]

        # helper: body-frame waypoint → image pixel
        # Project onto ground plane: camera is cam_height above ground,
        # so Y_cam = cam_height (positive = downward in camera frame).
        def to_pixel(x_fwd, y_left):
            if x_fwd < 0.05:
                return None
            cam_x = int(FX * (-y_left) / x_fwd + CX)
            cam_y = int(FY * cam_height / x_fwd + CY)
            if 0 <= cam_x < vis.shape[1] and 0 <= cam_y < h:
                return (cam_x, cam_y)
            return None

        # helper: critic value → BGR via OpenCV jet colourmap
        def critic_color(value):
            norm = np.clip(-value * 0.1, 0.0, 1.0)
            lut = np.array([[[int(norm * 255)]]], dtype=np.uint8)
            bgr = cv2.applyColorMap(lut, cv2.COLORMAP_JET)
            return tuple(int(c) for c in bgr[0, 0])

        # origin = bottom-center of image (camera / drone position)
        origin = (int(CX), h - 1)

        # draw all candidates (thin lines)
        for traj, val in zip(all_traj, all_vals):
            colour = critic_color(val)
            pts = [to_pixel(wp[0], wp[1]) for wp in traj]
            # connect origin to first valid point
            first = next((p for p in pts if p is not None), None)
            if first:
                cv2.line(vis, origin, first, colour, 2)
            for i in range(len(pts) - 1):
                if pts[i] and pts[i + 1]:
                    cv2.line(vis, pts[i], pts[i + 1], colour, 2)

        # draw best trajectory on top (thick white + green)
        best_pts = [to_pixel(wp[0], wp[1]) for wp in best_traj]
        first_best = next((p for p in best_pts if p is not None), None)
        if first_best:
            cv2.line(vis, origin, first_best, (255, 255, 255), 4)
            cv2.line(vis, origin, first_best, (0, 255, 0), 2)
        for i in range(len(best_pts) - 1):
            if best_pts[i] and best_pts[i + 1]:
                cv2.line(vis, best_pts[i], best_pts[i + 1], (255, 255, 255), 4)
                cv2.line(vis, best_pts[i], best_pts[i + 1], (0, 255, 0), 2)

        # draw click / goal marker
        if click_px is not None:
            cv2.circle(vis, click_px, 14, (0, 255, 0), 3)
            cv2.circle(vis, click_px, 5, (0, 255, 0), -1)

        # label
        cv2.putText(vis, "NavDP output", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis, f"best critic: {all_vals.max():.2f}",
                    (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 1)

        return vis

    # ─── drone motion ────────────────────────────────────────────
    def stop(self):
        t = Twist()
        for _ in range(5):
            self.cmd_pub.publish(t)
            time.sleep(0.02)
        self.following = False

    def control_step(self):
        """One iteration of pure-pursuit trajectory following."""
        if not self.following or self.world_wps is None or self.drone_pose is None:
            return
        px, py, pz, yaw = self.drone_pose

        # advance past reached waypoints
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

        # look a couple waypoints ahead for smoother pursuit
        look_idx = min(self.wp_idx + 3, len(self.world_wps) - 1)
        tx, ty = self.world_wps[look_idx]

        dx, dy = tx - px, ty - py
        dist = math.hypot(dx, dy)
        if dist < 0.03:
            return

        # goal distance (to final wp)
        fx, fy = self.world_wps[-1]
        d_goal = math.hypot(fx - px, fy - py)

        speed = self.cruise_speed * (0.3 + 0.7 * min(d_goal, 1.0))
        speed = max(0.08, speed)

        vx_w = dx / dist * speed
        vy_w = dy / dist * speed

        # altitude P
        vz = max(-self.max_vz, min(self.alt_kp * (self.cruise_alt - pz), self.max_vz))

        # yaw toward target
        des_yaw = math.atan2(dy, dx)
        yerr = des_yaw - yaw
        while yerr >  math.pi: yerr -= 2 * math.pi
        while yerr < -math.pi: yerr += 2 * math.pi
        yr = max(-self.max_yaw_rate, min(self.yaw_kp * yerr, self.max_yaw_rate))

        # world → body velocity
        c, s = math.cos(yaw), math.sin(yaw)
        tw = Twist()
        tw.linear.x =  vx_w * c + vy_w * s
        tw.linear.y = -vx_w * s + vy_w * c
        tw.linear.z = vz
        tw.angular.z = yr
        self.cmd_pub.publish(tw)

    # ─── map visualisation ───────────────────────────────────────
    def render_map(self) -> np.ndarray | None:
        if self.map_img is None:
            return None
        if self.map_img.ndim == 2:
            vis = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        else:
            vis = self.map_img.copy()

        # draw trajectory
        if self.world_wps:
            pts = [self._world_to_map_cv(x, y) for x, y in self.world_wps]
            for i in range(len(pts) - 1):
                cv2.line(vis, pts[i], pts[i + 1], (0, 255, 255), 2)
            for p in pts:
                cv2.circle(vis, p, 3, (0, 200, 255), -1)
            cv2.circle(vis, pts[-1], 8, (0, 0, 255), 2)   # goal
            cv2.circle(vis, pts[0], 8, (255, 0, 0), 2)     # start

        # draw drone
        if self.drone_pose:
            dc, dr = self._world_to_map_cv(self.drone_pose[0], self.drone_pose[1])
            cv2.circle(vis, (dc, dr), 6, (0, 0, 255), -1)
            yaw = self.drone_pose[3]
            a = 20  # arrow length in pixels
            cv2.arrowedLine(vis, (dc, dr),
                            (dc + int(a * math.cos(yaw)),
                             dr - int(a * math.sin(yaw))),
                            (0, 0, 255), 2, tipLength=0.4)

        return vis


# ====================================================================
# main loop (runs in the main thread, OpenCV needs that)
# ====================================================================
click_pos = None
live_width = 640  # updated dynamically

def mouse_cb(event, x, y, flags, param):
    global click_pos
    if event == cv2.EVENT_LBUTTONDOWN and x < live_width:
        click_pos = (x, y)


def main():
    global click_pos, live_width

    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--map_yaml", type=str, default="")
    ap.add_argument("--rgb_topic", type=str,
                    default="/simple_drone/front_depth/image_raw")
    ap.add_argument("--depth_topic", type=str,
                    default="/simple_drone/front_depth/depth/image_raw")
    ap.add_argument("--alt", type=float, default=1.5)
    args = ap.parse_args()

    rclpy.init()
    node = NavDPDroneController(
        navdp_port=args.port,
        map_yaml=args.map_yaml if args.map_yaml else None,
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
    )
    node.cruise_alt = args.alt

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # reset NavDP
    if not node.navdp_reset():
        node.get_logger().error("Cannot reach NavDP server — is it running?")
        rclpy.shutdown()
        return

    # wait for first frames
    print("Waiting for camera + pose ...")
    while node.rgb_frame is None or node.depth_frame is None or node.drone_pose is None:
        time.sleep(0.1)
    print("Ready. Click on the camera image to set a goal, then press ENTER.")

    cv2.namedWindow("NavDP Camera", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("NavDP Camera", mouse_cb)
    if node.map_img is not None:
        cv2.namedWindow("NavDP Map", cv2.WINDOW_NORMAL)

    last_ctrl = time.time()
    ctrl_dt = 1.0 / 30.0

    try:
        while rclpy.ok():
            # ── control step ─────────────────────────────────────
            now = time.time()
            if now - last_ctrl >= ctrl_dt:
                node.control_step()
                last_ctrl = now

            # ── camera display ───────────────────────────────────
            rgb = node.rgb_frame
            if rgb is None:
                time.sleep(0.01)
                continue
            live = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            live_width = live.shape[1]  # track for mouse callback

            # draw click marker on live view
            if click_pos is not None:
                px, py = click_pos
                cv2.circle(live, (px, py), 14, (0, 255, 255), 3)
                cv2.circle(live, (px, py), 5, (0, 255, 255), -1)
                if node.depth_frame is not None:
                    gx, gy, d = node.pixel_to_pointgoal(px, py, node.depth_frame)
                    lr = "LEFT" if gy > 0 else "RIGHT"
                    cv2.putText(live,
                                f"{gx:.1f}m fwd, {abs(gy):.1f}m {lr} (d={d:.1f}m)",
                                (px + 18, py - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

            # label live panel
            cv2.putText(live, "Live camera", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # status bar on live panel
            h, w = live.shape[:2]
            cv2.rectangle(live, (0, h - 28), (w, h), (0, 0, 0), -1)
            status = "FOLLOWING" if node.following else "IDLE"
            if node.following and node.world_wps:
                status += f"  wp {node.wp_idx}/{len(node.world_wps)}"
            cv2.putText(live,
                        f"{status}  |  CLICK + ENTER = go  |  Q = quit",
                        (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (200, 200, 200), 1)

            # build side-by-side: [live | navdp result]
            if node.navdp_vis is not None:
                right = node.navdp_vis
                # resize right panel to match live height if needed
                if right.shape[0] != h:
                    scale = h / right.shape[0]
                    right = cv2.resize(right, (int(right.shape[1] * scale), h))
                composite = np.hstack([live, right])
            else:
                # before first inference, show placeholder
                placeholder = np.zeros_like(live)
                cv2.putText(placeholder, "Click + ENTER to get NavDP prediction",
                            (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (120, 120, 120), 1)
                composite = np.hstack([live, placeholder])

            cv2.imshow("NavDP Camera", composite)

            # ── map display ──────────────────────────────────────
            map_vis = node.render_map()
            if map_vis is not None:
                cv2.imshow("NavDP Map", map_vis)

            # ── keyboard ─────────────────────────────────────────
            key = cv2.waitKey(30) & 0xFF

            if key in (ord("q"), 27):
                break

            elif key == ord("r"):          # clear click
                click_pos = None

            elif key in (13, 32):          # ENTER / SPACE → send to NavDP
                if click_pos is None:
                    print("Click on the image first!")
                    continue
                if node.rgb_frame is None or node.depth_frame is None:
                    print("No frames yet")
                    continue
                if node.drone_pose is None:
                    print("No pose yet")
                    continue

                # 1) stop any current trajectory
                node.stop()

                # 2) snapshot the frame that will go to the model
                snap_rgb = node.rgb_frame.copy()
                snap_depth = node.depth_frame.copy()
                snap_bgr = cv2.cvtColor(snap_rgb, cv2.COLOR_RGB2BGR)

                # 3) convert pixel → pointgoal
                px, py = click_pos
                gx, gy, d = node.pixel_to_pointgoal(px, py, snap_depth)
                lr = "LEFT" if gy > 0 else "RIGHT"
                print(f"Goal: {gx:.2f}m fwd, {abs(gy):.2f}m {lr}  "
                      f"(pixel {px},{py}  depth {d:.1f}m)")

                # 4) send to NavDP
                print("Sending to NavDP …")
                result = node.navdp_pointgoal(snap_rgb, snap_depth, gx, gy,
                                              click_px=px, click_py=py)
                if result is None:
                    print("NavDP returned no result")
                    continue

                best_traj = np.array(result["trajectory"])[0]   # (24, 3)
                all_traj  = np.array(result["all_trajectory"])[0]  # (16, 24, 3)
                all_vals  = np.array(result["all_values"])[0]      # (16,)
                print(f"Got trajectory: 24 waypoints, "
                      f"endpoint=({best_traj[-1,0]:.2f}, {best_traj[-1,1]:.2f}), "
                      f"critic [{all_vals.min():.2f} .. {all_vals.max():.2f}]")

                # 5) use the server's own trajectory visualisation
                node.inference_frame = snap_bgr
                if "trajectory_mask" in result:
                    mask_bytes = base64.b64decode(result["trajectory_mask"])
                    mask_arr = np.frombuffer(mask_bytes, dtype=np.uint8)
                    node.navdp_vis = cv2.imdecode(mask_arr, cv2.IMREAD_COLOR)
                else:
                    # fallback: client-side projection if server doesn't send mask
                    node.navdp_vis = node.project_trajectories(
                        snap_bgr, all_traj, all_vals, best_traj,
                        click_px=(px, py),
                        cam_height=node.cruise_alt)

                # 6) convert body-frame → world
                ref = node.drone_pose
                node.world_wps = node.traj_to_world(best_traj,
                                                     ref[0], ref[1], ref[3])
                node.wp_idx = 0
                node.following = True
                click_pos = None
                print(f"Following {len(node.world_wps)} waypoints …")

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