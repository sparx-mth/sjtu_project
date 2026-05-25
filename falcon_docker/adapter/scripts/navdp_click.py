#!/usr/bin/env python3
"""
navdp_click.py — minimal click-to-NavDP viewer (ROS 1 / rospy).

ONE OpenCV window "NavDP live", three panels side-by-side:
    [ live RGB  |  live colorized depth  |  snapshot + best trajectory ]
    [ status bar ]

  • LEFT-CLICK on the RGB panel  → set the goal pixel (yellow dot)
  • ENTER / SPACE                → send (RGB, depth, gx, gy) to NavDP,
                                    snapshot the current frame, and
                                    draw the best trajectory on it
  • r                            → clear the click + the snapshot panel
  • q / ESC                      → quit

The third panel is frozen — it shows the RGB frame that was sent to
NavDP plus the chosen best trajectory drawn over it. It updates only
on the next inference. NavDP was trained on ground robots at a
camera height of ~0.5 m, so its 2D body-frame trajectory is projected
back into the image using that training cam-height
(`~render_cam_height`, default 0.5 m) — not the drone's true
altitude. Otherwise the near-range waypoints land far below the
504×392 image and only the trajectory tail is visible. The drone's
live altitude is still shown on the RGB panel next to the click
readout and in the status bar.

Nothing flies. This file is only the visual loop:
   click pixel → body-frame (gx, gy) pointgoal → NavDP → draw.

Run:
    rosrun falcon_adapter navdp_click.py

Defaults match the real Xtend on the /xtend/rgb + /xtend/depth_m
topics. Override any of these via rosparam if you need to:
    _port _rgb_topic _depth_topic _pose_topic
    _fx _fy _cx _cy
    _default_altitude          (m above ground until pose arrives)
    _render_cam_height         (camera height used for the line
                                drawing; ≤ 0 means track live altitude)
"""

import io
import os
import json
import time
import base64

import cv2
import numpy as np
import requests
from PIL import Image as PILImage

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped


# ── shared mouse state (mouse_cb writes, main loop reads) ──────────
click_px = None        # (px, py) on the RGB panel
hover_px = None        # (px, py) on the depth panel, depth-image coords
rgb_w    = 0           # width of the RGB panel — splits the composite


def mouse_cb(event, x, y, flags, param):
    global click_px, hover_px
    if event == cv2.EVENT_MOUSEMOVE:
        # Depth panel only (the middle one). Trajectory panel (3rd)
        # is read-only.
        if rgb_w <= x < 2 * rgb_w:
            hover_px = (x - rgb_w, y)
        else:
            hover_px = None
    elif event == cv2.EVENT_LBUTTONDOWN and x < rgb_w:
        click_px = (x, y)


def colorize_depth(depth, dmax=10.0):
    d = np.clip(depth, 0.0, dmax)
    return cv2.applyColorMap((d / dmax * 255.0).astype(np.uint8),
                             cv2.COLORMAP_TURBO)


class NavDPClick:
    def __init__(self):
        rospy.init_node("navdp_click")
        G = rospy.get_param

        self.port        = int(G("~port", 8888))
        self.rgb_topic   = G("~rgb_topic",   "/xtend/rgb")
        self.depth_topic = G("~depth_topic", "/xtend/depth_m")
        self.pose_topic  = G("~pose_topic",  "/flow_depth/pose_est")

        # Camera intrinsics (real Xtend defaults). Override via
        # rosparam if you point this at a different camera.
        self.fx = float(G("~fx", 390.715))
        self.fy = float(G("~fy", 395.828))
        self.cx = float(G("~cx", 222.273))
        self.cy = float(G("~cy", 108.548))
        self.intrinsic = [[self.fx, 0.0,     self.cx],
                          [0.0,     self.fy, self.cy],
                          [0.0,     0.0,     1.0]]

        # Drone altitude (m above ground), updated from pose. Used
        # for the click readout (dz vs drone) and the status bar,
        # and sent to the server every step as the render plane.
        # NOT used for the client-side trajectory drawing — that
        # uses self.render_cam_height below. The default applies
        # only until the first pose message arrives.
        self.altitude = float(G("~default_altitude", 0.8))

        # Camera height used for projecting NavDP's 24 waypoints
        # back into the snapshot panel on this client. The
        # projection
        #   v = fy · cam_h / x_fwd + cy
        # only places the line on the ACTUAL ground when cam_h
        # equals the camera's true altitude. With cam_h = 0.8 m
        # (the drone's real altitude), the image bottom maps to
        # ground at ~1.12 m forward and ~9 of NavDP's 24 waypoints
        # land below the image, so the visible line is squashed
        # near the bottom — physically honest, visually cramped.
        # With cam_h = 0.5 m, ~19 of the 24 waypoints fit in-frame
        # and the line stretches from the lower edge up to roughly
        # the middle of the image — not on the real floor, but a
        # clearer picture of what the policy intends. We default
        # to 0.5 m for visibility; set to your altitude (or pass a
        # value ≤ 0) if you'd rather track the live altitude and
        # accept a shorter visible line.
        self.render_cam_height = float(G("~render_cam_height", 0.5))

        self.url   = "http://127.0.0.1:%d" % self.port
        self.rgb   = None
        self.depth = None

        rospy.Subscriber(self.rgb_topic,   Image,
                         self._rgb_cb,   queue_size=5)
        rospy.Subscriber(self.depth_topic, Image,
                         self._depth_cb, queue_size=5)
        rospy.Subscriber(self.pose_topic,  PoseStamped,
                         self._pose_cb,  queue_size=10)

        rospy.loginfo("=" * 64)
        rospy.loginfo("navdp_click ready")
        rospy.loginfo("  rgb   = %s", self.rgb_topic)
        rospy.loginfo("  depth = %s", self.depth_topic)
        rospy.loginfo("  pose  = %s", self.pose_topic)
        rospy.loginfo("  navdp = %s", self.url)
        rospy.loginfo("  intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                      self.fx, self.fy, self.cx, self.cy)
        rospy.loginfo("  default altitude (until pose arrives): %.2f m",
                      self.altitude)
        if self.render_cam_height > 0:
            rospy.loginfo("  render cam-height (fixed override):       %.2f m",
                          self.render_cam_height)
        else:
            rospy.loginfo("  render cam-height: tracks live altitude")
        rospy.loginfo("=" * 64)


    # ── click pixel + depth → body-frame goal + height delta ───────
    def pixel_to_pointgoal(self, px, py, depth):
        """Returns (gx, gy, d, bz).

        gx, gy : the 2D horizontal body-frame pointgoal NavDP needs,
                 scaled so it fits inside NavDP's [0,10] m forward
                 and [-10,10] m lateral input range *while keeping
                 the click's angle from the drone*. If we clipped
                 bx and by independently, a click farther than 10 m
                 would get bx capped to 10 but by left at its true
                 magnitude — making the goal look much more lateral
                 than the user actually meant.
        d      : the raw depth at the click (optical Z, metres).
        bz     : the click's vertical offset from the drone in body
                 frame (+ above, - below). Not sent to NavDP — it's
                 here so the caller can log it.
        """
        h, w = depth.shape
        patch = depth[max(0, py - 10):min(h, py + 10),
                      max(0, px - 10):min(w, px + 10)]
        valid = patch[(patch > 0.1) & (patch < 50.0)]
        d  = float(np.median(valid)) if valid.size else 3.0

        # True body-frame point at the click, no clipping yet.
        bx_raw =  d
        by_raw = -(px - self.cx) * d / self.fx
        bz     = -(py - self.cy) * d / self.fy

        # Shrink the goal vector uniformly so bx ≤ 10 and |by| ≤ 10.
        # This preserves atan2(by, bx) — the direction the drone
        # would head — which is what NavDP actually steers on.
        if bx_raw < 0.1:
            return 0.1, 0.0, d, bz
        scale_fwd = 10.0 / bx_raw if bx_raw > 10.0 else 1.0
        scale_lat = 10.0 / abs(by_raw) if abs(by_raw) > 10.0 else 1.0
        scale = min(scale_fwd, scale_lat)
        bx = float(bx_raw * scale)
        by = float(by_raw * scale)
        return bx, by, d, bz

    # ── draw NavDP's best trajectory on the snapshot image ─────────
    def project_trajectory(self, bgr, traj):
        """`traj` is (T, ≥2) — body-frame (forward, left) waypoints.

        Projected back into the image as if every waypoint sits on a
        ground plane `cam_h` below the camera, using
            u = fx * (-y_left) / x_fwd + cx
            v = fy *   cam_h   / x_fwd + cy

        `cam_h` is the live altitude (self.altitude) by default. That
        makes the rendered line physically consistent with the floor
        — a click on the floor at depth d and the trajectory endpoint
        at (bx≈d, by≈click_lateral) project to the SAME pixel. The
        cost is that near-range waypoints (x_fwd ≲ cam_h) fall below
        the image; only the visible tail of the line is drawn, which
        is honest: those near waypoints are at the drone's feet,
        outside the forward camera's FOV.

        If self.render_cam_height was set to a positive value via
        rosparam (e.g. 0.5), that fixed value is used instead — the
        line then looks like NavDP's training render (all 24 points
        in-frame) but is no longer aligned with the true floor.

        Off-image waypoints are clamped to a wide range rather than
        dropped, so cv2.line clips partial segments at the image
        edge and the visible portion stays connected.
        """
        out = bgr.copy()
        h, w = out.shape[:2]
        if self.render_cam_height > 0:
            cam_h = self.render_cam_height
            mode  = "fixed"
        else:
            cam_h = max(self.altitude, 0.1)
            mode  = "live"

        # Body (x_fwd, y_left, -cam_h) → camera (Xc=-y_left, Yc=cam_h, Zc=x_fwd)
        # → pixel u = fx*Xc/Zc + cx, v = fy*Yc/Zc + cy. Clamp to a wide
        # range (not 0..w/0..h) so cv2.line clips partial lines at the
        # image edge instead of us throwing the whole segment away.
        def to_px(x_fwd, y_left):
            if x_fwd < 0.05:                       # behind / on top of camera
                return None
            u = self.fx * (-y_left) / x_fwd + self.cx
            v = self.fy *   cam_h   / x_fwd + self.cy
            return (int(np.clip(u, -8000, 8000)),
                    int(np.clip(v, -8000, 8000)))

        pts = [to_px(float(p[0]), float(p[1])) for p in traj]

        # Drone origin = bottom-centre column at the drone's pixel-X.
        origin = (int(self.cx), h - 1)
        first = next((p for p in pts if p is not None), None)
        if first is not None:
            cv2.line(out, origin, first, (255, 255, 255), 4, cv2.LINE_AA)
            cv2.line(out, origin, first, (0, 255, 0),     2, cv2.LINE_AA)
        for i in range(len(pts) - 1):
            if pts[i] is not None and pts[i + 1] is not None:
                cv2.line(out, pts[i], pts[i + 1],
                         (255, 255, 255), 4, cv2.LINE_AA)
                cv2.line(out, pts[i], pts[i + 1],
                         (0, 255, 0),     2, cv2.LINE_AA)
        # A dot at every waypoint so all 24 points are individually
        # visible, even where the line is straight.
        for p in pts:
            if p is not None:
                cv2.circle(out, p, 3, (0, 255, 0), -1, cv2.LINE_AA)

        cv2.putText(out,
                    "NavDP best trajectory  render_h=%.2fm (%s)"
                    % (cam_h, mode),
                    (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return out

    # ── subscribers ────────────────────────────────────────────────
    def _rgb_cb(self, msg):
        arr = np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
        self.rgb = arr.copy()

    def _depth_cb(self, msg):
        if msg.encoding == "32FC1":
            self.depth = np.frombuffer(
                msg.data, np.float32).reshape(msg.height, msg.width).copy()
        elif msg.encoding == "16UC1":
            self.depth = np.frombuffer(
                msg.data, np.uint16).reshape(
                    msg.height, msg.width).astype(np.float32) / 1000.0

    def _pose_cb(self, msg):
        # Only z (altitude) matters for our projection.
        self.altitude = float(msg.pose.position.z)

    # ── NavDP HTTP ─────────────────────────────────────────────────
    def navdp_reset(self):
        try:
            r = requests.post(self.url + "/navigator_reset",
                              json={"intrinsic": self.intrinsic,
                                    "stop_threshold": -999,
                                    "batch_size": 1},
                              timeout=30)
            return r.status_code == 200
        except Exception as e:
            rospy.logerr("NavDP reset: %s", e)
            return False

    def navdp_pointgoal(self, rgb, depth, gx, gy, px, py):
        rgb_buf = io.BytesIO()
        PILImage.fromarray(rgb, "RGB").save(rgb_buf, format="PNG")
        rgb_buf.seek(0)

        # Depth encoding contract with NavDP server:
        #   client:   uint16 PNG, value = depth_m * 10000
        #   server:   value / 10000.0 = depth_m
        # uint16 caps at 65535, so any clip > 6.5535 m before
        # multiplying by 10000 OVERFLOWS SILENTLY. A 7 m pixel
        # becomes 4464 → server reads 0.45 m → NavDP sees a
        # phantom wall right in front of the camera exactly
        # where the user clicked the far floor. That collapses
        # the predicted trajectory to a meter or two.
        # NavDP's own process_depth then zeroes anything > 5 m
        # anyway (see policy_agent.process_depth), so 5 m is the
        # right honest cap — beyond it we have no usable signal
        # to send and clipping there is what NavDP was trained
        # against. Don't change to 10 unless you also widen the
        # encoding (e.g. uint32, or scale by 6000 instead of
        # 10000 with a matching server-side division).
        NAVDP_MAX_DEPTH_M = 5.0
        d_int = (np.clip(depth, 0.0, NAVDP_MAX_DEPTH_M)
                 * 10000).astype(np.uint16)
        d_buf = io.BytesIO()
        PILImage.fromarray(d_int, mode="I;16").save(d_buf, format="PNG")
        d_buf.seek(0)

        try:
            r = requests.post(
                self.url + "/pointgoal_step",
                files={"image": ("rgb.png",   rgb_buf, "image/png"),
                       "depth": ("depth.png", d_buf,   "image/png")},
                data={"goal_data": json.dumps({
                    "goal_x":   [gx], "goal_y":   [gy],
                    "click_px":  px,  "click_py":  py,
                    # Send the drone's live altitude every call so the
                    # server can render its trajectory mask onto the
                    # SAME ground plane the click was projected from.
                    # If --render_cam_height was set on the server CLI
                    # and we'd rather override it from here, send any
                    # positive value; the server uses this when > 0
                    # and falls back to its CLI default otherwise.
                    "altitude": float(self.altitude)})},
                timeout=30)
            return r.json() if r.status_code == 200 else None
        except Exception as e:
            rospy.logerr("NavDP step: %s", e)
            return None


def main():
    global click_px, hover_px, rgb_w

    node = NavDPClick()

    if not node.navdp_reset():
        rospy.logfatal("Could not reach NavDP at %s", node.url)
        return

    rospy.loginfo("Waiting for RGB + depth frames ...")
    while not rospy.is_shutdown() and (node.rgb is None or node.depth is None):
        time.sleep(0.1)
    if rospy.is_shutdown():
        return
    rospy.loginfo("Ready. Click on the RGB panel, then press ENTER.")

    cv2.namedWindow("NavDP live", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("NavDP live", mouse_cb)

    snap_vis = None     # third panel: snapshot RGB + drawn trajectory

    while not rospy.is_shutdown():
        rgb, depth = node.rgb, node.depth
        if rgb is None or depth is None:
            time.sleep(0.01); continue

        live      = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        depth_vis = colorize_depth(depth)
        rgb_w     = live.shape[1]

        # click marker + body-frame readout on the RGB panel
        if click_px is not None:
            gx, gy, d, bz = node.pixel_to_pointgoal(
                click_px[0], click_px[1], depth)
            cv2.circle(live, click_px, 12, (0, 255, 255), 2)
            cv2.circle(live, click_px, 4,  (0, 255, 255), -1)
            side = "left" if gy > 0 else "right"
            cv2.putText(live,
                        "%.1fm fwd  %.1fm %s  d=%.2fm  dz=%+.2fm" %
                        (gx, abs(gy), side, d, bz),
                        (click_px[0] + 14, click_px[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # hover crosshair + depth readout on the depth panel
        status = ("ENTER = send to NavDP   r = clear   q = quit   "
                  "alt=%.2fm" % node.altitude)
        if hover_px is not None:
            hx, hy = hover_px
            if 0 <= hx < depth.shape[1] and 0 <= hy < depth.shape[0]:
                cv2.drawMarker(depth_vis, (hx, hy), (255, 255, 255),
                               cv2.MARKER_CROSS, 14, 1)
                status = "depth(%d,%d) = %.2f m" % (hx, hy, depth[hy, hx])

        cv2.putText(live,      "RGB  (left-click)", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(depth_vis, "Depth  (hover)",    (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        # Third panel: frozen snapshot + best trajectory. Placeholder
        # until the first inference.
        if snap_vis is not None:
            third = snap_vis
        else:
            third = np.zeros_like(live)
            cv2.putText(third, "Click + ENTER for trajectory",
                        (16, third.shape[0] // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

        top = np.hstack([live, depth_vis, third])
        bar = np.zeros((28, top.shape[1], 3), np.uint8)
        cv2.putText(bar, status, (8, 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        cv2.imshow("NavDP live", np.vstack([top, bar]))

        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('r'):
            click_px  = None
            snap_vis  = None
        elif key in (13, 32):                       # ENTER / SPACE
            if click_px is None:
                rospy.loginfo("Click on the RGB panel first.")
                continue
            snap_rgb, snap_depth = node.rgb.copy(), node.depth.copy()
            px, py = click_px
            gx, gy, d, bz = node.pixel_to_pointgoal(px, py, snap_depth)
            rospy.loginfo(
                "Goal: gx=%.2fm fwd  gy=%+.2fm  "
                "(depth %.2fm, dz=%+.2fm vs drone, alt=%.2fm)",
                gx, gy, d, bz, node.altitude)
            result = node.navdp_pointgoal(snap_rgb, snap_depth,
                                          gx, gy, px, py)
            if result is None:
                rospy.logwarn("NavDP returned no result.")
                continue
            best = np.array(result["trajectory"])[0]
            vals = np.array(result["all_values"])[0]
            # Project the trajectory endpoint into pixel coords with
            # the SAME camera-height the server used to render its
            # mask, so you can compare directly to the click pixel.
            end_x = float(best[-1, 0])
            end_y = float(best[-1, 1])
            if end_x > 0.05:
                cam_h = max(node.render_cam_height
                            if node.render_cam_height > 0
                            else node.altitude, 0.1)
                end_u = int(node.fx * (-end_y) / end_x + node.cx)
                end_v = int(node.fy *   cam_h  / end_x + node.cy)
            else:
                end_u, end_v = -1, -1
            rospy.loginfo(
                "  goal=(%.2f, %.2f)  endpoint=(%.2f, %.2f)  "
                "range=%.2fm  critic max=%.2f",
                gx, gy, end_x, end_y,
                float(np.hypot(end_x, end_y)), vals.max())
            rospy.loginfo(
                "  click pixel=(%d, %d)  endpoint pixel=(%d, %d)  "
                "click depth=%.2fm  endpoint range=%.2fm",
                px, py, end_u, end_v,
                d, float(np.hypot(end_x, end_y)))

            # Full 24-waypoint dump in body frame. Columns are
            #   i      step index 0..23
            #   fwd    metres forward (NavDP X+)
            #   left   metres left    (NavDP Y+)
            #   yaw    heading change (rad) if present, else "  -  "
            # Useful for sanity-checking whether NavDP is producing
            # waypoints that actually march outward, or stalling
            # near the origin. Compact two-decimal format so all 24
            # rows fit on a normal terminal.
            has_yaw = best.shape[1] >= 3
            rospy.loginfo(
                "  waypoints (n=%d): %s",
                best.shape[0],
                "fwd/left/yaw" if has_yaw else "fwd/left")
            for i, wp in enumerate(best):
                if has_yaw:
                    rospy.loginfo(
                        "    [%2d]  fwd=%+5.2fm  left=%+5.2fm  yaw=%+5.2frad",
                        i, float(wp[0]), float(wp[1]), float(wp[2]))
                else:
                    rospy.loginfo(
                        "    [%2d]  fwd=%+5.2fm  left=%+5.2fm",
                        i, float(wp[0]), float(wp[1]))

            # Dump what we actually POSTed to NavDP, byte-for-byte,
            # so you can open and verify. /tmp/navdp_sent/rgb.png is
            # the cropped 504×392 RGB; depth_raw.png is the 16-bit
            # PNG the server decodes via /10000; depth_vis.png is a
            # human-friendly colorized version of the SAME clipped
            # depth (after clip to 5 m). If the crop looks wrong or
            # depth values look off, fix that before touching the
            # model.
            sent_dir = "/tmp/navdp_sent"
            os.makedirs(sent_dir, exist_ok=True)
            cv2.imwrite(os.path.join(sent_dir, "rgb.png"),
                        cv2.cvtColor(snap_rgb, cv2.COLOR_RGB2BGR))
            d_clipped = np.clip(snap_depth, 0.0, 5.0)
            d_raw = (d_clipped * 10000).astype(np.uint16)
            PILImage.fromarray(d_raw, mode="I;16").save(
                os.path.join(sent_dir, "depth_raw.png"))
            cv2.imwrite(os.path.join(sent_dir, "depth_vis.png"),
                        colorize_depth(d_clipped, dmax=5.0))

            # Build the snapshot panel client-side: snapshot RGB +
            # best trajectory (with a green dot at every waypoint),
            # nothing else. The server's trajectory_mask draws all
            # 16 candidates colour-coded by critic value, which is
            # useful for the MP4 but too busy for this viewer. If
            # you ever want NavDP's full visualization back, swap
            # this for the trajectory_mask branch.
            snap_bgr = cv2.cvtColor(snap_rgb, cv2.COLOR_RGB2BGR)
            snap_vis = node.project_trajectory(snap_bgr, best)

            # Click marker (yellow) — where the user pointed.
            cv2.circle(snap_vis, (px, py), 12, (0, 255, 255), 2,
                       cv2.LINE_AA)
            cv2.circle(snap_vis, (px, py),  4, (0, 255, 255), -1,
                       cv2.LINE_AA)
            # Endpoint marker (magenta) — where NavDP actually stops.
            # A short dashed line between the two makes the
            # "model didn't reach my click" case obvious.
            if 0 <= end_u < snap_vis.shape[1] and 0 <= end_v < snap_vis.shape[0]:
                cv2.circle(snap_vis, (end_u, end_v), 10, (255, 0, 255), 2,
                           cv2.LINE_AA)
                cv2.circle(snap_vis, (end_u, end_v),  3, (255, 0, 255), -1,
                           cv2.LINE_AA)
                cv2.line(snap_vis, (px, py), (end_u, end_v),
                         (255, 0, 255), 1, cv2.LINE_AA)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass