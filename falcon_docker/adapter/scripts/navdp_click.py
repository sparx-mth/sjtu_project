#!/usr/bin/env python3
"""
navdp_click.py — minimal click-to-NavDP viewer (ROS 1 / rospy).

ONE OpenCV window "NavDP live":
    [ live RGB  |  live colorized depth ]
    [ status bar: depth value under cursor when hovering on depth ]

  • LEFT-CLICK on the RGB panel  → set the goal pixel (yellow dot)
  • ENTER / SPACE                → send (RGB, depth, gx, gy) to NavDP
  • r                            → clear the click + the NavDP result
  • q / ESC                      → quit

A second window "NavDP trajectory" opens after the first inference and
shows the trajectory overlay returned in result["trajectory_mask"]
(the same picture the NavDP server draws internally).

Nothing flies. This file is only the visual loop:
   click pixel → body-frame (gx, gy) pointgoal → NavDP → draw.

Run:
    rosrun falcon_adapter navdp_click.py \\
        _port:=8888 \\
        _rgb_topic:=/simple_drone/front_depth/image_raw \\
        _depth_topic:=/simple_drone/front_depth/depth/image_raw

Intrinsics default to the sjtu_drone front camera (90° HFOV, 640×480).
Override on a real drone:
    _fx:=... _fy:=... _cx:=... _cy:=...
"""

import base64
import io
import json
import time

import cv2
import numpy as np
import requests
from PIL import Image as PILImage

import rospy
from sensor_msgs.msg import Image


# ── shared mouse state (mouse_cb writes, main loop reads) ──────────
click_px = None        # (px, py) on the RGB panel
hover_px = None        # (px, py) on the depth panel, depth-image coords
rgb_w    = 0           # width of the RGB panel — splits the composite


def mouse_cb(event, x, y, flags, param):
    global click_px, hover_px
    if event == cv2.EVENT_MOUSEMOVE:
        hover_px = (x - rgb_w, y) if x >= rgb_w else None
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
        self.rgb_topic   = G("~rgb_topic",
                             "/simple_drone/front_depth/image_raw")
        self.depth_topic = G("~depth_topic",
                             "/simple_drone/front_depth/depth/image_raw")

        # Camera intrinsics. Defaults: sjtu_drone front camera
        # (90° HFOV, 640×480). Override per-drone via rosparam.
        self.fx = float(G("~fx", 320.0))
        self.fy = float(G("~fy", 320.0))
        self.cx = float(G("~cx", 320.5))
        self.cy = float(G("~cy", 240.5))
        self.intrinsic = [[self.fx, 0.0,     self.cx],
                          [0.0,     self.fy, self.cy],
                          [0.0,     0.0,     1.0]]

        self.url   = "http://127.0.0.1:%d" % self.port
        self.rgb   = None
        self.depth = None

        rospy.Subscriber(self.rgb_topic,   Image,
                         self._rgb_cb,   queue_size=5)
        rospy.Subscriber(self.depth_topic, Image,
                         self._depth_cb, queue_size=5)

        rospy.loginfo("=" * 64)
        rospy.loginfo("navdp_click ready")
        rospy.loginfo("  rgb   = %s", self.rgb_topic)
        rospy.loginfo("  depth = %s", self.depth_topic)
        rospy.loginfo("  navdp = %s", self.url)
        rospy.loginfo("  intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                      self.fx, self.fy, self.cx, self.cy)
        rospy.loginfo("=" * 64)

    # ── click pixel + depth → body-frame (gx, gy) ─────────────────
    def pixel_to_pointgoal(self, px, py, depth):
        """gx = forward (m), gy = +left (m). Median over a 21×21 patch
        around the click to ignore depth holes."""
        h, w = depth.shape
        patch = depth[max(0, py - 10):min(h, py + 10),
                      max(0, px - 10):min(w, px + 10)]
        valid = patch[(patch > 0.1) & (patch < 50.0)]
        d  = float(np.median(valid)) if valid.size else 3.0
        gx = float(np.clip(d, 0.1, 10.0))
        gy = float(np.clip(-(px - self.cx) * d / self.fx, -10.0, 10.0))
        return gx, gy, d

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

        d_int = (np.clip(depth, 0.0, 10.0) * 10000).astype(np.uint16)
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
                    "click_px":  px,  "click_py":  py})},
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

    navdp_vis = None     # last server-rendered trajectory image (BGR)

    while not rospy.is_shutdown():
        rgb, depth = node.rgb, node.depth
        if rgb is None or depth is None:
            time.sleep(0.01); continue

        live      = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        depth_vis = colorize_depth(depth)
        rgb_w     = live.shape[1]

        # click marker + body-frame readout on the RGB panel
        if click_px is not None:
            gx, gy, d = node.pixel_to_pointgoal(
                click_px[0], click_px[1], depth)
            cv2.circle(live, click_px, 12, (0, 255, 255), 2)
            cv2.circle(live, click_px, 4,  (0, 255, 255), -1)
            side = "left" if gy > 0 else "right"
            cv2.putText(live,
                        "%.1fm fwd  %.1fm %s  d=%.2fm" % (gx, abs(gy), side, d),
                        (click_px[0] + 14, click_px[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # hover crosshair + depth readout on the depth panel
        status = "ENTER = send to NavDP    r = clear    q = quit"
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

        top = np.hstack([live, depth_vis])
        bar = np.zeros((28, top.shape[1], 3), np.uint8)
        cv2.putText(bar, status, (8, 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        cv2.imshow("NavDP live", np.vstack([top, bar]))

        if navdp_vis is not None:
            cv2.imshow("NavDP trajectory", navdp_vis)

        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('r'):
            click_px  = None
            navdp_vis = None
            try: cv2.destroyWindow("NavDP trajectory")
            except cv2.error: pass
        elif key in (13, 32):                       # ENTER / SPACE
            if click_px is None:
                rospy.loginfo("Click on the RGB panel first.")
                continue
            snap_rgb, snap_depth = node.rgb.copy(), node.depth.copy()
            px, py    = click_px
            gx, gy, d = node.pixel_to_pointgoal(px, py, snap_depth)
            rospy.loginfo("Goal: gx=%.2fm fwd  gy=%+.2fm  (depth %.2fm)",
                          gx, gy, d)
            result = node.navdp_pointgoal(snap_rgb, snap_depth,
                                          gx, gy, px, py)
            if result is None:
                rospy.logwarn("NavDP returned no result.")
                continue
            best = np.array(result["trajectory"])[0]
            vals = np.array(result["all_values"])[0]
            rospy.loginfo("  traj endpoint=(%.2f, %.2f)  critic max=%.2f",
                          best[-1, 0], best[-1, 1], vals.max())
            if "trajectory_mask" in result:
                buf = np.frombuffer(
                    base64.b64decode(result["trajectory_mask"]), np.uint8)
                navdp_vis = cv2.imdecode(buf, cv2.IMREAD_COLOR)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass