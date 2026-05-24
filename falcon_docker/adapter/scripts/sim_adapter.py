#!/usr/bin/env python3
"""
sim_adapter.py — make Gazebo look like the real Xtend on every topic.

This node makes the sim publish on the EXACT same topic names the real
drone publishes (/xtend/rgb, /xtend/depth_m, /flow_depth/pose_est),
AND crops the camera images so the cropped output has the real Xtend's
off-centre principal point, not Gazebo's centred one. With this node
running, real_drone.launch can be reused for sim with zero topic
remapping anywhere downstream.

It also stands in for the ROS2-owned DemoMode state machine: in sim,
this node OWNS /xtend/demo_mode (publishes the current state, latched,
default "fly_straight") and listens on /xtend/demo_mode_request for
transitions requested by ROS1 nodes (waypoint_follower, falcon_adapter).

And it relays the cmd_vel rename: FALCON publishes /cmd_vel (ROS1), but
Gazebo's sjtu_drone listens on /simple_drone/cmd_vel (ROS2). sim_adapter
subscribes to /cmd_vel and republishes on /simple_drone/cmd_vel; the
bridge then forwards that ROS1→ROS2 so Gazebo receives it.

Why cropping at all
-------------------
Stock libgazebo_ros_camera.so emits cx=W/2, cy=H/2 and fx=fy — those
are hardwired. The real Xtend has cx=222.273, cy=108.548 (way off-
centre, cy=108 vs centre=196 is 22% of image height). Closing the gap
without writing a custom Gazebo plugin:

  1. The SDF cameras render BIGGER than the target (default 600×600)
     at HFOV = 2·atan(600/(2·390.715)) ≈ 1.3098 rad. The rendered fx
     comes out at ≈ 390.7, matching the real Xtend.
  2. This node crops every frame asymmetrically (crop_x=78, crop_y=191
     for the defaults) down to the target 504×392. Cropping doesn't
     change focal length per pixel, so cropped fx is still 390.7 —
     but the principal point now lands at cx=222, cy=109 in the
     cropped image, matching the real camera.

Residual error after all this: fy=390.7 in sim vs 395.8 on the real
camera — a 1.3% asymmetry stock Gazebo can't reproduce (square pixels
only). Everything else matches.

Pose
----
Gazebo publishes geometry_msgs/Pose on /simple_drone/gt_pose. The real
drone publishes geometry_msgs/PoseStamped on /flow_depth/pose_est.
This node also handles that conversion so the real-drone topic naming
is consistent — don't run pose_adapter alongside this in sim, or
there'll be a duplicate publisher on /simple_drone/gt_pose.

Topic plumbing (defaults; all configurable via rosparam)
--------------------------------------------------------
IN  /simple_drone/front/image_raw             (Image, rgb8,  render_w × render_h)
IN  /simple_drone/front_depth/depth/image_raw (Image, 32FC1, render_w × render_h)
IN  /simple_drone/gt_pose                     (Pose)
IN  /cmd_vel                                  (Twist, from FALCON)
IN  /xtend/demo_mode_request                  (String, transitions requested by ROS1)

OUT /xtend/rgb              (Image, rgb8,  target_w × target_h)
OUT /xtend/depth_m          (Image, 32FC1, target_w × target_h)
OUT /flow_depth/pose_est    (PoseStamped)
OUT /simple_drone/cmd_vel   (Twist, bridged to ROS2 → Gazebo)
OUT /xtend/demo_mode        (String, latched — current DemoMode state)
"""

import numpy as np
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import String


# ── DemoMode constants ───────────────────────────────────────────────
# Strings match the ROS2 DemoMode(str, Enum) payloads bridged over
# /xtend/demo_mode and /xtend/demo_mode_request on the real drone.
class DemoMode:
    FLY_STRAIGHT    = "fly_straight"
    TURNING         = "turning"
    VISUAL_SERVOING = "visual_servoing"

    ALL = {FLY_STRAIGHT, TURNING, VISUAL_SERVOING}


class SimAdapter:
    def __init__(self):
        rospy.init_node("sim_adapter")
        G = rospy.get_param

        # ── topic plumbing ─────────────────────────────────────────
        self.in_rgb_t    = G("~in_rgb_topic",
                             "/simple_drone/front/image_raw")
        self.in_depth_t  = G("~in_depth_topic",
                             "/simple_drone/front_depth/depth/image_raw")
        self.in_pose_t   = G("~in_pose_topic",  "/simple_drone/gt_pose")
        self.in_cmd_t    = G("~in_cmd_topic",   "/cmd_vel")
        self.out_rgb_t   = G("~out_rgb_topic",   "/xtend/rgb")
        self.out_depth_t = G("~out_depth_topic", "/xtend/depth_m")
        self.out_pose_t  = G("~out_pose_topic",  "/flow_depth/pose_est")
        self.out_cmd_t   = G("~out_cmd_topic",   "/simple_drone/cmd_vel")

        # ── target image geometry (real Xtend defaults) ────────────
        self.target_w = int(G("~target_width",  504))
        self.target_h = int(G("~target_height", 392))
        self.cx       = float(G("~cx", 222.273))
        self.cy       = float(G("~cy", 108.548))

        # ── rendered image geometry (must match the SDF cameras) ───
        self.render_w = int(G("~render_width",  600))
        self.render_h = int(G("~render_height", 600))

        # ── derived crop offsets ───────────────────────────────────
        # Rendered camera's optical axis is at (render_w/2, render_h/2).
        # After cropping, we want it at (cx, cy) in the cropped frame,
        # so the crop starts at:
        self.crop_x = int(round(self.render_w / 2.0 - self.cx))
        self.crop_y = int(round(self.render_h / 2.0 - self.cy))

        # ── pose frame id ──────────────────────────────────────────
        self.pose_frame = G("~pose_frame_id", "world")

        # ── initial DemoMode state ─────────────────────────────────
        self.demo_mode = str(G("~initial_demo_mode", DemoMode.FLY_STRAIGHT))
        if self.demo_mode not in DemoMode.ALL:
            rospy.logwarn(
                "sim_adapter: ~initial_demo_mode=%r is not one of %s; "
                "forcing %s",
                self.demo_mode, sorted(DemoMode.ALL), DemoMode.FLY_STRAIGHT)
            self.demo_mode = DemoMode.FLY_STRAIGHT

        # Sanity-check the crop window fits inside the rendered image.
        # Misconfigured render_width/height vs SDF is the most common
        # bug, so fail loudly at startup rather than silently dropping
        # every frame later.
        if (self.crop_x < 0 or self.crop_y < 0 or
            self.crop_x + self.target_w > self.render_w or
            self.crop_y + self.target_h > self.render_h):
            rospy.logfatal(
                "sim_adapter: crop window out of bounds. "
                "render=%dx%d  target=%dx%d  crop=(%d,%d)  "
                "right=%d  bottom=%d  → enlarge the SDF render size "
                "OR adjust ~cx/~cy",
                self.render_w, self.render_h,
                self.target_w, self.target_h,
                self.crop_x, self.crop_y,
                self.crop_x + self.target_w,
                self.crop_y + self.target_h)
            raise RuntimeError("bad crop geometry")

        # ── pub/sub ────────────────────────────────────────────────
        self.pub_rgb   = rospy.Publisher(
            self.out_rgb_t,   Image,       queue_size=1)
        self.pub_depth = rospy.Publisher(
            self.out_depth_t, Image,       queue_size=1)
        self.pub_pose  = rospy.Publisher(
            self.out_pose_t,  PoseStamped, queue_size=10)
        self.pub_cmd   = rospy.Publisher(
            self.out_cmd_t,   Twist,       queue_size=10)

        # DemoMode topics. latch=True ≈ transient_local on the real
        # ROS2 publisher, so late-joining subscribers (waypoint_follower
        # restart, etc.) immediately get the current state.
        self.pub_demo = rospy.Publisher(
            "/xtend/demo_mode", String, queue_size=10, latch=True)

        rospy.Subscriber(self.in_rgb_t,   Image,
                         self._rgb_cb,   queue_size=1)
        rospy.Subscriber(self.in_depth_t, Image,
                         self._depth_cb, queue_size=1)
        rospy.Subscriber(self.in_pose_t,  Pose,
                         self._pose_cb,  queue_size=10)
        rospy.Subscriber(self.in_cmd_t,   Twist,
                         self._cmd_cb,   queue_size=10)
        rospy.Subscriber("/xtend/demo_mode_request", String,
                         self._demo_request_cb, queue_size=10)

        # Publish initial state so any (current or future) latched
        # subscriber sees a valid mode without waiting for a request.
        self.pub_demo.publish(String(data=self.demo_mode))

        rospy.loginfo("=" * 64)
        rospy.loginfo("sim_adapter ready")
        rospy.loginfo("  rgb   in : %s", self.in_rgb_t)
        rospy.loginfo("  depth in : %s", self.in_depth_t)
        rospy.loginfo("  pose  in : %s", self.in_pose_t)
        rospy.loginfo("  cmd   in : %s", self.in_cmd_t)
        rospy.loginfo("  rgb   out: %s", self.out_rgb_t)
        rospy.loginfo("  depth out: %s", self.out_depth_t)
        rospy.loginfo("  pose  out: %s   (frame_id=%s)",
                      self.out_pose_t, self.pose_frame)
        rospy.loginfo("  cmd   out: %s", self.out_cmd_t)
        rospy.loginfo("  render   : %d × %d  (must match SDF)",
                      self.render_w, self.render_h)
        rospy.loginfo("  target   : %d × %d   cx=%.3f cy=%.3f",
                      self.target_w, self.target_h, self.cx, self.cy)
        rospy.loginfo("  crop     : x=%d  y=%d   "
                      "(right=%d bottom=%d)",
                      self.crop_x, self.crop_y,
                      self.crop_x + self.target_w,
                      self.crop_y + self.target_h)
        rospy.loginfo("  demo mode: %s   (initial; latched on "
                      "/xtend/demo_mode)", self.demo_mode)
        rospy.loginfo("=" * 64)

    # ── byte-level crop, encoding-agnostic ──────────────────────────
    # We never decode the image — we just slice the raw byte buffer.
    # This works for rgb8, bgr8, mono8, 32FC1, 16UC1, anything: every
    # row is a fixed-size stripe of `step` bytes, and we want a
    # contiguous sub-rectangle of full byte-rows.
    def _crop(self, msg):
        if msg.width != self.render_w or msg.height != self.render_h:
            rospy.logwarn_throttle(
                5.0,
                "sim_adapter: incoming %dx%d, expected %dx%d "
                "— SDF render size doesn't match ~render_width/height. "
                "Dropping frame.",
                msg.width, msg.height, self.render_w, self.render_h)
            return None

        bpp = msg.step // msg.width      # bytes per pixel
        # View the bytes as a (height, width*bpp) array. No copy yet.
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.step)
        # Slice rows + byte columns, then copy out to make it contiguous.
        crop = arr[
            self.crop_y : self.crop_y + self.target_h,
            self.crop_x * bpp : (self.crop_x + self.target_w) * bpp
        ].copy()

        out = Image()
        out.header       = msg.header     # preserve stamp + frame_id
        out.height       = self.target_h
        out.width        = self.target_w
        out.encoding     = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step         = self.target_w * bpp
        out.data         = crop.tobytes()
        return out

    # ── callbacks ──────────────────────────────────────────────────
    def _rgb_cb(self, msg):
        out = self._crop(msg)
        if out is not None:
            self.pub_rgb.publish(out)

    def _depth_cb(self, msg):
        out = self._crop(msg)
        if out is not None:
            self.pub_depth.publish(out)

    def _pose_cb(self, msg):
        s = PoseStamped()
        s.header.stamp = rospy.Time.now()
        s.header.frame_id = self.pose_frame
        s.pose = msg
        self.pub_pose.publish(s)

    def _cmd_cb(self, msg):
        # Pure passthrough — same Twist message, just under the topic
        # name Gazebo's sjtu_drone listens on. The bridge handles the
        # ROS1→ROS2 hop.
        self.pub_cmd.publish(msg)

    def _demo_request_cb(self, msg):
        req = msg.data.strip()
        if req not in DemoMode.ALL:
            rospy.logwarn(
                "sim_adapter: ignoring demo_mode_request %r — "
                "not one of %s", req, sorted(DemoMode.ALL))
            return
        if req == self.demo_mode:
            return                       # no-op, suppress chatter
        rospy.loginfo("sim_adapter: demo_mode  %s → %s",
                      self.demo_mode, req)
        self.demo_mode = req
        self.pub_demo.publish(String(data=self.demo_mode))


if __name__ == "__main__":
    try:
        SimAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass