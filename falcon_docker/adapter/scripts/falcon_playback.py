#!/usr/bin/env python3
"""
falcon_playback.py — feed FALCON from a recorded dataset (no Gazebo).

Drop-in replacement for falcon_adapter.py + the simulated drone for the
case where you already have a recording from a real drone:

    poses.json     list of {"image": "frame_NNNNNN.jpg",
                            "pose": {"x": .., "y": .., "z": .., "yaw": ..}}
    frames/        folder containing depth_NNNNNN.npy alongside the rgb
                   frames (one .npy per .jpg, same stem).

For each entry, in order, this node publishes the SAME inputs FALCON's
voxel mapper expects — i.e. exactly what falcon_adapter.py publishes from
the live drone:

    /odom_world                       nav_msgs/Odometry      body in world
    /map_ros/pose                     geometry_msgs/PoseStamped  camera in world
    /map_ros/depth                    sensor_msgs/Image      32FC1 depth (m)
    /map_ros/depth/camera_info        sensor_msgs/CameraInfo
    TF: world -> body -> camera

FALCON sees these and builds the map the same way it would during a live
run. The planner output (cmd_to_vel) is ignored — the trajectory is
fixed by the recording.

All knobs (camera intrinsics, body->camera offset, depth scale, playback
rate, ...) are ROS params with sensible defaults.

JSON pose convention assumed:
    x, y, z   metres, world frame (ENU-ish: x forward, y left, z up)
    yaw       radians, rotation about world +Z
The body frame is FLU (x forward, y left, z up), camera frame is RDF
(x right, y down, z forward) — same as falcon_adapter.py.
"""

import json
import os

import numpy as np
import rospy
import tf
import tf.transformations as tft

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image


def _yaw_to_quat(yaw):
    """ROS quaternion (x, y, z, w) for a pure yaw rotation about world +Z."""
    return tft.quaternion_from_euler(0.0, 0.0, yaw)


class FalconPlayback:
    def __init__(self):
        rospy.init_node("falcon_playback")

        # ── Dataset ─────────────────────────────────────────────────
        self.poses_json   = rospy.get_param("~poses_json")
        self.frames_dir   = rospy.get_param("~frames_dir")
        self.depth_suffix = rospy.get_param("~depth_suffix", ".npy")
        self.depth_scale  = float(rospy.get_param("~depth_scale", 1.0))   # multiplier -> metres
        self.depth_max    = float(rospy.get_param("~depth_max",   0.0))   # 0 = no clip
        self.start_index  = int(rospy.get_param("~start_index",   0))
        self.stride       = int(rospy.get_param("~stride",        1))
        self.loop         = bool(rospy.get_param("~loop",         False))

        # ── Camera intrinsics (pinhole) ─────────────────────────────
        self.fx     = float(rospy.get_param("~fx", 320.0))
        self.fy     = float(rospy.get_param("~fy", 320.0))
        self.cx     = float(rospy.get_param("~cx", 320.0))
        self.cy     = float(rospy.get_param("~cy", 240.0))
        self.width  = int(rospy.get_param("~image_width",  640))
        self.height = int(rospy.get_param("~image_height", 480))

        # ── Frames ──────────────────────────────────────────────────
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.body_frame  = rospy.get_param("~body_frame",  "body")
        self.cam_frame   = rospy.get_param("~cam_frame",   "camera")

        # Body (FLU) -> camera (RDF). Default rotation is identical to
        # falcon_adapter.py; lever-arm defaults to zero (pose is assumed
        # to already be at the camera optical centre — common for VIO
        # outputs). Override cam_offset_* if your pose is the IMU/body
        # centre and the camera sits some distance away.
        cam_x = float(rospy.get_param("~cam_offset_x", 0.0))
        cam_y = float(rospy.get_param("~cam_offset_y", 0.0))
        cam_z = float(rospy.get_param("~cam_offset_z", 0.0))
        self.T_b_c = np.array([
            [ 0.0,  0.0, 1.0, cam_x],
            [-1.0,  0.0, 0.0, cam_y],
            [ 0.0, -1.0, 0.0, cam_z],
            [ 0.0,  0.0, 0.0, 1.0 ],
        ])
        self.T_b_c_quat  = tft.quaternion_from_matrix(self.T_b_c)
        self.T_b_c_trans = (cam_x, cam_y, cam_z)

        # ── Playback ────────────────────────────────────────────────
        self.rate_hz       = float(rospy.get_param("~playback_rate_hz", 10.0))
        self.startup_delay = float(rospy.get_param("~startup_delay_sec", 2.0))

        # ── Load dataset ────────────────────────────────────────────
        if not os.path.isfile(self.poses_json):
            rospy.logfatal("falcon_playback: poses_json not found: %s", self.poses_json)
            raise SystemExit(1)
        if not os.path.isdir(self.frames_dir):
            rospy.logfatal("falcon_playback: frames_dir not found: %s", self.frames_dir)
            raise SystemExit(1)

        with open(self.poses_json, "r") as f:
            self.entries = json.load(f)
        if not isinstance(self.entries, list) or not self.entries:
            rospy.logfatal("falcon_playback: empty/invalid JSON %s", self.poses_json)
            raise SystemExit(1)

        rospy.loginfo("falcon_playback: %d frames  intrinsics=(fx=%.1f fy=%.1f cx=%.1f cy=%.1f) "
                      "size=%dx%d  rate=%.1fHz  scale=%.4g  cam_offset=(%.2f,%.2f,%.2f)",
                      len(self.entries), self.fx, self.fy, self.cx, self.cy,
                      self.width, self.height, self.rate_hz, self.depth_scale,
                      cam_x, cam_y, cam_z)

        # ── Publishers (must match falcon_adapter.py exactly) ───────
        self.odom_pub     = rospy.Publisher("/odom_world",                 Odometry,    queue_size=10)
        self.pose_pub     = rospy.Publisher("/map_ros/pose",               PoseStamped, queue_size=10)
        self.depth_pub    = rospy.Publisher("/map_ros/depth",              Image,       queue_size=2)
        self.cam_info_pub = rospy.Publisher("/map_ros/depth/camera_info",  CameraInfo,  queue_size=2)
        self.tf_br = tf.TransformBroadcaster()

        # State for finite-difference velocity
        self._last_xyz   = None
        self._last_stamp = None

    # ────────────────────────────────────────────────────────────────
    # Frame loaders
    # ────────────────────────────────────────────────────────────────
    def _depth_path(self, image_name):
        stem = os.path.splitext(image_name)[0]
        return os.path.join(self.frames_dir, stem + self.depth_suffix)

    def _load_depth(self, image_name):
        path = self._depth_path(image_name)
        if not os.path.isfile(path):
            return None
        try:
            arr = np.load(path)
        except Exception as e:
            rospy.logwarn("falcon_playback: failed to load %s: %s", path, e)
            return None
        arr = np.asarray(arr, dtype=np.float32)
        if self.depth_scale != 1.0:
            arr = arr * np.float32(self.depth_scale)
        # Sanitize: NaN/Inf -> 0 (= "no return"), and optional clip.
        arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
        if self.depth_max > 0.0:
            arr[arr > self.depth_max] = 0.0
        return arr

    # ────────────────────────────────────────────────────────────────
    # Message builders
    # ────────────────────────────────────────────────────────────────
    def _camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp     = stamp
        info.header.frame_id  = self.cam_frame
        info.height           = self.height
        info.width            = self.width
        info.distortion_model = "plumb_bob"
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [self.fx, 0.0,     self.cx,
                  0.0,     self.fy, self.cy,
                  0.0,     0.0,     1.0]
        info.R = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.P = [self.fx, 0.0,     self.cx, 0.0,
                  0.0,     self.fy, self.cy, 0.0,
                  0.0,     0.0,     1.0,     0.0]
        return info

    def _depth_image(self, depth_arr, stamp):
        h, w = depth_arr.shape[:2]
        if (h, w) != (self.height, self.width):
            rospy.logwarn_throttle(5.0,
                "falcon_playback: depth shape %dx%d != intrinsics %dx%d "
                "(check fx/fy/cx/cy + image_width/image_height)",
                h, w, self.height, self.width)
        img = Image()
        img.header.stamp    = stamp
        img.header.frame_id = self.cam_frame
        img.height       = h
        img.width        = w
        img.encoding     = "32FC1"
        img.is_bigendian = 0
        img.step         = 4 * w   # bytes per row, float32
        img.data         = depth_arr.astype(np.float32, copy=False).tobytes()
        return img

    # ────────────────────────────────────────────────────────────────
    # Publish one frame
    # ────────────────────────────────────────────────────────────────
    def _publish_frame(self, entry):
        image = entry.get("image", "")
        pose  = entry.get("pose",  {})
        try:
            x   = float(pose["x"])
            y   = float(pose["y"])
            z   = float(pose["z"])
            yaw = float(pose["yaw"])
        except (KeyError, TypeError, ValueError) as e:
            rospy.logwarn("falcon_playback: skipping malformed entry %r: %s", image, e)
            return False

        depth = self._load_depth(image)
        if depth is None:
            rospy.logwarn_throttle(5.0,
                "falcon_playback: skipping %s (depth file missing/unreadable)", image)
            return False

        stamp = rospy.Time.now()

        # Body pose in world
        q_b = _yaw_to_quat(yaw)              # (x, y, z, w)
        T_w_b = tft.quaternion_matrix(q_b)
        T_w_b[0, 3], T_w_b[1, 3], T_w_b[2, 3] = x, y, z

        # Camera pose in world  =  T_w_b · T_b_c
        T_w_c    = T_w_b @ self.T_b_c
        cam_pos  = T_w_c[:3, 3]
        cam_quat = tft.quaternion_from_matrix(T_w_c)

        # Velocity (finite diff, world frame). Used by /odom_world.twist.
        vel = (0.0, 0.0, 0.0)
        if self._last_xyz is not None and self._last_stamp is not None:
            dt = (stamp - self._last_stamp).to_sec()
            if dt > 1e-6:
                vel = ((x - self._last_xyz[0]) / dt,
                       (y - self._last_xyz[1]) / dt,
                       (z - self._last_xyz[2]) / dt)
        self._last_xyz   = (x, y, z)
        self._last_stamp = stamp

        # 1) /odom_world (body in world)
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id  = self.body_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = q_b[0]
        odom.pose.pose.orientation.y = q_b[1]
        odom.pose.pose.orientation.z = q_b[2]
        odom.pose.pose.orientation.w = q_b[3]
        odom.twist.twist.linear.x = vel[0]
        odom.twist.twist.linear.y = vel[1]
        odom.twist.twist.linear.z = vel[2]
        self.odom_pub.publish(odom)

        # 2) /map_ros/pose (camera in world — FALCON's sensor pose)
        ps = PoseStamped()
        ps.header.stamp    = stamp
        ps.header.frame_id = self.world_frame
        ps.pose.position.x    = float(cam_pos[0])
        ps.pose.position.y    = float(cam_pos[1])
        ps.pose.position.z    = float(cam_pos[2])
        ps.pose.orientation.x = float(cam_quat[0])
        ps.pose.orientation.y = float(cam_quat[1])
        ps.pose.orientation.z = float(cam_quat[2])
        ps.pose.orientation.w = float(cam_quat[3])
        self.pose_pub.publish(ps)

        # 3) /map_ros/depth + /map_ros/depth/camera_info
        self.cam_info_pub.publish(self._camera_info(stamp))
        self.depth_pub.publish(self._depth_image(depth, stamp))

        # 4) TF: world -> body -> camera
        self.tf_br.sendTransform(
            (x, y, z), q_b, stamp, self.body_frame, self.world_frame)
        self.tf_br.sendTransform(
            self.T_b_c_trans, self.T_b_c_quat, stamp, self.cam_frame, self.body_frame)

        return True

    # ────────────────────────────────────────────────────────────────
    # Main loop
    # ────────────────────────────────────────────────────────────────
    def run(self):
        rospy.loginfo("falcon_playback: waiting %.1fs for FALCON to come up...",
                      self.startup_delay)
        rospy.sleep(self.startup_delay)

        rate = rospy.Rate(self.rate_hz)
        n = len(self.entries)
        i = self.start_index
        published = 0
        skipped   = 0
        last_log  = rospy.Time.now()

        rospy.loginfo("falcon_playback: starting (frames=%d, stride=%d, loop=%s)",
                      n, self.stride, self.loop)

        while not rospy.is_shutdown():
            if i >= n:
                if self.loop:
                    i = self.start_index
                    rospy.loginfo("falcon_playback: looping (published=%d)", published)
                else:
                    rospy.loginfo("falcon_playback: done — published=%d skipped=%d",
                                  published, skipped)
                    break

            ok = self._publish_frame(self.entries[i])
            if ok:
                published += 1
            else:
                skipped += 1

            now = rospy.Time.now()
            if (now - last_log).to_sec() >= 5.0:
                rospy.loginfo("falcon_playback: progress %d/%d  (skipped=%d)",
                              i, n, skipped)
                last_log = now

            i += self.stride
            rate.sleep()


if __name__ == "__main__":
    try:
        FalconPlayback().run()
    except rospy.ROSInterruptException:
        pass