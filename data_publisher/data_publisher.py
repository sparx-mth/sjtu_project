#!/usr/bin/env python3
"""
data_publisher.py  --  "Office replay" stand-in for the live XTEND drone.

Publishes the same two topics the real drone exposes to the FALCON bridge, with
the same types, QoS, rate, frames and timestamp synchronization as the office:

    /xtend/depth_m         sensor_msgs/Image          best_effort | volatile | depth=1
    /flow_depth/pose_est   geometry_msgs/PoseStamped  best_effort | volatile | depth=5

Timestamp synchronization (traced through the real pipeline)
------------------------------------------------------------
  online_nav_bridge_publisher : /xtend/rgb   stamp = capture time S, frame xtend_camera
  depth_processor_node        : /xtend/depth_m  copies the RGB header  -> stamp = S
  flow_depth_velocity_node    : /flow_depth/velocity  stamp = rgb stamp -> S
  velocity_integrator         : /flow_depth/pose_est  stamp = vel stamp -> S, frame odom

So BOTH topics for one frame carry the IDENTICAL header.stamp S, but pose_est is
delivered later in wall-clock time (it waits for depth inference + optical flow +
integration). Downstream matches them by header.stamp, not by arrival order.

This replayer reproduces that exactly. For each recorded frame i:
  * one shared stamp S_i is taken once,
  * the depth image is published with header.stamp = S_i,
  * the matching pose (yaw -> quaternion) is published with header.stamp = S_i,
  * the pose is delivered after a wall-clock gap ~ Normal(mean, std) (depth first
    on average; ~0 = together; negative = pose slightly first), all clamped.

No DDS configuration, no middleware override: it uses ROS 2's default RMW, exactly
like the Jetson (Humble, ROS_DOMAIN_ID=5). cv_bridge is not used; the Image is
built directly to keep the container minimal.

Recorded data layout (auto-discovered under /data, or set explicitly):
    <take>/depth_npy/*.npy              depth in METERS, float
    estimated_trajectory_<take>.json    [{"image": "...", "pose": {x,y,z,yaw}}, ...]
"""

from __future__ import annotations

import glob
import heapq
import itertools
import json
import math
import os
import random
import time

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32


class DataPublisher(Node):
    def __init__(self):
        super().__init__("data_publisher")

        # ---- Topics / frames (match the office) -----------------------------
        self.declare_parameter("depth_topic", "/xtend/depth_m")
        self.declare_parameter("pose_topic", "/flow_depth/pose_est")
        self.declare_parameter("depth_frame_id", "xtend_camera")
        self.declare_parameter("pose_frame_id", "odom")

        # ---- Depth encoding: office runs 16UC1 (mm). 32FC1 = meters. --------
        self.declare_parameter("depth_encoding", "16UC1")   # "16UC1" or "32FC1"

        # ---- CameraInfo (office publishes it from a calibration YAML) -------
        self.declare_parameter("publish_camera_info", True)
        self.declare_parameter("camera_info_topic", "/xtend/camera_info")
        self.declare_parameter("camera_info_yaml", "")      # "" => auto-discover under /data
        self.declare_parameter("camera_info_frame_id", "")  # "" => use depth_frame_id
        self.declare_parameter("camera_info_qos_depth", 5)

        # ---- Bearing (node 1 publishes /xtend/bearing; here from recorded yaw) --
        self.declare_parameter("publish_bearing", True)
        self.declare_parameter("bearing_topic", "/xtend/bearing")
        self.declare_parameter("bearing_qos_depth", 10)

        # ---- Timing ---------------------------------------------------------
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("pose_delay_mean", 0.020)    # sec, depth->pose gap
        self.declare_parameter("pose_delay_std", 0.010)
        self.declare_parameter("pose_delay_min", -0.050)
        self.declare_parameter("pose_delay_max", 0.200)
        self.declare_parameter("scheduler_resolution", 0.002)
        self.declare_parameter("seed", -1)                  # <0 = nondeterministic

        # ---- Looping --------------------------------------------------------
        self.declare_parameter("loop", True)
        self.declare_parameter("max_loops", 0)              # 0 = infinite

        # ---- QoS queue depths (match the source nodes) ----------------------
        self.declare_parameter("depth_qos_depth", 1)        # depth_processor: 1
        self.declare_parameter("pose_qos_depth", 5)         # velocity_integrator: 5

        # ---- Data location --------------------------------------------------
        # Defaults point at take_003 inside the container (host ~/Desktop -> /data).
        # If these don't exist (e.g. a different take is mounted), the node falls
        # back to auto-discovery under data_root.
        self.declare_parameter("data_root", "/data")
        self.declare_parameter(
            "depth_dir",
            "/data/xtend_rectified_depth_take_003_20260429_160647/depth_npy")
        self.declare_parameter(
            "json_path",
            "/data/estimated_trajectory_xtend_rectified_depth_take_003_20260429_160647.json")
        self.declare_parameter("depth_glob", "*.npy")

        gp = lambda n: self.get_parameter(n).value
        self.depth_topic = gp("depth_topic")
        self.pose_topic = gp("pose_topic")
        self.depth_frame_id = gp("depth_frame_id")
        self.pose_frame_id = gp("pose_frame_id")

        self.depth_encoding = str(gp("depth_encoding")).upper()
        if self.depth_encoding not in ("16UC1", "32FC1"):
            raise ValueError(f"depth_encoding must be 16UC1 or 32FC1, got {self.depth_encoding}")

        self.rate_hz = float(gp("publish_rate_hz"))
        self.delay_mean = float(gp("pose_delay_mean"))
        self.delay_std = float(gp("pose_delay_std"))
        self.delay_min = float(gp("pose_delay_min"))
        self.delay_max = float(gp("pose_delay_max"))
        self.sched_res = float(gp("scheduler_resolution"))

        self.loop = bool(gp("loop"))
        self.max_loops = int(gp("max_loops"))

        self.data_root = gp("data_root")
        self.depth_dir = gp("depth_dir")
        self.json_path = gp("json_path")
        self.depth_glob = gp("depth_glob")

        seed = int(gp("seed"))
        self.rng = random.Random(None if seed < 0 else seed)

        # ---- QoS exactly like the office nodes (best_effort, volatile) ------
        depth_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(gp("depth_qos_depth")),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(gp("pose_qos_depth")),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.depth_pub = self.create_publisher(Image, self.depth_topic, depth_qos)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, pose_qos)

        # ---- CameraInfo (loaded from the office calibration YAML) -----------
        self.publish_camera_info = bool(gp("publish_camera_info"))
        self.camera_info_topic = gp("camera_info_topic")
        self.camera_info_frame_id = gp("camera_info_frame_id") or self.depth_frame_id
        self._cam_fields = None
        self.caminfo_pub = None
        if self.publish_camera_info:
            cam_yaml = gp("camera_info_yaml")
            if not (cam_yaml and os.path.isfile(cam_yaml)):
                cam_yaml = self._auto_camera_info_yaml()
            if cam_yaml:
                try:
                    self._cam_fields = self._load_camera_info_fields(cam_yaml)
                    caminfo_qos = QoSProfile(
                        history=HistoryPolicy.KEEP_LAST,
                        depth=int(gp("camera_info_qos_depth")),
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                    )
                    self.caminfo_pub = self.create_publisher(
                        CameraInfo, self.camera_info_topic, caminfo_qos)
                    self.get_logger().info(
                        f"CameraInfo: {self.camera_info_topic} from {cam_yaml} "
                        f"({self._cam_fields['width']}x{self._cam_fields['height']})")
                except Exception as e:
                    self.publish_camera_info = False
                    self.get_logger().error(f"CameraInfo disabled, YAML parse failed: {e}")
            else:
                self.publish_camera_info = False
                self.get_logger().warn(
                    "CameraInfo requested but no YAML found. Set -p camera_info_yaml:=/config/camera_info.yaml")

        # ---- Bearing publisher (Float32) -----------------------------------
        self.publish_bearing = bool(gp("publish_bearing"))
        self.bearing_pub = None
        if self.publish_bearing:
            bearing_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=int(gp("bearing_qos_depth")),
                reliability=ReliabilityPolicy.RELIABLE,   # integrator subscribes RELIABLE
                durability=DurabilityPolicy.VOLATILE,
            )
            self.bearing_pub = self.create_publisher(Float32, gp("bearing_topic"), bearing_qos)

        # ---- Resolve + load data --------------------------------------------
        self._resolve_paths()
        with open(self.json_path, "r") as f:
            self.trajectory_data = json.load(f)
        self.depth_files = sorted(glob.glob(os.path.join(self.depth_dir, self.depth_glob)))
        if not self.depth_files:
            raise FileNotFoundError(f"No '{self.depth_glob}' in {self.depth_dir}")

        self.max_idx = min(len(self.trajectory_data), len(self.depth_files))
        if len(self.trajectory_data) != len(self.depth_files):
            self.get_logger().warn(
                f"Count mismatch: {len(self.depth_files)} depth vs "
                f"{len(self.trajectory_data)} poses; using first {self.max_idx} by index."
            )

        self.get_logger().info(
            f"Depth dir : {self.depth_dir}\n"
            f"JSON      : {self.json_path}\n"
            f"Frames    : {self.max_idx}  @ {self.rate_hz:.2f} Hz\n"
            f"Depth enc : {self.depth_encoding}"
            f"{' (mm)' if self.depth_encoding == '16UC1' else ' (m)'}\n"
            f"Pose delay: N(mean={self.delay_mean*1e3:.1f} ms, std={self.delay_std*1e3:.1f} ms)\n"
            f"Topics    : {self.depth_topic} | {self.pose_topic}  (best_effort/volatile)"
        )

        # ---- Scheduler (min-heap dispatched by a fast timer) ----------------
        self._heap = []
        self._seq = itertools.count()
        self._sched_timer = self.create_timer(self.sched_res, self._dispatch)

        self.current_idx = 0
        self.loops_done = 0
        self._log_ctr = 0

        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.1
        self.frame_timer = self.create_timer(period, self.frame_callback)

    # ------------------------------------------------------------------ paths
    def _resolve_paths(self):
        root = self.data_root
        if not self.depth_dir or not os.path.isdir(self.depth_dir):
            if self.depth_dir:
                self.get_logger().warn(
                    f"depth_dir '{self.depth_dir}' not found; auto-discovering under {root}")
                self.depth_dir = ""
            cands = sorted(glob.glob(os.path.join(root, "**", "depth_npy"), recursive=True))
            if not cands:
                for d in sorted(glob.glob(os.path.join(root, "*"))):
                    if os.path.isdir(d) and glob.glob(os.path.join(d, self.depth_glob)):
                        cands.append(d)
            if not cands and glob.glob(os.path.join(root, self.depth_glob)):
                cands.append(root)
            if not cands:
                raise FileNotFoundError(
                    f"No depth folder under {root}. Set -p depth_dir:=/path/to/depth_npy"
                )
            self.depth_dir = cands[0]

        base = os.path.basename(os.path.normpath(self.depth_dir))
        take = (os.path.basename(os.path.dirname(os.path.normpath(self.depth_dir)))
                if base == "depth_npy" else base)
        take_parent = os.path.dirname(os.path.normpath(self.depth_dir))

        if not self.json_path or not os.path.isfile(self.json_path):
            if self.json_path:
                self.get_logger().warn(
                    f"json_path '{self.json_path}' not found; auto-discovering under {root}")
                self.json_path = ""
            for p in (os.path.join(root, f"estimated_trajectory_{take}.json"),
                      os.path.join(root, f"{take}.json"),
                      os.path.join(take_parent, f"estimated_trajectory_{take}.json"),
                      os.path.join(take_parent, f"{take}.json")):
                if os.path.isfile(p):
                    self.json_path = p
                    break
            if not self.json_path:
                loose = sorted(glob.glob(os.path.join(root, "*.json")))
                if loose:
                    self.json_path = loose[0]
                    self.get_logger().warn(f"Falling back to {self.json_path}")
            if not self.json_path:
                raise FileNotFoundError(
                    f"No trajectory JSON for '{take}' under {root}. Set -p json_path:=..."
                )

    # ------------------------------------------------------------ camera info
    def _auto_camera_info_yaml(self):
        for pat in ("*calib*.yaml", "*camera*.yaml", "*.yaml"):
            hits = sorted(glob.glob(os.path.join(self.data_root, pat)))
            if hits:
                return hits[0]
        return None

    def _load_camera_info_fields(self, path: str) -> dict:
        with open(path, "r") as f:
            cfg = yaml.safe_load(f)

        def mat(name):
            node = cfg.get(name)
            if isinstance(node, dict) and "data" in node:
                return [float(x) for x in node["data"]]
            return None

        k, d, r, p = mat("camera_matrix"), mat("distortion_coefficients"), \
            mat("rectification_matrix"), mat("projection_matrix")

        if k is None and all(key in cfg for key in ("fx", "fy", "cx", "cy")):
            fx, fy, cx, cy = (float(cfg["fx"]), float(cfg["fy"]),
                              float(cfg["cx"]), float(cfg["cy"]))
            k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        if k is None:
            raise ValueError("camera_matrix or fx/fy/cx/cy missing")

        if r is None:
            r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        if p is None:
            p = [k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0]
        if d is None:
            d = []

        return {
            "width": int(cfg.get("image_width", 0)),
            "height": int(cfg.get("image_height", 0)),
            "distortion_model": str(cfg.get("distortion_model", "plumb_bob")),
            "k": k, "d": d, "r": r, "p": p,
        }

    def _build_camera_info_msg(self, stamp) -> CameraInfo:
        f = self._cam_fields
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_info_frame_id
        msg.height = f["height"]
        msg.width = f["width"]
        msg.distortion_model = f["distortion_model"]
        msg.d = list(f["d"])
        msg.k = list(f["k"])
        msg.r = list(f["r"])
        msg.p = list(f["p"])
        return msg

    # ----------------------------------------------------------- msg builders
    def _build_depth_msg(self, idx: int, stamp) -> Image:
        arr = np.squeeze(np.load(self.depth_files[idx])).astype(np.float32)  # meters
        if self.depth_encoding == "16UC1":
            mm = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
            data = np.ascontiguousarray(np.clip(mm, 0.0, 65535.0).astype(np.uint16))
            encoding, itemsize = "16UC1", 2
        else:
            data = np.ascontiguousarray(arr)  # 32FC1, meters
            encoding, itemsize = "32FC1", 4

        h, w = data.shape
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.depth_frame_id
        msg.height = int(h)
        msg.width = int(w)
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(w * itemsize)
        msg.data = data.tobytes()
        return msg

    def _build_pose_msg(self, idx: int, stamp) -> PoseStamped:
        pose = self.trajectory_data[idx]["pose"]
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.pose_frame_id
        msg.pose.position.x = float(pose.get("x", 0.0))
        msg.pose.position.y = float(pose.get("y", 0.0))
        msg.pose.position.z = float(pose.get("z", 0.0))
        half = 0.5 * float(pose.get("yaw", 0.0))
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        return msg

    # ----------------------------------------------------------- scheduling
    def _schedule(self, fire_monotonic: float, action):
        heapq.heappush(self._heap, (fire_monotonic, next(self._seq), action))

    def _dispatch(self):
        now = time.monotonic()
        while self._heap and self._heap[0][0] <= now:
            _, _, action = heapq.heappop(self._heap)
            action()

    def frame_callback(self):
        if self.current_idx >= self.max_idx:
            self.loops_done += 1
            if (not self.loop) or (self.max_loops and self.loops_done >= self.max_loops):
                self.get_logger().info(f"Finished {self.loops_done} loop(s). Stopping.")
                self.frame_timer.cancel()
                return
            self.get_logger().info("End of data. Restarting loop...")
            self.current_idx = 0

        idx = self.current_idx
        self.current_idx += 1

        stamp = self.get_clock().now().to_msg()   # one shared stamp for BOTH msgs
        now_mono = time.monotonic()

        try:
            depth_msg = self._build_depth_msg(idx, stamp)
        except Exception as e:
            self.get_logger().error(f"depth frame {idx} failed: {e}")
            return
        pose_msg = self._build_pose_msg(idx, stamp)
        caminfo_msg = self._build_camera_info_msg(stamp) if self.publish_camera_info else None

        delay = max(self.delay_min, min(self.delay_max,
                                        self.rng.gauss(self.delay_mean, self.delay_std)))
        depth_off = max(0.0, -delay)
        pose_off = max(0.0, delay)

        def _emit_depth(d=depth_msg, c=caminfo_msg):
            self.depth_pub.publish(d)
            if c is not None:
                self.caminfo_pub.publish(c)   # CameraInfo travels with depth, same stamp

        self._schedule(now_mono + depth_off, _emit_depth)
        self._schedule(now_mono + pose_off, lambda m=pose_msg: self.pose_pub.publish(m))

        if self.publish_bearing:
            bmsg = Float32()
            bmsg.data = float(self.trajectory_data[idx]["pose"].get("yaw", 0.0))
            self.bearing_pub.publish(bmsg)

        self._log_ctr += 1
        if self._log_ctr % 50 == 0:
            order = "depth->pose" if delay > 0 else "pose->depth" if delay < 0 else "together"
            self.get_logger().info(f"frame {idx}: gap {delay*1e3:+.1f} ms ({order})")


def main(args=None):
    rclpy.init(args=args)
    node = DataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()