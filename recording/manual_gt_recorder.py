#!/usr/bin/env python3
import csv
import json
import os
import sys
import threading
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def pose_to_c2w(tx: float, ty: float, tz: float, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    T[:3, 3] = [tx, ty, tz]
    return T


def invert_se3(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


class ManualGTRecorder(Node):
    def __init__(self) -> None:
        super().__init__("manual_gt_recorder")

        self.declare_parameter("rgb_topic", "/simple_drone/front/image_raw")
        self.declare_parameter("depth_topic", "/simple_drone/front_depth/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/simple_drone/front/camera_info")
        self.declare_parameter("odom_topic", "/simple_drone/odom")
        self.declare_parameter("pose_topic", "/simple_drone/gt_pose")
        self.declare_parameter("use_odometry", False)
        self.declare_parameter("output_dir", "./gazebo_capture")
        self.declare_parameter("max_frames", -1)
        self.declare_parameter("depth_scale_to_meters", 1.0)

        self.rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.use_odometry = self.get_parameter("use_odometry").get_parameter_value().bool_value
        self.output_dir = Path(self.get_parameter("output_dir").get_parameter_value().string_value)
        self.max_frames = self.get_parameter("max_frames").get_parameter_value().integer_value
        self.depth_scale_to_meters = self.get_parameter("depth_scale_to_meters").get_parameter_value().double_value

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.rgb_dir = self.output_dir / "rgb"
        self.depth_dir = self.output_dir / "depth"
        self.rgb_dir.mkdir(parents=True, exist_ok=True)
        self.depth_dir.mkdir(parents=True, exist_ok=True)

        self.frames_csv_path = self.output_dir / "frames.csv"
        self.camera_info_json_path = self.output_dir / "camera_info.json"

        self.latest_rgb_msg: Optional[Image] = None
        self.latest_depth_msg: Optional[Image] = None
        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_pose_msg: Optional[Odometry | PoseStamped] = None

        self.frame_idx = 0

        self.csv_file = open(self.frames_csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "frame_idx",
            "rgb_file",
            "depth_file",
            "rgb_stamp_ns",
            "depth_stamp_ns",
            "pose_stamp_ns",
            "tx", "ty", "tz",
            "qx", "qy", "qz", "qw",
        ])

        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        if self.use_odometry:
            self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)
            self.pose_mode = "odometry"
            self.get_logger().info(f"Using odometry topic: {self.odom_topic}")
        else:
            if not self.pose_topic:
                raise ValueError("pose_topic must be set when use_odometry is false")

            pose_type = self.get_topic_names_and_types()
            topic_type = None
            for name, types in pose_type:
                if name == self.pose_topic and len(types) > 0:
                    topic_type = types[0]
                    break

            if topic_type == "geometry_msgs/msg/PoseStamped":
                self.create_subscription(PoseStamped, self.pose_topic, self.pose_stamped_callback, 50)
                self.pose_mode = "pose_stamped"
            elif topic_type == "geometry_msgs/msg/Pose":
                self.create_subscription(Pose, self.pose_topic, self.pose_callback, 50)
                self.pose_mode = "pose"
            else:
                raise ValueError(f"Unsupported pose topic type for {self.pose_topic}: {topic_type}")

            self.get_logger().info(f"Using pose topic: {self.pose_topic} ({self.pose_mode})")

        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"Depth topic: {self.depth_topic}")
        self.get_logger().info(f"CameraInfo topic: {self.camera_info_topic}")
        self.get_logger().info(f"Output dir: {self.output_dir}")
        self.get_logger().info("Press Enter in this terminal to save a frame. Type q then Enter to quit.")

        self.input_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.input_thread.start()

    def rgb_callback(self, msg: Image) -> None:
        with self.lock:
            self.latest_rgb_msg = msg

    def depth_callback(self, msg: Image) -> None:
        with self.lock:
            self.latest_depth_msg = msg

    def camera_info_callback(self, msg: CameraInfo) -> None:
        with self.lock:
            self.latest_camera_info = msg

        data = {
            "width": msg.width,
            "height": msg.height,
            "k": list(msg.k),
            "p": list(msg.p),
            "distortion_model": msg.distortion_model,
            "d": list(msg.d),
            "frame_id": msg.header.frame_id,
        }
        with open(self.camera_info_json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    def odom_callback(self, msg: Odometry) -> None:
        with self.lock:
            self.latest_pose_msg = msg

    def pose_stamped_callback(self, msg: PoseStamped) -> None:
        with self.lock:
            self.latest_pose_msg = msg

    def pose_callback(self, msg: Pose) -> None:
        with self.lock:
            self.latest_pose_msg = msg

    def _stamp_to_ns(self, msg) -> int:
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _extract_pose(self) -> tuple[float, float, float, float, float, float, float, int]:
        if self.latest_pose_msg is None:
            raise RuntimeError("No pose received yet")

        if isinstance(self.latest_pose_msg, Odometry):
            p = self.latest_pose_msg.pose.pose.position
            q = self.latest_pose_msg.pose.pose.orientation
            stamp_ns = self._stamp_to_ns(self.latest_pose_msg)

        elif isinstance(self.latest_pose_msg, PoseStamped):
            p = self.latest_pose_msg.pose.position
            q = self.latest_pose_msg.pose.orientation
            stamp_ns = self._stamp_to_ns(self.latest_pose_msg)

        elif isinstance(self.latest_pose_msg, Pose):
            p = self.latest_pose_msg.position
            q = self.latest_pose_msg.orientation
            stamp_ns = -1  # Pose has no header/timestamp

        else:
            raise RuntimeError(f"Unsupported pose message type: {type(self.latest_pose_msg)}")

        return p.x, p.y, p.z, q.x, q.y, q.z, q.w, stamp_ns

    def _convert_depth_to_meters(self, depth_msg: Image) -> np.ndarray:
        """
        Supports common ROS encodings:
        - 32FC1: assumed already in meters
        - 16UC1: scaled by depth_scale_to_meters
        """
        if depth_msg.encoding == "32FC1":
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth = np.array(depth, dtype=np.float32)
            depth[~np.isfinite(depth)] = 0.0
            depth[depth < 0.0] = 0.0
            return depth

        if depth_msg.encoding == "16UC1":
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth = np.array(depth, dtype=np.float32) * float(self.depth_scale_to_meters)
            return depth

        raise RuntimeError(f"Unsupported depth encoding: {depth_msg.encoding}")

    def save_current_sample(self) -> None:
        with self.lock:
            rgb_msg = self.latest_rgb_msg
            depth_msg = self.latest_depth_msg
            camera_info = self.latest_camera_info
            pose_msg = self.latest_pose_msg

        if rgb_msg is None:
            self.get_logger().warn("Cannot save: no RGB image yet")
            return
        if depth_msg is None:
            self.get_logger().warn("Cannot save: no depth image yet")
            return
        if camera_info is None:
            self.get_logger().warn("Cannot save: no CameraInfo yet")
            return
        if pose_msg is None:
            self.get_logger().warn("Cannot save: no pose yet")
            return

        if self.max_frames > 0 and self.frame_idx >= self.max_frames:
            self.get_logger().info("Reached max_frames")
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth_m = self._convert_depth_to_meters(depth_msg)
            tx, ty, tz, qx, qy, qz, qw, pose_stamp_ns = self._extract_pose()
        except Exception as e:
            self.get_logger().error(f"Failed to save sample: {e}")
            return

        frame_name = f"{self.frame_idx:06d}"
        rgb_path = self.rgb_dir / f"{frame_name}.png"
        depth_path = self.depth_dir / f"{frame_name}.npy"

        cv2.imwrite(str(rgb_path), bgr)
        np.save(depth_path, depth_m.astype(np.float32))

        rgb_stamp_ns = self._stamp_to_ns(rgb_msg)
        depth_stamp_ns = self._stamp_to_ns(depth_msg)

        self.csv_writer.writerow([
            self.frame_idx,
            rgb_path.name,
            depth_path.name,
            rgb_stamp_ns,
            depth_stamp_ns,
            pose_stamp_ns,
            tx, ty, tz,
            qx, qy, qz, qw,
        ])
        self.csv_file.flush()

        self.get_logger().info(f"Saved frame {self.frame_idx}: {rgb_path.name}, {depth_path.name}")
        self.frame_idx += 1

    def keyboard_loop(self) -> None:
        while rclpy.ok():
            try:
                line = sys.stdin.readline()
            except Exception:
                break

            if not line:
                continue

            line = line.strip().lower()
            if line == "q":
                self.get_logger().info("Quit requested from keyboard")
                rclpy.shutdown()
                break

            self.save_current_sample()

    def close(self) -> None:
        if not self.csv_file.closed:
            self.csv_file.close()


def main() -> None:
    rclpy.init()
    node = ManualGTRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()