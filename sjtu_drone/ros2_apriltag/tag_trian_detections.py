#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped

# apriltag_ros (christianrauch) publishes:
# topic: /detections
# type : apriltag_msgs/msg/AprilTagDetectionArray
try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None


def euler_matrix(roll, pitch, yaw):
    """Create 4x4 rotation matrix from roll, pitch, yaw."""
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]])

    Ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]])

    Rz = np.array([[cy, -sy, 0],
                   [sy, cy, 0],
                   [0, 0, 1]])

    R = Rz @ Ry @ Rx

    M = np.eye(4)
    M[:3, :3] = R
    return M


def quaternion_matrix(q):
    """Create 4x4 matrix from quaternion [x, y, z, w]."""
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        return np.eye(4)
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    R = np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
    ])

    M = np.eye(4)
    M[:3, :3] = R
    return M


def quaternion_from_matrix(M):
    """Extract quaternion [x, y, z, w] from 4x4 rotation matrix."""
    R = M[:3, :3]
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

    return [x, y, z, w]


class TagTriangulationNode(Node):
    def __init__(self):
        super().__init__("tag_triangulation_node")

        if AprilTagDetectionArray is None:
            self.get_logger().error(
                "apriltag_msgs not found. Install apriltag_msgs or update message import."
            )
            raise RuntimeError("Missing apriltag_msgs")

        # === Configuration ===
        self.world_frame = "world"

        # MUST match what you set in apriltag_ros:
        # -p camera_frame:=simple_drone/front_cam_optical
        self.camera_frame = "simple_drone/front_cam_link"

        # Known tag frames
        self.tag_frames = ["tag36h11:14", "tag36h11:15"]

        # Static tag poses in world (Gazebo truth)
        self.tag_world_poses = {
            "tag36h11:14": {"xyz": (11.195, 0.0, 1.5), "rpy": (0.0, -1.5708, 0.0)},
            "tag36h11:15": {"xyz": (11.495, 1.0, 1.5), "rpy": (0.0, -1.5708, 0.0)},
        }

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher: pose based on tags
        self.pose_pub = self.create_publisher(PoseStamped, "/tag_pose", 10)

        # Subscriber: AprilTag detections (EVENT-DRIVEN)
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            "/detections",     # <-- YOUR topic
            self.detections_cb,
            10,
        )

        self.get_logger().info("TagTriangulationNode started (event-driven, /detections).")

    # ---------- Helpers ----------

    @staticmethod
    def transform_to_matrix(t: TransformStamped) -> np.ndarray:
        q = t.transform.rotation
        tr = t.transform.translation
        M = quaternion_matrix([q.x, q.y, q.z, q.w])
        M[0, 3] = tr.x
        M[1, 3] = tr.y
        M[2, 3] = tr.z
        return M

    @staticmethod
    def pose_to_matrix(xyz, rpy) -> np.ndarray:
        x, y, z = xyz
        roll, pitch, yaw = rpy
        M = euler_matrix(roll, pitch, yaw)
        M[0, 3] = x
        M[1, 3] = y
        M[2, 3] = z
        return M

    @staticmethod
    def matrix_to_pose(M: np.ndarray):
        x = M[0, 3]
        y = M[1, 3]
        z = M[2, 3]
        qx, qy, qz, qw = quaternion_from_matrix(M)
        return (x, y, z), (qx, qy, qz, qw)

    # ---------- Event-driven callback ----------



    def detections_cb(self, msg: AprilTagDetectionArray):
        """
        Called ONLY when apriltag_ros publishes new detections on /detections.
        We compute pose immediately and publish /tag_pose.
        header.stamp in /tag_pose is the REAL measurement time.
        """
        if not msg.detections:
            return

        detection_stamp = msg.header.stamp  # time of detection

        cam_poses_world = []
        measurement_stamps = []

        for det in msg.detections:
            tag_id = det.id if isinstance(det.id, int) else det.id
            tag_frame = f"tag36h11:{tag_id}"

            if tag_frame not in self.tag_world_poses:
                continue

            xyz = self.tag_world_poses[tag_frame]["xyz"]
            rpy = self.tag_world_poses[tag_frame]["rpy"]
            world_T_tag = self.pose_to_matrix(xyz, rpy)

            try:
                cam_T_tag_tf = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    tag_frame,
                    rclpy.time.Time.from_msg(detection_stamp),
                )
            except Exception as e:
                self.get_logger().warn(
                    f"TF lookup failed for {self.camera_frame}->{tag_frame}: {e}"
                )
                continue

            measurement_stamps.append(cam_T_tag_tf.header.stamp)

            cam_T_tag = self.transform_to_matrix(cam_T_tag_tf)
            tag_T_cam = np.linalg.inv(cam_T_tag)
            world_T_cam_i = world_T_tag @ tag_T_cam
            cam_poses_world.append(world_T_cam_i)

        if not cam_poses_world:
            return

        # Combine poses (avg translation, rotation from first)
        if len(cam_poses_world) == 1:
            world_T_cam_avg = cam_poses_world[0]
        else:
            translations = np.array([M[:3, 3] for M in cam_poses_world])
            avg_t = np.mean(translations, axis=0)
            R = cam_poses_world[0][:3, :3]

            world_T_cam_avg = np.eye(4)
            world_T_cam_avg[:3, :3] = R
            world_T_cam_avg[:3, 3] = avg_t

        (x, y, z), (qx, qy, qz, qw) = self.matrix_to_pose(world_T_cam_avg)

        measurement_stamp = measurement_stamps[-1] if measurement_stamps else detection_stamp

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.world_frame
        pose_msg.header.stamp = measurement_stamp  # real measurement time

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        self.get_logger().info(
            f"[TAG POSE PUBLISHED] x={x:.3f}, y={y:.3f}, z={z:.3f} "
            f"quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f}) "
            f"meas_time={measurement_stamp.sec}.{measurement_stamp.nanosec:09d}"
        )


def main():
    rclpy.init()
    node = TagTriangulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
