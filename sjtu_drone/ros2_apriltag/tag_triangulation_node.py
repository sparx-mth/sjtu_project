#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped


def euler_matrix(roll, pitch, yaw):
    """Create 4x4 rotation matrix from roll, pitch, yaw."""
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)

    # Rotation matrices around x, y, z
    Rx = np.array(
        [
            [1, 0, 0],
            [0, cr, -sr],
            [0, sr, cr],
        ]
    )

    Ry = np.array(
        [
            [cp, 0, sp],
            [0, 1, 0],
            [-sp, 0, cp],
        ]
    )

    Rz = np.array(
        [
            [cy, -sy, 0],
            [sy, cy, 0],
            [0, 0, 1],
        ]
    )

    # Standard convention: R = Rz * Ry * Rx
    R = Rz @ Ry @ Rx

    M = np.eye(4)
    M[:3, :3] = R
    return M


def quaternion_matrix(q):
    """Create 4x4 matrix from quaternion [x, y, z, w]."""
    x, y, z, w = q
    # Normalize quaternion
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

    R = np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ]
    )

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
        # Find the largest diagonal element
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

        # === Configuration ===
        # World frame (Gazebo's global frame)
        self.world_frame = "world"

        # Camera frame (the one apriltag_ros uses as parent of the tag frames)
        self.camera_frame = "simple_drone/front_cam_link"

        # AprilTag frames as published by apriltag_ros
        self.tag_frames = ["tag36h11:14", "tag36h11:15"]

        # Static world poses of the tags (from your Gazebo models)
        # Each entry: xyz in meters, rpy in radians
        self.tag_world_poses = {
            "tag36h11:14": {
                "xyz": (11.195, 0.0, 1.5),
                "rpy": (0.0, -1.5708, 0.0),
            },
            "tag36h11:15": {
                "xyz": (11.495, 1.0, 1.5),
                "rpy": (0.0, -1.5708, 0.0),
            },
        }

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically compute triangulated pose
        self.timer = self.create_timer(0.1, self.compute_triangulated_pose)

        self.get_logger().info("TagTriangulationNode started.")

    # ---------- Helper functions ----------

    @staticmethod
    def transform_to_matrix(t: TransformStamped) -> np.ndarray:
        """Convert a TransformStamped into a 4x4 homogeneous matrix."""
        q = t.transform.rotation
        tr = t.transform.translation

        M = quaternion_matrix([q.x, q.y, q.z, q.w])  # 4x4
        M[0, 3] = tr.x
        M[1, 3] = tr.y
        M[2, 3] = tr.z
        return M

    @staticmethod
    def pose_to_matrix(xyz, rpy) -> np.ndarray:
        """Build a 4x4 matrix from xyz and rpy (roll, pitch, yaw)."""
        x, y, z = xyz
        roll, pitch, yaw = rpy

        M = euler_matrix(roll, pitch, yaw)  # 4x4
        M[0, 3] = x
        M[1, 3] = y
        M[2, 3] = z
        return M

    @staticmethod
    def matrix_to_pose(M: np.ndarray):
        """Extract translation and quaternion from a 4x4 matrix."""
        x = M[0, 3]
        y = M[1, 3]
        z = M[2, 3]
        qx, qy, qz, qw = quaternion_from_matrix(M)
        return (x, y, z), (qx, qy, qz, qw)

    # ---------- Main logic ----------

    def compute_triangulated_pose(self):
        """
        For each tag:
          - Build world_T_tag from static pose (known from Gazebo)
          - Read cam_T_tag from TF (from apriltag_ros)
          - Compute world_T_cam_i = world_T_tag * inv(cam_T_tag)
        Then combine all available world_T_cam_i and print a triangulated pose.
        """
        cam_poses_world = []

        for tag_frame in self.tag_frames:
            # 1) Static world pose of this tag (from your Gazebo model)
            if tag_frame not in self.tag_world_poses:
                self.get_logger().warn(f"No static world pose defined for {tag_frame}")
                continue

            xyz = self.tag_world_poses[tag_frame]["xyz"]
            rpy = self.tag_world_poses[tag_frame]["rpy"]
            world_T_tag = self.pose_to_matrix(xyz, rpy)

            # 2) Measured cam -> tag from apriltag_ros via TF
            try:
                # lookup_transform(target_frame, source_frame, time)
                # Here we want T_cam_tag: parent = camera_frame, child = tag_frame.
                cam_T_tag_tf = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    tag_frame,
                    rclpy.time.Time(),  # latest available
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Could not get transform {self.camera_frame} -> {tag_frame}: {e}"
                )
                continue

            cam_T_tag = self.transform_to_matrix(cam_T_tag_tf)

            # 3) Invert cam_T_tag to get tag_T_cam
            tag_T_cam = np.linalg.inv(cam_T_tag)

            # 4) Compute world_T_cam_i = world_T_tag * tag_T_cam
            world_T_cam_i = np.dot(world_T_tag, tag_T_cam)
            cam_poses_world.append(world_T_cam_i)

        if len(cam_poses_world) == 0:
            # No tags visible / no transforms available
            return

        # If we only have one tag, use that directly.
        if len(cam_poses_world) == 1:
            world_T_cam_avg = cam_poses_world[0]
        else:
            # Simple combination: average translation, take rotation from the first.
            translations = np.array([M[:3, 3] for M in cam_poses_world])
            avg_t = np.mean(translations, axis=0)

            # For simplicity: take rotation of the first solution.
            R = cam_poses_world[0][:3, :3]

            world_T_cam_avg = np.eye(4)
            world_T_cam_avg[:3, :3] = R
            world_T_cam_avg[:3, 3] = avg_t

        # Convert back to position + quaternion for logging
        (x, y, z), (qx, qy, qz, qw) = self.matrix_to_pose(world_T_cam_avg)

        self.get_logger().info(
            f"[TRIANGULATED CAMERA POSE]\n"
            f"  Frame: {self.world_frame} -> {self.camera_frame}\n"
            f"  Position: x={x:.3f}, y={y:.3f}, z={z:.3f}\n"
            f"  Orientation (quat xyzw): "
            f"({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})\n"
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
