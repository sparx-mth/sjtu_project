#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
from vision_msgs.msg import Detection2DArray 


def quaternion_to_rotation_matrix(x, y, z, w) -> np.ndarray:
    """
    Convert quaternion (x, y, z, w) to 3x3 rotation matrix.
    """
    # normalized just in case
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3)
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    # standard formula
    R = np.zeros((3, 3), dtype=float)
    R[0, 0] = 1 - 2 * (y * y + z * z)
    R[0, 1] = 2 * (x * y - z * w)
    R[0, 2] = 2 * (x * z + y * w)

    R[1, 0] = 2 * (x * y + z * w)
    R[1, 1] = 1 - 2 * (x * x + z * z)
    R[1, 2] = 2 * (y * z - x * w)

    R[2, 0] = 2 * (x * z - y * w)
    R[2, 1] = 2 * (y * z + x * w)
    R[2, 2] = 1 - 2 * (x * x + y * y)
    return R


class ObjectSizeEstimator(Node):
    """
    Node that:
    - Listens to AprilTag detections (pose of tag)
    - Listens to camera info (intrinsics)
    - Listens to 2D detections (bounding boxes)
    - For each object, estimates:
        * distance_world (m)
        * size_world (width, height in m)
    under the assumption the object lies on the same plane as the tag.
    """

    def __init__(self):
        super().__init__("object_size_estimator")

        # Parameters
        # ros2 run ... --ros-args -p tag_id:=15
        self.declare_parameter("tag_id", -1)  # -1 = use first detection
        self.tag_id = self.get_parameter("tag_id").get_parameter_value().integer_value

        # camera intrinsics
        self.camera_info_received = False
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # plane of the tag in camera frame: n · (X - p0) = 0
        self.plane_normal: Optional[np.ndarray] = None  # shape (3,)
        self.plane_point: Optional[np.ndarray] = None   # shape (3,)

        # Subscriptions
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera_info",
            self.camera_info_callback,
            10,
        )

        self.apriltag_sub = self.create_subscription(
            AprilTagDetectionArray,
            "/apriltag/detections",
            self.apriltag_callback,
            10,
        )

        self.objects_sub = self.create_subscription(
            Detection2DArray,
            "/objects",
            self.objects_callback,
            10,
        )

        self.get_logger().info("ObjectSizeEstimator node started")

    # ---------------- Camera Info ---------------- #

    def camera_info_callback(self, msg: CameraInfo):
        # msg.k is [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_received = True

    # ---------------- AprilTag detections ---------------- #

    def apriltag_callback(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            return

        detection = None
        if self.tag_id < 0:
            detection = msg.detections[0]
        else:
            for det in msg.detections:
                if det.id == self.tag_id:
                    detection = det
                    break

        if detection is None:
            return

        # pose is PoseWithCovarianceStamped
        pose = detection.pose.pose
        t = pose.position
        q = pose.orientation

        # translation vector (camera frame)
        p0 = np.array([t.x, t.y, t.z], dtype=float)

        # rotation matrix from tag frame to camera frame
        R = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)

        # assume tag's +Z axis is the normal (perpendicular) to the tag plane
        tag_normal_in_tag_frame = np.array([0.0, 0.0, 1.0])
        n = R @ tag_normal_in_tag_frame

        self.plane_point = p0
        self.plane_normal = n

        distance_tag = np.linalg.norm(p0)
        self.get_logger().debug(
            f"Updated tag plane. Tag distance ~ {distance_tag:.3f} m"
        )

    # ---------------- Helper functions ---------------- #

    def pixel_to_ray(self, u: float, v: float) -> np.ndarray:
        """
        Convert pixel (u,v) to a normalized ray direction in camera frame.
        Using pinhole model: X = (u-cx)/fx * Z, Y = (v-cy)/fy * Z, Z>0
        """
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        z = 1.0
        vec = np.array([x, y, z], dtype=float)
        norm = np.linalg.norm(vec)
        if norm == 0.0:
            return np.array([0.0, 0.0, 1.0])
        return vec / norm

    def intersect_ray_with_plane(self, ray: np.ndarray) -> Optional[np.ndarray]:
        """
        Intersect ray originating at camera origin (0,0,0) with the tag plane.
        Plane: n · (X - p0) = 0 -> n·X = n·p0
        Ray: X = s * ray, s>0
        => s = (n·p0) / (n·ray)
        """
        if self.plane_normal is None or self.plane_point is None:
            return None

        n = self.plane_normal
        p0 = self.plane_point

        denom = np.dot(n, ray)
        if abs(denom) < 1e-6:
            # ray is almost parallel to plane
            return None

        s = np.dot(n, p0) / denom
        if s <= 0:
            # intersection is behind the camera or exactly at origin
            return None

        return s * ray

    def estimate_object_size_and_distance(
        self,
        u_center: float,
        v_center: float,
        width_px: float,
        height_px: float,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Estimate:
            distance_world (m),
            width_world (m),
            height_world (m)
        for an object whose 2D bounding box is given by center (u,v) and size (w,h) in pixels,
        assuming the object lies on the same plane as the tag.
        """

        ray_center = self.pixel_to_ray(u_center, v_center)
        p_center = self.intersect_ray_with_plane(ray_center)
        if p_center is None:
            return None

        distance_world = float(np.linalg.norm(p_center))

        u_left = u_center - width_px / 2.0
        u_right = u_center + width_px / 2.0
        v_mid = v_center

        ray_left = self.pixel_to_ray(u_left, v_mid)
        ray_right = self.pixel_to_ray(u_right, v_mid)

        p_left = self.intersect_ray_with_plane(ray_left)
        p_right = self.intersect_ray_with_plane(ray_right)

        if p_left is None or p_right is None:
            return None

        width_vec = p_right - p_left
        width_world = float(np.linalg.norm(width_vec))

        v_top = v_center - height_px / 2.0
        v_bottom = v_center + height_px / 2.0
        u_mid = u_center

        ray_top = self.pixel_to_ray(u_mid, v_top)
        ray_bottom = self.pixel_to_ray(u_mid, v_bottom)

        p_top = self.intersect_ray_with_plane(ray_top)
        p_bottom = self.intersect_ray_with_plane(ray_bottom)

        if p_top is None or p_bottom is None:
            return None

        height_vec = p_bottom - p_top
        height_world = float(np.linalg.norm(height_vec))

        return distance_world, width_world, height_world

    # ---------------- Objects (2D detections) ---------------- #

    def objects_callback(self, msg: Detection2DArray):
        if not self.camera_info_received:
            self.get_logger().warn_throttle(
                5000, "No CameraInfo yet – cannot estimate sizes"
            )
            return

        if self.plane_normal is None:
            self.get_logger().warn_throttle(
                5000, "No AprilTag plane yet – wait for apriltag detections"
            )
            return

        if not msg.detections:
            return

        for i, det in enumerate(msg.detections):
            bbox = det.bbox
            u_center = bbox.center.x
            v_center = bbox.center.y
            width_px = bbox.size_x
            height_px = bbox.size_y

            result = self.estimate_object_size_and_distance(
                u_center, v_center, width_px, height_px
            )

            if result is None:
                self.get_logger().warn(
                    f"Could not estimate size/distance for object #{i}"
                )
                continue

            distance_world, width_world, height_world = result

            self.get_logger().info(
                f"Object #{i}: "
                f"distance = {distance_world:.3f} m, "
                f"width = {width_world:.3f} m, "
                f"height = {height_world:.3f} m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSizeEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
