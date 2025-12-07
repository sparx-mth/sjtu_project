#!/usr/bin/env python3
import math
import json
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo

import tf2_ros
from tf2_ros import TransformException


def quaternion_to_rotation_matrix(x, y, z, w) -> np.ndarray:
    """
    Convert quaternion (x, y, z, w) to a 3x3 rotation matrix.
    """
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3)
    x /= norm
    y /= norm
    z /= norm
    w /= norm

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


class ObjectSizeFromJson(Node):
    """
    Node that:
    - Listens to CameraInfo (intrinsics)
    - Uses TF to get the AprilTag pose (camera_frame -> tag_frame)
    - Periodically reads a JSON file with bounding boxes
    - For each bbox, estimates:
        * distance_world (m)
        * width_world (m)
        * height_world (m)
      assuming the object lies on the same plane as the tag.
    """

    def __init__(self):
        super().__init__("object_size_from_json")

        # Parameters
        # tag_id is used to build the tag frame name (e.g., "tag36h11:15")
        self.declare_parameter("tag_id", 15)
        self.declare_parameter("bbox_json_path", "/ros2_ws/bboxes.json")
        self.declare_parameter("tag_family", "36h11")

        self.tag_id = self.get_parameter("tag_id").get_parameter_value().integer_value
        self.bbox_json_path = (
            self.get_parameter("bbox_json_path").get_parameter_value().string_value
        )
        self.tag_family = (
            self.get_parameter("tag_family").get_parameter_value().string_value
        )

        # Camera intrinsics
        self.camera_info_received = False
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Camera frame (will be taken from CameraInfo.header.frame_id)
        self.camera_frame: Optional[str] = None

        # Tag plane in camera frame: n · (X - p0) = 0
        self.plane_normal: Optional[np.ndarray] = None  # shape (3,)
        self.plane_point: Optional[np.ndarray] = None   # shape (3,)

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/simple_drone/front/camera_info",
            self.camera_info_callback,
            10,
        )

        # Timer – every 0.5 sec: update plane from TF, read JSON, compute sizes
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(
            f"ObjectSizeFromJson node started.\n"
            f"  bbox_json_path: {self.bbox_json_path}\n"
            f"  tag_family:     {self.tag_family}\n"
            f"  tag_id:         {self.tag_id}"
        )

        self.camera_frame_logged = False

    # ---------------- Camera Info ---------------- #

    def camera_info_callback(self, msg: CameraInfo):
        # intrinsics
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_received = True

        # frame id (strip leading '/')
        frame = msg.header.frame_id
        if frame.startswith("/"):
            frame = frame[1:]
        self.camera_frame = frame

        # log only once
        if not self.camera_frame_logged:
            self.get_logger().info(
                f"CameraInfo received. Using camera_frame = '{self.camera_frame}'"
            )
            self.camera_frame_logged = True


    # ---------------- TF: get tag plane ---------------- #

    def update_tag_plane_from_tf(self) -> bool:
        """
        Use TF to get the transform from camera_frame to tag_frame,
        and from that build the plane (point p0 and normal n) in camera frame.
        We try a few common naming patterns for the tag frame.
        """
        if self.camera_frame is None:
            self.get_logger().warn(
                "Camera frame is not known yet (no CameraInfo). Cannot look up TF."
            )
            return False

        # Possible frame name patterns. Adjust if needed.
        candidate_frames = [
            f"tag{self.tag_family}:{self.tag_id}",  # e.g., "tag36h11:15"
            f"tag_{self.tag_id}",                  # e.g., "tag_15"
            f"tag{self.tag_id}",                   # e.g., "tag15"
        ]

        transform = None
        last_error = None

        for tag_frame in candidate_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.camera_frame,  # target
                    tag_frame,          # source
                    Time()
                )
                self.get_logger().debug(f"Found TF for {tag_frame}")
                break
            except TransformException as ex:
                last_error = ex

        if transform is None:
            self.get_logger().warn(
                "Could not find TF from %s to any tag frame (tried: %s). Last error: %s"
                % (self.camera_frame, candidate_frames, last_error)
            )
            return False

        t = transform.transform.translation
        q = transform.transform.rotation

        # translation = point on the tag plane in camera frame
        p0 = np.array([t.x, t.y, t.z], dtype=float)

        # rotation: tag frame -> camera frame
        R = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)

        # assume tag's +Z axis is the normal (perpendicular to the tag plane)
        tag_normal_in_tag_frame = np.array([0.0, 0.0, 1.0])
        n = R @ tag_normal_in_tag_frame

        self.plane_point = p0
        self.plane_normal = n

        return True

    # ---------------- Geometry helpers ---------------- #

    def pixel_to_ray(self, u: float, v: float) -> np.ndarray:
        """
        Convert pixel (u,v) to a normalized ray direction in camera frame
        using the pinhole camera model.
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
        Intersect a ray originating at the camera origin with the tag plane.
        Ray:   X = s * ray, s > 0
        Plane: n · (X - p0) = 0  ->  n · X = n · p0
        =>     s = (n · p0) / (n · ray)
        """
        if self.plane_normal is None or self.plane_point is None:
            return None

        n = self.plane_normal
        p0 = self.plane_point

        denom = np.dot(n, ray)
        if abs(denom) < 1e-6:
            # Ray is almost parallel to the plane
            return None

        s = np.dot(n, p0) / denom
        if s <= 0:
            # Intersection behind the camera or at the origin
            return None

        return s * ray

    def estimate_object_from_bbox(
        self,
        u_min: float,
        v_min: float,
        u_max: float,
        v_max: float,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Estimate (distance_world, width_world, height_world) in meters
        from bbox corners in pixels:
            (u_min, v_min) top-left
            (u_max, v_max) bottom-right
        assuming the object lies on the same plane as the tag.
        """
        u_center = 0.5 * (u_min + u_max)
        v_center = 0.5 * (v_min + v_max)

        # 1) distance: use the center ray
        ray_center = self.pixel_to_ray(u_center, v_center)
        p_center = self.intersect_ray_with_plane(ray_center)
        if p_center is None:
            return None

        distance_world = float(np.linalg.norm(p_center))

        # 2) width: left/right edges
        ray_left = self.pixel_to_ray(u_min, v_center)
        ray_right = self.pixel_to_ray(u_max, v_center)
        p_left = self.intersect_ray_with_plane(ray_left)
        p_right = self.intersect_ray_with_plane(ray_right)
        if p_left is None or p_right is None:
            return None
        width_world = float(np.linalg.norm(p_right - p_left))

        # 3) height: top/bottom edges
        ray_top = self.pixel_to_ray(u_center, v_min)
        ray_bottom = self.pixel_to_ray(u_center, v_max)
        p_top = self.intersect_ray_with_plane(ray_top)
        p_bottom = self.intersect_ray_with_plane(ray_bottom)
        if p_top is None or p_bottom is None:
            return None
        height_world = float(np.linalg.norm(p_bottom - p_top))

        return distance_world, width_world, height_world

    # ---------------- Timer: read JSON + compute ---------------- #

    def timer_callback(self):
        if not self.camera_info_received:
            self.get_logger().warn("No CameraInfo yet – cannot estimate sizes")
            return

        # Update tag plane (plane_point + plane_normal) from TF
        if not self.update_tag_plane_from_tf():
            # Warning already logged inside update_tag_plane_from_tf
            return

        try:
            with open(self.bbox_json_path, "r") as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().warn(
                f"Could not read JSON file '{self.bbox_json_path}': {e}"
            )
            return

        objects = data.get("objects", [])
        if not objects:
            return

        for obj in objects:
            # Expected JSON keys (change here if your format is different)
            u_min = float(obj["u_min"])
            v_min = float(obj["v_min"])
            u_max = float(obj["u_max"])
            v_max = float(obj["v_max"])
            obj_id = obj.get("id", None)

            result = self.estimate_object_from_bbox(u_min, v_min, u_max, v_max)
            if result is None:
                self.get_logger().warn(
                    f"Could not estimate object (id={obj_id}) from bbox "
                    f"({u_min},{v_min})-({u_max},{v_max})"
                )
                continue

            distance_world, width_world, height_world = result
            self.get_logger().info(
                f"Object id={obj_id}: "
                f"distance = {distance_world:.3f} m, "
                f"width = {width_world:.3f} m, "
                f"height = {height_world:.3f} m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSizeFromJson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
