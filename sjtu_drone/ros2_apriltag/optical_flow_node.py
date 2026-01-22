#!/usr/bin/env python3
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo


class OpticalFlowNode(Node):
    def __init__(self):
        super().__init__("optical_flow_node")

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True
        )])

        self.declare_parameter("image_topic", "/simple_drone/front/image_raw")
        self.declare_parameter("output_topic", "/optical_flow_velocity")
        self.declare_parameter("max_corners", 200)
        self.declare_parameter("min_corners", 20)
        self.declare_parameter("show_debug", False)
        self.declare_parameter("camera_frame", "simple_drone/front_cam_link")
        self.declare_parameter("height_m", 1.0)
        self.declare_parameter("camera_info_topic", "/simple_drone/front/camera_info")

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.max_corners = self.get_parameter("max_corners").get_parameter_value().integer_value
        self.min_corners = self.get_parameter("min_corners").get_parameter_value().integer_value
        self.show_debug = self.get_parameter("show_debug").get_parameter_value().bool_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        self.get_logger().info(f"[OpticalFlow] Subscribing to: {image_topic}")
        self.get_logger().info(f"[OpticalFlow] Publishing to: {output_topic}")

        self.bridge = CvBridge()

        self.prev_gray = None
        self.prev_pts = None
        self.prev_stamp = None 

        self.height_m = self.get_parameter("height_m").get_parameter_value().double_value

        self.fx = None
        self.fy = None

        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value

        self.caminfo_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )

        # Publisher
        self.vel_pub = self.create_publisher(Vector3Stamped, output_topic, 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

    def detect_features(self, gray):
        pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=0.01,
            minDistance=7,
            blockSize=7
        )
        return pts

    @staticmethod
    def robust_velocity_from_flow(good_old, good_new, dt):
     
        if len(good_old) == 0 or dt <= 0.0:
            return 0.0, 0.0

        flow = good_new - good_old  # [N, 2]
        dx = flow[:, 0]
        dy = flow[:, 1]

        vx_px_per_frame = np.median(dx)
        vy_px_per_frame = np.median(dy)

        vx_px_per_sec = vx_px_per_frame / dt
        vy_px_per_sec = vy_px_per_frame / dt

        return float(vx_px_per_sec), float(vy_px_per_sec)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_pts = self.detect_features(self.prev_gray)
            self.prev_stamp = msg.header.stamp
            return

        curr_stamp = msg.header.stamp
        dt_ns = (curr_stamp.sec - self.prev_stamp.sec) * 1e9 + \
                (curr_stamp.nanosec - self.prev_stamp.nanosec)
        dt = dt_ns * 1e-9

        if dt <= 0.0:
            self.get_logger().warn(f"Non-positive dt={dt:.6f}, skipping")
            self.prev_gray = gray
            self.prev_stamp = curr_stamp
            return

        if self.prev_pts is None or len(self.prev_pts) < self.min_corners:
            self.prev_pts = self.detect_features(self.prev_gray)
            if self.prev_pts is None:
                self.prev_gray = gray
                self.prev_stamp = curr_stamp
                return

        # calcOpticalFlow
        next_pts, st, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None, **self.lk_params
        )

        if next_pts is None or st is None:
            self.get_logger().warn("LK returned no points")
            self.prev_gray = gray
            self.prev_stamp = curr_stamp
            self.prev_pts = None
            return

        good_new = next_pts[st == 1]
        good_old = self.prev_pts[st == 1]

        if len(good_new) == 0:
            self.get_logger().warn("No valid flow points")
            self.prev_gray = gray
            self.prev_stamp = curr_stamp
            self.prev_pts = None
            return

        vx, vy = self.robust_velocity_from_flow(good_old, good_new, dt)

        if self.fx is None or self.fy is None:
            self.get_logger().warn("No CameraInfo yet (fx/fy missing). Publishing px/s.")
            self.prev_gray = gray
            self.prev_pts = good_new.reshape(-1, 1, 2)
            self.prev_stamp = curr_stamp
            return        
        else:
            Z = self.height_m
            vx_mps = Z * (vx / self.fx)
            vy_mps = Z * (vy / self.fy)

        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = msg.header.stamp
        vel_msg.header.frame_id = self.camera_frame
        vel_msg.vector.x = vx_mps
        vel_msg.vector.y = vy_mps
        vel_msg.vector.z = 0.0
        self.vel_pub.publish(vel_msg)

        if self.show_debug:
            vis = frame.copy()
            for (x_new, y_new), (x_old, y_old) in zip(good_new, good_old):
                x_new, y_new = int(x_new), int(y_new)
                x_old, y_old = int(x_old), int(y_old)
                cv2.arrowedLine(vis, (x_old, y_old), (x_new, y_new),
                                (0, 255, 0), 1, tipLength=0.3)
            text = f"vx={vx_mps:.3f} m/s vy={vy_mps:.3f} m/s N={len(good_new)}"

            cv2.putText(vis, text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("Optical Flow", vis)
            cv2.waitKey(1)

        self.prev_gray = gray
        self.prev_pts = good_new.reshape(-1, 1, 2)
        self.prev_stamp = curr_stamp

    def camera_info_callback(self, msg: CameraInfo):
        K = msg.k
        self.fx = float(K[0])
        self.fy = float(K[4])

def main():
    rclpy.init()
    node = OpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
