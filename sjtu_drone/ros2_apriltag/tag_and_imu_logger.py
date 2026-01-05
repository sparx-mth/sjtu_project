#!/usr/bin/env python3
import csv
import os
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu  


class TagPoseLogger(Node):
    def __init__(self):
        super().__init__("tag_imu_pose_logger")

        # Params (can be overridden by ros2 run ... --ros-args -p ...) 
        self.declare_parameter("output_path", "/ros2_ws/tag_imu_logger.csv")
        self.output_path = self.get_parameter("output_path").value

        # Keep latest IDs message
        self.latest_ids = []
        self.latest_ids_stamp = None

        # Keep latest IMU message
        self.latest_imu = None
        self.latest_imu_stamp = None

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, "/tag_pose", self.pose_cb, 10
        )
        self.ids_sub = self.create_subscription(
            Int32MultiArray, "/tag_pose_ids", self.ids_cb, 10
        )
        self.imu_sub = self.create_subscription(      # 👈 חדש
            Imu, "/simple_drone/imu/out", self.imu_cb, 50
        )

        # Prepare file + header if needed
        file_exists = os.path.isfile(self.output_path)
        self.csv_file = open(self.output_path, "a", newline="")
        self.writer = csv.writer(self.csv_file)

        if not file_exists:
            self.writer.writerow([
                "stamp_sec",
                "stamp_nanosec",
                "x", "y", "z",
                "qx", "qy", "qz", "qw",
                "tag_ids",
                # --- IMU fields ---
                "imu_stamp_sec",
                "imu_stamp_nanosec",
                "imu_qx", "imu_qy", "imu_qz", "imu_qw",
                "ang_vel_x", "ang_vel_y", "ang_vel_z",
                "lin_acc_x", "lin_acc_y", "lin_acc_z",
            ])
            self.csv_file.flush()

        self.get_logger().info(f"Logging /tag_pose + /imu to: {self.output_path}")

    def ids_cb(self, msg: Int32MultiArray):
        self.latest_ids = list(msg.data)
        self.latest_ids_stamp = self.get_clock().now()

    def imu_cb(self, msg: Imu):
        self.latest_imu = msg
        self.latest_imu_stamp = msg.header.stamp

    def pose_cb(self, msg: PoseStamped):
        stamp_sec = msg.header.stamp.sec
        stamp_ns = msg.header.stamp.nanosec

        p = msg.pose.position
        q = msg.pose.orientation

        # Use latest ids we got (good enough since both topics published same cycle)
        tag_ids_str = ",".join(str(i) for i in self.latest_ids) if self.latest_ids else ""

        # Prepare IMU fields
        if self.latest_imu is not None:
            imu = self.latest_imu
            imu_stamp_sec = imu.header.stamp.sec
            imu_stamp_ns = imu.header.stamp.nanosec

            imu_q = imu.orientation
            ang = imu.angular_velocity
            acc = imu.linear_acceleration

            imu_row = [
                imu_stamp_sec,
                imu_stamp_ns,
                imu_q.x, imu_q.y, imu_q.z, imu_q.w,
                ang.x, ang.y, ang.z,
                acc.x, acc.y, acc.z,
            ]
        else:
            imu_row = ["", "", "", "", "", "", "", "", "", "", "", ""]

        row = [
            stamp_sec,
            stamp_ns,
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
            tag_ids_str,
        ] + imu_row

        self.writer.writerow(row)
        self.csv_file.flush()

        self.get_logger().info(
            f"[LOGGED] t={stamp_sec}.{stamp_ns:09d} "
            f"pose=({p.x:.2f},{p.y:.2f},{p.z:.2f}) "
            f"ids={self.latest_ids}"        
        )

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TagPoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
