#!/usr/bin/env python3
import csv
import os
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray


class TagPoseLogger(Node):
    def __init__(self):
        super().__init__("tag_pose_logger")
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True
        )])
        
        # Params (can be overridden by ros2 run ... --ros-args -p ...)
        self.declare_parameter("output_path", "/ros2_ws/tag_pose_log.csv")
        self.output_path = self.get_parameter("output_path").value

        # Keep latest IDs message
        self.latest_ids = []
        self.latest_ids_stamp = None
        
        # [M_sec, M_nanosec, P_sec, P_nanosec]
        self.latest_times_data = [] 
        # -------------------------------------------------------------

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, "/tag_pose", self.pose_cb, 10
        )
        self.ids_sub = self.create_subscription(
            Int32MultiArray, "/tag_pose_ids", self.ids_cb, 10
        )
        
        self.time_sub = self.create_subscription(
            Int32MultiArray, "/pose_publish_time", self.time_cb, 10
        )
        # -------------------------------------------------------------

        # Prepare file + header if needed
        file_exists = os.path.isfile(self.output_path)
        self.csv_file = open(self.output_path, "a", newline="")
        self.writer = csv.writer(self.csv_file)

        if not file_exists:
            self.writer.writerow([
                "measurement_sec",         # (M)  
                "measurement_nanosec",
                "publish_sec",             # (P) 
                "publish_nanosec",
                "x", "y", "z",
                "qx", "qy", "qz", "qw",
                "tag_ids"
            ])
            self.csv_file.flush()
            # -----------------------------------------------------------------

        self.get_logger().info(f"Logging /tag_pose to: {self.output_path}")

    def ids_cb(self, msg: Int32MultiArray):
        self.latest_ids = list(msg.data)
        self.latest_ids_stamp = self.get_clock().now()
        
    def time_cb(self, msg: Int32MultiArray):
        # [M_sec, M_nanosec, P_sec, P_nanosec]
        if len(msg.data) == 4:
            self.latest_times_data = list(msg.data)
        else:
            self.get_logger().warn("Received /pose_publish_time with incorrect data length.")
    # -------------------------------------------------------------

    def pose_cb(self, msg: PoseStamped):
        
        if len(self.latest_times_data) != 4:
            self.get_logger().warn(
                "Skipping logging: Missing or incomplete /pose_publish_time data."
            )
            measurement_sec = msg.header.stamp.sec
            measurement_ns = msg.header.stamp.nanosec
            publish_sec = 0
            publish_ns = 0
        else:
            measurement_sec = self.latest_times_data[0]
            measurement_ns = self.latest_times_data[1]
            publish_sec = self.latest_times_data[2]
            publish_ns = self.latest_times_data[3]
            
        p = msg.pose.position
        q = msg.pose.orientation

        # Use latest ids we got (good enough since both topics published same cycle)
        tag_ids_str = ",".join(str(i) for i in self.latest_ids) if self.latest_ids else ""

        self.writer.writerow([
            measurement_sec,
            measurement_ns,
            publish_sec,       
            publish_ns,         
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
            tag_ids_str
        ])
        self.csv_file.flush()

        self.get_logger().info(
            f"[LOGGED] M_t={measurement_sec}.{measurement_ns:09d} P_t={publish_sec}.{publish_ns:09d} pose=({p.x:.2f},{p.y:.2f},{p.z:.2f}) ids={self.latest_ids}"
        )
        # -------------------------------------------------------------

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