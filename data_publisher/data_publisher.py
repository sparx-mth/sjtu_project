import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import math
import json
import os
import glob

class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')

        # Define Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/flow_depth/pose_est', 10)
        self.depth_pub = self.create_publisher(Image, '/xtend/depth_m', 10)

        self.bridge = CvBridge()

        # Paths - Docker maps the host's Desktop folder to /data
        self.depth_dir = '/data/xtend_rectified_depth_take_003_20260429_160647/depth_npy'
        self.json_path = '/data/estimated_trajectory_xtend_rectified_depth_take_003_20260429_160647.json'

        # Load localization file (JSON)
        with open(self.json_path, 'r') as f:
            self.trajectory_data = json.load(f)

        # Create a sorted list of all .npy depth files
        self.depth_files = sorted(glob.glob(os.path.join(self.depth_dir, '*.npy')))

        self.current_idx = 0
        self.max_idx = min(len(self.trajectory_data), len(self.depth_files))

        self.get_logger().info(f"Found {len(self.depth_files)} depth images and {len(self.trajectory_data)} poses. Publishing in an infinite loop...")

        # Timer to publish messages (e.g., 10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Reset index to 0 to loop indefinitely
        if self.current_idx >= self.max_idx:
            self.get_logger().info("Reached the end of data. Restarting loop...")
            self.current_idx = 0

        now = self.get_clock().now().to_msg()

        # --- Create and publish localization (PoseStamped) ---
        # JSON format: [{"image": "...", "pose": {"x": ..., "y": ..., "z": ..., "yaw": ...}}, ...]
        # Pose is nested under "pose"; orientation is a single yaw angle (rad).
        pose_data = self.trajectory_data[self.current_idx]['pose']
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = float(pose_data.get('x', 0.0))
        pose_msg.pose.position.y = float(pose_data.get('y', 0.0))
        pose_msg.pose.position.z = float(pose_data.get('z', 0.0))

        # Yaw (Z-axis rotation) → quaternion. qx=qy=0 since roll/pitch=0.
        yaw = float(pose_data.get('yaw', 0.0))
        half = 0.5 * yaw
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(half)
        pose_msg.pose.orientation.w = math.cos(half)

        self.pose_pub.publish(pose_msg)

        # --- Create and publish depth image (Image) ---
        depth_file = self.depth_files[self.current_idx]
        depth_array = np.load(depth_file)

        # Ensure the numpy array is float32 (standard for ROS depth in meters)
        if depth_array.dtype == np.float64:
            depth_array = depth_array.astype(np.float32)

        # Convert numpy array to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(depth_array, encoding="passthrough")
        image_msg.header.stamp = now
        image_msg.header.frame_id = "camera_depth_frame"

        self.depth_pub.publish(image_msg)

        self.current_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = DataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()