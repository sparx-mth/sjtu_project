#!/usr/bin/env python3
"""
navigate_to_point.py
--------------------
Simple manual navigation controller:
 - Reads current pose from /simple_drone/gt_pose
 - Asks user for target (x, y) in world coordinates (meters)
 - Flies there with constant altitude and prints position feedback
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import math, time


class ManualNavigator(Node):
    def __init__(self):
        super().__init__('manual_navigator')

        # ROS setup
        self.pose = Pose()
        self.pose_sub = self.create_subscription(Pose, '/simple_drone/gt_pose', self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)

        # parameters
        self.z_fixed = 1.5  # constant altitude (m)
        self.kp = 0.8
        self.tol = 0.05  # stop within 5cm

        self.get_logger().info("Manual navigator initialized. Waiting for pose data...")

        # Wait until pose arrives
        while rclpy.ok() and self.pose.position.x == 0.0 and self.pose.position.y == 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.loop()

    def pose_cb(self, msg: Pose):
        self.pose = msg

    def yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def loop(self):
        """Main user loop"""
        while rclpy.ok():
            # Print current position
            x, y, z = self.pose.position.x, self.pose.position.y, self.pose.position.z
            self.get_logger().info(f"Current position: ({x:.3f}, {y:.3f}, {z:.3f})")

            try:
                target_x = float(input("Enter target X (meters): "))
                target_y = float(input("Enter target Y (meters): "))
            except ValueError:
                print("Invalid input. Please enter numeric values.")
                continue

            self.move_to(target_x, target_y, self.z_fixed)

    def move_to(self, tx, ty, tz):
        self.get_logger().info(f"Navigating to target ({tx:.2f}, {ty:.2f}, {tz:.2f})...")
        stable_counter = 0
        rate = 0.02  # 50 Hz

        while rclpy.ok():
            rclpy.spin_once(self)
            x, y, z = self.pose.position.x, self.pose.position.y, self.pose.position.z

            dx, dy, dz = tx - x, ty - y, tz - z
            dist = math.sqrt(dx**2 + dy**2)

            if dist < self.tol:
                stable_counter += 1
                if stable_counter > 10:
                    self.stop()
                    self.get_logger().info(f"Reached target ({x:.2f}, {y:.2f}) ✓")
                    break
            else:
                stable_counter = 0
                vx = self.kp * dx
                vy = self.kp * dy
                vz = self.kp * dz

                vx = max(min(vx, 0.3), -0.3)
                vy = max(min(vy, 0.3), -0.3)
                vz = max(min(vz, 0.3), -0.3)

                twist = Twist()
                twist.linear.x = vx
                twist.linear.y = vy
                twist.linear.z = vz
                self.cmd_pub.publish(twist)

                self.get_logger().info(f"→ pos=({x:.3f},{y:.3f}) dist={dist:.3f} vx={vx:.2f} vy={vy:.2f}")
            time.sleep(rate)

    def stop(self):
        twist = Twist()
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)


def main():
    rclpy.init()
    ManualNavigator()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
