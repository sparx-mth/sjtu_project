#!/usr/bin/env python3
"""
navigate_waypoints.py
---------------------
Simple waypoint navigation controller:
 - Reads current pose from /simple_drone/gt_pose
 - Uses a predefined list of waypoints (x, y) in world coordinates (meters)
 - Flies to each waypoint sequentially at constant altitude
 - Prints feedback after each waypoint is reached
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import math, time


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # ROS setup
        self.pose = Pose()
        self.pose_sub = self.create_subscription(Pose, '/simple_drone/gt_pose', self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)

        # parameters
        self.z_fixed = 1.5  # constant altitude
        self.kp = 0.8
        self.tol = 0.05  # stop within 5cm

        # === define list of waypoints (world coordinates) ===
        self.waypoints = [
            (4.5, 1.0),
            (4.5, -9.0),
            (3.0, -9.0),
            (2.8, -11.0),
            (1.5, -11.2),
        ]  # 355, 593

        self.get_logger().info("Waiting for pose data...")
        # wait for pose data
        while rclpy.ok() and self.pose.position.x == 0.0 and self.pose.position.y == 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.run_mission()

    # === callbacks ===
    def pose_cb(self, msg: Pose):
        self.pose = msg

    def yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # === mission loop ===
    def run_mission(self):
        self.get_logger().info(f"Starting mission with {len(self.waypoints)} waypoints...")
        for i, (tx, ty) in enumerate(self.waypoints):
            self.get_logger().info(f"\n→ Waypoint {i+1}/{len(self.waypoints)}: ({tx:.2f}, {ty:.2f})")
            self.move_to(tx, ty, self.z_fixed)
            time.sleep(1.0)
        self.get_logger().info("Mission completed ✓")

    # === movement ===
    def move_to(self, tx, ty, tz):
        stable_counter = 0
        rate = 0.02  # 50Hz

        while rclpy.ok():
            rclpy.spin_once(self)
            x, y, z = self.pose.position.x, self.pose.position.y, self.pose.position.z

            dx, dy, dz = tx - x, ty - y, tz - z
            dist = math.sqrt(dx**2 + dy**2)

            if dist < self.tol:
                stable_counter += 1
                if stable_counter > 10:
                    self.stop()
                    self.get_logger().info(f"Reached waypoint ({x:.2f}, {y:.2f}) ✓")
                    break
            else:
                stable_counter = 0
                vx = self.kp * dx
                vy = self.kp * dy
                vz = self.kp * dz

                # limit speeds
                vx = max(min(vx, 0.3), -0.3)
                vy = max(min(vy, 0.3), -0.3)
                vz = max(min(vz, 0.3), -0.3)

                twist = Twist()
                twist.linear.x = vx
                twist.linear.y = vy
                twist.linear.z = vz
                self.cmd_pub.publish(twist)

                self.get_logger().info(f"→ pos=({x:.2f},{y:.2f}) dist={dist:.2f} vx={vx:.2f} vy={vy:.2f}")
            time.sleep(rate)

    def stop(self):
        twist = Twist()
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)


def main():
    rclpy.init()
    WaypointNavigator()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
