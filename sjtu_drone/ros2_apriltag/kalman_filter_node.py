#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class DroneKalmanFilter(Node):
    def __init__(self):
        super().__init__("drone_kalman_filter")

        # State vector: [x, y, z, vx, vy, vz]
        self.x = np.zeros((6, 1), dtype=float)
        self.P = np.eye(6) * 1.0

        self.initialized = False

        # Process noise (Q)
        self.q_pos = 1e-3
        self.q_vel = 1e-2
        self.Q = np.diag([
            self.q_pos, self.q_pos, self.q_pos,
            self.q_vel, self.q_vel, self.q_vel
        ])

        # Measurement noise (R) for triangulation
        pos_noise = 0.05  # 5 cm
        self.R = np.diag([pos_noise**2, pos_noise**2, pos_noise**2])

        # Measurement matrix H: we only measure x,y,z
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ])

        # IMU tracking
        self.last_imu_time = None
        self.last_acc = np.zeros(3)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, "/tag_pose", self.pose_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, "/simple_drone/imu/out", self.imu_callback, 100
        )

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry, "/drone/filtered_odom", 10
        )

        self.get_logger().info("DroneKalmanFilter node started.")

    # ----------------- IMU: Prediction Step -----------------
    def imu_callback(self, msg: Imu):
        # Current timestamp
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Linear acceleration (subtract gravity approximation on z)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z - 9.81

        acc = np.array([ax, ay, az])

        if self.last_imu_time is None:
            self.last_imu_time = t
            self.last_acc = acc
            return

        dt = t - self.last_imu_time
        if dt <= 0.0 or dt > 1.0:
            dt = 0.01  # fallback for unstable timing

        self.last_imu_time = t
        self.last_acc = acc

        if not self.initialized:
            return

        # State transition matrix F
        F = np.array([
            [1, 0, 0, dt,  0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1],
        ])

        a = acc.reshape((3, 1))

        # Predict position and velocity
        pos = self.x[0:3]
        vel = self.x[3:6]

        pos_new = pos + vel * dt + 0.5 * a * (dt ** 2)
        vel_new = vel + a * dt

        self.x[0:3] = pos_new
        self.x[3:6] = vel_new

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q

        # Publish predicted state
        self.publish_odom(msg.header.stamp)

    # ----------------- Tag Pose: Update Step -----------------
    def pose_callback(self, msg: PoseStamped):
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        z = np.array([[px], [py], [pz]])

        if not self.initialized:
            # Initialization from triangulation
            self.x[0:3, 0] = np.array([px, py, pz])
            self.x[3:6, 0] = np.array([0.0, 0.0, 0.0])
            self.P = np.eye(6) * 0.1
            self.initialized = True

            self.get_logger().info(
                f"EKF initialized at ({px:.2f}, {py:.2f}, {pz:.2f})"
            )
        else:
            # Innovation
            y = z - self.H @ self.x
            S = self.H @ self.P @ self.H.T + self.R
            K = self.P @ self.H.T @ np.linalg.inv(S)

            # Update state
            self.x = self.x + K @ y

            # Update covariance
            I = np.eye(6)
            self.P = (I - K @ self.H) @ self.P

        self.publish_odom(msg.header.stamp)

    # ----------------- Publish Odometry -----------------
    def publish_odom(self, stamp):
        if not self.initialized:
            return

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.x[0, 0])
        odom.pose.pose.position.y = float(self.x[1, 0])
        odom.pose.pose.position.z = float(self.x[2, 0])

        # Neutral orientation for now
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0

        odom.twist.twist.linear.x = float(self.x[3, 0])
        odom.twist.twist.linear.y = float(self.x[4, 0])
        odom.twist.twist.linear.z = float(self.x[5, 0])

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = DroneKalmanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
