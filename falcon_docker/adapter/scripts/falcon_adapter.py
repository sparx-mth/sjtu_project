#!/usr/bin/env python3
"""
falcon_adapter.py
Bridges FALCON planner (ROS1) <-> sjtu_drone (ROS2 via ros1_bridge).

Data flow:
  IN:  /simple_drone/gt_pose  (Pose)   --> /odom_world  (Odometry) + TF
  IN:  /simple_drone/depth/*  (Image)  --> /map_ros/depth (Image)
  OUT: /planning/pos_cmd (PositionCommand) --> /simple_drone/cmd_vel (Twist)
  INIT: sends /simple_drone/takeoff + disables posctrl
"""
import rospy
import tf
import math
import numpy as np

from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Bool


class FalconAdapter:
    def __init__(self):
        rospy.init_node("falcon_adapter")

        # Parameters
        self.drone_ns = rospy.get_param("~drone_ns", "/simple_drone")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.body_frame = rospy.get_param("~body_frame", "body")
        self.cam_frame = rospy.get_param("~cam_frame", "camera")

        # PD gains for pos_cmd -> cmd_vel conversion
        self.kp_xy = rospy.get_param("~kp_xy", 1.5)
        self.kd_xy = rospy.get_param("~kd_xy", 0.3)
        self.kp_z = rospy.get_param("~kp_z", 1.5)
        self.kd_z = rospy.get_param("~kd_z", 0.3)
        self.kp_yaw = rospy.get_param("~kp_yaw", 1.0)
        self.max_vel_xy = rospy.get_param("~max_vel_xy", 1.0)
        self.max_vel_z = rospy.get_param("~max_vel_z", 0.5)
        self.max_yaw_rate = rospy.get_param("~max_yaw_rate", 1.0)

        # Whether to auto-takeoff or assume drone is already airborne
        self.auto_takeoff = rospy.get_param("~auto_takeoff", True)

        # State
        self.cur_pose = None
        self.prev_time = None
        self.vel = np.zeros(3)
        self.target_pos = None
        self.target_yaw = 0.0
        self.airborne = False

        # TF
        self.tf_br = tf.TransformBroadcaster()

        # Publishers — to FALCON
        self.odom_pub = rospy.Publisher("/odom_world", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("/map_ros/pose", PoseStamped, queue_size=10)
        self.depth_pub = rospy.Publisher("/map_ros/depth", Image, queue_size=2)

        # Publishers — to drone
        self.cmd_pub = rospy.Publisher(self.drone_ns + "/cmd_vel", Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher(self.drone_ns + "/takeoff", Empty, queue_size=1)
        self.posctrl_pub = rospy.Publisher(self.drone_ns + "/posctrl", Bool, queue_size=1, latch=True)

        # Subscribers — from drone (arrive via bridge as ROS1 topics)
        rospy.Subscriber(self.drone_ns + "/gt_pose", Pose, self.gt_pose_cb)
        rospy.Subscriber(self.drone_ns + "/front_depth/depth/image_raw", Image, self.depth_cb)

        # Subscriber — from FALCON traj_server
        try:
            from quadrotor_msgs.msg import PositionCommand
            rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.pos_cmd_cb)
            rospy.loginfo("[Adapter] Using quadrotor_msgs/PositionCommand")
        except ImportError:
            rospy.logwarn("[Adapter] quadrotor_msgs not found, using PoseStamped fallback")
            rospy.Subscriber("/planning/pos_cmd_pose", PoseStamped, self.pos_cmd_pose_cb)

        # Control loop at 30 Hz
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.control_loop)

        # Startup: wait for pose data before taking off
        if self.auto_takeoff:
            rospy.Timer(rospy.Duration(2.0), self.try_takeoff)
        else:
            rospy.loginfo("[Adapter] auto_takeoff=false — assuming drone is already airborne")
            self.airborne = True

        rospy.loginfo("══════════════════════════════════")
        rospy.loginfo("  FALCON <-> Drone Adapter")
        rospy.loginfo("  Drone: %s", self.drone_ns)
        rospy.loginfo("  auto_takeoff: %s", self.auto_takeoff)
        rospy.loginfo("══════════════════════════════════")

    def try_takeoff(self, _):
        """Retry takeoff until pose confirms the drone is up."""
        if self.airborne:
            return

        if self.cur_pose is None:
            rospy.logwarn("[Adapter] No pose yet — bridge may not be ready. Retrying in 3s...")
            rospy.Timer(rospy.Duration(3.0), self.try_takeoff, oneshot=True)
            return

        # Disable position control mode (drone accepts cmd_vel)
        self.posctrl_pub.publish(Bool(data=False))
        rospy.sleep(0.5)

        rospy.loginfo("[Adapter] Sending takeoff...")
        self.takeoff_pub.publish(Empty())

        # Wait and check altitude
        rospy.sleep(4.0)
        if self.cur_pose is not None and self.cur_pose.position.z > 0.3:
            rospy.loginfo("[Adapter] Drone is airborne (z=%.2f)", self.cur_pose.position.z)
            self.airborne = True
        else:
            z = self.cur_pose.position.z if self.cur_pose else 0.0
            rospy.logwarn("[Adapter] Drone may not be airborne (z=%.2f). Retrying...", z)
            rospy.Timer(rospy.Duration(3.0), self.try_takeoff, oneshot=True)

    # ── Drone -> FALCON ──────────────────────────────────────────

    def gt_pose_cb(self, msg):
        now = rospy.Time.now()

        # Estimate velocity via finite difference
        if self.cur_pose is not None and self.prev_time is not None:
            dt = (now - self.prev_time).to_sec()
            if dt > 1e-6:
                self.vel = np.array([
                    (msg.position.x - self.cur_pose.position.x) / dt,
                    (msg.position.y - self.cur_pose.position.y) / dt,
                    (msg.position.z - self.cur_pose.position.z) / dt,
                ])
        self.prev_time = now
        self.cur_pose = msg

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose = msg
        odom.twist.twist.linear.x = self.vel[0]
        odom.twist.twist.linear.y = self.vel[1]
        odom.twist.twist.linear.z = self.vel[2]
        self.odom_pub.publish(odom)

        # Publish sensor pose
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.world_frame
        ps.pose = msg
        self.pose_pub.publish(ps)

        # Broadcast TF
        p, o = msg.position, msg.orientation
        self.tf_br.sendTransform(
            (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
            now, self.body_frame, self.world_frame,
        )
        self.tf_br.sendTransform(
            (0, 0, 0), (0, 0, 0, 1),
            now, self.cam_frame, self.body_frame,
        )

    def depth_cb(self, msg):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.cam_frame
        self.depth_pub.publish(msg)

    # ── FALCON -> Drone ──────────────────────────────────────────

    def pos_cmd_cb(self, msg):
        self.target_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.target_yaw = msg.yaw

    def pos_cmd_pose_cb(self, msg):
        p = msg.pose.position
        self.target_pos = np.array([p.x, p.y, p.z])
        q = msg.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.target_yaw = yaw

    @staticmethod
    def _world_to_body(vx_world, vy_world, yaw):
        """Transform world-frame velocity to body-frame velocity."""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_body = vx_world * cos_yaw + vy_world * sin_yaw
        vy_body = -vx_world * sin_yaw + vy_world * cos_yaw
        return vx_body, vy_body

    def control_loop(self, _):
        if self.cur_pose is None or self.target_pos is None:
            return
        if not self.airborne:
            return

        cur = np.array([self.cur_pose.position.x,
                        self.cur_pose.position.y,
                        self.cur_pose.position.z])
        err = self.target_pos - cur

        # PD on XY (world frame)
        vxy = self.kp_xy * err[:2] - self.kd_xy * self.vel[:2]
        sp = np.linalg.norm(vxy)
        if sp > self.max_vel_xy:
            vxy = vxy / sp * self.max_vel_xy

        # PD on Z
        vz = np.clip(self.kp_z * err[2] - self.kd_z * self.vel[2],
                      -self.max_vel_z, self.max_vel_z)

        # P on yaw
        q = self.cur_pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        ye = (self.target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        yr = np.clip(self.kp_yaw * ye, -self.max_yaw_rate, self.max_yaw_rate)

        # ── FIX: transform world-frame XY velocity to body frame ──
        vx_body, vy_body = self._world_to_body(vxy[0], vxy[1], yaw)

        cmd = Twist()
        cmd.linear.x = vx_body
        cmd.linear.y = vy_body
        cmd.linear.z = vz
        cmd.angular.z = yr
        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    try:
        FalconAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass