#!/usr/bin/env python3
"""
falcon_adapter.py  (v7)
Bridges FALCON planner (ROS1) <-> sjtu_drone (ROS2 via ros1_bridge).

v7 fixes:
  - CRITICAL: Fixed T_b_c quaternion bug. Previous versions used
    (0.5, -0.5, 0.5, 0.5) which is the INVERSE of the correct rotation.
    Now builds T_b_c directly from the matrix (same as FALCON's YAML)
    to eliminate any quaternion convention confusion.
  - Camera at 0.2m forward of body center (matching xacro joint).
  - Depth 32FC1 passthrough (FALCON handles conversion internally).

Data flow:
  IN:  /simple_drone/gt_pose (Pose)
         → /odom_world (Odometry, body frame, frame_id="world")
         → /map_ros/pose (PoseStamped, CAMERA frame via T_w_c = T_w_b * T_b_c)

  IN:  /simple_drone/front_depth/depth/image_raw (32FC1)
         → /map_ros/depth (passthrough, frame_id="camera")

  OUT: /planning/pos_cmd (PositionCommand) → /simple_drone/cmd_vel (Twist)
"""

import rospy
import tf
import tf.transformations as tft
import math
import numpy as np

from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, Bool


class FalconAdapter:
    def __init__(self):
        rospy.init_node("falcon_adapter")

        # ── Parameters ──
        self.drone_ns = rospy.get_param("~drone_ns", "/simple_drone")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.body_frame = rospy.get_param("~body_frame", "body")
        self.cam_frame = rospy.get_param("~cam_frame", "camera")
        self.mapping_only = rospy.get_param("~mapping_only", False)

        # Camera offset from body center (in body frame, meters)
        self.cam_offset_x = rospy.get_param("~cam_offset_x", 0.2)
        self.cam_offset_y = rospy.get_param("~cam_offset_y", 0.0)
        self.cam_offset_z = rospy.get_param("~cam_offset_z", 0.0)

        # PD gains for pos_cmd -> cmd_vel
        self.kp_xy = rospy.get_param("~kp_xy", 1.5)
        self.kd_xy = rospy.get_param("~kd_xy", 0.3)
        self.kp_z = rospy.get_param("~kp_z", 1.5)
        self.kd_z = rospy.get_param("~kd_z", 0.3)
        self.kp_yaw = rospy.get_param("~kp_yaw", 1.0)
        self.max_vel_xy = rospy.get_param("~max_vel_xy", 1.0)
        self.max_vel_z = rospy.get_param("~max_vel_z", 0.5)
        self.max_yaw_rate = rospy.get_param("~max_yaw_rate", 1.0)
        self.auto_takeoff = rospy.get_param("~auto_takeoff", True)
        self.odom_min_dt = rospy.get_param("~odom_min_dt", 0.02)

        if self.mapping_only:
            self.auto_takeoff = False

        # ══════════════════════════════════════════════════════════
        # T_b_c: camera-to-body transform (4x4 homogeneous)
        #
        # Built DIRECTLY from the matrix — no quaternion conversion
        # that can go wrong. This is identical to every FALCON map
        # YAML (octa_maze.yaml, complex_office.yaml, hospital.yaml).
        #
        # Columns = where each camera axis points in body frame:
        #   Camera X (right)   → Body -Y (left)    : col0 = [ 0,-1, 0]
        #   Camera Y (down)    → Body -Z (down)    : col1 = [ 0, 0,-1]
        #   Camera Z (forward) → Body  X (forward) : col2 = [ 1, 0, 0]
        #
        # Translation = camera origin in body frame coordinates.
        # ══════════════════════════════════════════════════════════
        self.T_b_c = np.array([
            [ 0.0,  0.0, 1.0, self.cam_offset_x],
            [-1.0,  0.0, 0.0, self.cam_offset_y],
            [ 0.0, -1.0, 0.0, self.cam_offset_z],
            [ 0.0,  0.0, 0.0, 1.0]
        ])

        # Extract quaternion from the rotation part for TF broadcast
        self.T_b_c_quat = tft.quaternion_from_matrix(self.T_b_c)
        self.T_b_c_trans = (self.cam_offset_x, self.cam_offset_y, self.cam_offset_z)

        # ── State ──
        self.cur_pose = None
        self.prev_time = None
        self.vel = np.zeros(3)
        self.target_pos = None
        self.target_yaw = 0.0
        self.airborne = False

        # ── TF ──
        self.tf_br = tf.TransformBroadcaster()

        # ── Publishers: to FALCON ──
        self.odom_pub = rospy.Publisher("/odom_world", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("/map_ros/pose", PoseStamped, queue_size=10)
        self.depth_pub = rospy.Publisher("/map_ros/depth", Image, queue_size=2)
        self.cam_info_pub = rospy.Publisher(
            "/map_ros/depth/camera_info", CameraInfo, queue_size=2
        )

        # ── Publishers: to drone (exploration mode only) ──
        if not self.mapping_only:
            self.cmd_pub = rospy.Publisher(
                self.drone_ns + "/cmd_vel", Twist, queue_size=10
            )
            self.takeoff_pub = rospy.Publisher(
                self.drone_ns + "/takeoff", Empty, queue_size=1
            )
            self.posctrl_pub = rospy.Publisher(
                self.drone_ns + "/posctrl", Bool, queue_size=1, latch=True
            )

        # ── Subscribers: from drone ──
        rospy.Subscriber(self.drone_ns + "/gt_pose", Pose, self.gt_pose_cb)
        rospy.Subscriber(
            self.drone_ns + "/front_depth/depth/image_raw", Image, self.depth_cb
        )
        rospy.Subscriber(
            self.drone_ns + "/front_depth/depth/camera_info",
            CameraInfo,
            self.cam_info_cb,
        )

        # ── Subscribers: from FALCON (exploration mode only) ──
        if not self.mapping_only:
            try:
                from quadrotor_msgs.msg import PositionCommand

                rospy.Subscriber(
                    "/planning/pos_cmd", PositionCommand, self.pos_cmd_cb
                )
                rospy.loginfo("[Adapter] Using quadrotor_msgs/PositionCommand")
            except ImportError:
                rospy.logwarn(
                    "[Adapter] quadrotor_msgs not found, PoseStamped fallback"
                )
                rospy.Subscriber(
                    "/planning/pos_cmd_pose", PoseStamped, self.pos_cmd_pose_cb
                )
            rospy.Timer(rospy.Duration(1.0 / 30.0), self.control_loop)

        # ── Startup ──
        if self.auto_takeoff:
            rospy.Timer(rospy.Duration(2.0), self.try_takeoff)
        else:
            self.airborne = True

        # ── Banner ──
        mode_str = (
            "MAPPING ONLY" if self.mapping_only else "FULL EXPLORATION"
        )
        rospy.loginfo("=" * 54)
        rospy.loginfo("  FALCON <-> Drone Adapter (v7)")
        rospy.loginfo("  Mode: %s", mode_str)
        rospy.loginfo("  Drone: %s", self.drone_ns)
        rospy.loginfo(
            "  Camera offset (body frame): (%.2f, %.2f, %.2f)",
            self.cam_offset_x, self.cam_offset_y, self.cam_offset_z,
        )
        rospy.loginfo("  T_b_c (from matrix, not quaternion):")
        for row in self.T_b_c:
            rospy.loginfo("    [%6.2f %6.2f %6.2f %6.2f]",
                          row[0], row[1], row[2], row[3])
        rospy.loginfo("  sensor_pose = T_w_b * T_b_c  (camera frame)")
        rospy.loginfo("  Depth: 32FC1 passthrough")
        rospy.loginfo("=" * 54)

    # ── Takeoff ──────────────────────────────────────────────────

    def try_takeoff(self, _):
        if self.airborne:
            return
        if self.cur_pose is None:
            rospy.logwarn("[Adapter] No pose yet — retrying in 3s...")
            rospy.Timer(rospy.Duration(3.0), self.try_takeoff, oneshot=True)
            return

        self.posctrl_pub.publish(Bool(data=False))
        rospy.sleep(0.5)

        rospy.loginfo("[Adapter] Sending takeoff...")
        self.takeoff_pub.publish(Empty())
        rospy.sleep(4.0)

        if self.cur_pose is not None and self.cur_pose.position.z > 0.3:
            rospy.loginfo(
                "[Adapter] Airborne (z=%.2f)", self.cur_pose.position.z
            )
            self.airborne = True
        else:
            z = self.cur_pose.position.z if self.cur_pose else 0.0
            rospy.logwarn("[Adapter] Not airborne (z=%.2f). Retrying...", z)
            rospy.Timer(rospy.Duration(3.0), self.try_takeoff, oneshot=True)

    # ── Drone → FALCON ───────────────────────────────────────────

    def gt_pose_cb(self, msg):
        now = rospy.Time.now()

        # Throttle
        if self.prev_time is not None:
            dt = (now - self.prev_time).to_sec()
            if dt < self.odom_min_dt:
                return
        else:
            dt = 0.0

        # Velocity estimate
        if self.cur_pose is not None and dt > 1e-6:
            self.vel = np.array([
                (msg.position.x - self.cur_pose.position.x) / dt,
                (msg.position.y - self.cur_pose.position.y) / dt,
                (msg.position.z - self.cur_pose.position.z) / dt,
            ])
        self.prev_time = now
        self.cur_pose = msg

        p = msg.position
        o = msg.orientation

        # ── 1. Odometry (body frame) for FALCON FSM ──
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose = msg
        odom.twist.twist.linear.x = self.vel[0]
        odom.twist.twist.linear.y = self.vel[1]
        odom.twist.twist.linear.z = self.vel[2]
        self.odom_pub.publish(odom)

        # ── 2. Sensor pose (CAMERA frame) for voxel_mapping ──
        #
        # FALCON's transformer feeds this directly to voxel_mapping
        # as T_w_c. The planner NEVER applies T_b_c — we must do it.
        #
        # T_w_c = T_w_b * T_b_c
        #
        T_w_b = tft.quaternion_matrix([o.x, o.y, o.z, o.w])
        T_w_b[0, 3] = p.x
        T_w_b[1, 3] = p.y
        T_w_b[2, 3] = p.z

        T_w_c = T_w_b @ self.T_b_c

        cam_quat = tft.quaternion_from_matrix(T_w_c)
        cam_pos = T_w_c[:3, 3]

        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.world_frame
        ps.pose.position.x = cam_pos[0]
        ps.pose.position.y = cam_pos[1]
        ps.pose.position.z = cam_pos[2]
        ps.pose.orientation.x = cam_quat[0]
        ps.pose.orientation.y = cam_quat[1]
        ps.pose.orientation.z = cam_quat[2]
        ps.pose.orientation.w = cam_quat[3]
        self.pose_pub.publish(ps)

        # ── 3. TF: world→body and body→camera ──
        self.tf_br.sendTransform(
            (p.x, p.y, p.z),
            (o.x, o.y, o.z, o.w),
            now,
            self.body_frame,
            self.world_frame,
        )
        self.tf_br.sendTransform(
            self.T_b_c_trans,
            self.T_b_c_quat,
            now,
            self.cam_frame,
            self.body_frame,
        )

    def depth_cb(self, msg):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.cam_frame
        self.depth_pub.publish(msg)

    def cam_info_cb(self, msg):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.cam_frame
        self.cam_info_pub.publish(msg)

    # ── FALCON → Drone ───────────────────────────────────────────

    def pos_cmd_cb(self, msg):
        self.target_pos = np.array(
            [msg.position.x, msg.position.y, msg.position.z]
        )
        self.target_yaw = msg.yaw

    def pos_cmd_pose_cb(self, msg):
        p = msg.pose.position
        self.target_pos = np.array([p.x, p.y, p.z])
        q = msg.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.target_yaw = yaw

    def control_loop(self, _):
        if self.cur_pose is None or self.target_pos is None:
            return
        if not self.airborne:
            return

        cur = np.array([
            self.cur_pose.position.x,
            self.cur_pose.position.y,
            self.cur_pose.position.z,
        ])
        err = self.target_pos - cur

        vxy = self.kp_xy * err[:2] - self.kd_xy * self.vel[:2]
        sp = np.linalg.norm(vxy)
        if sp > self.max_vel_xy:
            vxy = vxy / sp * self.max_vel_xy

        vz = np.clip(
            self.kp_z * err[2] - self.kd_z * self.vel[2],
            -self.max_vel_z, self.max_vel_z,
        )

        q = self.cur_pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        ye = (self.target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        yr = np.clip(self.kp_yaw * ye, -self.max_yaw_rate, self.max_yaw_rate)

        cmd = Twist()
        cmd.linear.x = float(vxy[0])
        cmd.linear.y = float(vxy[1])
        cmd.linear.z = float(vz)
        cmd.angular.z = float(yr)
        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    try:
        FalconAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
