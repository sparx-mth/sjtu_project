#!/usr/bin/env python3
"""
falcon_adapter.py  (v5)
Bridges FALCON planner (ROS1) <-> sjtu_drone (ROS2 via ros1_bridge).

Data flow:
  IN:  /simple_drone/gt_pose                  (Pose)  --> /odom_world (Odometry) + TF
  IN:  /simple_drone/front_depth/depth/image_raw      --> /map_ros/depth
  IN:  /simple_drone/front_depth/depth/camera_info    --> /map_ros/depth/camera_info
  OUT: /planning/pos_cmd (PositionCommand)   --> /simple_drone/cmd_vel (Twist)
  INIT: sends /simple_drone/takeoff + disables posctrl

Modes:
  mapping_only=false (default): Full exploration — adapter takes off, FALCON
    plans trajectories, adapter converts them to cmd_vel.
  mapping_only=true: Map-only mode — adapter forwards pose + depth to FALCON
    for mapping, but NEVER publishes cmd_vel or takeoff. Fly the drone yourself.

Frames:
  world -> body    : drone pose  (published as Odometry + PoseStamped)
  body  -> camera  : FALCON uses T_b_c from hospital.yaml for depth back-projection

Depth encoding fix (v5):
  FALCON's voxel_mapping reads depth as uint16 (mm) and hardcodes * 0.001.
  Gazebo publishes 32FC1 (float32, meters). The adapter now converts
  32FC1 → 16UC1 (millimeters) so FALCON can process depth correctly.
"""
import rospy
import tf
import tf.transformations as tft
import math
import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, Bool

# body (FLU):    x-forward, y-left,  z-up
# optical (RDF): z-forward, x-right, y-down
_BODY_TO_OPTICAL_QUAT = (0.5, -0.5, 0.5, 0.5)


class FalconAdapter:
    def __init__(self):
        rospy.init_node("falcon_adapter")

        # Parameters
        self.drone_ns = rospy.get_param("~drone_ns", "/simple_drone")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.body_frame = rospy.get_param("~body_frame", "body")
        self.cam_frame = rospy.get_param("~cam_frame", "camera")

        # ── Mapping-only mode ──
        # When true: forwards pose+depth for mapping, but never sends
        # cmd_vel or takeoff. You fly the drone manually.
        self.mapping_only = rospy.get_param("~mapping_only", False)

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

        # In mapping_only mode, force auto_takeoff off
        if self.mapping_only:
            self.auto_takeoff = False

        # ── Odom throttle: max rate for gt_pose processing ──
        self.odom_min_dt = rospy.get_param("~odom_min_dt", 0.02)  # 50 Hz max

        # State
        self.cur_pose = None
        self.prev_time = None
        self.vel = np.zeros(3)
        self.target_pos = None
        self.target_yaw = 0.0
        self.airborne = False

        # TF
        self.tf_br = tf.TransformBroadcaster()
        self.cv_bridge = CvBridge()

        # Publishers — to FALCON (always active, even in mapping_only)
        self.odom_pub = rospy.Publisher("/odom_world", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("/map_ros/pose", PoseStamped, queue_size=10)
        self.depth_pub = rospy.Publisher("/map_ros/depth", Image, queue_size=2)
        self.cam_info_pub = rospy.Publisher("/map_ros/depth/camera_info", CameraInfo, queue_size=2)

        # Publishers — to drone (only used when NOT in mapping_only mode)
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

        # Subscribers — from drone (always active)
        rospy.Subscriber(self.drone_ns + "/gt_pose", Pose, self.gt_pose_cb)
        rospy.Subscriber(
            self.drone_ns + "/front_depth/depth/image_raw", Image, self.depth_cb
        )
        rospy.Subscriber(
            self.drone_ns + "/front_depth/depth/camera_info", CameraInfo, self.cam_info_cb
        )

        # Subscriber — from FALCON traj_server (only when NOT mapping_only)
        if not self.mapping_only:
            try:
                from quadrotor_msgs.msg import PositionCommand

                rospy.Subscriber(
                    "/planning/pos_cmd", PositionCommand, self.pos_cmd_cb
                )
                rospy.loginfo("[Adapter] Using quadrotor_msgs/PositionCommand")
            except ImportError:
                rospy.logwarn(
                    "[Adapter] quadrotor_msgs not found, using PoseStamped fallback"
                )
                rospy.Subscriber(
                    "/planning/pos_cmd_pose", PoseStamped, self.pos_cmd_pose_cb
                )

            # Control loop at 30 Hz (only needed for autonomous flight)
            rospy.Timer(rospy.Duration(1.0 / 30.0), self.control_loop)

        # Startup: wait for pose data before taking off
        if self.auto_takeoff:
            rospy.Timer(rospy.Duration(2.0), self.try_takeoff)
        else:
            if not self.mapping_only:
                rospy.loginfo(
                    "[Adapter] auto_takeoff=false — assuming drone is already airborne"
                )
            self.airborne = True

        # ── Banner ──
        mode_str = "MAPPING ONLY (no cmd_vel)" if self.mapping_only else "FULL EXPLORATION"
        rospy.loginfo("══════════════════════════════════")
        rospy.loginfo("  FALCON <-> Drone Adapter (v5)")
        rospy.loginfo("  Mode: %s", mode_str)
        rospy.loginfo("  Drone: %s", self.drone_ns)
        if not self.mapping_only:
            rospy.loginfo("  auto_takeoff: %s", self.auto_takeoff)
        else:
            rospy.loginfo("  Fly the drone manually — adapter only forwards pose+depth")
        rospy.loginfo(
            "  odom_min_dt: %.3f s (max %.0f Hz)",
            self.odom_min_dt,
            1.0 / self.odom_min_dt,
        )
        rospy.loginfo("  pose type: PoseStamped (body frame)")
        rospy.loginfo("  T_b_c applied by FALCON (not adapter)")
        rospy.loginfo("══════════════════════════════════")

    def try_takeoff(self, _):
        """Retry takeoff until pose confirms the drone is up."""
        if self.airborne:
            return

        if self.cur_pose is None:
            rospy.logwarn(
                "[Adapter] No pose yet — bridge may not be ready. Retrying in 3s..."
            )
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
            rospy.loginfo(
                "[Adapter] Drone is airborne (z=%.2f)", self.cur_pose.position.z
            )
            self.airborne = True
        else:
            z = self.cur_pose.position.z if self.cur_pose else 0.0
            rospy.logwarn(
                "[Adapter] Drone may not be airborne (z=%.2f). Retrying...", z
            )
            rospy.Timer(rospy.Duration(3.0), self.try_takeoff, oneshot=True)

    # ── Drone -> FALCON ──────────────────────────────────────────

    def gt_pose_cb(self, msg):
        now = rospy.Time.now()

        # ── THROTTLE: skip if called too fast (bridge can push 800+ Hz) ──
        if self.prev_time is not None:
            dt = (now - self.prev_time).to_sec()
            if dt < self.odom_min_dt:
                return
        else:
            dt = 0.0

        # Estimate velocity via finite difference
        if self.cur_pose is not None and dt > 1e-6:
            self.vel = np.array(
                [
                    (msg.position.x - self.cur_pose.position.x) / dt,
                    (msg.position.y - self.cur_pose.position.y) / dt,
                    (msg.position.z - self.cur_pose.position.z) / dt,
                ]
            )
        self.prev_time = now
        self.cur_pose = msg

        # ── Publish Odometry (body frame) ──
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose = msg
        odom.twist.twist.linear.x = self.vel[0]
        odom.twist.twist.linear.y = self.vel[1]
        odom.twist.twist.linear.z = self.vel[2]
        self.odom_pub.publish(odom)

        # ── Publish sensor pose as PoseStamped (BODY pose, not camera) ──
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.world_frame
        ps.pose = msg
        self.pose_pub.publish(ps)

        # ── Broadcast TF ──
        p, o = msg.position, msg.orientation
        self.tf_br.sendTransform(
            (p.x, p.y, p.z),
            (o.x, o.y, o.z, o.w),
            now,
            self.body_frame,
            self.world_frame,
        )
        self.tf_br.sendTransform(
            (0, 0, 0),
            _BODY_TO_OPTICAL_QUAT,
            now,
            self.cam_frame,
            self.body_frame,
        )

    def depth_cb(self, msg):
        now = rospy.Time.now()

        # ── Convert 32FC1 (float meters) → 16UC1 (uint16 millimeters) ──
        # FALCON's voxel_mapping reads depth as uint16_t and hardcodes * 0.001
        # to convert mm→m. Gazebo publishes 32FC1 in meters, so we must convert.
        if msg.encoding == "32FC1":
            depth_m = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            # Replace NaN/inf with 0 (no measurement)
            depth_m = np.where(np.isfinite(depth_m), depth_m, 0.0)
            # Convert meters → millimeters, clamp to uint16 range
            depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
            msg = self.cv_bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1")

        msg.header.stamp = now
        msg.header.frame_id = self.cam_frame
        self.depth_pub.publish(msg)

    def cam_info_cb(self, msg):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.cam_frame
        self.cam_info_pub.publish(msg)

    # ── FALCON -> Drone (disabled in mapping_only mode) ──────────

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

        cur = np.array(
            [
                self.cur_pose.position.x,
                self.cur_pose.position.y,
                self.cur_pose.position.z,
            ]
        )
        err = self.target_pos - cur

        # PD on XY (world frame)
        vxy = self.kp_xy * err[:2] - self.kd_xy * self.vel[:2]
        sp = np.linalg.norm(vxy)
        if sp > self.max_vel_xy:
            vxy = vxy / sp * self.max_vel_xy

        # PD on Z
        vz = np.clip(
            self.kp_z * err[2] - self.kd_z * self.vel[2],
            -self.max_vel_z,
            self.max_vel_z,
        )

        # P on yaw
        q = self.cur_pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        ye = (self.target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        yr = np.clip(self.kp_yaw * ye, -self.max_yaw_rate, self.max_yaw_rate)

        # ── sjtu_drone cmd_vel expects WORLD-frame velocities ──
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
