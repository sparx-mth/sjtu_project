#!/usr/bin/env python3
"""
falcon_adapter.py  (v8 — 3D Pure Pursuit controller)

Replaces the simple PD controller with a 3D Pure Pursuit path follower,
adapted from the user's proven SmoothPathFollower.

Key improvements over v7:
  - Pure Pursuit tracking: steers toward a lookahead point on the
    buffered trajectory instead of directly at the target position.
    Produces smooth, rounded turns instead of oscillating corrections.
  - Adaptive lookahead: shorter on tight curves, longer on straights.
  - Curvature-based speed reduction: automatically slows before turns.
  - Smooth yaw control with deadband and exponential smoothing.
  - Distance-to-goal deceleration: slows down approaching waypoints.
  - 3D extension: altitude tracked with separate P controller.
  - Speed smoothing: prevents jerky acceleration changes.

Architecture:
  FALCON traj_server publishes PositionCommand at 100Hz with
  position, velocity, yaw. We buffer these into a rolling trajectory
  and use Pure Pursuit to track it.
"""

import rospy
import tf
import tf.transformations as tft
import math
import numpy as np
from collections import deque

from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, Bool


class TrajectoryBuffer:
    """
    Rolling buffer of PositionCommand points forming a local trajectory.
    Used by Pure Pursuit to find closest point and lookahead point.
    """

    def __init__(self, max_points=200, min_spacing=0.02):
        self.points = deque(maxlen=max_points)  # (x, y, z, yaw, stamp)
        self.min_spacing = min_spacing

    def add(self, x, y, z, yaw, stamp):
        """Add point if far enough from the last one."""
        if self.points:
            last = self.points[-1]
            dist = math.sqrt((x - last[0])**2 + (y - last[1])**2 + (z - last[2])**2)
            if dist < self.min_spacing:
                return
        self.points.append((x, y, z, yaw, stamp))

    def find_closest(self, px, py, pz, start_idx=0):
        """Find closest point index and distance."""
        if not self.points:
            return 0, float('inf')
        best_idx, best_dist = start_idx, float('inf')
        for i in range(start_idx, len(self.points)):
            pt = self.points[i]
            d = math.sqrt((pt[0]-px)**2 + (pt[1]-py)**2 + (pt[2]-pz)**2)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx, best_dist

    def find_lookahead(self, start_idx, lookahead_dist):
        """
        Find the point on the buffered trajectory at approximately
        lookahead_dist ahead of start_idx.
        """
        if not self.points or start_idx >= len(self.points) - 1:
            if self.points:
                return self.points[-1], len(self.points) - 1
            return None, 0

        accumulated = 0.0
        for i in range(start_idx, len(self.points) - 1):
            p0 = self.points[i]
            p1 = self.points[i + 1]
            seg_len = math.sqrt(
                (p1[0]-p0[0])**2 + (p1[1]-p0[1])**2 + (p1[2]-p0[2])**2
            )
            accumulated += seg_len
            if accumulated >= lookahead_dist:
                return p1, i + 1

        # Not enough trajectory buffered — return the last point
        return self.points[-1], len(self.points) - 1

    def estimate_curvature(self, idx):
        """
        Estimate local curvature at idx using three-point method.
        Returns curvature in 1/m (0 = straight).
        """
        if len(self.points) < 3 or idx < 1 or idx >= len(self.points) - 1:
            return 0.0

        p0 = self.points[idx - 1]
        p1 = self.points[idx]
        p2 = self.points[idx + 1]

        # Vectors
        v1 = np.array([p1[0]-p0[0], p1[1]-p0[1]])
        v2 = np.array([p2[0]-p1[0], p2[1]-p1[1]])

        # Cross product magnitude / product of lengths
        cross = abs(v1[0]*v2[1] - v1[1]*v2[0])
        l1, l2 = np.linalg.norm(v1), np.linalg.norm(v2)

        if l1 < 1e-6 or l2 < 1e-6:
            return 0.0

        # Menger curvature approximation
        area = cross / 2.0
        d = np.linalg.norm(np.array([p2[0]-p0[0], p2[1]-p0[1]]))
        if d < 1e-6:
            return 0.0

        return 4.0 * area / (l1 * l2 * d)

    @property
    def empty(self):
        return len(self.points) == 0

    @property
    def last(self):
        return self.points[-1] if self.points else None


class FalconAdapter:
    def __init__(self):
        rospy.init_node("falcon_adapter")

        # ── Parameters ──
        self.drone_ns = rospy.get_param("~drone_ns", "/simple_drone")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.body_frame = rospy.get_param("~body_frame", "body")
        self.cam_frame = rospy.get_param("~cam_frame", "camera")
        self.mapping_only = rospy.get_param("~mapping_only", False)

        # Camera offset
        self.cam_offset_x = rospy.get_param("~cam_offset_x", 0.2)
        self.cam_offset_y = rospy.get_param("~cam_offset_y", 0.0)
        self.cam_offset_z = rospy.get_param("~cam_offset_z", 0.0)

        self.auto_takeoff = rospy.get_param("~auto_takeoff", True)
        self.odom_min_dt = rospy.get_param("~odom_min_dt", 0.02)

        if self.mapping_only:
            self.auto_takeoff = False

        # ════════════════════════════════════════════════════════
        # Pure Pursuit parameters (from SmoothPathFollower)
        # ════════════════════════════════════════════════════════

        # Lookahead
        self.base_lookahead = rospy.get_param("~base_lookahead", 0.6)
        self.min_lookahead = rospy.get_param("~min_lookahead", 0.3)
        self.max_lookahead = rospy.get_param("~max_lookahead", 1.5)
        self.lookahead_speed_gain = rospy.get_param("~lookahead_speed_gain", 0.5)

        # Speed control
        self.cruise_speed = rospy.get_param("~cruise_speed", 0.4)
        self.min_speed = rospy.get_param("~min_speed", 0.08)
        self.max_speed = rospy.get_param("~max_speed", 0.5)
        self.curvature_speed_factor = rospy.get_param("~curvature_speed_factor", 0.3)
        self.speed_smoothing = rospy.get_param("~speed_smoothing", 0.3)

        # Altitude control
        self.altitude_kp = rospy.get_param("~altitude_kp", 1.2)
        self.max_vertical_speed = rospy.get_param("~max_vertical_speed", 0.3)

        # Yaw control (smooth, subtle)
        self.yaw_kp = rospy.get_param("~yaw_kp", 0.5)
        self.max_yaw_rate = rospy.get_param("~max_yaw_rate", 0.35)
        self.yaw_deadband = rospy.get_param("~yaw_deadband", 0.15)
        self.yaw_speed_threshold = rospy.get_param("~yaw_speed_threshold", 0.05)
        self.yaw_rate_smoothing = rospy.get_param("~yaw_rate_smoothing", 0.15)

        # Path tolerance
        self.path_tolerance = rospy.get_param("~path_tolerance", 2.0)

        # ── T_b_c (from matrix, same as every FALCON YAML) ──
        self.T_b_c = np.array([
            [ 0.0,  0.0, 1.0, self.cam_offset_x],
            [-1.0,  0.0, 0.0, self.cam_offset_y],
            [ 0.0, -1.0, 0.0, self.cam_offset_z],
            [ 0.0,  0.0, 0.0, 1.0]
        ])
        self.T_b_c_quat = tft.quaternion_from_matrix(self.T_b_c)
        self.T_b_c_trans = (self.cam_offset_x, self.cam_offset_y, self.cam_offset_z)

        # ── State ──
        self.cur_pose = None
        self.prev_time = None
        self.vel = np.zeros(3)
        self.airborne = False

        # Pure Pursuit state
        self.traj_buffer = TrajectoryBuffer(max_points=200, min_spacing=0.02)
        self.current_speed = 0.0
        self._current_yaw_rate = 0.0
        self.closest_idx = 0
        self.target_altitude = None  # Learned from first pos_cmd z

        # TF
        self.tf_br = tf.TransformBroadcaster()

        # ── Publishers: to FALCON ──
        self.odom_pub = rospy.Publisher("/odom_world", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("/map_ros/pose", PoseStamped, queue_size=10)
        self.depth_pub = rospy.Publisher("/map_ros/depth", Image, queue_size=2)
        self.cam_info_pub = rospy.Publisher(
            "/map_ros/depth/camera_info", CameraInfo, queue_size=2
        )

        # ── Publishers: to drone ──
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
            CameraInfo, self.cam_info_cb,
        )

        # ── Subscribers: from FALCON ──
        if not self.mapping_only:
            try:
                from quadrotor_msgs.msg import PositionCommand
                rospy.Subscriber(
                    "/planning/pos_cmd", PositionCommand, self.pos_cmd_cb
                )
                rospy.loginfo("[Adapter] Using quadrotor_msgs/PositionCommand")
            except ImportError:
                rospy.logwarn("[Adapter] quadrotor_msgs not found")
                rospy.Subscriber(
                    "/planning/pos_cmd_pose", PoseStamped, self.pos_cmd_pose_cb
                )

            # Control loop at 50 Hz (matching your SmoothPathFollower)
            rospy.Timer(rospy.Duration(1.0 / 50.0), self.control_loop)

        # ── Startup ──
        if self.auto_takeoff:
            rospy.Timer(rospy.Duration(2.0), self.try_takeoff)
        else:
            self.airborne = True

        # Logging counter
        self._loop_count = 0

        # ── Banner ──
        mode_str = "MAPPING ONLY" if self.mapping_only else "FULL EXPLORATION"
        rospy.loginfo("=" * 54)
        rospy.loginfo("  FALCON <-> Drone Adapter (v8 — Pure Pursuit)")
        rospy.loginfo("  Mode: %s", mode_str)
        rospy.loginfo("  Drone: %s", self.drone_ns)
        rospy.loginfo("  --- Pure Pursuit ---")
        rospy.loginfo("  Lookahead: %.1f-%.1fm", self.min_lookahead, self.max_lookahead)
        rospy.loginfo("  Speed: %.2f-%.2f m/s (cruise %.2f)",
                      self.min_speed, self.max_speed, self.cruise_speed)
        rospy.loginfo("  Yaw: kp=%.2f, max_rate=%.2f, deadband=%.2f",
                      self.yaw_kp, self.max_yaw_rate, self.yaw_deadband)
        rospy.loginfo("  Altitude: kp=%.2f, max_vz=%.2f",
                      self.altitude_kp, self.max_vertical_speed)
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
            rospy.loginfo("[Adapter] Airborne (z=%.2f)", self.cur_pose.position.z)
            self.airborne = True
        else:
            z = self.cur_pose.position.z if self.cur_pose else 0.0
            rospy.logwarn("[Adapter] Not airborne (z=%.2f). Retrying...", z)
            rospy.Timer(rospy.Duration(3.0), self.try_takeoff, oneshot=True)

    # ── Drone → FALCON ───────────────────────────────────────────

    def gt_pose_cb(self, msg):
        now = rospy.Time.now()

        if self.prev_time is not None:
            dt = (now - self.prev_time).to_sec()
            if dt < self.odom_min_dt:
                return
        else:
            dt = 0.0

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

        # 1. Odometry (body frame)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose = msg
        odom.twist.twist.linear.x = self.vel[0]
        odom.twist.twist.linear.y = self.vel[1]
        odom.twist.twist.linear.z = self.vel[2]
        self.odom_pub.publish(odom)

        # 2. Sensor pose (camera frame) — T_w_c = T_w_b * T_b_c
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

        # 3. TF
        self.tf_br.sendTransform(
            (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
            now, self.body_frame, self.world_frame,
        )
        self.tf_br.sendTransform(
            self.T_b_c_trans, self.T_b_c_quat,
            now, self.cam_frame, self.body_frame,
        )

    def depth_cb(self, msg):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.cam_frame
        self.depth_pub.publish(msg)

    def cam_info_cb(self, msg):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.cam_frame
        self.cam_info_pub.publish(msg)

    # ── FALCON → Trajectory Buffer ───────────────────────────────

    def pos_cmd_cb(self, msg):
        """Buffer FALCON's trajectory commands for Pure Pursuit."""
        self.traj_buffer.add(
            msg.position.x, msg.position.y, msg.position.z,
            msg.yaw, rospy.Time.now().to_sec()
        )
        if self.target_altitude is None:
            self.target_altitude = msg.position.z

    def pos_cmd_pose_cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.traj_buffer.add(p.x, p.y, p.z, yaw, rospy.Time.now().to_sec())
        if self.target_altitude is None:
            self.target_altitude = p.z

    # ════════════════════════════════════════════════════════════
    # Pure Pursuit helpers (adapted from SmoothPathFollower)
    # ════════════════════════════════════════════════════════════

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _compute_lookahead(self, current_speed, curvature):
        """Adaptive lookahead: shorter on curves, longer on straights."""
        lookahead = self.base_lookahead + self.lookahead_speed_gain * current_speed
        if curvature > 0.5:
            lookahead *= 0.7
        return max(self.min_lookahead, min(lookahead, self.max_lookahead))

    def _compute_speed(self, dist_to_goal, curvature):
        """Speed profiling: slow on curves, decelerate near goal."""
        speed = self.cruise_speed

        # Decelerate approaching the latest trajectory point
        if dist_to_goal < 1.0:
            speed *= (0.3 + 0.7 * (dist_to_goal / 1.0))

        # Slow on curves
        curve_factor = 1.0 / (1.0 + self.curvature_speed_factor * curvature)
        speed *= curve_factor

        return max(self.min_speed, min(speed, self.max_speed))

    def _compute_yaw_rate(self, current_yaw, desired_yaw, current_speed):
        """
        Smooth yaw control with deadband (from SmoothPathFollower).
        Prevents oscillation via exponential smoothing.
        """
        # Don't adjust yaw when nearly stationary
        if current_speed < self.yaw_speed_threshold:
            self._current_yaw_rate *= 0.8
            return self._current_yaw_rate

        yaw_error = self._normalize_angle(desired_yaw - current_yaw)

        # Deadband: ignore small errors
        if abs(yaw_error) < self.yaw_deadband:
            self._current_yaw_rate *= 0.7
            return self._current_yaw_rate

        # P control with saturation
        target_rate = self.yaw_kp * yaw_error
        target_rate = max(-self.max_yaw_rate, min(target_rate, self.max_yaw_rate))

        # Exponential smoothing to prevent oscillation
        self._current_yaw_rate = (
            self.yaw_rate_smoothing * target_rate +
            (1.0 - self.yaw_rate_smoothing) * self._current_yaw_rate
        )

        return self._current_yaw_rate

    # ════════════════════════════════════════════════════════════
    # 3D Pure Pursuit Control Loop
    # ════════════════════════════════════════════════════════════

    def control_loop(self, _):
        """
        Pure Pursuit controller running at 50 Hz.

        Algorithm:
          1. Find closest point on buffered trajectory
          2. Compute adaptive lookahead distance
          3. Find lookahead point on trajectory
          4. Compute speed (curvature-aware, with deceleration)
          5. Steer toward lookahead point (world frame velocity)
          6. Smooth yaw to face direction of travel
          7. Altitude P controller (3D extension)
          8. Publish cmd_vel
        """
        if self.cur_pose is None or not self.airborne:
            return
        if self.traj_buffer.empty:
            return

        px = self.cur_pose.position.x
        py = self.cur_pose.position.y
        pz = self.cur_pose.position.z
        q = self.cur_pose.orientation
        current_yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # ── 1. Find closest point on trajectory ──
        closest_idx, cross_track = self.traj_buffer.find_closest(
            px, py, pz, start_idx=max(0, self.closest_idx - 5)
        )

        # Only advance forward (prevent backtracking)
        if closest_idx >= self.closest_idx:
            self.closest_idx = closest_idx

        # Safety: too far from path
        if cross_track > self.path_tolerance:
            rospy.logwarn_throttle(
                2.0, "[Pursuit] Cross-track error %.2fm > tolerance %.2fm",
                cross_track, self.path_tolerance
            )

        # ── 2. Estimate curvature at current position ──
        curvature = self.traj_buffer.estimate_curvature(self.closest_idx)

        # ── 3. Compute adaptive lookahead ──
        lookahead = self._compute_lookahead(self.current_speed, curvature)

        # ── 4. Find lookahead point ──
        lookahead_pt, lookahead_idx = self.traj_buffer.find_lookahead(
            self.closest_idx, lookahead
        )
        if lookahead_pt is None:
            return

        target_x, target_y, target_z, target_yaw = (
            lookahead_pt[0], lookahead_pt[1], lookahead_pt[2], lookahead_pt[3]
        )

        # ── 5. Compute distance to end of buffer (for deceleration) ──
        if self.traj_buffer.last:
            last = self.traj_buffer.last
            dist_to_end = math.sqrt(
                (last[0]-px)**2 + (last[1]-py)**2
            )
        else:
            dist_to_end = 10.0

        # ── 6. Compute target speed ──
        target_speed = self._compute_speed(dist_to_end, curvature)

        # Smooth speed changes
        self.current_speed = (
            self.speed_smoothing * target_speed +
            (1.0 - self.speed_smoothing) * self.current_speed
        )

        # ── 7. Compute XY velocity toward lookahead (world frame) ──
        dx = target_x - px
        dy = target_y - py
        dist_to_target = math.hypot(dx, dy)

        if dist_to_target > 0.01:
            vx_world = (dx / dist_to_target) * self.current_speed
            vy_world = (dy / dist_to_target) * self.current_speed
            desired_yaw = math.atan2(dy, dx)
        else:
            vx_world, vy_world = 0.0, 0.0
            desired_yaw = current_yaw

        # ── 8. Yaw rate (smooth, with deadband) ──
        yaw_rate = self._compute_yaw_rate(
            current_yaw, desired_yaw, self.current_speed
        )

        # ── 9. Altitude control (3D extension) ──
        # Use the lookahead point's z as the target altitude
        error_z = target_z - pz
        vz = self.altitude_kp * error_z
        vz = max(-self.max_vertical_speed, min(vz, self.max_vertical_speed))

        # ── 10. Publish cmd_vel (world frame for sjtu_drone) ──
        cmd = Twist()
        cmd.linear.x = float(vx_world)
        cmd.linear.y = float(vy_world)
        cmd.linear.z = float(vz)
        cmd.angular.z = float(yaw_rate)
        self.cmd_pub.publish(cmd)

        # ── Logging ──
        self._loop_count += 1
        if self._loop_count % 100 == 0:  # Every 2 seconds at 50Hz
            rospy.loginfo(
                "[Pursuit] spd=%.2f | pos=(%.1f,%.1f,%.1f) | "
                "xtrack=%.2f | curv=%.1f | la=%.2f | yaw=%d->%d",
                self.current_speed, px, py, pz,
                cross_track, curvature, lookahead,
                math.degrees(current_yaw), math.degrees(desired_yaw),
            )


if __name__ == "__main__":
    try:
        FalconAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass