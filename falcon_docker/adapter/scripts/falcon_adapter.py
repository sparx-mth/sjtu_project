#!/usr/bin/env python3
"""
falcon_adapter.py  (v12 — drift + jitter on localisation)

Bridges drone topics to FALCON topics. That's it.

  Drone gt_pose      -> /odom_world  +  /map_ros/pose  +  TF
  Drone depth        -> /map_ros/depth
  Drone camera_info  -> /map_ros/depth/camera_info

You fly the drone manually. FALCON builds the map and plans
exploration from the pose + depth it receives.

Localisation-noise injection (all default 0 = clean):

  jitter — i.i.d. Gaussian, sampled fresh each tick:
      ~noise_pos_std        position std-dev    [m]
      ~noise_yaw_std        yaw std-dev         [rad]

  drift  — random walk on the world-frame offset; ACCUMULATES.
           Variance grows as  σ_per_m² · Δd  +  σ_per_s² · Δt,
           so reported std-dev grows as √(distance, time).
      ~drift_pos_per_m      [m / sqrt(m)]   per-meter-of-travel pos drift
      ~drift_yaw_per_m      [rad / sqrt(m)] per-meter-of-travel yaw drift
      ~drift_pos_per_s      [m / sqrt(s)]   hover/bias pos drift
      ~drift_yaw_per_s      [rad / sqrt(s)] hover/bias yaw drift

Depth noise (~noise_depth_std, ~noise_depth_proportional) is
unrelated to localisation and unchanged.

TF is always published with ground-truth pose so RViz looks right;
only the topics fed into FALCON (/odom_world, /map_ros/pose) are
perturbed. That keeps the visualization a fair witness of how
much FALCON's *belief* deviates from reality.
"""

import rospy
import tf
import tf.transformations as tft
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image


class FalconAdapter:
    def __init__(self):
        rospy.init_node("falcon_adapter")

        # ── Parameters ──
        self.drone_ns    = rospy.get_param("~drone_ns", "/simple_drone")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.body_frame  = rospy.get_param("~body_frame", "body")
        self.cam_frame   = rospy.get_param("~cam_frame", "camera")
        self.odom_min_dt  = rospy.get_param("~odom_min_dt", 0.04)   # 25 Hz
        self.depth_min_dt = rospy.get_param("~depth_min_dt", 0.04)  # 25 Hz

        # Camera offset (must match your URDF/xacro)
        cam_x = rospy.get_param("~cam_offset_x", 0.2)
        cam_y = rospy.get_param("~cam_offset_y", 0.0)
        cam_z = rospy.get_param("~cam_offset_z", 0.0)

        # ── Noise parameters (all default 0 = off) ──
        # Jitter: i.i.d. Gaussian, fresh sample per callback.
        # (Names kept for backward compat with v10–v11.)
        self.noise_pos_std   = rospy.get_param("~noise_pos_std", 0.0)   # m   jitter
        self.noise_yaw_std   = rospy.get_param("~noise_yaw_std", 0.0)   # rad jitter
        self.noise_depth_std = rospy.get_param("~noise_depth_std", 0.0)
        self.noise_depth_proportional = rospy.get_param("~noise_depth_proportional", 0.0)

        # Drift: random walk on the world->body offset; ACCUMULATES.
        # Variance per unit distance traveled (the VIO-style term)
        self.drift_pos_per_m = rospy.get_param("~drift_pos_per_m", 0.0)  # m / sqrt(m)
        self.drift_yaw_per_m = rospy.get_param("~drift_yaw_per_m", 0.0)  # rad / sqrt(m)
        # Variance per unit time (gyro/bias drift while hovering)
        self.drift_pos_per_s = rospy.get_param("~drift_pos_per_s", 0.0)  # m / sqrt(s)
        self.drift_yaw_per_s = rospy.get_param("~drift_yaw_per_s", 0.0)  # rad / sqrt(s)

        self.pose_noise_enabled = (
            self.noise_pos_std   > 0 or self.noise_yaw_std   > 0 or
            self.drift_pos_per_m > 0 or self.drift_yaw_per_m > 0 or
            self.drift_pos_per_s > 0 or self.drift_yaw_per_s > 0
        )
        self.noise_enabled = (
            self.pose_noise_enabled or
            self.noise_depth_std > 0 or self.noise_depth_proportional > 0
        )

        # Drift state — accumulates from node start, never reset.
        self.drift_p          = np.zeros(3)   # accumulated position offset
        self.drift_yaw        = 0.0           # accumulated yaw offset (rad)
        self.drift_last_pos   = None
        self.drift_last_time  = None

        noise_seed = rospy.get_param("~noise_seed", -1)
        self.rng = np.random.RandomState(int(noise_seed) if noise_seed >= 0 else None)

        # ── Body-to-camera transform ──
        self.T_b_c = np.array([
            [ 0.0,  0.0, 1.0, cam_x],
            [-1.0,  0.0, 0.0, cam_y],
            [ 0.0, -1.0, 0.0, cam_z],
            [ 0.0,  0.0, 0.0, 1.0 ]
        ])
        self.T_b_c_quat  = tft.quaternion_from_matrix(self.T_b_c)
        self.T_b_c_trans  = (cam_x, cam_y, cam_z)

        # ── State ──
        self.cur_pose       = None
        self.prev_time      = None
        self.prev_depth_time = None
        self.vel            = np.zeros(3)

        # ── TF broadcaster ──
        self.tf_br = tf.TransformBroadcaster()

        # ── Publishers (to FALCON) ──
        self.odom_pub     = rospy.Publisher("/odom_world", Odometry, queue_size=10)
        self.pose_pub     = rospy.Publisher("/map_ros/pose", PoseStamped, queue_size=10)
        self.depth_pub    = rospy.Publisher("/map_ros/depth", Image, queue_size=2)
        self.cam_info_pub = rospy.Publisher("/map_ros/depth/camera_info", CameraInfo, queue_size=2)

        # ── Subscribers (from drone) ──
        rospy.Subscriber(self.drone_ns + "/gt_pose",
                         Pose, self.gt_pose_cb)
        rospy.Subscriber(self.drone_ns + "/front_depth/depth/image_raw",
                         Image, self.depth_cb)
        rospy.Subscriber(self.drone_ns + "/front_depth/depth/camera_info",
                         CameraInfo, self.cam_info_cb)

        # ── One-line banner ──
        if self.pose_noise_enabled:
            noise_desc = ("jitter(p=%.3g,yaw=%.3g) "
                          "drift(p/m=%.3g,yaw/m=%.3g,p/s=%.3g,yaw/s=%.3g)") % (
                self.noise_pos_std, self.noise_yaw_std,
                self.drift_pos_per_m, self.drift_yaw_per_m,
                self.drift_pos_per_s, self.drift_yaw_per_s)
        else:
            noise_desc = "off"
        rospy.loginfo(
            "falcon_adapter ready  drone=%s  odom=%.0fHz  depth=%.0fHz  pose_noise=%s",
            self.drone_ns, 1.0 / self.odom_min_dt, 1.0 / self.depth_min_dt,
            noise_desc)

    # ──────────────────────────────────────────────────────────
    #  Pose callback  (drone -> FALCON)
    # ──────────────────────────────────────────────────────────

    def gt_pose_cb(self, msg):
        now = rospy.Time.now()

        # Throttle rate
        if self.prev_time is not None:
            dt = (now - self.prev_time).to_sec()
            if dt < self.odom_min_dt:
                return
        else:
            dt = 0.0

        # Estimate velocity from consecutive poses
        if self.cur_pose is not None and dt > 1e-6:
            self.vel = np.array([
                (msg.position.x - self.cur_pose.position.x) / dt,
                (msg.position.y - self.cur_pose.position.y) / dt,
                (msg.position.z - self.cur_pose.position.z) / dt,
            ])
        self.prev_time = now
        self.cur_pose = msg

        # Choose what FALCON sees (noisy or clean).
        # IMPORTANT: drift must be advanced *before* the pose is built,
        # so that drift_p / drift_yaw reflect the latest random walk step.
        if self.pose_noise_enabled:
            self._update_drift(msg, now)
            falcon_pose = self._add_pose_noise(msg)
        else:
            falcon_pose = msg

        fp = falcon_pose.position
        fo = falcon_pose.orientation

        # 1. Odometry
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.world_frame
        odom.child_frame_id  = self.body_frame
        odom.pose.pose = falcon_pose
        odom.twist.twist.linear.x = self.vel[0]
        odom.twist.twist.linear.y = self.vel[1]
        odom.twist.twist.linear.z = self.vel[2]
        self.odom_pub.publish(odom)

        # 2. Camera-frame sensor pose  (T_w_c = T_w_b * T_b_c)
        T_w_b = tft.quaternion_matrix([fo.x, fo.y, fo.z, fo.w])
        T_w_b[0, 3], T_w_b[1, 3], T_w_b[2, 3] = fp.x, fp.y, fp.z
        T_w_c = T_w_b @ self.T_b_c

        cam_pos  = T_w_c[:3, 3]
        cam_quat = tft.quaternion_from_matrix(T_w_c)

        ps = PoseStamped()
        ps.header.stamp    = now
        ps.header.frame_id = self.world_frame
        ps.pose.position.x    = cam_pos[0]
        ps.pose.position.y    = cam_pos[1]
        ps.pose.position.z    = cam_pos[2]
        ps.pose.orientation.x = cam_quat[0]
        ps.pose.orientation.y = cam_quat[1]
        ps.pose.orientation.z = cam_quat[2]
        ps.pose.orientation.w = cam_quat[3]
        self.pose_pub.publish(ps)

        # 3. TF  (always ground truth so RViz is accurate)
        gt_p, gt_o = msg.position, msg.orientation
        self.tf_br.sendTransform(
            (gt_p.x, gt_p.y, gt_p.z),
            (gt_o.x, gt_o.y, gt_o.z, gt_o.w),
            now, self.body_frame, self.world_frame,
        )
        self.tf_br.sendTransform(
            self.T_b_c_trans, self.T_b_c_quat,
            now, self.cam_frame, self.body_frame,
        )

    # ──────────────────────────────────────────────────────────
    #  Depth callback
    # ──────────────────────────────────────────────────────────

    def depth_cb(self, msg):
        # Throttle to stay under sim rate
        now = rospy.Time.now()
        if self.prev_depth_time is not None:
            if (now - self.prev_depth_time).to_sec() < self.depth_min_dt:
                return
        self.prev_depth_time = now

        msg.header.stamp    = now
        msg.header.frame_id = self.cam_frame

        if self.noise_enabled and (self.noise_depth_std > 0
                                   or self.noise_depth_proportional > 0):
            msg = self._add_depth_noise(msg)

        self.depth_pub.publish(msg)

    # ──────────────────────────────────────────────────────────
    #  Camera info callback
    # ──────────────────────────────────────────────────────────

    def cam_info_cb(self, msg):
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = self.cam_frame
        self.cam_info_pub.publish(msg)

    # ──────────────────────────────────────────────────────────
    #  Noise helpers
    # ──────────────────────────────────────────────────────────

    def _update_drift(self, gt_pose, now):
        """Advance the accumulated drift offset by one tick.

        Uses a discrete random walk whose variance scales linearly with
        distance traveled and elapsed time, both of which are added in
        quadrature (independent noise sources).  After a trajectory of
        length D over time T, the resulting drift offset has std-dev

            σ_pos = sqrt(σ_per_m² · D + σ_per_s² · T)

        which is the standard √(distance) growth seen in real VIO/SLAM.
        """
        p = np.array([gt_pose.position.x,
                      gt_pose.position.y,
                      gt_pose.position.z])
        if self.drift_last_pos is None:
            self.drift_last_pos  = p
            self.drift_last_time = now
            return

        dd = float(np.linalg.norm(p - self.drift_last_pos))
        dt = max((now - self.drift_last_time).to_sec(), 0.0)
        self.drift_last_pos  = p
        self.drift_last_time = now

        var_pos = (self.drift_pos_per_m ** 2) * dd + (self.drift_pos_per_s ** 2) * dt
        var_yaw = (self.drift_yaw_per_m ** 2) * dd + (self.drift_yaw_per_s ** 2) * dt

        if var_pos > 0.0:
            self.drift_p   += self.rng.normal(0.0, np.sqrt(var_pos), 3)
        if var_yaw > 0.0:
            self.drift_yaw += self.rng.normal(0.0, np.sqrt(var_yaw))

    def _add_pose_noise(self, pose_msg):
        """Apply drift (accumulated) + jitter (i.i.d.) to a Pose.

            p_noisy   = p_gt   + drift_p   + N(0, noise_pos_std² · I)
            yaw_noisy = yaw_gt + drift_yaw + N(0, noise_yaw_std²)

        Roll/pitch are left clean — for an indoor drone they are
        well-stabilized by the IMU and are not what mapping cares about.
        """
        noisy = Pose()

        # Position: GT + accumulated drift offset + per-tick jitter
        noisy.position.x = pose_msg.position.x + self.drift_p[0]
        noisy.position.y = pose_msg.position.y + self.drift_p[1]
        noisy.position.z = pose_msg.position.z + self.drift_p[2]
        if self.noise_pos_std > 0:
            noisy.position.x += self.rng.normal(0.0, self.noise_pos_std)
            noisy.position.y += self.rng.normal(0.0, self.noise_pos_std)
            noisy.position.z += self.rng.normal(0.0, self.noise_pos_std)

        # Orientation: drift + jitter on yaw only
        q = pose_msg.orientation
        roll, pitch, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw += self.drift_yaw
        if self.noise_yaw_std > 0:
            yaw += self.rng.normal(0.0, self.noise_yaw_std)
        nq = tft.quaternion_from_euler(roll, pitch, yaw)
        noisy.orientation.x = nq[0]
        noisy.orientation.y = nq[1]
        noisy.orientation.z = nq[2]
        noisy.orientation.w = nq[3]
        return noisy

    def _add_depth_noise(self, depth_msg):
        """New Image with Gaussian noise on 32FC1 depth pixels."""
        if depth_msg.encoding != "32FC1":
            return depth_msg

        arr = np.frombuffer(depth_msg.data, dtype=np.float32).copy()
        arr = arr.reshape(depth_msg.height, depth_msg.width)
        valid = np.isfinite(arr) & (arr > 0)

        if self.noise_depth_std > 0:
            noise = self.rng.normal(0, self.noise_depth_std,
                                    arr.shape).astype(np.float32)
            arr[valid] += noise[valid]

        if self.noise_depth_proportional > 0:
            scale = self.rng.normal(1.0, self.noise_depth_proportional,
                                    arr.shape).astype(np.float32)
            arr[valid] *= scale[valid]

        np.maximum(arr, 0.0, out=arr)

        out = Image()
        out.header      = depth_msg.header
        out.height      = depth_msg.height
        out.width       = depth_msg.width
        out.encoding    = depth_msg.encoding
        out.is_bigendian = depth_msg.is_bigendian
        out.step        = depth_msg.step
        out.data        = arr.tobytes()
        return out


if __name__ == "__main__":
    try:
        FalconAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass