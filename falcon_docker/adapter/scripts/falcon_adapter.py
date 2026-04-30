#!/usr/bin/env python3
"""
falcon_adapter.py  (v14 — SE(3) drift composition + IMU-style noise)

Bridges drone topics to FALCON topics. That's it.

  Drone gt_pose      -> /odom_world  +  /map_ros/pose  +  TF
  Drone depth        -> /map_ros/depth
  Drone camera_info  -> /map_ros/depth/camera_info

You fly the drone manually. FALCON builds the map and plans
exploration from the pose + depth it receives.

Localisation noise model (all default 0 = clean). The model
emulates how a real IMU/VIO pipeline misperceives the world,
not just how a Gaussian smudges the truth. Three layers:

  (1) SE(3) DRIFT COMPOSITION
      Drift is a 4x4 transform T_drift accumulated tick by tick:
          T_belief = T_drift  ∘  T_gt
      Each tick adds a small body-frame increment (rotation + trans)
      whose statistics are configured per axis (x = forward, y = left,
      z = up, yaw). This couples yaw drift into position the way a
      real EKF does — if you accumulate 5° of yaw drift, then 10 m of
      forward GT flight produces ~87 cm of *implied* position error
      even with zero direct position drift.

      Per-axis knobs:
        ~drift_<ax>_mean_per_m   bias rate         [m/m or rad/m]
        ~drift_<ax>_std_per_m    random-walk rate  [m/√m or rad/√m]

  (2) PER-TICK JITTER (i.i.d. body-frame Gaussian)
      Sampled fresh every callback, applied as a body-frame offset
      that's rotated to world before being added.
        ~jitter_<ax>_mean   constant per-tick offset [m or rad]
        ~jitter_<ax>_std    per-tick std-dev          [m or rad]

  (3) SLOW TIME-VARYING IMU BIAS  (Ornstein–Uhlenbeck-style walk)
      Each axis has a bias state that itself slowly drifts. Models
      gyro/accelerometer bias dynamics — calibrated systems go bad
      over minutes even on similar trajectories.
        ~bias_<ax>_init     initial bias value           [m or rad]
        ~bias_<ax>_walk_per_s  bias random-walk std      [(m or rad)/√s]
      The bias is added to drift_<ax>_mean_per_m every tick, so a
      slowly-evolving bias becomes a slowly-evolving drift rate.

  (4) OUTLIER EVENTS (rare jumps; feature-loss / depth-dropout)
      Every tick, with probability `outlier_rate * dt`, inject a
      one-shot pose jump in body frame:
        ~outlier_rate_hz    expected events per second   [1/s]
        ~outlier_pos_std    jump magnitude (position)    [m]
        ~outlier_yaw_std    jump magnitude (yaw)         [rad]

TF is always published with ground-truth pose so RViz looks right;
only the topics fed into FALCON (/odom_world, /map_ros/pose) are
perturbed. Visualisation thus stays a fair witness of how much
FALCON's *belief* deviates from reality.

Depth noise (~noise_depth_std, ~noise_depth_proportional) is
unrelated to localisation and applied independently.
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

        # ── Localisation-noise parameters ─────────────────────────────────
        # See module docstring for the full model. All values default to 0
        # so a clean run is the default.
        # ─────────────────────────────────────────────────────────────────

        self.AXES     = ("x", "y", "z", "yaw")
        self.POS_AXES = ("x", "y", "z")

        # (1) Per-tick jitter (body frame): mean + std per axis
        self.jit_mean = {ax: rospy.get_param(f"~jitter_{ax}_mean", 0.0) for ax in self.AXES}
        self.jit_std  = {ax: rospy.get_param(f"~jitter_{ax}_std",  0.0) for ax in self.AXES}

        # (2) Drift increment statistics (per metre flown), per axis
        self.drift_mean_per_m = {ax: rospy.get_param(f"~drift_{ax}_mean_per_m", 0.0)
                                 for ax in self.AXES}
        self.drift_std_per_m  = {ax: rospy.get_param(f"~drift_{ax}_std_per_m",  0.0)
                                 for ax in self.AXES}

        # (3) Slow IMU-style bias: initial value + per-second random-walk std
        self.bias_init      = {ax: rospy.get_param(f"~bias_{ax}_init",        0.0)
                               for ax in self.AXES}
        self.bias_walk_per_s = {ax: rospy.get_param(f"~bias_{ax}_walk_per_s", 0.0)
                                for ax in self.AXES}
        self.bias = dict(self.bias_init)   # current bias state, evolves over time

        # (4) Outlier events
        self.outlier_rate_hz = rospy.get_param("~outlier_rate_hz", 0.0)
        self.outlier_pos_std = rospy.get_param("~outlier_pos_std", 0.0)
        self.outlier_yaw_std = rospy.get_param("~outlier_yaw_std", 0.0)

        # Depth noise (independent of localisation)
        self.noise_depth_std          = rospy.get_param("~noise_depth_std", 0.0)
        self.noise_depth_proportional = rospy.get_param("~noise_depth_proportional", 0.0)

        any_jit     = any(v != 0 for v in (*self.jit_mean.values(),
                                            *self.jit_std.values()))
        any_drift   = any(v != 0 for v in (*self.drift_mean_per_m.values(),
                                            *self.drift_std_per_m.values()))
        any_bias    = any(v != 0 for v in (*self.bias_init.values(),
                                            *self.bias_walk_per_s.values()))
        any_outlier = (self.outlier_rate_hz > 0 and
                       (self.outlier_pos_std > 0 or self.outlier_yaw_std > 0))
        self.pose_noise_enabled = any_jit or any_drift or any_bias or any_outlier
        self.noise_enabled = (
            self.pose_noise_enabled or
            self.noise_depth_std > 0 or self.noise_depth_proportional > 0
        )

        # ── Drift transform T_drift in SE(3): T_belief = T_drift @ T_gt ──
        # Identity at startup. Each tick is a small left-multiplied increment
        # built from the per-axis random-walk samples in the BODY frame.
        self.T_drift           = np.eye(4)
        self.drift_last_pos    = None
        self.drift_last_time   = None

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
        def _axes_summary(d, fmt="%.3g"):
            on = [f"{ax}={fmt % d[ax]}" for ax in self.AXES if d[ax] != 0]
            return "(" + ", ".join(on) + ")" if on else "off"

        if self.pose_noise_enabled:
            parts = [
                "jit_mean=%s"      % _axes_summary(self.jit_mean),
                "jit_std=%s"       % _axes_summary(self.jit_std),
                "drift_mean/m=%s"  % _axes_summary(self.drift_mean_per_m),
                "drift_std/m=%s"   % _axes_summary(self.drift_std_per_m),
            ]
            if any_bias:
                parts.append("bias_init=%s"   % _axes_summary(self.bias_init))
                parts.append("bias_walk/s=%s" % _axes_summary(self.bias_walk_per_s))
            if any_outlier:
                parts.append("outlier=%.2gHz(p=%.3g,yaw=%.3g)" % (
                    self.outlier_rate_hz, self.outlier_pos_std, self.outlier_yaw_std))
            noise_desc = " ".join(parts)
        else:
            noise_desc = "off"
        rospy.loginfo(
            "falcon_adapter ready  drone=%s  odom=%.0fHz  depth=%.0fHz  SE(3) pose_noise=%s",
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

    def _gt_pose_to_T(self, gt_pose):
        """Build a 4x4 SE(3) transform from a Pose message."""
        q = gt_pose.orientation
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0, 3] = gt_pose.position.x
        T[1, 3] = gt_pose.position.y
        T[2, 3] = gt_pose.position.z
        return T

    @staticmethod
    def _se3_from_xyz_yaw(x, y, z, yaw):
        """Build an SE(3) transform from a body-frame increment (translation
        + yaw rotation only). Roll/pitch increments are left at zero — see
        the docstring on _add_pose_noise for why."""
        T = tft.euler_matrix(0.0, 0.0, yaw)
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    def _update_drift(self, gt_pose, now):
        """Advance T_drift by one tick using a small body-frame SE(3) step.

        The accumulation rule is left-multiplicative:

            T_drift  ←  T_step  ∘  T_drift

        Each step is built from per-axis samples drawn in the body frame:

            step_<ax> = drift_<ax>_mean_per_m · Δd
                      + bias_<ax>(t)          · Δd       (slow IMU bias)
                      + N(0, std_per_m² · Δd)            (random walk)

        Plus, with rate `outlier_rate_hz`, an occasional one-shot jump of
        magnitude `outlier_pos_std` / `outlier_yaw_std`.

        The bias state itself does a slow random walk with scale
        `bias_<ax>_walk_per_s`, so even if the trajectory is identical
        across runs, the systematic error envelope changes over minutes
        — like a real IMU warming up or recalibrating in the background.

        Δd = distance flown since last tick (body-frame magnitude).
        Δt = time elapsed since last tick (for bias evolution / outliers).
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

        # ── Slow IMU-style bias evolution (always on, even when stationary) ──
        if dt > 0.0:
            for ax in self.AXES:
                walk = self.bias_walk_per_s[ax]
                if walk > 0.0:
                    self.bias[ax] += self.rng.normal(0.0, walk * np.sqrt(dt))

        # ── Per-tick body-frame step from drift + random walk + bias ──
        step_xyz = np.zeros(3)
        if dd > 0.0:
            for i, ax in enumerate(self.POS_AXES):
                step_xyz[i]  = (self.drift_mean_per_m[ax] + self.bias[ax]) * dd
                std = self.drift_std_per_m[ax]
                if std > 0.0:
                    step_xyz[i] += self.rng.normal(0.0, std * np.sqrt(dd))
            step_yaw  = (self.drift_mean_per_m["yaw"] + self.bias["yaw"]) * dd
            if self.drift_std_per_m["yaw"] > 0.0:
                step_yaw += self.rng.normal(0.0,
                                            self.drift_std_per_m["yaw"] * np.sqrt(dd))
        else:
            step_yaw = 0.0

        # ── Outlier injection (rare large body-frame jumps) ──
        if (self.outlier_rate_hz > 0.0 and dt > 0.0
                and self.rng.random_sample() < self.outlier_rate_hz * dt):
            if self.outlier_pos_std > 0.0:
                step_xyz += self.rng.normal(0.0, self.outlier_pos_std, 3)
            if self.outlier_yaw_std > 0.0:
                step_yaw += self.rng.normal(0.0, self.outlier_yaw_std)
            rospy.loginfo_throttle(5.0,
                "falcon_adapter: outlier event (pos_step=%.3fm yaw_step=%.3frad)",
                float(np.linalg.norm(step_xyz)), float(step_yaw))

        # ── Compose into the accumulated drift transform ──
        # Body-frame step → SE(3) increment, expressed in WORLD frame using
        # the GT body-to-world rotation. Then left-multiply T_drift.
        if step_xyz.any() or step_yaw != 0.0:
            T_gt   = self._gt_pose_to_T(gt_pose)
            T_step_body = self._se3_from_xyz_yaw(step_xyz[0], step_xyz[1],
                                                 step_xyz[2], step_yaw)
            # Conjugate the body-frame increment into the world frame:
            #   T_step_world = T_gt · T_step_body · T_gt⁻¹
            T_gt_inv = np.linalg.inv(T_gt)
            T_step_world = T_gt @ T_step_body @ T_gt_inv
            self.T_drift = T_step_world @ self.T_drift

    def _add_pose_noise(self, pose_msg):
        """Build the noisy Pose for FALCON.

        Composition order:
            1. T_belief = T_drift  ∘  T_gt        (SE(3) drift)
            2. add per-tick body-frame jitter to position + yaw (additive,
               small-angle, the standard high-frequency sensor-noise model)

        Roll/pitch are left at the GT values — for an indoor stabilised
        drone they're tightly observed by the IMU's accelerometer (gravity
        gives a direct measurement), and they're not what mapping cares
        about. If you want to perturb them, do it on `T_drift` directly.
        """
        # Step 1: SE(3) drift composition
        T_gt     = self._gt_pose_to_T(pose_msg)
        T_belief = self.T_drift @ T_gt

        belief_p = T_belief[:3, 3].copy()
        roll, pitch, yaw = tft.euler_from_matrix(T_belief)

        # Step 2: body-frame jitter
        body_jitter = np.zeros(3)
        for i, ax in enumerate(self.POS_AXES):
            m, s = self.jit_mean[ax], self.jit_std[ax]
            v = m
            if s > 0.0:
                v += self.rng.normal(0.0, s)
            body_jitter[i] = v

        if body_jitter.any():
            # Rotate the body-frame jitter into world using the BELIEF
            # orientation (consistent with what FALCON sees as its own frame)
            R_belief = T_belief[:3, :3]
            belief_p += R_belief @ body_jitter

        yaw += self.jit_mean["yaw"]
        if self.jit_std["yaw"] > 0.0:
            yaw += self.rng.normal(0.0, self.jit_std["yaw"])

        # Repack into Pose
        noisy = Pose()
        noisy.position.x = belief_p[0]
        noisy.position.y = belief_p[1]
        noisy.position.z = belief_p[2]
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