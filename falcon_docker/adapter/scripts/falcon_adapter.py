#!/usr/bin/env python3
"""
falcon_adapter.py  (v15 — body-frame dead-reckoning noise model)

Bridges drone topics to FALCON topics. That's it.

  Drone gt_pose      -> /odom_world  +  /map_ros/pose  +  TF
  Drone depth        -> /map_ros/depth

You fly the drone manually. FALCON builds the map and plans
exploration from the pose + depth it receives.

═══════════════════════════════════════════════════════════════════════
LOCALISATION NOISE MODEL
═══════════════════════════════════════════════════════════════════════

The model emulates what a real IMU/VIO pipeline without loop closure
would output: each tick, integrate a noisy body-frame motion estimate
into a self-propagating belief pose. No re-anchoring to GT, ever.

Concretely, at each callback:
  1. Read the *true* GT pose at this tick.
  2. Compute the body-frame increment since the last tick:
         ΔT_body  =  T_gt_prev⁻¹  ·  T_gt_now
     and decompose into (Δx_b, Δy_b, Δz_b, Δroll_b, Δpitch_b, Δyaw_b).
  3. Perturb each axis independently with three INDEPENDENT noise sources
     (every parameter defaults to 0 → clean run):

       a) SCALE-FACTOR DRIFT — error per unit of body-frame motion in
          that axis. Models IMU scale errors. The user's "I rotated 90°
          but actually 89°" is exactly this: drift_yaw_mean_per_rad=0.011
          gives ~1° of yaw error per radian of rotation.
            ~drift_<xyz>_mean_per_m   [m/m]    constant scale-factor bias
            ~drift_<xyz>_std_per_m    [m/√m]   random walk std
            ~drift_yaw_mean_per_rad   [rad/rad]  (NEW UNIT — see below)
            ~drift_yaw_std_per_rad    [rad/√rad]
          Old names drift_yaw_*_per_m are still read for back-compat and
          re-interpreted as per_rad with a one-time warning.

       b) TIME-BASED BIAS — drift per second, independent of motion.
          Models always-on gyro/accel bias. This is what makes the belief
          wander even when the drone is hovering perfectly still.
            ~bias_<ax>_per_s_mean     [m/s or rad/s]   constant bias rate
            ~bias_<ax>_per_s_std      [m/s/√s,...]     random-walk rate
          Old names bias_<ax>_init and bias_<ax>_walk_per_s also accepted.

       c) PER-TICK JITTER — i.i.d. Gaussian, applied to the published
          pose only (does NOT enter the integrated belief — models
          downstream measurement noise after the filter has already run).
            ~jitter_<ax>_mean   [m or rad]   constant per-tick offset
            ~jitter_<ax>_std    [m or rad]   per-tick std

  4. Build the noisy body-frame increment ΔT_body_noisy and propagate
     the belief:  T_belief ← T_belief · ΔT_body_noisy.

  5. Optionally inject a rare outlier (one-shot body-frame jump).

WHY THIS GIVES THE 89° → 90° BEHAVIOUR FOR FREE
───────────────────────────────────────────────
After a noisy yaw step, T_belief's rotation is wrong. The next forward
translation Δx_b > 0 is integrated through that wrong rotation, so it
moves the belief in the wrong world-frame direction. Position error
grows linearly with subsequent forward distance — no extra logic needed.
This is exactly how real dead-reckoning fails.

WHY YAW IS NOW PER_RAD INSTEAD OF PER_M
───────────────────────────────────────
A "rad of yaw error per metre flown" parameter cannot fire when the
drone yaws in place (e.g. during the mapping_scan), so a 360° spin
accumulated zero drift. Tying yaw drift to yaw rotation amount fixes
that, and is the physically meaningful scaling for gyro errors.

ROLL/PITCH are passed through cleanly — for an indoor stabilised
drone they're tightly observed by accelerometer gravity, so VIO
keeps them very accurate in practice. If you want to perturb them,
add the same pattern for roll/pitch.

Depth noise (~noise_depth_std, ~noise_depth_proportional) is unrelated
to localisation and applied independently.
"""

import rospy
import tf
import tf.transformations as tft
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


def _yaw_from_R(R):
    """Yaw (rad) from a 3x3 rotation matrix. Robust to small roll/pitch."""
    return float(np.arctan2(R[1, 0], R[0, 0]))


class FalconAdapter:
    def __init__(self):
        rospy.init_node("falcon_adapter")

        # ── Frames / rates ──
        self.drone_ns    = rospy.get_param("~drone_ns", "/simple_drone")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.body_frame  = rospy.get_param("~body_frame", "body")
        self.cam_frame   = rospy.get_param("~cam_frame", "camera")
        self.odom_min_dt  = rospy.get_param("~odom_min_dt", 0.04)
        self.depth_min_dt = rospy.get_param("~depth_min_dt", 0.04)

        cam_x = rospy.get_param("~cam_offset_x", 0.2)
        cam_y = rospy.get_param("~cam_offset_y", 0.0)
        cam_z = rospy.get_param("~cam_offset_z", 0.0)

        # ────────────────────────────────────────────────────────────
        # Noise parameters (per body axis)
        # ────────────────────────────────────────────────────────────
        self.AXES     = ("x", "y", "z", "yaw")
        self.POS_AXES = ("x", "y", "z")

        # Per-tick jitter (no memory; applied to published pose only)
        self.jit_mean = {a: rospy.get_param(f"~jitter_{a}_mean", 0.0) for a in self.AXES}
        self.jit_std  = {a: rospy.get_param(f"~jitter_{a}_std",  0.0) for a in self.AXES}

        # Scale-factor drift per unit body-frame motion in this axis.
        # x, y, z: per-m of body-x, body-y, body-z motion respectively.
        self.drift_mean_per_motion = {
            a: rospy.get_param(f"~drift_{a}_mean_per_m", 0.0) for a in self.POS_AXES
        }
        self.drift_std_per_motion = {
            a: rospy.get_param(f"~drift_{a}_std_per_m", 0.0) for a in self.POS_AXES
        }
        # Yaw: per radian of yaw rotation (NOT per metre — see docstring).
        # For back-compat, fall back to drift_yaw_*_per_m if per_rad isn't set.
        yaw_mean_per_rad = rospy.get_param("~drift_yaw_mean_per_rad", None)
        yaw_std_per_rad  = rospy.get_param("~drift_yaw_std_per_rad",  None)
        if yaw_mean_per_rad is None:
            yaw_mean_per_rad = rospy.get_param("~drift_yaw_mean_per_m", 0.0)
            if yaw_mean_per_rad != 0.0:
                rospy.logwarn(
                    "falcon_adapter: drift_yaw_mean_per_m is deprecated and being "
                    "re-interpreted as drift_yaw_mean_per_rad=%.4g. See docstring.",
                    yaw_mean_per_rad)
        if yaw_std_per_rad is None:
            yaw_std_per_rad = rospy.get_param("~drift_yaw_std_per_m", 0.0)
            if yaw_std_per_rad != 0.0:
                rospy.logwarn(
                    "falcon_adapter: drift_yaw_std_per_m is deprecated and being "
                    "re-interpreted as drift_yaw_std_per_rad=%.4g. See docstring.",
                    yaw_std_per_rad)
        self.drift_mean_per_motion["yaw"] = yaw_mean_per_rad
        self.drift_std_per_motion["yaw"]  = yaw_std_per_rad

        # Time-based bias (always-on, even when stationary).
        # New names take precedence; fall back to old bias_<ax>_init / bias_<ax>_walk_per_s.
        self.bias_per_s_mean = {}
        self.bias_per_s_std  = {}
        for a in self.AXES:
            m = rospy.get_param(f"~bias_{a}_per_s_mean", None)
            if m is None:
                m = rospy.get_param(f"~bias_{a}_init", 0.0)
            self.bias_per_s_mean[a] = m
            s = rospy.get_param(f"~bias_{a}_per_s_std", None)
            if s is None:
                s = rospy.get_param(f"~bias_{a}_walk_per_s", 0.0)
            self.bias_per_s_std[a] = s

        # Outliers (rare one-shot body-frame jumps)
        self.outlier_rate_hz = rospy.get_param("~outlier_rate_hz", 0.0)
        self.outlier_pos_std = rospy.get_param("~outlier_pos_std", 0.0)
        self.outlier_yaw_std = rospy.get_param("~outlier_yaw_std", 0.0)

        # Depth noise (independent of localisation)
        self.noise_depth_std          = rospy.get_param("~noise_depth_std", 0.0)
        self.noise_depth_proportional = rospy.get_param("~noise_depth_proportional", 0.0)

        any_jit     = any(v != 0 for v in (*self.jit_mean.values(), *self.jit_std.values()))
        any_drift   = any(v != 0 for v in (*self.drift_mean_per_motion.values(),
                                            *self.drift_std_per_motion.values()))
        any_bias    = any(v != 0 for v in (*self.bias_per_s_mean.values(),
                                            *self.bias_per_s_std.values()))
        any_outlier = (self.outlier_rate_hz > 0 and
                       (self.outlier_pos_std > 0 or self.outlier_yaw_std > 0))
        self.pose_noise_enabled = any_jit or any_drift or any_bias or any_outlier
        self.noise_enabled = (self.pose_noise_enabled or
                              self.noise_depth_std > 0 or
                              self.noise_depth_proportional > 0)

        # ────────────────────────────────────────────────────────────
        # Belief state — self-propagating, never re-anchored to GT.
        # ────────────────────────────────────────────────────────────
        self.T_belief    = None          # 4x4, set on first GT callback
        self.T_gt_prev   = None          # 4x4, last GT pose seen
        self.t_prev      = None          # last callback wall-clock time
        # Slowly-evolving bias state for each axis (rad/s or m/s).
        # Initialised to bias_per_s_mean and walks each tick.
        self.bias_state = dict(self.bias_per_s_mean)

        noise_seed = rospy.get_param("~noise_seed", -1)
        self.rng = np.random.RandomState(int(noise_seed) if noise_seed >= 0 else None)

        # Body-to-camera transform
        self.T_b_c = np.array([
            [ 0.0,  0.0, 1.0, cam_x],
            [-1.0,  0.0, 0.0, cam_y],
            [ 0.0, -1.0, 0.0, cam_z],
            [ 0.0,  0.0, 0.0, 1.0 ]
        ])
        self.T_b_c_quat  = tft.quaternion_from_matrix(self.T_b_c)
        self.T_b_c_trans = (cam_x, cam_y, cam_z)

        # Generic state
        self.cur_pose        = None
        self.prev_time       = None
        self.prev_depth_time = None
        self.vel             = np.zeros(3)

        self.tf_br = tf.TransformBroadcaster()

        # Publishers (to FALCON)
        self.odom_pub     = rospy.Publisher("/odom_world", Odometry, queue_size=10)
        self.pose_pub     = rospy.Publisher("/map_ros/pose", PoseStamped, queue_size=10)
        self.depth_pub    = rospy.Publisher("/map_ros/depth", Image, queue_size=2)

        # Subscribers (from drone)
        rospy.Subscriber(self.drone_ns + "/gt_pose", Pose, self.gt_pose_cb)
        rospy.Subscriber(self.drone_ns + "/front_depth/depth/image_raw",
                         Image, self.depth_cb)

        rospy.loginfo("falcon_adapter ready  drone=%s  pose_noise=%s",
                      self.drone_ns, self._summarize_noise())

    # ──────────────────────────────────────────────────────────
    # Pose callback (drone -> FALCON)
    # ──────────────────────────────────────────────────────────
    def gt_pose_cb(self, msg):
        now = rospy.Time.now()

        # Throttle
        if self.prev_time is not None:
            dt_throttle = (now - self.prev_time).to_sec()
            if dt_throttle < self.odom_min_dt:
                return
        self.prev_time = now

        # World-frame velocity, used in odom.twist
        if self.cur_pose is not None:
            dt_vel = max((now - getattr(self, "_last_vel_t", now)).to_sec(), 1e-3)
            self.vel = np.array([
                (msg.position.x - self.cur_pose.position.x) / dt_vel,
                (msg.position.y - self.cur_pose.position.y) / dt_vel,
                (msg.position.z - self.cur_pose.position.z) / dt_vel,
            ])
        self._last_vel_t = now
        self.cur_pose = msg

        if self.pose_noise_enabled:
            falcon_pose = self._propagate_belief_and_publish(msg, now)
        else:
            falcon_pose = msg

        fp = falcon_pose.position
        fo = falcon_pose.orientation

        # 1. Odometry (FALCON's pose belief)
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.world_frame
        odom.child_frame_id  = self.body_frame
        odom.pose.pose = falcon_pose
        odom.twist.twist.linear.x = self.vel[0]
        odom.twist.twist.linear.y = self.vel[1]
        odom.twist.twist.linear.z = self.vel[2]
        self.odom_pub.publish(odom)

        # 2. Camera-frame sensor pose (FALCON's mapping reference)
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

        # 3. TF (always GT, so RViz remains a fair witness)
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
    # Noise core: dead-reckoning belief propagation
    # ──────────────────────────────────────────────────────────
    def _propagate_belief_and_publish(self, gt_msg, now):
        """Advance T_belief by one tick using a noisy body-frame increment,
        then return the belief Pose to publish (with optional jitter on top).

        Math summary (per tick):
            ΔT_body  = T_gt_prev⁻¹ · T_gt_now            # truth in body frame
            (Δx_b, Δy_b, Δz_b, Δyaw_b) = decompose(ΔT_body)
            for each axis ax:
                noisy_<ax> = Δ<ax>
                          + drift_<ax>_mean_per_motion · |Δ<ax>|·sign(Δ<ax>)
                          + N(0, drift_<ax>_std_per_motion · √|Δ<ax>|)
                          + bias_state[<ax>] · dt
            ΔT_body_noisy = compose(noisy_*, ΔRPY_truth_for_roll_pitch)
            T_belief    ← T_belief · ΔT_body_noisy
        """
        T_gt = self._pose_to_T(gt_msg)

        # First call: belief == GT, nothing to integrate
        if self.T_belief is None:
            self.T_belief  = T_gt.copy()
            self.T_gt_prev = T_gt.copy()
            self.t_prev    = now
            return self._T_to_pose(T_gt)

        dt = max((now - self.t_prev).to_sec(), 0.0)
        self.t_prev = now

        # ── 1. Bias state random walk (always-on, motion-independent) ──
        for a in self.AXES:
            walk = self.bias_per_s_std[a]
            if walk > 0.0 and dt > 0.0:
                self.bias_state[a] += self.rng.normal(0.0, walk * np.sqrt(dt))

        # ── 2. True body-frame increment from GT ──
        T_gt_prev_inv = np.linalg.inv(self.T_gt_prev)
        dT_body = T_gt_prev_inv @ T_gt
        dx_b = dT_body[0, 3]
        dy_b = dT_body[1, 3]
        dz_b = dT_body[2, 3]
        # Roll/pitch/yaw of the increment. Using ZYX (yaw-pitch-roll) Euler.
        roll_b, pitch_b, dyaw_b = tft.euler_from_matrix(dT_body, axes='sxyz')

        self.T_gt_prev = T_gt.copy()

        # ── 3. Per-axis noisy increment in body frame ──
        true_vals = {"x": dx_b, "y": dy_b, "z": dz_b, "yaw": dyaw_b}
        noisy_vals = {}
        for a in self.AXES:
            true_d = true_vals[a]
            mag    = abs(true_d)

            # Scale-factor drift: per metre (xyz) or per radian (yaw) of motion
            # in this axis. Sign follows the motion so flying +x and -x both
            # produce drift of consistent sign (i.e. always overshoots, or
            # always undershoots, like a real scale error).
            mean_rate = self.drift_mean_per_motion[a]
            std_rate  = self.drift_std_per_motion[a]
            err = mean_rate * true_d            # ← signed, scales with motion
            if std_rate > 0.0 and mag > 0.0:
                err += self.rng.normal(0.0, std_rate * np.sqrt(mag))

            # Time-based bias (motion-independent)
            err += self.bias_state[a] * dt

            noisy_vals[a] = true_d + err

        # ── 4. Optional outlier (one-shot body-frame jump) ──
        if (self.outlier_rate_hz > 0.0 and dt > 0.0
                and self.rng.random_sample() < self.outlier_rate_hz * dt):
            if self.outlier_pos_std > 0.0:
                noisy_vals["x"] += self.rng.normal(0.0, self.outlier_pos_std)
                noisy_vals["y"] += self.rng.normal(0.0, self.outlier_pos_std)
                noisy_vals["z"] += self.rng.normal(0.0, self.outlier_pos_std)
            if self.outlier_yaw_std > 0.0:
                noisy_vals["yaw"] += self.rng.normal(0.0, self.outlier_yaw_std)
            rospy.loginfo_throttle(5.0, "falcon_adapter: outlier event")

        # ── 5. Build noisy body-frame increment ΔT_body_noisy ──
        # Roll/pitch are passed through cleanly (a stabilised drone observes
        # them from gravity; perturbing them is a separate modelling choice).
        dT_noisy = tft.euler_matrix(roll_b, pitch_b, noisy_vals["yaw"], axes='sxyz')
        dT_noisy[0, 3] = noisy_vals["x"]
        dT_noisy[1, 3] = noisy_vals["y"]
        dT_noisy[2, 3] = noisy_vals["z"]

        # ── 6. Propagate the belief — this is the dead-reckoning step ──
        self.T_belief = self.T_belief @ dT_noisy

        # ── 7. Per-tick jitter (no memory; published pose only) ──
        T_pub = self.T_belief.copy()
        if any(self.jit_std[a] > 0.0 or self.jit_mean[a] != 0.0 for a in self.AXES):
            # Body-frame jitter, applied as small SE(3) right-multiply
            jx = self.jit_mean["x"]
            jy = self.jit_mean["y"]
            jz = self.jit_mean["z"]
            jyaw = self.jit_mean["yaw"]
            if self.jit_std["x"] > 0.0:   jx   += self.rng.normal(0.0, self.jit_std["x"])
            if self.jit_std["y"] > 0.0:   jy   += self.rng.normal(0.0, self.jit_std["y"])
            if self.jit_std["z"] > 0.0:   jz   += self.rng.normal(0.0, self.jit_std["z"])
            if self.jit_std["yaw"] > 0.0: jyaw += self.rng.normal(0.0, self.jit_std["yaw"])
            J = tft.euler_matrix(0.0, 0.0, jyaw)
            J[0, 3], J[1, 3], J[2, 3] = jx, jy, jz
            T_pub = T_pub @ J

        return self._T_to_pose(T_pub)

    # ──────────────────────────────────────────────────────────
    # Depth
    # ──────────────────────────────────────────────────────────
    def depth_cb(self, msg):
        now = rospy.Time.now()
        if self.prev_depth_time is not None:
            if (now - self.prev_depth_time).to_sec() < self.depth_min_dt:
                return
        self.prev_depth_time = now

        msg.header.stamp    = now
        msg.header.frame_id = self.cam_frame

        if self.noise_depth_std > 0 or self.noise_depth_proportional > 0:
            msg = self._add_depth_noise(msg)
        self.depth_pub.publish(msg)

    # ──────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────
    @staticmethod
    def _pose_to_T(pose):
        q = pose.orientation
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        return T

    @staticmethod
    def _T_to_pose(T):
        p = Pose()
        p.position.x = float(T[0, 3])
        p.position.y = float(T[1, 3])
        p.position.z = float(T[2, 3])
        q = tft.quaternion_from_matrix(T)
        p.orientation.x = float(q[0])
        p.orientation.y = float(q[1])
        p.orientation.z = float(q[2])
        p.orientation.w = float(q[3])
        return p

    def _add_depth_noise(self, depth_msg):
        if depth_msg.encoding != "32FC1":
            return depth_msg
        arr = np.frombuffer(depth_msg.data, dtype=np.float32).copy()
        arr = arr.reshape(depth_msg.height, depth_msg.width)
        valid = np.isfinite(arr) & (arr > 0)
        if self.noise_depth_std > 0:
            noise = self.rng.normal(0, self.noise_depth_std, arr.shape).astype(np.float32)
            arr[valid] += noise[valid]
        if self.noise_depth_proportional > 0:
            scale = self.rng.normal(1.0, self.noise_depth_proportional, arr.shape).astype(np.float32)
            arr[valid] *= scale[valid]
        np.maximum(arr, 0.0, out=arr)
        out = Image()
        out.header       = depth_msg.header
        out.height       = depth_msg.height
        out.width        = depth_msg.width
        out.encoding     = depth_msg.encoding
        out.is_bigendian = depth_msg.is_bigendian
        out.step         = depth_msg.step
        out.data         = arr.tobytes()
        return out

    def _summarize_noise(self):
        if not self.pose_noise_enabled:
            return "off"
        def axes(d, fmt="%.3g"):
            on = [f"{a}={fmt % d[a]}" for a in self.AXES if d.get(a, 0) != 0]
            return "(" + ",".join(on) + ")" if on else "off"
        parts = [
            f"jit_mean={axes(self.jit_mean)}",
            f"jit_std={axes(self.jit_std)}",
            f"drift_mean/motion={axes(self.drift_mean_per_motion)}",
            f"drift_std/motion={axes(self.drift_std_per_motion)}",
            f"bias/s_mean={axes(self.bias_per_s_mean)}",
            f"bias/s_std={axes(self.bias_per_s_std)}",
        ]
        if self.outlier_rate_hz > 0:
            parts.append(f"outlier={self.outlier_rate_hz:.2g}Hz"
                         f"(p={self.outlier_pos_std:.3g},yaw={self.outlier_yaw_std:.3g})")
        return " ".join(parts)


if __name__ == "__main__":
    try:
        FalconAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass