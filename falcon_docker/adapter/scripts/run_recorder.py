#!/usr/bin/env python3
"""
run_recorder.py — captures everything needed to evaluate one FALCON run.

Subscribes to:
  ~voxel_topic        FALCON occupancy PointCloud2  (default /sdf_map/occupancy_all)
  ~gt_topic           drone ground-truth Pose       (default /simple_drone/gt_pose)
  ~falcon_odom_topic  pose FALCON believes          (default /odom_world)

On Ctrl-C / shutdown, writes  <output_dir>/<run_name>/ :
    voxels.npy            Nx3 float32 — FALCON's final voxel cloud
    coverage.csv          t,n_voxels  — discovery curve
    trajectory_gt.csv     t,x,y,z,yaw_rad — true path
    trajectory_falcon.csv t,x,y,z,yaw_rad — what FALCON believed
    summary.json          {duration, path_length, final_voxels, noise_params}
"""

import json
import math
import os
import threading
from collections import deque

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# Parameter names captured into summary.json. Kept aligned with falcon_adapter v15.
NOISE_PARAM_NAMES = (
    # Per-tick jitter
    "jitter_x_mean",   "jitter_x_std",
    "jitter_y_mean",   "jitter_y_std",
    "jitter_z_mean",   "jitter_z_std",
    "jitter_yaw_mean", "jitter_yaw_std",
    # Scale-factor drift (per-axis body-frame motion)
    "drift_x_mean_per_m",     "drift_x_std_per_m",
    "drift_y_mean_per_m",     "drift_y_std_per_m",
    "drift_z_mean_per_m",     "drift_z_std_per_m",
    "drift_yaw_mean_per_rad", "drift_yaw_std_per_rad",
    # Time-based bias drift
    "bias_x_per_s_mean",   "bias_x_per_s_std",
    "bias_y_per_s_mean",   "bias_y_per_s_std",
    "bias_z_per_s_mean",   "bias_z_per_s_std",
    "bias_yaw_per_s_mean", "bias_yaw_per_s_std",
    # Outliers
    "outlier_rate_hz", "outlier_pos_std", "outlier_yaw_std",
    # Reproducibility
    "noise_seed",
)


class RunRecorder:
    def __init__(self):
        rospy.init_node("run_recorder")

        self.run_name    = rospy.get_param("~run_name", "unnamed")
        self.output_dir  = rospy.get_param("~output_dir", "/home/falcon/runs")
        self.voxel_topic = rospy.get_param("~voxel_topic", "/sdf_map/occupancy_all")
        self.gt_topic    = rospy.get_param("~gt_topic", "/simple_drone/gt_pose")
        self.falcon_odom_topic = rospy.get_param("~falcon_odom_topic", "/odom_world")

        self.run_dir = os.path.join(self.output_dir, self.run_name)
        os.makedirs(self.run_dir, exist_ok=True)

        self.lock = threading.Lock()
        self.t0   = None
        self.latest_voxel_msg = None
        self.coverage    = deque(maxlen=200000)
        self.gt_traj     = deque(maxlen=200000)
        self.falcon_traj = deque(maxlen=200000)
        self.gt_path_length = 0.0
        self.gt_last_xyz    = None

        rospy.Subscriber(self.voxel_topic, PointCloud2,  self._voxel_cb, queue_size=2)
        rospy.Subscriber(self.gt_topic, Pose,            self._gt_cb,    queue_size=50)
        rospy.Subscriber(self.falcon_odom_topic, Odometry, self._falcon_cb, queue_size=50)

        rospy.on_shutdown(self._dump)
        rospy.Timer(rospy.Duration(10.0), self._heartbeat)
        rospy.loginfo("recorder: run=%s out=%s", self.run_name, self.run_dir)

    def _now(self):
        t = rospy.Time.now().to_sec()
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def _voxel_cb(self, msg):
        t = self._now()
        with self.lock:
            self.latest_voxel_msg = msg
            self.coverage.append((t, msg.width * msg.height))

    def _gt_cb(self, msg):
        t = self._now()
        xyz = np.array([msg.position.x, msg.position.y, msg.position.z])
        yaw = _yaw_from_quat(msg.orientation)
        with self.lock:
            self.gt_traj.append((t, xyz[0], xyz[1], xyz[2], yaw))
            if self.gt_last_xyz is not None:
                self.gt_path_length += float(np.linalg.norm(xyz - self.gt_last_xyz))
            self.gt_last_xyz = xyz

    def _falcon_cb(self, msg):
        t = self._now()
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        with self.lock:
            self.falcon_traj.append((t, p.x, p.y, p.z, yaw))

    def _heartbeat(self, _):
        with self.lock:
            n_vox = self.coverage[-1][1] if self.coverage else 0
            n_gt  = len(self.gt_traj)
        rospy.loginfo_throttle(20.0,
            "recorder[%s]: voxels=%d  gt_samples=%d  path=%.1fm",
            self.run_name, n_vox, n_gt, self.gt_path_length)

    def _dump(self):
        rospy.loginfo("recorder: saving to %s ...", self.run_dir)
        with self.lock:
            if self.latest_voxel_msg is not None:
                pts = np.array(list(pc2.read_points(
                    self.latest_voxel_msg,
                    field_names=("x", "y", "z"), skip_nans=True)),
                    dtype=np.float32)
            else:
                rospy.logwarn("recorder: no voxel msgs received! saving empty.")
                pts = np.zeros((0, 3), dtype=np.float32)
            np.save(os.path.join(self.run_dir, "voxels.npy"), pts)

            cov = np.array(self.coverage) if self.coverage else np.zeros((0, 2))
            np.savetxt(os.path.join(self.run_dir, "coverage.csv"),
                       cov, delimiter=",", header="t_sec,n_voxels", comments="")

            gt = np.array(self.gt_traj)     if self.gt_traj     else np.zeros((0, 5))
            fa = np.array(self.falcon_traj) if self.falcon_traj else np.zeros((0, 5))
            np.savetxt(os.path.join(self.run_dir, "trajectory_gt.csv"),
                       gt, delimiter=",", header="t_sec,x,y,z,yaw_rad", comments="")
            np.savetxt(os.path.join(self.run_dir, "trajectory_falcon.csv"),
                       fa, delimiter=",", header="t_sec,x,y,z,yaw_rad", comments="")

            duration = float(cov[-1, 0]) if len(cov) else 0.0
            n_voxels = int(len(pts))
            summary = {
                "run_name": self.run_name,
                "duration_sec": duration,
                "path_length_m": float(self.gt_path_length),
                "final_voxels": n_voxels,
                "avg_discovery_voxels_per_sec":
                    (n_voxels / duration) if duration > 0 else 0.0,
                "noise": {
                    name: rospy.get_param("/falcon_adapter/" + name, 0.0)
                    for name in NOISE_PARAM_NAMES
                },
            }
            with open(os.path.join(self.run_dir, "summary.json"), "w") as f:
                json.dump(summary, f, indent=2)

        rospy.loginfo("recorder: %d voxels, %.1fs, %.1fm path → %s",
                      n_voxels, duration, self.gt_path_length, self.run_dir)


if __name__ == "__main__":
    try:
        RunRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass