#!/usr/bin/env python3
import csv
import json
from pathlib import Path

import numpy as np


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def pose_to_c2w(tx, ty, tz, qx, qy, qz, qw):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    T[:3, 3] = [tx, ty, tz]
    return T


def invert_se3(T):
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


capture_dir = Path("/root/sjtu_project/recording")

with open(capture_dir / "camera_info.json", "r", encoding="utf-8") as f:
    cam = json.load(f)

K = np.array(cam["k"], dtype=np.float32).reshape(3, 3)

rows = []
with open(capture_dir / "frames.csv", "r", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    for row in reader:
        rows.append(row)

N = len(rows)
intrinsics = np.repeat(K[None, :, :], N, axis=0)

extrinsics = []
for row in rows:
    tx = float(row["tx"])
    ty = float(row["ty"])
    tz = float(row["tz"])
    qx = float(row["qx"])
    qy = float(row["qy"])
    qz = float(row["qz"])
    qw = float(row["qw"])

    c2w = pose_to_c2w(tx, ty, tz, qx, qy, qz, qw)
    w2c = invert_se3(c2w)
    extrinsics.append(w2c)

extrinsics = np.stack(extrinsics, axis=0).astype(np.float32)
np.save(capture_dir / "intrinsics.npy", intrinsics)
np.save(capture_dir / "extrinsics.npy", extrinsics)

print("Saved:")
print(capture_dir / "intrinsics.npy")
print(capture_dir / "extrinsics.npy")
