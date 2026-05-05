#!/usr/bin/env python3
"""
navdp_offline.py
================
Run NavDP trajectory inference on pre-recorded drone footage.

  1. Loads RGB images + depth .npy files + poses from JSON.
  2. Shows each frame — click a pixel to set a goal.
  3. Sends RGB + depth + goal to the NavDP server.
  4. Draws the predicted trajectory on the image.

Usage
-----
  # Terminal 1: start the NavDP server
  python navdp_server.py --port 8888 --checkpoint ./checkpoints/navdp-cross-modal.ckpt

  # Terminal 2: run this tool
  python navdp_offline.py \\
      --rgb_dir   /path/to/rgb_rectified \\
      --depth_dir /path/to/depth_npy \\
      --poses     /path/to/estimated_trajectory.json \\
      --port 8888

Controls
--------
  Click        — set goal pixel, run NavDP inference
  D / →        — next frame
  A / ←        — previous frame
  Q / Esc      — quit
"""

import argparse
import io
import json
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import requests
from PIL import Image as PILImage


# ── globals for mouse callback ──────────────────────────────────
click_xy = None


def mouse_cb(event, x, y, flags, param):
    global click_xy
    if event == cv2.EVENT_LBUTTONDOWN:
        click_xy = (x, y)


# ── NavDP server helpers ────────────────────────────────────────
def navdp_reset(url: str, intrinsic: list) -> bool:
    try:
        r = requests.post(f"{url}/navigator_reset",
                          json={"intrinsic": intrinsic,
                                "stop_threshold": -999,
                                "batch_size": 1}, timeout=30)
        print(f"NavDP reset: {'OK' if r.status_code == 200 else 'FAIL'}")
        return r.status_code == 200
    except Exception as e:
        print(f"NavDP reset error: {e}")
        return False


def navdp_pointgoal(url: str, rgb: np.ndarray, depth: np.ndarray,
                    gx: float, gy: float) -> dict | None:
    """Send RGB + depth + pointgoal to NavDP, return result dict."""
    rgb_buf = io.BytesIO()
    PILImage.fromarray(rgb, "RGB").save(rgb_buf, format="PNG")
    rgb_buf.seek(0)

    d_clip = np.clip(depth, 0.0, 10.0)
    d_int = (d_clip * 10000).astype(np.uint16)
    d_buf = io.BytesIO()
    PILImage.fromarray(d_int, mode="I;16").save(d_buf, format="PNG")
    d_buf.seek(0)

    files = {"image": ("rgb.png", rgb_buf, "image/png"),
             "depth": ("depth.png", d_buf, "image/png")}
    data = {"goal_data": json.dumps({"goal_x": [gx], "goal_y": [gy]})}

    try:
        r = requests.post(f"{url}/pointgoal_step",
                          files=files, data=data, timeout=30)
        return r.json() if r.status_code == 200 else None
    except Exception as e:
        print(f"NavDP step error: {e}")
        return None


def pixel_to_pointgoal(px: int, py: int, depth: np.ndarray,
                       fx: float, cx: float):
    """Clicked pixel + depth → body-frame (forward, left) goal."""
    h, w = depth.shape
    patch = depth[max(0, py - 10):min(h, py + 10),
                  max(0, px - 10):min(w, px + 10)]
    valid = patch[(patch > 0.1) & (patch < 50.0)]
    d = float(np.median(valid)) if len(valid) > 0 else 3.0
    d = float(np.clip(d, 0.1, 10.0))

    gx = d                                     # forward = depth
    gy = float(-(px - cx) * d / fx)             # left
    return gx, gy, d


# ── pose / file loading ────────────────────────────────────────
def load_poses(json_path: str) -> dict:
    """
    Load poses from JSON.  Returns dict: frame_stem → (x, y, z, yaw).
    Tries common formats automatically.
    """
    with open(json_path) as f:
        data = json.load(f)

    poses = {}
    for entry in data:
        img_name = entry.get("image", "")
        stem = Path(img_name).stem

        p = entry.get("pose", entry)
        x   = float(p.get("x", 0))
        y   = float(p.get("y", 0))
        z   = float(p.get("z", 0))
        yaw = float(p.get("yaw", 0))
        poses[stem] = (x, y, z, yaw)

    print(f"Loaded {len(poses)} poses from {json_path}")
    if poses:
        k, v = next(iter(poses.items()))
        print(f"  sample: {k} → x={v[0]:.2f} y={v[1]:.2f} z={v[2]:.2f} yaw={v[3]:.2f}")
    return poses


def find_frames(rgb_dir: str, depth_dir: str, poses: dict) -> list[dict]:
    """Match RGB, depth, and poses by frame stem. Returns sorted list."""
    rgb_files = {Path(f).stem: os.path.join(rgb_dir, f)
                 for f in os.listdir(rgb_dir)
                 if f.lower().endswith(('.jpg', '.jpeg', '.png'))}

    depth_files = {Path(f).stem: os.path.join(depth_dir, f)
                   for f in os.listdir(depth_dir)
                   if f.endswith('.npy')}

    common = sorted(set(rgb_files) & set(depth_files))
    if not common:
        print("No matching RGB + depth frames found. Check paths.")
        sys.exit(1)

    frames = []
    for stem in common:
        frames.append({
            "stem": stem,
            "rgb_path": rgb_files[stem],
            "depth_path": depth_files[stem],
            "pose": poses.get(stem, (0, 0, 0, 0)),
        })

    n_with_pose = sum(1 for stem in common if stem in poses)
    print(f"Found {len(frames)} frames ({n_with_pose} with poses)")
    return frames


# ── trajectory visualisation ────────────────────────────────────
def draw_all_trajectories(img: np.ndarray, all_traj: np.ndarray,
                          all_values: np.ndarray, best_idx: int,
                          fx: float, fy: float, cx: float, cy: float,
                          alt: float = 1.0):
    """
    Draw all candidate trajectories (faded) and the best one (green).
    all_traj: (K, N, 3)  all_values: (K,)
    alt: flight altitude in metres (ground is alt metres below camera).
    """
    vis = img.copy()
    K = all_traj.shape[0]
    v_min, v_max = float(all_values.min()), float(all_values.max())
    v_range = max(v_max - v_min, 1e-6)

    # Draw candidate trajectories
    for k in range(K):
        traj_k = all_traj[k]
        norm_v = (float(all_values[k]) - v_min) / v_range
        if k == best_idx:
            colour, thickness = (0, 255, 0), 2
        else:
            colour = (int((1 - norm_v) * 255), 80, int(norm_v * 255))
            thickness = 1

        pts = []
        for i in range(traj_k.shape[0]):
            fwd, left = float(traj_k[i, 0]), float(traj_k[i, 1])
            if fwd < 0.05:
                continue
            u = int(cx - (left * fx / fwd))
            v = int(cy + (alt * fy / fwd))     # ground is alt metres below camera
            pts.append((u, v))

        for i in range(len(pts) - 1):
            cv2.line(vis, pts[i], pts[i + 1], colour, thickness)

        if k == best_idx:
            for i, p in enumerate(pts):
                cv2.circle(vis, p, 3, (0, 255, 0), -1)
                if i % 4 == 0:
                    cv2.putText(vis, str(i), (p[0] + 4, p[1] - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

    return vis


# ── main ────────────────────────────────────────────────────────
def main():
    global click_xy

    ap = argparse.ArgumentParser(description="Offline NavDP on recorded drone footage")
    ap.add_argument("--rgb_dir",   required=True)
    ap.add_argument("--depth_dir", required=True)
    ap.add_argument("--poses",     required=True, help="JSON with per-frame poses")
    ap.add_argument("--port",      type=int,   default=8888)
    ap.add_argument("--fx",        type=float, default=361.52)
    ap.add_argument("--fy",        type=float, default=410.76)
    ap.add_argument("--cx",        type=float, default=229.34)
    ap.add_argument("--cy",        type=float, default=116.76)
    ap.add_argument("--alt",       type=float, default=1.0,
                    help="Flight altitude in metres (default 1.0)")
    args = ap.parse_args()

    navdp_url = f"http://127.0.0.1:{args.port}"
    fx, fy, cx, cy = args.fx, args.fy, args.cx, args.cy
    intrinsic = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]

    # Load data
    poses  = load_poses(args.poses)
    frames = find_frames(args.rgb_dir, args.depth_dir, poses)

    # Reset NavDP
    if not navdp_reset(navdp_url, intrinsic):
        print("Cannot reach NavDP server. Is it running?")
        sys.exit(1)

    # UI
    cv2.namedWindow("NavDP Offline", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("NavDP Offline", mouse_cb)

    idx = 0
    result_vis = None

    print(f"\n{len(frames)} frames ready.")
    print("Click on the image to set a goal.  D/→ next, A/← prev, Q/Esc quit.\n")

    while True:
        f = frames[idx]

        # Load frame
        rgb_bgr = cv2.imread(f["rgb_path"])
        if rgb_bgr is None:
            print(f"Cannot read {f['rgb_path']}")
            idx = (idx + 1) % len(frames)
            continue

        depth = np.load(f["depth_path"]).astype(np.float32)
        pose = f["pose"]

        # Resize RGB to match depth resolution (e.g. 720x420 → 504x280)
        # so the depth intrinsics are valid for pixel clicks and projection
        dh, dw = depth.shape
        if rgb_bgr.shape[:2] != (dh, dw):
            rgb_bgr = cv2.resize(rgb_bgr, (dw, dh), interpolation=cv2.INTER_LINEAR)

        # Build display image
        if result_vis is not None and result_vis.shape[:2] == rgb_bgr.shape[:2]:
            display = result_vis.copy()
        else:
            display = rgb_bgr.copy()

        # Info bar
        cv2.putText(display,
                    f"[{idx}/{len(frames)-1}] {f['stem']}  "
                    f"pose=({pose[0]:.1f},{pose[1]:.1f},{pose[2]:.1f}) "
                    f"yaw={math.degrees(pose[3]):.0f}deg",
                    (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(display, "Click to set goal | D/A = next/prev | Q = quit",
                    (6, display.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

        cv2.imshow("NavDP Offline", display)

        # Handle input
        key = cv2.waitKey(30) & 0xFF

        if key in (ord('q'), 27):
            break
        elif key in (ord('d'), 83):   # D or →
            idx = min(idx + 1, len(frames) - 1)
            result_vis = None
            click_xy = None
        elif key in (ord('a'), 81):   # A or ←
            idx = max(idx - 1, 0)
            result_vis = None
            click_xy = None

        # Process click
        if click_xy is not None:
            px, py = click_xy
            click_xy = None

            print(f"  Click at pixel ({px}, {py})")

            # Convert pixel → pointgoal
            gx, gy, d = pixel_to_pointgoal(px, py, depth, fx, cx)
            print(f"  → pointgoal: fwd={gx:.2f}m  left={gy:.2f}m  depth={d:.2f}m")

            # Draw goal marker
            goal_vis = rgb_bgr.copy()
            cv2.circle(goal_vis, (px, py), 8, (0, 0, 255), 2)
            cv2.putText(goal_vis, f"goal ({gx:.1f},{gy:.1f}) d={d:.1f}m",
                        (px + 10, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            cv2.imshow("NavDP Offline", goal_vis)
            cv2.waitKey(1)

            # Send RGB (NavDP expects RGB, not BGR)
            rgb_for_navdp = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)

            result = navdp_pointgoal(navdp_url, rgb_for_navdp, depth, gx, gy)

            if result is None:
                print("  NavDP returned no result.")
                result_vis = goal_vis
                continue

            # Parse result
            exec_traj  = np.array(result["trajectory"])       # (1, N, 3)
            all_traj   = np.array(result["all_trajectory"])   # (1, K, N, 3)
            all_values = np.array(result["all_values"])       # (1, K)

            # Remove batch dimension
            exec_traj  = exec_traj[0]                         # (N, 3)
            all_traj   = all_traj[0]                          # (K, N, 3)
            all_values = all_values[0]                        # (K,)

            best_idx = int(np.argmax(all_values))

            print(f"  NavDP returned {all_traj.shape[0]} candidates, "
                  f"best #{best_idx} (value={all_values[best_idx]:.3f})")
            print(f"  Exec trajectory: {exec_traj.shape[0]} waypoints, "
                  f"max fwd={exec_traj[:,0].max():.2f}m")

            # Draw all trajectories
            result_vis = draw_all_trajectories(
                goal_vis, all_traj, all_values, best_idx,
                fx, fy, cx, cy, args.alt)

    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()