#!/usr/bin/env python3
"""
compare_runs.py — offline analysis of FALCON exploration runs.

Reads a set of run directories produced by run_recorder.py and computes
mapping-quality metrics relative to a reference run (default: 'clean').

Usage:
    python3 compare_runs.py runs/clean runs/drift runs/jitter runs/both \\
            --reference clean --resolution 0.15 --output report/

Produces in report/:
    coverage.png        — voxels-mapped vs time, all runs overlaid
    discovery_pct.png   — % of reference voxels mapped over time
    trajectories.png    — GT vs FALCON-believed paths per run
    drift_error.png     — ‖p_falcon − p_gt‖ over time, per run
    yaw_error.png       — yaw drift over time per run (when yaw is recorded)
    pose_error_decomp.png — translation vs rotation error side by side
    map_quality.png     — bar chart of P / R / F1 / IoU + Chamfer per run
    map_quality_aligned.png — same, but after rigidly aligning each run's
                          voxels to the reference (separates "wrong frame"
                          from "wrong geometry" — drift-tolerant metric)
    voxel_diff.png      — top-down TP / FP / FN voxel scatter per run
    metrics.csv         — full table of all metrics
    summary.txt         — printable text report

Pure numpy/scipy/matplotlib. No ROS or open3d needed.

The recorder writes trajectory CSVs with 5 columns (t,x,y,z,yaw_rad).
This script also handles the older 4-column format (yaw plots are
skipped if yaw isn't present).
"""
import argparse
import json
import os

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree


# ──────────────────────────────────────────────────────────────────
# I/O
# ──────────────────────────────────────────────────────────────────

def _load_traj(path):
    """Load a trajectory CSV. Returns (xyz_array, yaw_array_or_None).
    Accepts both legacy 4-column (t,x,y,z) and new 5-column (t,x,y,z,yaw) files.
    """
    arr = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if arr.shape[1] >= 5:
        return arr[:, :4], arr[:, 4]
    elif arr.shape[1] == 4:
        return arr, None
    else:
        # empty
        return np.zeros((0, 4)), None


def load_run(path):
    name = os.path.basename(os.path.normpath(path))
    voxels   = np.load(os.path.join(path, "voxels.npy"))
    coverage = np.loadtxt(os.path.join(path, "coverage.csv"),
                          delimiter=",", skiprows=1, ndmin=2)
    gt_xyz, gt_yaw = _load_traj(os.path.join(path, "trajectory_gt.csv"))
    fa_xyz, fa_yaw = _load_traj(os.path.join(path, "trajectory_falcon.csv"))
    with open(os.path.join(path, "summary.json")) as f:
        summary = json.load(f)
    return {"name": name, "voxels": voxels, "coverage": coverage,
            "gt": gt_xyz, "gt_yaw": gt_yaw,
            "falcon": fa_xyz, "falcon_yaw": fa_yaw,
            "summary": summary}


# ──────────────────────────────────────────────────────────────────
# Metrics
# ──────────────────────────────────────────────────────────────────

def voxelize(points, res):
    """Map 3-D points to integer voxel indices and return as a set."""
    if len(points) == 0:
        return set()
    idx = np.floor(points / res).astype(np.int64)
    return set(map(tuple, idx))


def precision_recall_iou(test, ref):
    """Set-based metrics on voxelized maps. test/ref are sets of voxel keys."""
    if not ref:
        return dict(precision=0, recall=0, f1=0, iou=0, tp=0, fp=0, fn=0)
    tp = len(test & ref)
    fp = len(test - ref)
    fn = len(ref - test)
    p = tp / (tp + fp) if (tp + fp) else 0.0
    r = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = 2 * p * r / (p + r) if (p + r) else 0.0
    iou = tp / (tp + fp + fn) if (tp + fp + fn) else 0.0
    return dict(precision=p, recall=r, f1=f1, iou=iou, tp=tp, fp=fp, fn=fn)


def chamfer(test_pts, ref_pts):
    """Symmetric mean nearest-neighbour distance (m). 0 = identical."""
    if len(test_pts) == 0 or len(ref_pts) == 0:
        return float("inf")
    d1, _ = cKDTree(ref_pts).query(test_pts)
    d2, _ = cKDTree(test_pts).query(ref_pts)
    return 0.5 * (float(d1.mean()) + float(d2.mean()))


def trajectory_drift(gt, fa):
    """For each falcon timestamp, find nearest gt timestamp and report ‖Δp‖."""
    if len(gt) == 0 or len(fa) == 0:
        return np.zeros((0, 2))
    # Match by time, since they're recorded asynchronously
    gt_t  = gt[:, 0]
    fa_t  = fa[:, 0]
    idx = np.searchsorted(gt_t, fa_t).clip(0, len(gt_t) - 1)
    err = np.linalg.norm(fa[:, 1:4] - gt[idx, 1:4], axis=1)
    return np.column_stack([fa_t, err])


def yaw_error(gt, gt_yaw, fa, fa_yaw):
    """Per-falcon-tick yaw error in radians, wrapped to (-π, π]."""
    if (gt_yaw is None or fa_yaw is None
            or len(gt_yaw) == 0 or len(fa_yaw) == 0):
        return np.zeros((0, 2))
    idx = np.searchsorted(gt[:, 0], fa[:, 0]).clip(0, len(gt) - 1)
    dy  = (fa_yaw - gt_yaw[idx] + np.pi) % (2 * np.pi) - np.pi
    return np.column_stack([fa[:, 0], dy])


def horn_align(src, dst, sample=20000):
    """Closed-form least-squares rigid alignment (Horn's method): find
    rotation R and translation t that minimize ‖R·src + t − dst‖². Used
    to separate "wrong frame" from "wrong geometry" — if a noisy run's
    map is just rotated/translated, alignment recovers the true match.

    Subsamples both clouds for speed (rigid alignment is well-conditioned
    even with a few thousand point pairs).

    Returns: (R 3x3, t 3, src_aligned).
    """
    if len(src) == 0 or len(dst) == 0:
        return np.eye(3), np.zeros(3), src

    rng = np.random.default_rng(0)
    if len(src) > sample:
        src_s = src[rng.choice(len(src), sample, replace=False)]
    else:
        src_s = src
    if len(dst) > sample:
        dst_s = dst[rng.choice(len(dst), sample, replace=False)]
    else:
        dst_s = dst

    # For each source point, find the nearest dst point (one-pass ICP)
    tree = cKDTree(dst_s)
    _, nn = tree.query(src_s)
    pairs_src = src_s
    pairs_dst = dst_s[nn]

    mu_s = pairs_src.mean(axis=0)
    mu_d = pairs_dst.mean(axis=0)
    H = (pairs_src - mu_s).T @ (pairs_dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = mu_d - R @ mu_s
    return R, t, (R @ src.T).T + t


# ──────────────────────────────────────────────────────────────────
# Plots
# ──────────────────────────────────────────────────────────────────

def plot_coverage(runs, out_path):
    fig, ax = plt.subplots(figsize=(8, 5))
    for r in runs:
        c = r["coverage"]
        if len(c):
            ax.plot(c[:, 0], c[:, 1], lw=1.8, label=r["name"])
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Voxels mapped")
    ax.set_title("FALCON discovery curve"); ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)


def plot_discovery_pct(runs, ref, out_path):
    """Coverage as % of the reference run's final voxel count."""
    target = ref["summary"]["final_voxels"] or 1
    fig, ax = plt.subplots(figsize=(8, 5))
    for r in runs:
        c = r["coverage"]
        if len(c):
            ax.plot(c[:, 0], 100.0 * c[:, 1] / target, lw=1.8, label=r["name"])
    ax.set_xlabel("Time (s)"); ax.set_ylabel("% of reference voxel count")
    ax.set_title(f"Coverage progress (100% = {ref['name']} final = {target} voxels)")
    ax.legend(); ax.grid(alpha=0.3); ax.set_ylim(0, 130)
    fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)


def plot_trajectories(runs, out_path):
    n = len(runs)
    fig, axes = plt.subplots(1, n, figsize=(4.2 * n, 4.2), squeeze=False)
    for ax, r in zip(axes[0], runs):
        gt, fa = r["gt"], r["falcon"]
        if len(gt):
            ax.plot(gt[:, 1], gt[:, 2], "k-",  lw=1.8, label="ground truth")
        if len(fa):
            ax.plot(fa[:, 1], fa[:, 2], "r--", lw=1.0, label="FALCON belief")
        ax.set_title(r["name"]); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=0.3)
    fig.suptitle("Trajectory: ground truth vs FALCON's belief")
    fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)


def plot_drift_error(runs, out_path):
    fig, ax = plt.subplots(figsize=(8, 5))
    for r in runs:
        e = trajectory_drift(r["gt"], r["falcon"])
        if len(e):
            ax.plot(e[:, 0], e[:, 1], lw=1.5, label=r["name"])
    ax.set_xlabel("Time (s)"); ax.set_ylabel("‖p_falcon − p_gt‖  (m)")
    ax.set_title("Translation error (FALCON belief vs ground truth)")
    ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)


def plot_yaw_error(runs, out_path):
    """Yaw drift over time. Skips runs where yaw isn't recorded."""
    fig, ax = plt.subplots(figsize=(8, 5))
    any_data = False
    for r in runs:
        e = yaw_error(r["gt"], r["gt_yaw"], r["falcon"], r["falcon_yaw"])
        if len(e):
            ax.plot(e[:, 0], np.degrees(e[:, 1]), lw=1.5, label=r["name"])
            any_data = True
    if not any_data:
        ax.text(0.5, 0.5, "no yaw data recorded\n(legacy 4-column trajectory CSVs)",
                ha="center", va="center", transform=ax.transAxes,
                fontsize=11, color="gray")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("yaw_falcon − yaw_gt  (deg)")
    ax.set_title("Yaw error (FALCON belief vs ground truth)")
    ax.axhline(0, color="k", lw=0.5, alpha=0.5)
    if any_data:
        ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)


def plot_pose_error_decomp(runs, out_path):
    """Final translation vs rotation error per run, side by side. Quick read
    on which error mode dominates each noise configuration."""
    names = [r["name"] for r in runs]
    final_pos = []
    final_yaw = []
    for r in runs:
        et = trajectory_drift(r["gt"], r["falcon"])
        ey = yaw_error(r["gt"], r["gt_yaw"], r["falcon"], r["falcon_yaw"])
        final_pos.append(float(et[-1, 1]) if len(et) else 0.0)
        final_yaw.append(float(np.abs(ey[-1, 1])) if len(ey) else 0.0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))
    x = np.arange(len(names))

    bs1 = ax1.bar(x, final_pos, color="tab:blue")
    for b, v in zip(bs1, final_pos):
        ax1.text(b.get_x() + b.get_width()/2, v + 0.005,
                 f"{v:.2f} m", ha="center", va="bottom", fontsize=9)
    ax1.set_xticks(x); ax1.set_xticklabels(names)
    ax1.set_ylabel("‖Δp‖ at end of run (m)")
    ax1.set_title("Final translation error")
    ax1.grid(alpha=0.3, axis="y")

    bs2 = ax2.bar(x, np.degrees(final_yaw), color="tab:orange")
    for b, v in zip(bs2, np.degrees(final_yaw)):
        ax2.text(b.get_x() + b.get_width()/2, v + 0.05,
                 f"{v:.1f}°", ha="center", va="bottom", fontsize=9)
    ax2.set_xticks(x); ax2.set_xticklabels(names)
    ax2.set_ylabel("|Δyaw| at end of run (deg)")
    ax2.set_title("Final yaw error")
    ax2.grid(alpha=0.3, axis="y")

    fig.suptitle("Pose error decomposition (final values)")
    fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)


def plot_map_quality(rows, ref_name, out_path, title_suffix=""):
    """Grouped bar chart of P / R / F1 / IoU per run, plus chamfer below."""
    names   = [r["name"] for r in rows]
    P       = [r["metrics"]["precision"] for r in rows]
    R       = [r["metrics"]["recall"]    for r in rows]
    F1      = [r["metrics"]["f1"]        for r in rows]
    IoU     = [r["metrics"]["iou"]       for r in rows]
    chamfer = [r["chamfer"]              for r in rows]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7),
                                   gridspec_kw={"height_ratios": [3, 1.5]})

    x = np.arange(len(names))
    w = 0.20
    bars = [
        ("Precision", P,   "tab:blue"),
        ("Recall",    R,   "tab:orange"),
        ("F1",        F1,  "tab:green"),
        ("IoU",       IoU, "tab:red"),
    ]
    for i, (label, vals, color) in enumerate(bars):
        offset = (i - 1.5) * w
        bs = ax1.bar(x + offset, vals, w, label=label, color=color)
        for b, v in zip(bs, vals):
            ax1.text(b.get_x() + b.get_width()/2, v + 0.01,
                     f"{v:.2f}", ha="center", va="bottom", fontsize=8)
    ax1.set_xticks(x); ax1.set_xticklabels(names)
    ax1.set_ylabel("Score (1.0 = perfect match)")
    ax1.set_ylim(0, 1.10)
    title = f"Voxel-map quality vs reference run '{ref_name}'"
    if title_suffix:
        title += f"  [{title_suffix}]"
    ax1.set_title(title)
    ax1.legend(ncol=4, loc="upper right", fontsize=9)
    ax1.grid(alpha=0.3, axis="y")

    bs = ax2.bar(x, [c * 100 for c in chamfer], color="tab:purple")
    for b, v in zip(bs, chamfer):
        ax2.text(b.get_x() + b.get_width()/2, v * 100 + 0.2,
                 f"{v*100:.1f} cm", ha="center", va="bottom", fontsize=9)
    ax2.set_xticks(x); ax2.set_xticklabels(names)
    ax2.set_ylabel("Chamfer (cm)")
    ax2.set_title("Continuous-space mean nearest-neighbour distance "
                  "(0 = identical clouds)")
    ax2.grid(alpha=0.3, axis="y")

    fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)


def plot_voxel_diff(runs, ref, resolution, out_path):
    """Top-down view of TP/FP/FN voxels per run, projected to xy.

    Green   = true positives  (test ∩ ref)   — correctly mapped
    Red     = false positives (test − ref)   — phantom voxels (drift artefacts)
    Blue    = false negatives (ref − test)   — missed voxels
    """
    ref_set = voxelize(ref["voxels"], resolution)
    n = len(runs)
    fig, axes = plt.subplots(1, n, figsize=(4.5 * n, 4.5), squeeze=False)

    for ax, r in zip(axes[0], runs):
        test_set = voxelize(r["voxels"], resolution)
        tp = test_set & ref_set
        fp = test_set - ref_set
        fn = ref_set  - test_set

        def to_xy(s):
            if not s:
                return np.zeros((0, 2))
            arr = np.asarray(list(s))[:, :2] * resolution
            return arr

        # Plot order: TP first (background), then errors on top
        for pts, color, label, marker in [
            (to_xy(tp), "lightgray", f"TP ({len(tp)})", "."),
            (to_xy(fn), "tab:blue",  f"FN ({len(fn)})", "."),
            (to_xy(fp), "tab:red",   f"FP ({len(fp)})", "."),
        ]:
            if len(pts):
                ax.scatter(pts[:, 0], pts[:, 1], s=1.0, c=color,
                           marker=marker, label=label, alpha=0.7)

        ax.set_title(r["name"])
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.legend(loc="upper right", fontsize=8, markerscale=8)
        ax.grid(alpha=0.3)

    fig.suptitle(f"Voxel-set agreement with '{ref['name']}' "
                 f"(top-down, {resolution}m voxels)")
    fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)


# ──────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────

DEFAULT_RUNS_DIR  = "runs"
DEFAULT_RUN_NAMES = ("clean", "drift", "jitter", "both")


def main():
    ap = argparse.ArgumentParser(
        description="Compare FALCON runs (precision/recall/F1/IoU/chamfer + plots).")
    ap.add_argument("runs", nargs="*",
                    help="Run directories. If omitted, auto-discovers "
                         f"{DEFAULT_RUN_NAMES} under ./{DEFAULT_RUNS_DIR}/.")
    ap.add_argument("--reference", default=None,
                    help="run name to use as ground-truth (default: 'clean' or first)")
    ap.add_argument("--resolution", type=float, default=0.15,
                    help="voxel size in metres for set-based metrics")
    ap.add_argument("--output", default="report")
    args = ap.parse_args()

    # Auto-discover runs if none were given.
    if not args.runs:
        candidates = [os.path.join(DEFAULT_RUNS_DIR, name)
                      for name in DEFAULT_RUN_NAMES]
        args.runs = [p for p in candidates if os.path.isdir(p)]
        if not args.runs:
            ap.error(f"no run directories given and none of "
                     f"{DEFAULT_RUN_NAMES} found under ./{DEFAULT_RUNS_DIR}/")
        print(f"[auto] using runs: {', '.join(args.runs)}")

    os.makedirs(args.output, exist_ok=True)
    runs = [load_run(p) for p in args.runs]
    by_name = {r["name"]: r for r in runs}
    ref_name = args.reference or ("clean" if "clean" in by_name else runs[0]["name"])
    ref = by_name[ref_name]
    ref_vox = voxelize(ref["voxels"], args.resolution)

    # ── metrics table ──
    header = ("run,n_voxels,duration_s,path_m,"
              "precision,recall,F1,IoU,chamfer_m,FP_voxels,FN_voxels,"
              "final_drift_m,final_yaw_err_deg,"
              "precision_aligned,recall_aligned,F1_aligned,IoU_aligned,chamfer_aligned_m,"
              "align_translation_m,align_rotation_deg")
    lines = [header]
    text  = []
    rows         = []   # raw metrics — for plot_map_quality
    rows_aligned = []   # post-rigid-alignment metrics — for plot_map_quality_aligned
    for r in runs:
        # ── Raw (un-aligned) metrics ──
        v   = voxelize(r["voxels"], args.resolution)
        m   = precision_recall_iou(v, ref_vox)
        cd  = chamfer(r["voxels"], ref["voxels"])
        e   = trajectory_drift(r["gt"], r["falcon"])
        ey  = yaw_error(r["gt"], r["gt_yaw"], r["falcon"], r["falcon_yaw"])
        final_drift = float(e[-1, 1]) if len(e) else 0.0
        final_yaw   = float(np.abs(ey[-1, 1])) if len(ey) else 0.0

        # ── Drift-tolerant metrics: rigidly align this run's voxels to the
        # reference, then re-score. If alignment fixes things, the map is
        # internally consistent — just expressed in the wrong frame. ──
        if r["name"] == ref_name:
            v_aligned = v
            m_aligned, cd_aligned = m, cd
            tx_align, rot_align = 0.0, 0.0
        else:
            R_a, t_a, voxels_aligned = horn_align(r["voxels"], ref["voxels"])
            v_aligned     = voxelize(voxels_aligned, args.resolution)
            m_aligned     = precision_recall_iou(v_aligned, ref_vox)
            cd_aligned    = chamfer(voxels_aligned, ref["voxels"])
            tx_align      = float(np.linalg.norm(t_a))
            # rotation magnitude from R: angle = arccos((trace(R) - 1) / 2)
            tr            = np.clip((np.trace(R_a) - 1.0) / 2.0, -1.0, 1.0)
            rot_align     = float(np.degrees(np.arccos(tr)))

        s = r["summary"]
        rows.append(        {"name": r["name"], "metrics": m,         "chamfer": cd})
        rows_aligned.append({"name": r["name"], "metrics": m_aligned, "chamfer": cd_aligned})

        lines.append(
            f"{r['name']},{s['final_voxels']},{s['duration_sec']:.1f},"
            f"{s['path_length_m']:.1f},"
            f"{m['precision']:.4f},{m['recall']:.4f},"
            f"{m['f1']:.4f},{m['iou']:.4f},{cd:.4f},{m['fp']},{m['fn']},"
            f"{final_drift:.3f},{np.degrees(final_yaw):.2f},"
            f"{m_aligned['precision']:.4f},{m_aligned['recall']:.4f},"
            f"{m_aligned['f1']:.4f},{m_aligned['iou']:.4f},{cd_aligned:.4f},"
            f"{tx_align:.3f},{rot_align:.2f}")
        text.append(
            f"  {r['name']:<10} voxels={s['final_voxels']:>6}  "
            f"dur={s['duration_sec']:>6.1f}s  path={s['path_length_m']:>6.1f}m\n"
            f"             raw:     P={m['precision']:.3f} R={m['recall']:.3f} "
            f"F1={m['f1']:.3f} IoU={m['iou']:.3f}  chamfer={cd*100:.1f}cm\n"
            f"             aligned: P={m_aligned['precision']:.3f} R={m_aligned['recall']:.3f} "
            f"F1={m_aligned['f1']:.3f} IoU={m_aligned['iou']:.3f}  chamfer={cd_aligned*100:.1f}cm  "
            f"(shift={tx_align*100:.1f}cm, rot={rot_align:.1f}°)\n"
            f"             pose:    drift={final_drift:.2f}m  "
            f"yaw_err={np.degrees(final_yaw):.1f}°")

    with open(os.path.join(args.output, "metrics.csv"), "w") as f:
        f.write("\n".join(lines) + "\n")

    # ── plots ──
    plot_coverage         (runs,           os.path.join(args.output, "coverage.png"))
    plot_discovery_pct    (runs, ref,      os.path.join(args.output, "discovery_pct.png"))
    plot_trajectories     (runs,           os.path.join(args.output, "trajectories.png"))
    plot_drift_error      (runs,           os.path.join(args.output, "drift_error.png"))
    plot_yaw_error        (runs,           os.path.join(args.output, "yaw_error.png"))
    plot_pose_error_decomp(runs,           os.path.join(args.output, "pose_error_decomp.png"))
    plot_map_quality      (rows, ref_name, os.path.join(args.output, "map_quality.png"))
    plot_map_quality      (rows_aligned, ref_name,
                           os.path.join(args.output, "map_quality_aligned.png"),
                           title_suffix="rigidly aligned to reference")
    plot_voxel_diff       (runs, ref, args.resolution,
                           os.path.join(args.output, "voxel_diff.png"))

    # ── text summary ──
    summary = (
        f"FALCON run comparison\n"
        f"=====================\n"
        f"Reference run : {ref_name}\n"
        f"Voxel size    : {args.resolution} m\n"
        f"Runs          : {', '.join(r['name'] for r in runs)}\n\n"
        + "\n".join(text) + "\n\n"
        f"Wrote: {os.path.abspath(args.output)}/\n")
    with open(os.path.join(args.output, "summary.txt"), "w") as f:
        f.write(summary)
    print(summary)


if __name__ == "__main__":
    main()