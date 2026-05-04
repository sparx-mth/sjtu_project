#!/usr/bin/env python3
"""
batch_runner.py — drives N successful FALCON exploration runs in one environment.

Run this INSIDE the falcon-ros:noetic container after starting the
Gazebo+drone container with `./run.sh <env>` in another terminal.

Usage
═════
    python3 batch_runner.py <env_name> [n_successes=10] [timeout_sec=300]

Examples
────────
    python3 batch_runner.py hospital              # 10 runs, 300s each
    python3 batch_runner.py small_house 10 240    # 10 runs, 240s each
    python3 batch_runner.py playground 5 180      # 5 runs, 180s each

What "success" means
════════════════════
FALCON publishes Int32(data=2) on /planning/replan when no frontiers
remain. completion_watcher.py (launched alongside the FALCON nodes)
watches that topic and, after a settle period, touches:

    <output_dir>/<run_name>/.exploration_done

This script polls that file. If it appears before timeout_sec → success.
Otherwise the run is marked failed, its directory is deleted, and
another attempt is made under the same run index.

Output layout
═════════════
    /home/falcon/runs/<env>_batch_<TS>/
        run_01/
            voxels.npy             ← run_recorder
            coverage.csv           ← run_recorder
            trajectory_gt.csv      ← run_recorder
            trajectory_falcon.csv  ← run_recorder
            summary.json           ← run_recorder
            .exploration_done      ← completion_watcher
            roslaunch.log          ← this script
        run_02/...
        ...
        run_log.txt        one line per attempt (run_NN  outcome)
        runs.csv           per-run table of headline metrics
        aggregate.json     mean / std / min / max for the batch
        failures.json      list of timed-out / crashed attempts

Note on starting positions
══════════════════════════
This script does NOT physically respawn the drone between runs. The
drone starts each run from wherever the previous one ended (or from
the world's spawn point for the first run). That gives meaningful
run-to-run variability for free, because subsequent runs see the map
from a different starting pose. If you need controlled random starts,
fill in respawn_drone_hook() at the bottom of this file.
"""

import json
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime

DEFAULT_OUTPUT_ROOT = "/home/falcon/runs"
LAUNCH_PKG          = "falcon_adapter"
LAUNCH_FILE         = "gazebo_exploration.launch"

# After SIGINT, run_recorder.on_shutdown writes summary.json synchronously.
# The rest of the FALCON stack (planner threads, voxel mapper) takes longer
# to release. This is the wall-clock budget for that.
SHUTDOWN_GRACE_SEC = 30.0

# How often to check for the .exploration_done flag and roslaunch death.
POLL_INTERVAL_SEC = 0.5

# Quiet pause between iterations — lets ROS master release ports and stale
# tf2 transforms time out before the next launch.
INTER_RUN_PAUSE_SEC = 5.0


def now_ts():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def launch_one_run(env_name, run_name, output_dir, timeout_sec, log_file):
    """
    Spawn roslaunch for one experiment, watch for completion or timeout,
    then send SIGINT to cleanly shut everything down.

    Returns one of: 'success', 'timeout', 'crash'.
    """
    run_dir = os.path.join(output_dir, run_name)
    done_flag = os.path.join(run_dir, ".exploration_done")
    summary_path = os.path.join(run_dir, "summary.json")
    os.makedirs(run_dir, exist_ok=True)

    # Optional: physically respawn / reposition the drone for THIS run.
    # The hook is a no-op by default. See bottom of this file.
    respawn_drone_hook(run_name, env_name)

    cmd = [
        "roslaunch", LAUNCH_PKG, LAUNCH_FILE,
        "map_name:={}".format(env_name),
        "run_name:={}".format(run_name),
        "output_dir:={}".format(output_dir),
    ]
    print("[batch] launching: {}".format(" ".join(cmd)), flush=True)

    rl_log_path = os.path.join(run_dir, "roslaunch.log")
    rl_log_fp = open(rl_log_path, "w")
    proc = subprocess.Popen(
        cmd,
        preexec_fn=os.setsid,           # own process group → clean SIGINT
        stdout=rl_log_fp, stderr=subprocess.STDOUT,
    )

    t0 = time.time()
    deadline = t0 + timeout_sec
    outcome = None

    try:
        while True:
            time.sleep(POLL_INTERVAL_SEC)
            now = time.time()
            t_in_run = now - t0

            # 1. roslaunch died on its own → crash
            if proc.poll() is not None:
                print("[batch] roslaunch exited unexpectedly (rc={}, t={:.1f}s)"
                      .format(proc.returncode, t_in_run), flush=True)
                outcome = "crash"
                break

            # 2. completion_watcher signalled done → success
            if os.path.exists(done_flag):
                print("[batch] exploration_done at t={:.1f}s".format(t_in_run),
                      flush=True)
                outcome = "success"
                break

            # 3. wall-clock cap exceeded → timeout
            if now > deadline:
                print("[batch] timeout after {:.0f}s".format(timeout_sec),
                      flush=True)
                outcome = "timeout"
                break
    finally:
        # Always shut roslaunch down — SIGINT first so on_shutdown handlers
        # fire (run_recorder writes summary.json here), SIGKILL only if the
        # process refuses to exit within the grace window.
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=SHUTDOWN_GRACE_SEC)
            except subprocess.TimeoutExpired:
                print("[batch] roslaunch did not exit after {:.0f}s; SIGKILLing"
                      .format(SHUTDOWN_GRACE_SEC), flush=True)
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait()
        rl_log_fp.close()

    # If we declared success but run_recorder didn't manage to dump its
    # files, demote to crash — we'd otherwise have a dangling success
    # entry with no summary.json.
    if outcome == "success" and not os.path.exists(summary_path):
        print("[batch] success flagged but summary.json missing → reclassifying as crash",
              flush=True)
        outcome = "crash"

    log_file.write("{}  {}  {}\n".format(
        datetime.now().isoformat(timespec="seconds"), run_name, outcome))
    log_file.flush()
    return outcome


# ─────────────────── Aggregation ──────────────────────────────────────────

# Default radius (m) for "swept area" — area within this distance of the
# trajectory is considered well-mapped. Smaller than the depth camera's
# 5 m max range, so it's a conservative estimate of confidently-mapped
# floor area. Tune via SWEPT_RADIUS_M env var if needed.
SWEPT_RADIUS_M = float(os.environ.get("SWEPT_RADIUS_M", "2.0"))
SWEPT_GRID_CELL_M = 0.5  # 0.5 m grid for area accumulation


def _read_traj_xy(traj_csv_path):
    """
    Returns Nx2 (x, y) array from a trajectory_*.csv file written by
    run_recorder, or an empty array if the file is missing/malformed.
    Avoids importing numpy at module top so this script keeps working
    on a stripped-down container.
    """
    if not os.path.exists(traj_csv_path):
        return None
    try:
        import numpy as np
        # Columns: t, x, y, z, yaw_rad
        a = np.loadtxt(traj_csv_path, delimiter=",", skiprows=1)
        if a.size == 0:
            return None
        if a.ndim == 1:
            a = a.reshape(1, -1)
        if a.shape[1] < 3:
            return None
        return a[:, 1:3]  # x, y
    except (OSError, ValueError, ImportError):
        return None


def _swept_area_m2(traj_xy, swept_radius=SWEPT_RADIUS_M, cell_size=SWEPT_GRID_CELL_M):
    """
    Approximate the area the drone "explored" by computing the area
    of the union of disks of radius `swept_radius` around every
    trajectory point. Implemented by rasterising onto a 2D grid of
    `cell_size`-sized cells and counting cells whose centres fall
    within `swept_radius` of any trajectory point.

    Pure numpy. Trajectory is downsampled to one waypoint per
    swept_radius/2 of motion before the rasterisation, which keeps
    runtime bounded regardless of original sample rate.
    """
    if traj_xy is None or len(traj_xy) == 0:
        return 0.0
    import numpy as np

    xs = traj_xy[:, 0]
    ys = traj_xy[:, 1]

    # Downsample so consecutive waypoints are at least swept_radius/2 apart
    keep = [0]
    step2 = (swept_radius * 0.5) ** 2
    for i in range(1, len(traj_xy)):
        dx = xs[i] - xs[keep[-1]]
        dy = ys[i] - ys[keep[-1]]
        if dx * dx + dy * dy >= step2:
            keep.append(i)
    if keep[-1] != len(traj_xy) - 1:
        keep.append(len(traj_xy) - 1)
    pxs = xs[keep]
    pys = ys[keep]

    xmin = float(pxs.min() - swept_radius)
    xmax = float(pxs.max() + swept_radius)
    ymin = float(pys.min() - swept_radius)
    ymax = float(pys.max() + swept_radius)
    nx = int(np.ceil((xmax - xmin) / cell_size))
    ny = int(np.ceil((ymax - ymin) / cell_size))
    if nx <= 0 or ny <= 0:
        return 0.0

    gx = xmin + (np.arange(nx) + 0.5) * cell_size
    gy = ymin + (np.arange(ny) + 0.5) * cell_size

    grid = np.zeros((ny, nx), dtype=bool)
    r2 = swept_radius ** 2
    r_cells = int(np.ceil(swept_radius / cell_size))

    for x, y in zip(pxs, pys):
        cxi = int((x - xmin) / cell_size)
        cyi = int((y - ymin) / cell_size)
        x0 = max(0, cxi - r_cells)
        x1 = min(nx, cxi + r_cells + 1)
        y0 = max(0, cyi - r_cells)
        y1 = min(ny, cyi + r_cells + 1)
        sub_x = gx[x0:x1]
        sub_y = gy[y0:y1]
        dx = sub_x - x
        dy = sub_y - y
        d2 = dx[None, :] ** 2 + dy[:, None] ** 2
        grid[y0:y1, x0:x1] |= (d2 <= r2)

    return float(grid.sum() * cell_size * cell_size)


def _bbox_area_m2(traj_xy):
    """Width × height of the (x, y) bounding box of the trajectory."""
    if traj_xy is None or len(traj_xy) == 0:
        return 0.0
    xs, ys = traj_xy[:, 0], traj_xy[:, 1]
    return float((xs.max() - xs.min()) * (ys.max() - ys.min()))


def _stats(xs):
    if not xs:
        return {"mean": 0.0, "std": 0.0, "min": 0.0, "max": 0.0, "n": 0}
    return {
        "mean": float(statistics.mean(xs)),
        "std":  float(statistics.stdev(xs)) if len(xs) > 1 else 0.0,
        "min":  float(min(xs)),
        "max":  float(max(xs)),
        "n":    len(xs),
    }


def _coverage_milestones(coverage_csv_path, fractions=(0.50, 0.90, 0.99)):
    """
    Returns the wall-clock seconds at which the coverage curve first
    reaches each requested fraction of its FINAL value. Useful for
    answering "how long did it take to reach 90% of what FALCON
    eventually mapped". Returns None entries for fractions that the
    run didn't reach (shouldn't happen on successful runs but be
    defensive).
    """
    out = {}
    if not os.path.exists(coverage_csv_path):
        return {f: None for f in fractions}
    try:
        # coverage.csv is "t_sec,n_voxels" with one header line
        ts, ns = [], []
        with open(coverage_csv_path) as f:
            next(f, None)
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(",")
                if len(parts) < 2:
                    continue
                ts.append(float(parts[0]))
                ns.append(float(parts[1]))
    except (OSError, ValueError):
        return {f: None for f in fractions}

    if not ts or ns[-1] <= 0:
        return {f: None for f in fractions}

    final = ns[-1]
    for frac in fractions:
        threshold = frac * final
        out[frac] = None
        for t, n in zip(ts, ns):
            if n >= threshold:
                out[frac] = t
                break
    return out


def aggregate(output_dir, success_run_names, failures):
    durations, paths, vox = [], [], []
    vox_per_sec, vox_per_m, avg_speed = [], [], []
    t50, t90, t99 = [], [], []
    # Area metrics — primary "how much was actually mapped" denominators
    swept_areas, bbox_areas = [], []
    m2_per_sec, sec_per_m2, m_per_m2, vox_per_m2 = [], [], [], []

    rows = []

    for run_name in sorted(success_run_names):
        sp = os.path.join(output_dir, run_name, "summary.json")
        cp = os.path.join(output_dir, run_name, "coverage.csv")
        tp = os.path.join(output_dir, run_name, "trajectory_gt.csv")
        if not os.path.exists(sp):
            continue
        with open(sp) as f:
            summary = json.load(f)

        d = float(summary.get("duration_sec", 0.0))
        p = float(summary.get("path_length_m", 0.0))
        v = int(summary.get("final_voxels", 0))
        a = float(summary.get("avg_discovery_voxels_per_sec", 0.0))
        per_m = (v / p) if p > 1e-3 else 0.0
        spd   = (p / d) if d > 1e-3 else 0.0

        ms = _coverage_milestones(cp)

        # Area metrics — derived from trajectory_gt.csv
        traj_xy = _read_traj_xy(tp)
        swept = _swept_area_m2(traj_xy)
        bbox = _bbox_area_m2(traj_xy)
        m2ps = (swept / d) if d > 1e-3 else 0.0
        spm2 = (d / swept) if swept > 1e-3 else 0.0
        mpm2 = (p / swept) if swept > 1e-3 else 0.0
        vpm2 = (v / swept) if swept > 1e-3 else 0.0

        durations.append(d); paths.append(p); vox.append(v)
        vox_per_sec.append(a); vox_per_m.append(per_m); avg_speed.append(spd)
        if ms[0.50] is not None: t50.append(ms[0.50])
        if ms[0.90] is not None: t90.append(ms[0.90])
        if ms[0.99] is not None: t99.append(ms[0.99])
        if swept > 1e-3:
            swept_areas.append(swept); bbox_areas.append(bbox)
            m2_per_sec.append(m2ps); sec_per_m2.append(spm2)
            m_per_m2.append(mpm2);   vox_per_m2.append(vpm2)

        rows.append({
            "run_name":            run_name,
            "duration_sec":        d,
            "path_length_m":       p,
            "final_voxels":        v,
            "voxels_per_sec":      a,
            "voxels_per_meter":    per_m,
            "avg_speed_m_per_sec": spd,
            "swept_area_m2":       swept,
            "bbox_area_m2":        bbox,
            "m2_per_sec":          m2ps,
            "sec_per_m2":          spm2,
            "m_per_m2":            mpm2,
            "voxels_per_m2":       vpm2,
            "t_50pct_coverage":    ms[0.50] if ms[0.50] is not None else "",
            "t_90pct_coverage":    ms[0.90] if ms[0.90] is not None else "",
            "t_99pct_coverage":    ms[0.99] if ms[0.99] is not None else "",
        })

    n_total = len(success_run_names) + len(failures)
    aggregate_obj = {
        "n_successes":  len(success_run_names),
        "n_failures":   len(failures),
        "n_attempts":   n_total,
        "success_rate": (len(success_run_names) / n_total) if n_total else 0.0,
        "swept_radius_m": SWEPT_RADIUS_M,
        "metrics": {
            "duration_sec":           _stats(durations),
            "path_length_m":          _stats(paths),
            "final_voxels":           _stats(vox),
            "voxels_per_sec":         _stats(vox_per_sec),
            "voxels_per_meter":       _stats(vox_per_m),
            "avg_speed_m_per_sec":    _stats(avg_speed),
            "swept_area_m2":          _stats(swept_areas),
            "bbox_area_m2":           _stats(bbox_areas),
            "m2_per_sec":             _stats(m2_per_sec),
            "sec_per_m2":             _stats(sec_per_m2),
            "m_per_m2":               _stats(m_per_m2),
            "voxels_per_m2":          _stats(vox_per_m2),
            "t_50pct_coverage_sec":   _stats(t50),
            "t_90pct_coverage_sec":   _stats(t90),
            "t_99pct_coverage_sec":   _stats(t99),
        },
    }

    with open(os.path.join(output_dir, "aggregate.json"), "w") as f:
        json.dump(aggregate_obj, f, indent=2)
    with open(os.path.join(output_dir, "failures.json"), "w") as f:
        json.dump(failures, f, indent=2)

    keys = ["run_name", "duration_sec", "path_length_m", "final_voxels",
            "voxels_per_sec", "voxels_per_meter", "avg_speed_m_per_sec",
            "swept_area_m2", "bbox_area_m2", "m2_per_sec", "sec_per_m2",
            "m_per_m2", "voxels_per_m2",
            "t_50pct_coverage", "t_90pct_coverage", "t_99pct_coverage"]
    with open(os.path.join(output_dir, "runs.csv"), "w") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join(
                ("{:.4f}".format(r[k]) if isinstance(r[k], float) else str(r[k]))
                for k in keys) + "\n")

    # Pretty console summary
    print("\n" + "=" * 60, flush=True)
    print("BATCH SUMMARY  (swept_radius={:.1f}m)".format(SWEPT_RADIUS_M), flush=True)
    print("=" * 60, flush=True)
    print("successes: {} / {}  (success rate {:.1%})".format(
        aggregate_obj["n_successes"], n_total, aggregate_obj["success_rate"]),
          flush=True)
    if rows:
        m = aggregate_obj["metrics"]
        print("duration_sec       : mean={:.1f}  std={:.1f}  min={:.1f}  max={:.1f}".format(
              m["duration_sec"]["mean"], m["duration_sec"]["std"],
              m["duration_sec"]["min"], m["duration_sec"]["max"]), flush=True)
        print("path_length_m      : mean={:.2f}  std={:.2f}".format(
              m["path_length_m"]["mean"], m["path_length_m"]["std"]), flush=True)
        print("swept_area_m2      : mean={:.1f}  std={:.1f}".format(
              m["swept_area_m2"]["mean"], m["swept_area_m2"]["std"]), flush=True)
        print("m2_per_sec         : mean={:.3f}  std={:.3f}".format(
              m["m2_per_sec"]["mean"], m["m2_per_sec"]["std"]), flush=True)
        print("sec_per_m2         : mean={:.2f}  std={:.2f}".format(
              m["sec_per_m2"]["mean"], m["sec_per_m2"]["std"]), flush=True)
        print("m_per_m2           : mean={:.2f}  std={:.2f}".format(
              m["m_per_m2"]["mean"], m["m_per_m2"]["std"]), flush=True)
        print("avg_speed_m_per_s  : mean={:.3f}  std={:.3f}".format(
              m["avg_speed_m_per_sec"]["mean"], m["avg_speed_m_per_sec"]["std"]),
              flush=True)
        print("final_voxels       : mean={:.0f}  std={:.0f}".format(
              m["final_voxels"]["mean"], m["final_voxels"]["std"]), flush=True)
        print("voxels_per_sec     : mean={:.1f}  std={:.1f}".format(
              m["voxels_per_sec"]["mean"], m["voxels_per_sec"]["std"]), flush=True)
        print("voxels_per_m2      : mean={:.0f}  std={:.0f}".format(
              m["voxels_per_m2"]["mean"], m["voxels_per_m2"]["std"]), flush=True)
        if m["t_90pct_coverage_sec"]["n"] > 0:
            print("t_90pct_coverage_s : mean={:.1f}  std={:.1f}".format(
                  m["t_90pct_coverage_sec"]["mean"],
                  m["t_90pct_coverage_sec"]["std"]), flush=True)


# ─────────────────── Optional: drone respawn hook ────────────────────────

def respawn_drone_hook(run_name, env_name):
    """
    Override this to physically reposition the drone before each run.

    The default implementation is a no-op: the drone starts each run
    from wherever the previous one ended (and from the world's spawn
    point on the first run).

    To actually teleport, you need a way to call Gazebo's
    set_entity_state service from inside the FALCON container — which
    requires that you bridge that service across your ros1_bridge /
    ros2 setup. Once you've done that, replace this body with something
    like:

        import rospy
        from gazebo_msgs.srv import SetModelState
        # ... pick a random pose from a per-env starts list ...
        rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
        srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        srv(...)

    Or do it from the host with `docker exec sjtu_drone_<env> ros2 ...`
    BEFORE invoking this batch script, and just leave this stub empty.
    """
    return


# ─────────────────── Main ─────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        sys.stderr.write(__doc__)
        sys.exit(2)

    env_name      = sys.argv[1]
    n_succ_target = int(sys.argv[2])   if len(sys.argv) > 2 else 10
    timeout_sec   = float(sys.argv[3]) if len(sys.argv) > 3 else 300.0

    batch_id = "{}_batch_{}".format(env_name, now_ts())
    output_dir = os.path.join(DEFAULT_OUTPUT_ROOT, batch_id)
    os.makedirs(output_dir, exist_ok=True)

    print("[batch] env={}  target_successes={}  timeout={:.0f}s  output={}"
          .format(env_name, n_succ_target, timeout_sec, output_dir),
          flush=True)

    successes, failures = [], []
    log_path = os.path.join(output_dir, "run_log.txt")

    with open(log_path, "w") as log_file:
        log_file.write("# batch={} env={} target={} timeout={}\n".format(
            batch_id, env_name, n_succ_target, timeout_sec))
        log_file.flush()

        attempt = 0
        while len(successes) < n_succ_target:
            attempt += 1
            run_idx = len(successes) + 1
            run_name = "run_{:02d}".format(run_idx)

            print("\n[batch] ──── attempt {}  (success {}/{}, failed {}) ────"
                  .format(attempt, run_idx, n_succ_target, len(failures)),
                  flush=True)

            outcome = launch_one_run(env_name, run_name, output_dir,
                                     timeout_sec, log_file)

            if outcome == "success":
                successes.append(run_name)
            else:
                failures.append({
                    "run_name":      run_name,
                    "outcome":       outcome,
                    "attempt_index": attempt,
                })
                run_dir = os.path.join(output_dir, run_name)
                if os.path.isdir(run_dir):
                    shutil.rmtree(run_dir, ignore_errors=True)
                print("[batch] discarded {} ({})".format(run_name, outcome),
                      flush=True)

            time.sleep(INTER_RUN_PAUSE_SEC)

    aggregate(output_dir, successes, failures)
    print("\n[batch] DONE. {} successes, {} failures.".format(
        len(successes), len(failures)), flush=True)
    print("[batch] results: {}".format(output_dir), flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[batch] interrupted by user", flush=True)
        sys.exit(130)