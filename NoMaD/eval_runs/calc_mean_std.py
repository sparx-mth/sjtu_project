import os
import json
import numpy as np
import csv
from datetime import datetime

RUNS_DIR = "/home/user1/sjtu_project/NoMaD/runs_nomad"

rows = []

for run_name in sorted(os.listdir(RUNS_DIR)):
    run_path = os.path.join(RUNS_DIR, run_name)

    if not run_name.startswith("run_"):
        continue
    if not os.path.isdir(run_path):
        continue

    summary_path = os.path.join(run_path, "summary.json")
    if not os.path.isfile(summary_path):
        continue

    try:
        with open(summary_path, "r") as f:
            data = json.load(f)
    except Exception:
        continue

    final_dist = data.get("final_dist_to_goal_xy_m", None)
    success = data.get("success", None)

    # רק אם יש לנו ערך מספרי
    if isinstance(final_dist, (int, float)):
        rows.append({
            "run_name": run_name,
            "timestamp_unix": data.get("timestamp_unix", None),
            "model": data.get("model", None),
            "topomap_dir": data.get("topomap_dir", None),
            "goal_x": (data.get("goal_xy", {}) or {}).get("x", None),
            "goal_y": (data.get("goal_xy", {}) or {}).get("y", None),
            "final_dist_to_goal_xy_m": float(final_dist),
            "success": success
        })

# ---------- Save CSV ----------
out_csv = os.path.join(RUNS_DIR, "final_dist_summary.csv")

with open(out_csv, "w", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=[
            "run_name",
            "timestamp_unix",
            "model",
            "topomap_dir",
            "goal_x",
            "goal_y",
            "final_dist_to_goal_xy_m",
            "success",
        ],
    )
    writer.writeheader()
    writer.writerows(rows)

print(f"[SAVED] {out_csv}")
print(f"Rows written: {len(rows)}")

# ---------- Stats ----------
def print_stats(name, vals):
    vals = np.array(vals, dtype=float)
    if len(vals) == 0:
        print(f"{name}: no samples")
        return
    print(f"{name}: n={len(vals)} mean={vals.mean():.3f} std={vals.std(ddof=0):.3f} (meters)")

all_vals = [r["final_dist_to_goal_xy_m"] for r in rows]
print_stats("ALL", all_vals)

true_vals = [r["final_dist_to_goal_xy_m"] for r in rows if r["success"] is True]
false_vals = [r["final_dist_to_goal_xy_m"] for r in rows if r["success"] is False]
none_vals = [r["final_dist_to_goal_xy_m"] for r in rows if r["success"] is None]

print_stats("SUCCESS=True", true_vals)
print_stats("SUCCESS=False", false_vals)
print_stats("SUCCESS=None (missing)", none_vals)
