#!/usr/bin/env python3
"""
visualize_routes.py
-------------------
Visualize RRT* benchmark routes on the map.

Reads the benchmark JSON file and creates images showing:
- First solution found (blue)
- Final optimized solution (green)

Usage:
  python3 visualize_routes.py results/benchmark_XXXX.json
  python3 visualize_routes.py results/benchmark_XXXX.json -m maps/hospital_map_cropped.yaml

PyCharm-friendly:
  - If run with no arguments, uses latest ../results/benchmark_*.json
"""

import os
import sys
import json
import argparse
from typing import Tuple
from pathlib import Path

import cv2
import yaml
import numpy as np
import matplotlib.pyplot as plt


# Your known-good map path (default when -m is not provided)
DEFAULT_MAP_YAML = "/home/nadavc/PycharmProjects/sjtu_project/rrt_benchmark/maps/hospital_map_cropped.yaml"


def load_map(yaml_path: str) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """Load map from YAML file. Returns (map_data, resolution, (origin_x, origin_y))."""
    with open(yaml_path, "r") as f:
        info = yaml.safe_load(f)

    resolution = float(info["resolution"])
    origin_x = float(info["origin"][0])
    origin_y = float(info["origin"][1])

    map_image_path = info["image"]
    if not map_image_path.startswith("/"):
        map_image_path = os.path.join(os.path.dirname(yaml_path), map_image_path)

    img = cv2.imread(map_image_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"Failed to load map image: {map_image_path}")

    # Binary occupancy for display (flip Y to match ROS convention)
    map_data = np.zeros_like(img, dtype=np.uint8)
    map_data[img < 50] = 1  # obstacles
    map_data = np.flipud(map_data)

    return map_data, resolution, (origin_x, origin_y)


def world_to_map(wx: float, wy: float, resolution: float, origin: Tuple[float, float]) -> Tuple[float, float]:
    """Convert world coordinates to map (grid) coordinates."""
    ox, oy = origin
    return (wx - ox) / resolution, (wy - oy) / resolution


def visualize_pair(map_data: np.ndarray, resolution: float, origin: Tuple[float, float],
                   pair_data: dict, output_path: str) -> None:
    """Create visualization for a single pair showing first and final routes."""
    fig, ax = plt.subplots(figsize=(10, 10))

    # Draw map
    ax.imshow(map_data, cmap="gray", origin="lower")

    # Get paths
    first_x = pair_data.get("example_first_path_x", [])
    first_y = pair_data.get("example_first_path_y", [])
    final_x = pair_data.get("example_final_path_x", [])
    final_y = pair_data.get("example_final_path_y", [])

    # Convert to map coordinates
    if first_x and first_y:
        first_mx = [world_to_map(x, y, resolution, origin)[0] for x, y in zip(first_x, first_y)]
        first_my = [world_to_map(x, y, resolution, origin)[1] for x, y in zip(first_x, first_y)]
        ax.plot(first_mx, first_my, 'b-', linewidth=2, alpha=0.7, label='First solution')
        ax.plot(first_mx, first_my, 'b.', markersize=4, alpha=0.5)

    if final_x and final_y:
        final_mx = [world_to_map(x, y, resolution, origin)[0] for x, y in zip(final_x, final_y)]
        final_my = [world_to_map(x, y, resolution, origin)[1] for x, y in zip(final_x, final_y)]
        ax.plot(final_mx, final_my, 'g-', linewidth=2.5, alpha=0.8, label='Final solution')
        ax.plot(final_mx, final_my, 'g.', markersize=4, alpha=0.5)

    # Mark start and goal
    start_grid = pair_data.get("start_grid", [])
    goal_grid = pair_data.get("goal_grid", [])

    if start_grid:
        ax.plot(start_grid[0], start_grid[1], 'ro', markersize=12, label='Start', zorder=10)
    if goal_grid:
        ax.plot(goal_grid[0], goal_grid[1], 'r*', markersize=15, label='Goal', zorder=10)

    # Add info text
    pair_id = pair_data.get("pair_id", 0) + 1
    air_dist = pair_data.get("air_distance_world", 0)
    success_rate = pair_data.get("success_rate", 0) * 100
    mean_time = pair_data.get("mean_first_solution_time_ms", 0)
    mean_length = pair_data.get("mean_final_length", 0)
    improvement = pair_data.get("mean_improvement_percent", 0)

    info_text = (
        f"Pair {pair_id}\n"
        f"Air distance: {air_dist:.2f}m\n"
        f"Success rate: {success_rate:.0f}%\n"
        f"Mean first solution: {mean_time:.1f}ms\n"
        f"Mean final length: {mean_length:.2f}m\n"
        f"Improvement: {improvement:.1f}%"
    )

    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    ax.legend(loc='upper right')
    ax.set_title(f"RRT* Route - Pair {pair_id}")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {output_path}")


def _find_latest_benchmark_json() -> str | None:
    """Find latest benchmark_*.json in ../results relative to this script."""
    results_dir = (Path(__file__).resolve().parent.parent / "results")
    if not results_dir.exists():
        return None
    candidates = sorted(results_dir.glob("benchmark_*.json"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        return None
    return str(candidates[-1])


def _resolve_path_relative_to_json(path_str: str, json_path: str) -> str:
    """If path_str is relative, resolve it relative to the JSON directory."""
    if not path_str:
        return path_str
    p = Path(path_str)
    if p.is_absolute():
        return str(p)
    return str(Path(json_path).resolve().parent / p)


def main():
    parser = argparse.ArgumentParser(description="Visualize RRT* benchmark routes")
    parser.add_argument("json_file", nargs='?', help="Benchmark JSON file (default: latest in ../results)")
    parser.add_argument("-m", "--map", help="Map YAML file (overrides JSON, default: hardcoded hospital map)")
    parser.add_argument("-o", "--output", default=None, help="Output directory (default: same as JSON)")
    args = parser.parse_args()

    # PyCharm-friendly default: pick latest JSON if none provided
    if not args.json_file:
        latest = _find_latest_benchmark_json()
        if not latest:
            parser.error("json_file is required (no args given and no ../results/benchmark_*.json found)")
        args.json_file = latest
        print(f"[INFO] No json_file provided. Using latest benchmark JSON: {args.json_file}")

    # Load benchmark results
    with open(args.json_file, "r") as f:
        data = json.load(f)

    # Determine map path:
    # 1) CLI -m overrides
    # 2) JSON map_file if present
    # 3) fallback to DEFAULT_MAP_YAML
    if args.map:
        map_path = _resolve_path_relative_to_json(args.map, args.json_file)
    else:
        map_path = data.get("map_file", "") or DEFAULT_MAP_YAML
        # If JSON provided relative path, resolve relative to JSON dir
        if map_path != DEFAULT_MAP_YAML:
            map_path = _resolve_path_relative_to_json(map_path, args.json_file)

    if not os.path.exists(map_path):
        print(f"[WARN] Map not found at {map_path}")
        print(f"[INFO] Falling back to default map: {DEFAULT_MAP_YAML}")
        map_path = DEFAULT_MAP_YAML

    if not os.path.exists(map_path):
        print(f"[ERROR] Default map also not found: {map_path}")
        sys.exit(1)

    # Load map
    print(f"Loading map: {map_path}")
    map_data, resolution, origin = load_map(map_path)
    print(f"Map size: {map_data.shape[1]}x{map_data.shape[0]}, resolution: {resolution}m/px")

    # Output directory
    if args.output:
        output_dir = args.output
    else:
        output_dir = os.path.dirname(args.json_file) or "."
    os.makedirs(output_dir, exist_ok=True)

    # Get base name for output files
    base_name = os.path.splitext(os.path.basename(args.json_file))[0]

    # Visualize each pair
    pair_results = data.get("pair_results", [])
    print(f"Visualizing {len(pair_results)} pairs...")

    for pair_data in pair_results:
        pair_id = pair_data.get("pair_id", 0)
        output_path = os.path.join(output_dir, f"{base_name}_pair{pair_id + 1:02d}.png")
        visualize_pair(map_data, resolution, origin, pair_data, output_path)

    print(f"\nDone! Created {len(pair_results)} images in {output_dir}/")


if __name__ == "__main__":
    main()
