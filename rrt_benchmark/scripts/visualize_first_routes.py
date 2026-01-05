#!/usr/bin/env python3
"""
visualize_first_routes.py
-------------------------
Visualize RRT first-solution benchmark routes on the map.

Creates visualizations showing:
- The first successful route for each point pair
- Start/goal markers
- Path statistics

Also creates a combined overview showing all routes.

Usage:
  python3 visualize_first_routes.py results/rrt_first_XXXX.json
  python3 visualize_first_routes.py results/rrt_first_XXXX.json -m maps/custom.yaml
"""

import os
import sys
import json
import argparse
from typing import Tuple, List, Dict, Any
from pathlib import Path

import cv2
import yaml
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.patches as mpatches

# Default map path
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


def visualize_single_pair(map_data: np.ndarray, resolution: float, origin: Tuple[float, float],
                          pair_data: Dict[str, Any], output_path: str) -> None:
    """Create visualization for a single pair showing the first successful route."""
    fig, ax = plt.subplots(figsize=(12, 10))

    # Draw map (gray background)
    ax.imshow(map_data, cmap="gray", origin="lower", alpha=0.7)

    # Get example path (first successful route)
    path_x = pair_data.get("example_path_x", [])
    path_y = pair_data.get("example_path_y", [])

    if path_x and path_y:
        # Convert to map coordinates
        path_mx = [world_to_map(x, y, resolution, origin)[0] for x, y in zip(path_x, path_y)]
        path_my = [world_to_map(x, y, resolution, origin)[1] for x, y in zip(path_x, path_y)]

        # Draw path
        ax.plot(path_mx, path_my, 'b-', linewidth=2.5, alpha=0.9, label='First solution')
        ax.plot(path_mx, path_my, 'b.', markersize=3, alpha=0.4)

    # Mark start and goal
    start_grid = pair_data.get("start_grid", [])
    goal_grid = pair_data.get("goal_grid", [])

    if start_grid:
        ax.plot(start_grid[0], start_grid[1], 'go', markersize=15,
                markeredgecolor='darkgreen', markeredgewidth=2, label='Start')
    if goal_grid:
        ax.plot(goal_grid[0], goal_grid[1], 'r*', markersize=18,
                markeredgecolor='darkred', markeredgewidth=1, label='Goal')

    # Statistics
    pair_id = pair_data.get("pair_id", 0) + 1
    air_dist = pair_data.get("air_distance", 0)
    success_rate = pair_data.get("success_rate", 0) * 100
    mean_time = pair_data.get("mean_planning_time_ms", 0)
    std_time = pair_data.get("std_planning_time_ms", 0)
    mean_length = pair_data.get("mean_path_length", 0)
    std_length = pair_data.get("std_path_length", 0)
    min_length = pair_data.get("min_path_length", 0)
    max_length = pair_data.get("max_path_length", 0)
    num_success = pair_data.get("num_success", 0)
    total_iters = len(pair_data.get("iterations", []))

    info_text = (
        f"Pair {pair_id}\n"
        f"Air distance: {air_dist:.2f}m\n"
        f"Success: {num_success}/{total_iters} ({success_rate:.0f}%)\n"
        f"Planning time: {mean_time:.1f} ± {std_time:.1f} ms\n"
        f"Path length: {mean_length:.2f} ± {std_length:.2f} m\n"
        f"Length range: [{min_length:.2f}, {max_length:.2f}] m"
    )

    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=11,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

    ax.legend(loc='upper right', fontsize=10)
    ax.set_title(f"RRT First Solution - Pair {pair_id}", fontsize=14, fontweight='bold')
    ax.set_xlabel("Grid X")
    ax.set_ylabel("Grid Y")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {output_path}")


def visualize_all_routes(map_data: np.ndarray, resolution: float, origin: Tuple[float, float],
                         data: Dict[str, Any], output_path: str) -> None:
    """Create overview visualization showing all routes on one map."""
    fig, ax = plt.subplots(figsize=(14, 12))

    # Draw map
    ax.imshow(map_data, cmap="gray", origin="lower", alpha=0.6)

    # Color map for different pairs
    colors = plt.cm.tab20(np.linspace(0, 1, 20))

    pair_results = data.get("pair_results", [])

    for idx, pair_data in enumerate(pair_results):
        path_x = pair_data.get("example_path_x", [])
        path_y = pair_data.get("example_path_y", [])

        if not path_x or not path_y:
            continue

        color = colors[idx % 20]

        # Convert to map coordinates
        path_mx = [world_to_map(x, y, resolution, origin)[0] for x, y in zip(path_x, path_y)]
        path_my = [world_to_map(x, y, resolution, origin)[1] for x, y in zip(path_x, path_y)]

        # Draw path
        ax.plot(path_mx, path_my, '-', color=color, linewidth=1.5, alpha=0.8)

        # Mark start and goal
        start_grid = pair_data.get("start_grid", [])
        goal_grid = pair_data.get("goal_grid", [])

        if start_grid:
            ax.plot(start_grid[0], start_grid[1], 'o', color=color, markersize=8,
                    markeredgecolor='black', markeredgewidth=1)
        if goal_grid:
            ax.plot(goal_grid[0], goal_grid[1], '*', color=color, markersize=12,
                    markeredgecolor='black', markeredgewidth=0.5)

    # Summary statistics
    total_runs = data.get("total_runs", 0)
    total_successes = data.get("total_successes", 0)
    success_rate = data.get("overall_success_rate", 0) * 100
    mean_time = data.get("mean_planning_time_ms", 0)
    mean_length = data.get("mean_path_length", 0)
    num_pairs = data.get("num_pairs", 0)
    iters = data.get("iterations_per_pair", 0)
    timeout = data.get("planning_timeout", 0)

    info_text = (
        f"RRT First-Solution Benchmark Overview\n"
        f"{'=' * 40}\n"
        f"Pairs: {num_pairs}, Iterations: {iters}\n"
        f"Timeout: {timeout}s\n"
        f"Success: {total_successes}/{total_runs} ({success_rate:.1f}%)\n"
        f"Mean planning time: {mean_time:.1f} ms\n"
        f"Mean path length: {mean_length:.2f} m"
    )

    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=11,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

    # Legend
    start_marker = mpatches.Patch(color='gray', label='Start (circle)')
    goal_marker = mpatches.Patch(color='gray', label='Goal (star)')
    ax.legend(handles=[start_marker, goal_marker], loc='upper right', fontsize=10)

    ax.set_title("All RRT First-Solution Routes", fontsize=14, fontweight='bold')
    ax.set_xlabel("Grid X")
    ax.set_ylabel("Grid Y")

    plt.tight_layout()
    plt.savefig(output_path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved overview: {output_path}")


def visualize_timing_histogram(data: Dict[str, Any], output_path: str) -> None:
    """Create histogram of planning times."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Collect all planning times
    times = []
    lengths = []
    for pair_data in data.get("pair_results", []):
        for ir in pair_data.get("iterations", []):
            if ir.get("success", False):
                times.append(ir.get("planning_time_ms", 0))
                lengths.append(ir.get("path_length", 0))

    # Planning time histogram
    ax1 = axes[0]
    if times:
        ax1.hist(times, bins=50, color='steelblue', edgecolor='black', alpha=0.7)
        ax1.axvline(np.mean(times), color='red', linestyle='--', linewidth=2,
                    label=f'Mean: {np.mean(times):.1f} ms')
        ax1.axvline(np.median(times), color='orange', linestyle='--', linewidth=2,
                    label=f'Median: {np.median(times):.1f} ms')
    ax1.set_xlabel("Planning Time (ms)", fontsize=12)
    ax1.set_ylabel("Frequency", fontsize=12)
    ax1.set_title("RRT Planning Time Distribution", fontsize=13, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)

    # Path length histogram
    ax2 = axes[1]
    if lengths:
        ax2.hist(lengths, bins=50, color='seagreen', edgecolor='black', alpha=0.7)
        ax2.axvline(np.mean(lengths), color='red', linestyle='--', linewidth=2,
                    label=f'Mean: {np.mean(lengths):.2f} m')
        ax2.axvline(np.median(lengths), color='orange', linestyle='--', linewidth=2,
                    label=f'Median: {np.median(lengths):.2f} m')
    ax2.set_xlabel("Path Length (m)", fontsize=12)
    ax2.set_ylabel("Frequency", fontsize=12)
    ax2.set_title("Path Length Distribution", fontsize=13, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved histograms: {output_path}")


def visualize_pair_comparison(data: Dict[str, Any], output_path: str) -> None:
    """Create bar chart comparing pairs."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    pair_results = data.get("pair_results", [])
    n_pairs = len(pair_results)

    if n_pairs == 0:
        plt.close(fig)
        return

    pair_ids = [p.get("pair_id", i) + 1 for i, p in enumerate(pair_results)]
    success_rates = [p.get("success_rate", 0) * 100 for p in pair_results]
    mean_times = [p.get("mean_planning_time_ms", 0) for p in pair_results]
    std_times = [p.get("std_planning_time_ms", 0) for p in pair_results]
    mean_lengths = [p.get("mean_path_length", 0) for p in pair_results]
    std_lengths = [p.get("std_path_length", 0) for p in pair_results]
    air_distances = [p.get("air_distance", 0) for p in pair_results]

    x = np.arange(n_pairs)
    width = 0.7

    # Success rate
    ax1 = axes[0, 0]
    bars1 = ax1.bar(x, success_rates, width, color='steelblue', edgecolor='black')
    ax1.set_ylabel("Success Rate (%)", fontsize=11)
    ax1.set_title("Success Rate per Pair", fontsize=12, fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels(pair_ids)
    ax1.set_xlabel("Pair ID", fontsize=11)
    ax1.set_ylim(0, 110)
    ax1.axhline(100, color='green', linestyle='--', alpha=0.5)
    ax1.grid(True, axis='y', alpha=0.3)

    # Planning time
    ax2 = axes[0, 1]
    bars2 = ax2.bar(x, mean_times, width, yerr=std_times, color='coral',
                    edgecolor='black', capsize=3)
    ax2.set_ylabel("Planning Time (ms)", fontsize=11)
    ax2.set_title("Mean Planning Time per Pair", fontsize=12, fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(pair_ids)
    ax2.set_xlabel("Pair ID", fontsize=11)
    ax2.grid(True, axis='y', alpha=0.3)

    # Path length
    ax3 = axes[1, 0]
    bars3 = ax3.bar(x, mean_lengths, width, yerr=std_lengths, color='seagreen',
                    edgecolor='black', capsize=3)
    ax3.plot(x, air_distances, 'r--o', markersize=6, label='Air distance')
    ax3.set_ylabel("Path Length (m)", fontsize=11)
    ax3.set_title("Mean Path Length vs Air Distance", fontsize=12, fontweight='bold')
    ax3.set_xticks(x)
    ax3.set_xticklabels(pair_ids)
    ax3.set_xlabel("Pair ID", fontsize=11)
    ax3.legend(fontsize=9)
    ax3.grid(True, axis='y', alpha=0.3)

    # Path length ratio (actual / air)
    ax4 = axes[1, 1]
    ratios = [ml / ad if ad > 0 else 0 for ml, ad in zip(mean_lengths, air_distances)]
    bars4 = ax4.bar(x, ratios, width, color='mediumpurple', edgecolor='black')
    ax4.axhline(1.0, color='green', linestyle='--', alpha=0.7, label='Direct path (1.0)')
    ax4.set_ylabel("Path Length / Air Distance", fontsize=11)
    ax4.set_title("Path Efficiency Ratio", fontsize=12, fontweight='bold')
    ax4.set_xticks(x)
    ax4.set_xticklabels(pair_ids)
    ax4.set_xlabel("Pair ID", fontsize=11)
    ax4.legend(fontsize=9)
    ax4.grid(True, axis='y', alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved comparison: {output_path}")


def _find_latest_json() -> str | None:
    """Find latest rrt_first_*.json in results directory."""
    results_dir = Path("results")
    if not results_dir.exists():
        results_dir = Path(__file__).resolve().parent.parent / "results"
    if not results_dir.exists():
        return None
    candidates = sorted(results_dir.glob("rrt_first_*.json"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        return None
    return str(candidates[-1])


def _resolve_path(path_str: str, json_path: str) -> str:
    """Resolve relative path relative to JSON file location."""
    if not path_str:
        return path_str
    p = Path(path_str)
    if p.is_absolute():
        return str(p)
    return str(Path(json_path).resolve().parent / p)


def main():
    parser = argparse.ArgumentParser(description="Visualize RRT first-solution benchmark routes")
    parser.add_argument("json_file", nargs='?', help="Benchmark JSON file")
    parser.add_argument("-m", "--map", help="Map YAML file (overrides JSON)")
    parser.add_argument("-o", "--output", default=None, help="Output directory")
    parser.add_argument("--no-individual", action="store_true", help="Skip individual pair images")
    args = parser.parse_args()

    # Find JSON file
    if not args.json_file:
        latest = _find_latest_json()
        if not latest:
            parser.error("json_file required (no rrt_first_*.json found in results/)")
        args.json_file = latest
        print(f"[INFO] Using latest: {args.json_file}")

    # Load benchmark results
    print(f"Loading: {args.json_file}")
    with open(args.json_file, "r") as f:
        data = json.load(f)

    # Determine map path
    if args.map:
        map_path = _resolve_path(args.map, args.json_file)
    else:
        map_path = data.get("map_file", "") or DEFAULT_MAP_YAML
        if map_path != DEFAULT_MAP_YAML:
            map_path = _resolve_path(map_path, args.json_file)

    if not os.path.exists(map_path):
        print(f"[WARN] Map not found: {map_path}")
        print(f"[INFO] Trying default: {DEFAULT_MAP_YAML}")
        map_path = DEFAULT_MAP_YAML

    if not os.path.exists(map_path):
        print(f"[ERROR] Map not found: {map_path}")
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

    base_name = os.path.splitext(os.path.basename(args.json_file))[0]

    pair_results = data.get("pair_results", [])
    print(f"\nGenerating visualizations for {len(pair_results)} pairs...")

    # Individual pair visualizations
    if not args.no_individual:
        for pair_data in pair_results:
            pair_id = pair_data.get("pair_id", 0)
            output_path = os.path.join(output_dir, f"{base_name}_pair{pair_id + 1:02d}.png")
            visualize_single_pair(map_data, resolution, origin, pair_data, output_path)

    # Overview with all routes
    overview_path = os.path.join(output_dir, f"{base_name}_overview.png")
    visualize_all_routes(map_data, resolution, origin, data, overview_path)

    # Histograms
    hist_path = os.path.join(output_dir, f"{base_name}_histograms.png")
    visualize_timing_histogram(data, hist_path)

    # Pair comparison
    comp_path = os.path.join(output_dir, f"{base_name}_comparison.png")
    visualize_pair_comparison(data, comp_path)

    print(f"\nDone! Created visualizations in {output_dir}/")


if __name__ == "__main__":
    main()