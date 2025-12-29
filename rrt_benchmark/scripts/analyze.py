#!/usr/bin/env python3
"""
analyze.py
----------
Analyze RRT* benchmark results and generate plots.

Usage:
    python3 analyze.py results/benchmark_XXXX.json
    python3 analyze.py results/benchmark_XXXX.json --no-show
"""

import sys
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def load_results(filepath):
    """Load benchmark results from JSON."""
    with open(filepath) as f:
        return json.load(f)


def extract_timelines(data):
    """Extract all improvement timelines (time_ms, path_length) for each run."""
    timelines = []
    for pair in data.get('pair_results', []):
        for it in pair.get('iteration_results', []):
            if it.get('success'):
                timeline = it.get('improvement_timeline', [])
                if timeline:
                    t = [p['timestamp_ms'] for p in timeline]
                    l = [p['path_length'] for p in timeline]
                    timelines.append((t, l))
    return timelines


def extract_first_times(data):
    """Extract all first solution times."""
    times = []
    for pair in data.get('pair_results', []):
        for it in pair.get('iteration_results', []):
            if it.get('success'):
                times.append(it['first_solution_time_ms'])
    return times


def extract_final_lengths(data):
    """Extract all final path lengths."""
    lengths = []
    for pair in data.get('pair_results', []):
        for it in pair.get('iteration_results', []):
            if it.get('success'):
                lengths.append(it['final_path_length'])
    return lengths


def extract_improvements(data):
    """Extract improvement percentages."""
    imps = []
    for pair in data.get('pair_results', []):
        for it in pair.get('iteration_results', []):
            if it.get('success') and 'improvement_percent' in it:
                imps.append(it['improvement_percent'])
    return imps


def plot_improvement_over_time(data, output_dir, show=True):
    """
    Main plot: Path distance vs time showing RRT* optimization.
    X-axis: time (ms), Y-axis: path distance (m)
    """
    timelines = extract_timelines(data)
    if not timelines:
        print("No timeline data found")
        return

    fig, ax = plt.subplots(figsize=(12, 7))

    # Plot individual runs (faded)
    for t, l in timelines:
        ax.plot(t, l, 'b-', alpha=0.08, linewidth=0.8)

    # Compute aggregate at time bins
    max_time = max(t[-1] for t, _ in timelines)
    bins = np.linspace(0, max_time, 100)

    lengths_at_bin = []
    for b in bins:
        vals = []
        for times, lengths in timelines:
            # Get length at this time (most recent before b)
            val = lengths[0]
            for i, t in enumerate(times):
                if t <= b:
                    val = lengths[i]
            vals.append(val)
        lengths_at_bin.append(vals)

    means = [np.mean(v) for v in lengths_at_bin]
    stds = [np.std(v) for v in lengths_at_bin]

    ax.plot(bins, means, 'b-', linewidth=2.5, label='Mean path length')
    ax.fill_between(bins,
                    [m - s for m, s in zip(means, stds)],
                    [m + s for m, s in zip(means, stds)],
                    alpha=0.3, color='blue', label='±1 std')

    # Mark median first solution time
    first_times = extract_first_times(data)
    if first_times:
        median = np.median(first_times)
        ax.axvline(median, color='green', linestyle='--', linewidth=2,
                   label=f'Median first solution: {median:.0f}ms')

    ax.set_xlabel('Time (milliseconds)', fontsize=12)
    ax.set_ylabel('Path Distance (meters)', fontsize=12)
    ax.set_title('RRT* Path Optimization Over Time', fontsize=14)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, max_time)

    plt.tight_layout()
    plt.savefig(output_dir / 'improvement_over_time.png', dpi=150)
    print(f"Saved: {output_dir / 'improvement_over_time.png'}")

    if show:
        plt.show()
    plt.close()


def plot_first_solution_distribution(data, output_dir, show=True):
    """Plot distribution of first solution times."""
    times = extract_first_times(data)
    if not times:
        print("No first solution data")
        return

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Histogram
    ax = axes[0]
    ax.hist(times, bins=30, color='steelblue', edgecolor='black', alpha=0.7)
    ax.axvline(np.mean(times), color='red', linestyle='--',
               label=f'Mean: {np.mean(times):.1f}ms')
    ax.axvline(np.median(times), color='green', linestyle='--',
               label=f'Median: {np.median(times):.1f}ms')
    ax.set_xlabel('First Solution Time (ms)')
    ax.set_ylabel('Frequency')
    ax.set_title('Distribution of First Solution Times')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Box plot with stats
    ax = axes[1]
    bp = ax.boxplot(times, vert=True, patch_artist=True)
    bp['boxes'][0].set_facecolor('steelblue')
    bp['boxes'][0].set_alpha(0.7)
    ax.set_ylabel('Time (ms)')
    ax.set_title('First Solution Time')
    ax.grid(True, alpha=0.3)

    stats = (f"N = {len(times)}\n"
             f"Mean: {np.mean(times):.1f}ms\n"
             f"Median: {np.median(times):.1f}ms\n"
             f"Std: {np.std(times):.1f}ms\n"
             f"P5: {np.percentile(times, 5):.1f}ms\n"
             f"P95: {np.percentile(times, 95):.1f}ms")
    ax.text(1.4, 0.5, stats, transform=ax.transAxes, fontsize=10,
            verticalalignment='center',
            bbox=dict(boxstyle='round', facecolor='wheat'))

    plt.tight_layout()
    plt.savefig(output_dir / 'first_solution_dist.png', dpi=150)
    print(f"Saved: {output_dir / 'first_solution_dist.png'}")

    if show:
        plt.show()
    plt.close()


def plot_quality_metrics(data, output_dir, show=True):
    """Plot path quality metrics."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # Final path lengths
    lengths = extract_final_lengths(data)
    if lengths:
        ax = axes[0]
        ax.hist(lengths, bins=30, color='forestgreen', edgecolor='black', alpha=0.7)
        ax.axvline(np.mean(lengths), color='red', linestyle='--',
                   label=f'Mean: {np.mean(lengths):.2f}m')
        ax.set_xlabel('Path Length (m)')
        ax.set_ylabel('Frequency')
        ax.set_title('Final Path Lengths')
        ax.legend()
        ax.grid(True, alpha=0.3)

    # Improvements
    imps = extract_improvements(data)
    if imps:
        ax = axes[1]
        ax.hist(imps, bins=30, color='coral', edgecolor='black', alpha=0.7)
        ax.axvline(np.mean(imps), color='red', linestyle='--',
                   label=f'Mean: {np.mean(imps):.1f}%')
        ax.set_xlabel('Improvement (%)')
        ax.set_ylabel('Frequency')
        ax.set_title('Path Improvement (First → Final)')
        ax.legend()
        ax.grid(True, alpha=0.3)

    # Path length vs air distance
    ax = axes[2]
    for pair in data.get('pair_results', []):
        air = pair.get('air_distance_world', 0)
        for it in pair.get('iteration_results', []):
            if it.get('success'):
                ax.scatter(air, it['final_path_length'], alpha=0.3,
                           color='purple', s=20)

    lim = ax.get_xlim()[1]
    ax.plot([0, lim], [0, lim], 'k--', alpha=0.5, label='Path = Air')
    ax.set_xlabel('Air Distance (m)')
    ax.set_ylabel('Path Length (m)')
    ax.set_title('Path Length vs Air Distance')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / 'quality_metrics.png', dpi=150)
    print(f"Saved: {output_dir / 'quality_metrics.png'}")

    if show:
        plt.show()
    plt.close()


def plot_per_pair_summary(data, output_dir, show=True):
    """Plot per-pair statistics."""
    pairs = data.get('pair_results', [])
    if not pairs:
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    ids = [p['pair_id'] for p in pairs]

    # Success rate
    ax = axes[0, 0]
    rates = [p['success_rate'] * 100 for p in pairs]
    colors = ['green' if r >= 90 else 'orange' if r >= 70 else 'red' for r in rates]
    ax.bar(ids, rates, color=colors, edgecolor='black')
    ax.axhline(100, color='gray', linestyle='--', alpha=0.5)
    ax.set_xlabel('Pair ID')
    ax.set_ylabel('Success Rate (%)')
    ax.set_title('Success Rate per Pair')
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.3, axis='y')

    # First solution time
    ax = axes[0, 1]
    means = [p['mean_first_solution_time_ms'] for p in pairs]
    stds = [p['std_first_solution_time_ms'] for p in pairs]
    ax.errorbar(ids, means, yerr=stds, fmt='o', capsize=5,
                color='steelblue', markersize=8)
    ax.set_xlabel('Pair ID')
    ax.set_ylabel('First Solution Time (ms)')
    ax.set_title('First Solution Time per Pair (±1 std)')
    ax.grid(True, alpha=0.3)

    # Path length vs air distance
    ax = axes[1, 0]
    air = [p['air_distance_world'] for p in pairs]
    length = [p['mean_final_length'] for p in pairs]
    ax.scatter(air, length, c='forestgreen', s=100, edgecolor='black')
    maxv = max(max(air), max(length)) * 1.1
    ax.plot([0, maxv], [0, maxv], 'k--', alpha=0.5, label='Path = Air')
    ax.set_xlabel('Air Distance (m)')
    ax.set_ylabel('Mean Path Length (m)')
    ax.set_title('Path Efficiency per Pair')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Improvement
    ax = axes[1, 1]
    imps = [p['mean_improvement_percent'] for p in pairs]
    ax.bar(ids, imps, color='coral', edgecolor='black')
    ax.set_xlabel('Pair ID')
    ax.set_ylabel('Mean Improvement (%)')
    ax.set_title('Path Improvement per Pair')
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(output_dir / 'per_pair_summary.png', dpi=150)
    print(f"Saved: {output_dir / 'per_pair_summary.png'}")

    if show:
        plt.show()
    plt.close()


def print_statistics(data):
    """Print statistical summary."""
    print("\n" + "=" * 60)
    print("STATISTICAL SUMMARY")
    print("=" * 60)

    print(f"\nTimestamp: {data.get('timestamp', 'N/A')}")
    print(f"Total runs: {data.get('total_runs', 0)}")
    print(f"Successes: {data.get('total_successes', 0)}")
    print(f"Success rate: {data.get('overall_success_rate', 0) * 100:.1f}%")

    first_times = extract_first_times(data)
    if first_times:
        print(f"\nFirst Solution Time:")
        print(f"  Mean: {np.mean(first_times):.1f}ms")
        print(f"  Median: {np.median(first_times):.1f}ms")
        print(f"  Std: {np.std(first_times):.1f}ms")
        print(f"  P5: {np.percentile(first_times, 5):.1f}ms")
        print(f"  P95: {np.percentile(first_times, 95):.1f}ms")

    lengths = extract_final_lengths(data)
    if lengths:
        print(f"\nFinal Path Length:")
        print(f"  Mean: {np.mean(lengths):.2f}m")
        print(f"  Std: {np.std(lengths):.2f}m")

    imps = extract_improvements(data)
    if imps:
        print(f"\nPath Improvement:")
        print(f"  Mean: {np.mean(imps):.1f}%")
        print(f"  Max: {np.max(imps):.1f}%")

    print("\n" + "=" * 60)

    # Recommendations
    print("\nRECOMMENDATIONS:")
    if first_times:
        p50 = np.median(first_times)
        p95 = np.percentile(first_times, 95)
        print(f"  - 50% of paths found within {p50:.0f}ms")
        print(f"  - 95% of paths found within {p95:.0f}ms")

        if imps and np.mean(imps) > 5:
            print(f"  - RRT* improves paths by {np.mean(imps):.1f}% on average")
            print(f"  - Consider using full timeout for quality")
        elif imps:
            print(f"  - Only {np.mean(imps):.1f}% improvement - shorter timeout OK")

    print("=" * 60 + "\n")


def main():
    parser = argparse.ArgumentParser(description='Analyze RRT* benchmark results')
    parser.add_argument('input', nargs='?', help='JSON results file (default: latest in ../results)')
    parser.add_argument('--output', '-o', help='Output directory (default: same as input)')
    parser.add_argument('--no-show', action='store_true', help="Don't display plots")
    args = parser.parse_args()

    # If no input provided, pick latest benchmark_*.json from ../results
    if not args.input:
        results_dir = (Path(__file__).resolve().parent.parent / "results")
        candidates = sorted(results_dir.glob("benchmark_*.json"), key=lambda p: p.stat().st_mtime)
        if not candidates:
            parser.error(f"No input provided and no benchmark_*.json found in: {results_dir}")
        args.input = str(candidates[-1])
        print(f"[INFO] No input given, using latest: {args.input}")

    # Load data
    data = load_results(args.input)

    # Output directory
    if args.output:
        output_dir = Path(args.output)
    else:
        output_dir = Path(args.input).parent
    output_dir.mkdir(parents=True, exist_ok=True)

    show = not args.no_show

    # Print statistics
    print_statistics(data)

    # Generate plots
    print("Generating plots...\n")
    plot_improvement_over_time(data, output_dir, show)
    plot_first_solution_distribution(data, output_dir, show)
    plot_quality_metrics(data, output_dir, show)
    plot_per_pair_summary(data, output_dir, show)

    print("\nDone!")


if __name__ == '__main__':
    main()