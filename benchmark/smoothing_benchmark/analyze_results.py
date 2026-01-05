#!/usr/bin/env python3
"""
Smoothing Benchmark Analyzer
============================
Analyzes smoothing benchmark results and displays statistics with visualizations.

Usage:
    python analyze_results.py                           # Analyze most recent results
    python analyze_results.py path/to/smoothing_*.csv  # Specific file
    python analyze_results.py --all                     # Analyze all result files
    python analyze_results.py --plot                    # Generate visualization plots
"""

import argparse
import glob
import os
import sys
from pathlib import Path
from typing import List, Dict, Optional, Tuple
import csv
import statistics

# Visualization imports
try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import numpy as np

    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Warning: matplotlib not installed. Plotting disabled. Run: pip install matplotlib")

# Default paths
DEFAULT_RESULTS_DIR = "/home/nadavc/PycharmProjects/sjtu_project/benchmark/smoothing_benchmark/results"


def find_most_recent_results(results_dir: str) -> Optional[str]:
    """Find the most recent smoothing results CSV file."""
    pattern = os.path.join(results_dir, "smoothing_*.csv")
    files = glob.glob(pattern)

    if not files:
        return None

    files.sort(key=os.path.getmtime, reverse=True)
    return files[0]


def find_all_results(results_dir: str) -> List[str]:
    """Find all smoothing results CSV files."""
    pattern = os.path.join(results_dir, "smoothing_*.csv")
    files = glob.glob(pattern)
    files.sort(key=os.path.getmtime, reverse=True)
    return files


def load_csv(filepath: str) -> List[Dict]:
    """Load CSV file into list of dictionaries."""
    rows = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def compute_stats(values: List[float]) -> Dict[str, float]:
    """Compute statistics for a list of values."""
    if not values:
        return {
            'count': 0,
            'mean': 0,
            'median': 0,
            'std': 0,
            'min': 0,
            'max': 0,
            'p25': 0,
            'p75': 0,
            'p95': 0,
            'p99': 0,
        }

    sorted_vals = sorted(values)
    n = len(sorted_vals)

    def percentile(p: float) -> float:
        k = (n - 1) * p / 100
        f = int(k)
        c = f + 1 if f + 1 < n else f
        return sorted_vals[f] + (k - f) * (sorted_vals[c] - sorted_vals[f])

    return {
        'count': n,
        'mean': statistics.mean(values),
        'median': statistics.median(values),
        'std': statistics.stdev(values) if n > 1 else 0,
        'min': min(values),
        'max': max(values),
        'p25': percentile(25),
        'p75': percentile(75),
        'p95': percentile(95),
        'p99': percentile(99),
    }


def analyze_file(filepath: str, verbose: bool = True) -> Dict:
    """
    Analyze a single results file.

    Returns dictionary with statistics for each smoother.
    """
    rows = load_csv(filepath)

    if not rows:
        print(f"Warning: No data in {filepath}")
        return {}

    # Extract timing data
    rrt_times = []
    cubic_spline_times = []
    cubic_bezier_times = []
    min_snap_times = []

    # Extract path info
    num_waypoints_list = []
    path_lengths = []

    # Success counts
    cubic_spline_success = 0
    cubic_bezier_success = 0
    min_snap_success = 0

    for row in rows:
        # RRT time
        rrt_time = float(row.get('rrt_time_ms', 0))
        if rrt_time > 0:
            rrt_times.append(rrt_time)

        # Path info
        num_wp = int(row.get('num_waypoints', 0))
        if num_wp > 0:
            num_waypoints_list.append(num_wp)

        path_len = float(row.get('path_length', 0))
        if path_len > 0:
            path_lengths.append(path_len)

        # Cubic Spline
        if int(row.get('cubic_spline_success', 0)) == 1:
            cubic_spline_success += 1
            time_val = float(row.get('cubic_spline_time_ms', 0))
            if time_val > 0:
                cubic_spline_times.append(time_val)

        # Cubic Bezier
        if int(row.get('cubic_bezier_success', 0)) == 1:
            cubic_bezier_success += 1
            time_val = float(row.get('cubic_bezier_time_ms', 0))
            if time_val > 0:
                cubic_bezier_times.append(time_val)

        # Min Snap
        if int(row.get('min_snap_success', 0)) == 1:
            min_snap_success += 1
            time_val = float(row.get('min_snap_time_ms', 0))
            if time_val > 0:
                min_snap_times.append(time_val)

    total_paths = len(rows)

    results = {
        'file': filepath,
        'total_paths': total_paths,
        'rrt': {
            'stats': compute_stats(rrt_times),
        },
        'cubic_spline': {
            'success_count': cubic_spline_success,
            'success_rate': cubic_spline_success / total_paths if total_paths > 0 else 0,
            'stats': compute_stats(cubic_spline_times),
        },
        'cubic_bezier': {
            'success_count': cubic_bezier_success,
            'success_rate': cubic_bezier_success / total_paths if total_paths > 0 else 0,
            'stats': compute_stats(cubic_bezier_times),
        },
        'min_snap': {
            'success_count': min_snap_success,
            'success_rate': min_snap_success / total_paths if total_paths > 0 else 0,
            'stats': compute_stats(min_snap_times),
        },
        'path_info': {
            'waypoints': compute_stats(num_waypoints_list),
            'lengths': compute_stats(path_lengths),
        }
    }

    return results


def print_stats_table(name: str, stats: Dict[str, float], unit: str = "ms"):
    """Print a formatted statistics table."""
    if stats['count'] == 0:
        print(f"  {name}: No data")
        return

    print(f"  {name}:")
    print(f"    Count:    {stats['count']:>10}")
    print(f"    Mean:     {stats['mean']:>10.3f} {unit}")
    print(f"    Median:   {stats['median']:>10.3f} {unit}")
    print(f"    Std Dev:  {stats['std']:>10.3f} {unit}")
    print(f"    Min:      {stats['min']:>10.3f} {unit}")
    print(f"    Max:      {stats['max']:>10.3f} {unit}")
    print(f"    P25:      {stats['p25']:>10.3f} {unit}")
    print(f"    P75:      {stats['p75']:>10.3f} {unit}")
    print(f"    P95:      {stats['p95']:>10.3f} {unit}")
    print(f"    P99:      {stats['p99']:>10.3f} {unit}")


def print_comparison_table(results: Dict):
    """Print a comparison table of all smoothers."""
    print("\n" + "=" * 80)
    print("TIMING COMPARISON (milliseconds)")
    print("=" * 80)

    headers = ["Metric", "RRT Planning", "Cubic Spline", "Cubic Bezier", "Min Snap"]

    # Get stats
    rrt = results['rrt']['stats']
    cs = results['cubic_spline']['stats']
    cb = results['cubic_bezier']['stats']
    ms = results['min_snap']['stats']

    # Format row
    def fmt(val):
        if val == 0:
            return "N/A"
        return f"{val:.3f}"

    rows = [
        ["Count", str(rrt['count']), str(cs['count']), str(cb['count']), str(ms['count'])],
        ["Mean", fmt(rrt['mean']), fmt(cs['mean']), fmt(cb['mean']), fmt(ms['mean'])],
        ["Median", fmt(rrt['median']), fmt(cs['median']), fmt(cb['median']), fmt(ms['median'])],
        ["Std Dev", fmt(rrt['std']), fmt(cs['std']), fmt(cb['std']), fmt(ms['std'])],
        ["Min", fmt(rrt['min']), fmt(cs['min']), fmt(cb['min']), fmt(ms['min'])],
        ["Max", fmt(rrt['max']), fmt(cs['max']), fmt(cb['max']), fmt(ms['max'])],
        ["P25", fmt(rrt['p25']), fmt(cs['p25']), fmt(cb['p25']), fmt(ms['p25'])],
        ["P75", fmt(rrt['p75']), fmt(cs['p75']), fmt(cb['p75']), fmt(ms['p75'])],
        ["P95", fmt(rrt['p95']), fmt(cs['p95']), fmt(cb['p95']), fmt(ms['p95'])],
        ["P99", fmt(rrt['p99']), fmt(cs['p99']), fmt(cb['p99']), fmt(ms['p99'])],
    ]

    # Calculate column widths
    col_widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            col_widths[i] = max(col_widths[i], len(cell))

    # Print header
    header_str = " | ".join(h.ljust(col_widths[i]) for i, h in enumerate(headers))
    print(header_str)
    print("-" * len(header_str))

    # Print rows
    for row in rows:
        row_str = " | ".join(cell.ljust(col_widths[i]) for i, cell in enumerate(row))
        print(row_str)


def print_summary(results: Dict):
    """Print a summary of the analysis."""
    print("\n" + "=" * 80)
    print("SMOOTHING BENCHMARK ANALYSIS")
    print("=" * 80)

    print(f"\nFile: {results['file']}")
    print(f"Total Paths: {results['total_paths']}")

    # Success rates
    print("\n" + "-" * 40)
    print("SUCCESS RATES")
    print("-" * 40)

    for name, key in [("Cubic Spline", "cubic_spline"),
                      ("Cubic Bezier", "cubic_bezier"),
                      ("Minimum Snap", "min_snap")]:
        data = results[key]
        rate = data['success_rate'] * 100
        count = data['success_count']
        total = results['total_paths']
        print(f"  {name:15}: {count:>5}/{total} ({rate:>5.1f}%)")

    # Path info
    print("\n" + "-" * 40)
    print("PATH INFORMATION")
    print("-" * 40)
    wp_stats = results['path_info']['waypoints']
    len_stats = results['path_info']['lengths']
    print(f"  Waypoints per path:")
    print(f"    Mean: {wp_stats['mean']:.1f}, Median: {wp_stats['median']:.1f}, "
          f"Min: {wp_stats['min']:.0f}, Max: {wp_stats['max']:.0f}")
    print(f"  Path lengths (meters):")
    print(f"    Mean: {len_stats['mean']:.2f}, Median: {len_stats['median']:.2f}, "
          f"Min: {len_stats['min']:.2f}, Max: {len_stats['max']:.2f}")

    # Detailed timing stats
    print("\n" + "-" * 40)
    print("TIMING STATISTICS")
    print("-" * 40)

    print("\nRRT Planning Time:")
    print_stats_table("RRT", results['rrt']['stats'])

    print("\nSmoothing Times:")
    print_stats_table("Cubic Spline", results['cubic_spline']['stats'])
    print()
    print_stats_table("Cubic Bezier", results['cubic_bezier']['stats'])
    print()
    print_stats_table("Minimum Snap", results['min_snap']['stats'])

    # Comparison table
    print_comparison_table(results)

    # Speed comparison
    print("\n" + "-" * 40)
    print("SPEED COMPARISON (relative to RRT planning)")
    print("-" * 40)

    rrt_mean = results['rrt']['stats']['mean']
    if rrt_mean > 0:
        for name, key in [("Cubic Spline", "cubic_spline"),
                          ("Cubic Bezier", "cubic_bezier"),
                          ("Minimum Snap", "min_snap")]:
            smoother_mean = results[key]['stats']['mean']
            if smoother_mean > 0:
                ratio = rrt_mean / smoother_mean
                pct_of_rrt = (smoother_mean / rrt_mean) * 100
                print(f"  {name:15}: {ratio:>8.1f}x faster ({pct_of_rrt:.2f}% of RRT time)")
            else:
                print(f"  {name:15}: N/A")

    # Total pipeline time estimate
    print("\n" + "-" * 40)
    print("TOTAL PIPELINE TIME (RRT + Smoothing)")
    print("-" * 40)

    rrt_mean = results['rrt']['stats']['mean']
    for name, key in [("Cubic Spline", "cubic_spline"),
                      ("Cubic Bezier", "cubic_bezier"),
                      ("Minimum Snap", "min_snap")]:
        smoother_mean = results[key]['stats']['mean']
        if smoother_mean > 0 and rrt_mean > 0:
            total = rrt_mean + smoother_mean
            overhead = (smoother_mean / rrt_mean) * 100
            print(f"  RRT + {name:12}: {total:>8.3f} ms (smoothing adds {overhead:.1f}% overhead)")


def save_analysis(results: Dict, output_path: str):
    """Save analysis results to a text file."""
    import io
    import sys

    # Capture print output
    old_stdout = sys.stdout
    sys.stdout = buffer = io.StringIO()

    print_summary(results)

    output = buffer.getvalue()
    sys.stdout = old_stdout

    with open(output_path, 'w') as f:
        f.write(output)

    print(f"Analysis saved to: {output_path}")


# =============================================================================
# Visualization Functions
# =============================================================================

# Color scheme
COLORS = {
    'rrt': '#2ecc71',  # Green
    'cubic_spline': '#3498db',  # Blue
    'cubic_bezier': '#9b59b6',  # Purple
    'min_snap': '#e74c3c',  # Red
    'background': '#f8f9fa',
    'grid': '#dee2e6',
    'text': '#2c3e50',
}

SMOOTHER_NAMES = {
    'rrt': 'RRT Planning',
    'cubic_spline': 'Cubic Spline',
    'cubic_bezier': 'Cubic Bezier',
    'min_snap': 'Minimum Snap',
}


def create_timing_bar_chart(results: Dict, output_path: str):
    """Create a bar chart comparing mean timing of all methods."""
    if not PLOTTING_AVAILABLE:
        return

    fig, ax = plt.subplots(figsize=(10, 6))
    fig.patch.set_facecolor(COLORS['background'])
    ax.set_facecolor(COLORS['background'])

    methods = ['rrt', 'cubic_spline', 'cubic_bezier', 'min_snap']
    names = [SMOOTHER_NAMES[m] for m in methods]
    colors = [COLORS[m] for m in methods]

    means = []
    stds = []
    for method in methods:
        if method == 'rrt':
            stats = results['rrt']['stats']
        else:
            stats = results[method]['stats']
        means.append(stats['mean'])
        stds.append(stats['std'])

    x = np.arange(len(names))
    bars = ax.bar(x, means, yerr=stds, capsize=5, color=colors, edgecolor='white', linewidth=2)

    # Add value labels on bars
    for bar, mean, std in zip(bars, means, stds):
        height = bar.get_height()
        if height > 0:
            ax.annotate(f'{mean:.2f}ms',
                        xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 5),
                        textcoords="offset points",
                        ha='center', va='bottom',
                        fontsize=11, fontweight='bold', color=COLORS['text'])

    ax.set_ylabel('Time (ms)', fontsize=12, fontweight='bold', color=COLORS['text'])
    ax.set_title('Mean Processing Time Comparison', fontsize=14, fontweight='bold', color=COLORS['text'], pad=20)
    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=11, color=COLORS['text'])
    ax.grid(axis='y', linestyle='--', alpha=0.7, color=COLORS['grid'])
    ax.set_axisbelow(True)

    # Remove top and right spines
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color(COLORS['grid'])
    ax.spines['bottom'].set_color(COLORS['grid'])

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, facecolor=COLORS['background'], edgecolor='none', bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def create_timing_box_plot(raw_data: Dict, output_path: str):
    """Create box plots showing timing distributions with appropriate Y-axis scales for each method."""
    if not PLOTTING_AVAILABLE:
        return

    # High quality settings
    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['font.size'] = 11
    plt.rcParams['axes.linewidth'] = 1.2

    fig = plt.figure(figsize=(20, 10), dpi=200)
    fig.patch.set_facecolor('#ffffff')

    # Create grid: top row has overview, bottom row has individual methods
    gs = fig.add_gridspec(2, 4, height_ratios=[1, 1.2], hspace=0.35, wspace=0.3,
                          left=0.05, right=0.98, top=0.90, bottom=0.08)

    # =========================================================================
    # Top row: Overview with log scale (spans all columns)
    # =========================================================================
    ax_overview = fig.add_subplot(gs[0, :])
    ax_overview.set_facecolor('#ffffff')

    all_data = [
        raw_data.get('rrt_times', []),
        raw_data.get('cubic_spline_times', []),
        raw_data.get('cubic_bezier_times', []),
        raw_data.get('min_snap_times', []),
    ]
    labels = ['RRT Planning', 'Cubic Spline', 'Cubic Bezier', 'Minimum Snap']
    colors = [COLORS['rrt'], COLORS['cubic_spline'], COLORS['cubic_bezier'], COLORS['min_snap']]

    valid_data = [(d, l, c) for d, l, c in zip(all_data, labels, colors) if d]

    if valid_data:
        bp_overview = ax_overview.boxplot([d[0] for d in valid_data],
                                          tick_labels=[d[1] for d in valid_data],
                                          patch_artist=True, widths=0.5)

        for patch, (_, _, color) in zip(bp_overview['boxes'], valid_data):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)
            patch.set_edgecolor('#333333')
            patch.set_linewidth(1.5)

        for element in ['whiskers', 'caps']:
            for line in bp_overview[element]:
                line.set_color('#333333')
                line.set_linewidth(1.5)

        for line in bp_overview['medians']:
            line.set_color('#ffffff')
            line.set_linewidth(2)

        # Add median value annotations
        for i, (data, label, color) in enumerate(valid_data):
            median = statistics.median(data)
            mean = statistics.mean(data)
            ax_overview.annotate(f'μ={mean:.2f}', xy=(i + 1, mean), xytext=(0, 10),
                                 textcoords='offset points', ha='center', fontsize=9,
                                 fontweight='bold', color=color)

        # Use log scale
        ax_overview.set_yscale('log')
        ax_overview.set_ylabel('Time (ms) - Log Scale', fontsize=13, fontweight='bold', color='#1a1a2e')

    ax_overview.set_title('All Methods Overview (Log Scale)', fontsize=14, fontweight='bold',
                          color='#1a1a2e', pad=15)
    ax_overview.grid(axis='y', linestyle='--', alpha=0.5, color='#cccccc')
    ax_overview.set_axisbelow(True)
    ax_overview.spines['top'].set_visible(False)
    ax_overview.spines['right'].set_visible(False)
    ax_overview.spines['left'].set_color('#cccccc')
    ax_overview.spines['bottom'].set_color('#cccccc')
    ax_overview.tick_params(axis='x', labelsize=11)

    # =========================================================================
    # Bottom row: Individual methods with appropriate Y-axis scales
    # =========================================================================

    # Calculate statistics for each method to set appropriate Y-axis limits
    method_configs = [
        ('rrt_times', 'RRT Planning', COLORS['rrt'], 'RRT'),
        ('cubic_spline_times', 'Cubic Spline', COLORS['cubic_spline'], 'Spline'),
        ('cubic_bezier_times', 'Cubic Bezier', COLORS['cubic_bezier'], 'Bezier'),
        ('min_snap_times', 'Minimum Snap', COLORS['min_snap'], 'MinSnap'),
    ]

    for idx, (data_key, title, color, short_name) in enumerate(method_configs):
        ax = fig.add_subplot(gs[1, idx])
        ax.set_facecolor('#ffffff')

        data = raw_data.get(data_key, [])

        if data:
            mean_val = statistics.mean(data)
            median_val = statistics.median(data)
            std_val = statistics.stdev(data) if len(data) > 1 else 0
            p25 = np.percentile(data, 25)
            p75 = np.percentile(data, 75)
            p95 = np.percentile(data, 95)
            p99 = np.percentile(data, 99)
            min_val = min(data)
            max_val = max(data)

            # Create boxplot
            bp = ax.boxplot([data], tick_labels=[short_name], patch_artist=True, widths=0.4)

            bp['boxes'][0].set_facecolor(color)
            bp['boxes'][0].set_alpha(0.7)
            bp['boxes'][0].set_edgecolor('#333333')
            bp['boxes'][0].set_linewidth(1.5)

            for element in ['whiskers', 'caps']:
                for line in bp[element]:
                    line.set_color('#333333')
                    line.set_linewidth(1.5)

            bp['medians'][0].set_color('#ffffff')
            bp['medians'][0].set_linewidth(2.5)

            # Set Y-axis limits based on mean ± 3*std or P99, whichever shows data better
            # Use a scale that shows the bulk of the distribution well
            y_upper = min(mean_val + 3 * std_val, p99 * 1.1, max_val)
            y_lower = max(0, min_val * 0.9)

            # Ensure we show at least to P95
            y_upper = max(y_upper, p95 * 1.15)

            ax.set_ylim(y_lower, y_upper)

            # Add statistics annotation box
            stats_text = (f'Mean: {mean_val:.3f}ms\n'
                          f'Median: {median_val:.3f}ms\n'
                          f'Std: {std_val:.3f}ms\n'
                          f'P95: {p95:.3f}ms')

            ax.text(0.98, 0.98, stats_text, transform=ax.transAxes, fontsize=9,
                    verticalalignment='top', horizontalalignment='right',
                    bbox=dict(boxstyle='round,pad=0.4', facecolor='white',
                              edgecolor=color, alpha=0.9, linewidth=1.5),
                    family='monospace')

            # Add horizontal lines for mean and std boundaries
            ax.axhline(mean_val, color=color, linestyle='--', linewidth=1.5, alpha=0.8, label='Mean')
            ax.axhline(mean_val + std_val, color=color, linestyle=':', linewidth=1, alpha=0.5)
            ax.axhline(max(0, mean_val - std_val), color=color, linestyle=':', linewidth=1, alpha=0.5)

            # Fill the mean ± std region
            ax.axhspan(max(0, mean_val - std_val), mean_val + std_val,
                       alpha=0.1, color=color, label='±1 Std Dev')

        ax.set_ylabel('Time (ms)', fontsize=11, fontweight='bold', color='#1a1a2e')
        ax.set_title(title, fontsize=13, fontweight='bold', color=color, pad=12)
        ax.grid(axis='y', linestyle='--', alpha=0.5, color='#cccccc')
        ax.set_axisbelow(True)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_color('#cccccc')
        ax.spines['bottom'].set_color('#cccccc')

    # Add note about different scales
    fig.text(0.5, 0.01, '⚠ Note: Bottom panels use different Y-axis scales optimized for each method\'s distribution',
             ha='center', fontsize=10, style='italic', color='#888888')

    plt.suptitle('Timing Distributions Comparison', fontsize=18, fontweight='bold', color='#1a1a2e', y=0.96)
    plt.savefig(output_path, dpi=200, facecolor='#ffffff', edgecolor='none', bbox_inches='tight')
    plt.close()
    plt.rcParams.update(plt.rcParamsDefault)
    print(f"Saved: {output_path}")


def create_histogram_grid(raw_data: Dict, output_path: str):
    """Create histograms for each method's timing distribution with improved visualization."""
    if not PLOTTING_AVAILABLE:
        return

    # High quality settings
    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['font.size'] = 11
    plt.rcParams['axes.linewidth'] = 1.2

    fig, axes = plt.subplots(2, 2, figsize=(16, 14), dpi=200)
    fig.patch.set_facecolor('#ffffff')

    data_sets = [
        ('rrt_times', 'RRT Planning Time', COLORS['rrt']),
        ('cubic_spline_times', 'Cubic Spline Time', COLORS['cubic_spline']),
        ('cubic_bezier_times', 'Cubic Bezier Time', COLORS['cubic_bezier']),
        ('min_snap_times', 'Minimum Snap Time', COLORS['min_snap']),
    ]

    for ax, (key, title, color) in zip(axes.flat, data_sets):
        ax.set_facecolor(COLORS['background'])
        data = raw_data.get(key, [])

        if data:
            mean_val = statistics.mean(data)
            median_val = statistics.median(data)

            # Check if data is highly skewed (mean >> median or large range)
            data_range = max(data) - min(data)
            is_skewed = (mean_val > median_val * 2) or (max(data) > median_val * 10)

            if is_skewed and min(data) > 0:
                # Use log scale for skewed data
                log_data = np.log10(np.array(data))
                n, bins, patches = ax.hist(log_data, bins=40, color=color, edgecolor='white', alpha=0.8)

                # Add statistics lines on log scale
                ax.axvline(np.log10(mean_val), color='red', linestyle='--', linewidth=2,
                           label=f'Mean: {mean_val:.3f}ms')
                ax.axvline(np.log10(median_val), color='orange', linestyle=':', linewidth=2,
                           label=f'Median: {median_val:.3f}ms')

                # Format x-axis with actual values
                ax.set_xlabel('Time (ms) - Log Scale', fontsize=10, color=COLORS['text'])

                # Custom tick labels showing actual ms values
                tick_locs = ax.get_xticks()
                ax.set_xticks(tick_locs)  # Fix: set ticks first
                tick_labels = [f'{10 ** x:.2f}' if x < 1 else f'{10 ** x:.1f}' for x in tick_locs]
                ax.set_xticklabels(tick_labels, fontsize=8)

            else:
                # Normal histogram for non-skewed data
                n, bins, patches = ax.hist(data, bins=40, color=color, edgecolor='white', alpha=0.8)
                ax.axvline(mean_val, color='red', linestyle='--', linewidth=2,
                           label=f'Mean: {mean_val:.3f}ms')
                ax.axvline(median_val, color='orange', linestyle=':', linewidth=2,
                           label=f'Median: {median_val:.3f}ms')
                ax.set_xlabel('Time (ms)', fontsize=10, color=COLORS['text'])

            ax.legend(loc='upper right', fontsize=9)

            # Add P95/P99 annotation
            p95 = np.percentile(data, 95)
            p99 = np.percentile(data, 99)
            ax.text(0.98, 0.75, f'P95: {p95:.2f}ms\nP99: {p99:.2f}ms',
                    transform=ax.transAxes, fontsize=9, ha='right', va='top',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        else:
            ax.text(0.5, 0.5, 'No Data', ha='center', va='center', fontsize=14,
                    color=COLORS['text'], transform=ax.transAxes)

        ax.set_ylabel('Frequency', fontsize=11, color=COLORS['text'])
        ax.set_title(title, fontsize=13, fontweight='bold', color=COLORS['text'], pad=10)
        ax.grid(axis='y', linestyle='--', alpha=0.5, color=COLORS['grid'])
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    plt.suptitle('Timing Distributions (Log Scale for Skewed Data)', fontsize=16, fontweight='bold',
                 color=COLORS['text'], y=1.02)
    plt.tight_layout()
    plt.savefig(output_path, dpi=200, facecolor='#ffffff', edgecolor='none', bbox_inches='tight')
    plt.close()
    plt.rcParams.update(plt.rcParamsDefault)
    print(f"Saved: {output_path}")


def create_histogram_zoomed(raw_data: Dict, output_path: str):
    """Create zoomed histograms clipped to P99 to show main distribution clearly."""
    if not PLOTTING_AVAILABLE:
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.patch.set_facecolor(COLORS['background'])

    data_sets = [
        ('rrt_times', 'RRT Planning Time', COLORS['rrt']),
        ('cubic_spline_times', 'Cubic Spline Time', COLORS['cubic_spline']),
        ('cubic_bezier_times', 'Cubic Bezier Time', COLORS['cubic_bezier']),
        ('min_snap_times', 'Minimum Snap Time', COLORS['min_snap']),
    ]

    for ax, (key, title, color) in zip(axes.flat, data_sets):
        ax.set_facecolor(COLORS['background'])
        data = raw_data.get(key, [])

        if data:
            mean_val = statistics.mean(data)
            median_val = statistics.median(data)
            p95 = np.percentile(data, 95)
            p99 = np.percentile(data, 99)

            # Clip data to P99 for better visualization
            clipped_data = [x for x in data if x <= p99]
            outliers_count = len(data) - len(clipped_data)
            outliers_pct = (outliers_count / len(data)) * 100

            # Create histogram with clipped data
            n, bins, patches = ax.hist(clipped_data, bins=50, color=color, edgecolor='white', alpha=0.8)

            # Add statistics lines
            if mean_val <= p99:
                ax.axvline(mean_val, color='red', linestyle='--', linewidth=2,
                           label=f'Mean: {mean_val:.3f}ms')
            ax.axvline(median_val, color='orange', linestyle=':', linewidth=2,
                       label=f'Median: {median_val:.3f}ms')
            ax.axvline(p95, color='purple', linestyle='-.', linewidth=1.5, alpha=0.7,
                       label=f'P95: {p95:.3f}ms')

            ax.legend(loc='upper right', fontsize=9)
            ax.set_xlabel('Time (ms)', fontsize=10, color=COLORS['text'])

            # Add info about clipped outliers
            if outliers_count > 0:
                ax.text(0.98, 0.60, f'{outliers_count} outliers\n({outliers_pct:.1f}%) > P99\nnot shown',
                        transform=ax.transAxes, fontsize=8, ha='right', va='top',
                        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
        else:
            ax.text(0.5, 0.5, 'No Data', ha='center', va='center', fontsize=14,
                    color=COLORS['text'], transform=ax.transAxes)

        ax.set_ylabel('Frequency', fontsize=10, color=COLORS['text'])
        ax.set_title(f'{title} (≤P99)', fontsize=12, fontweight='bold', color=COLORS['text'], pad=10)
        ax.grid(axis='y', linestyle='--', alpha=0.5, color=COLORS['grid'])
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    plt.suptitle('Timing Distributions (Zoomed to P99 - Outliers Excluded)', fontsize=14, fontweight='bold',
                 color=COLORS['text'], y=1.02)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, facecolor=COLORS['background'], edgecolor='none', bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def create_success_rate_chart(results: Dict, output_path: str):
    """Create a horizontal bar chart showing success rates."""
    if not PLOTTING_AVAILABLE:
        return

    fig, ax = plt.subplots(figsize=(10, 5))
    fig.patch.set_facecolor(COLORS['background'])
    ax.set_facecolor(COLORS['background'])

    methods = ['cubic_spline', 'cubic_bezier', 'min_snap']
    names = [SMOOTHER_NAMES[m] for m in methods]
    colors = [COLORS[m] for m in methods]

    success_rates = [results[m]['success_rate'] * 100 for m in methods]

    y = np.arange(len(names))
    bars = ax.barh(y, success_rates, color=colors, edgecolor='white', linewidth=2, height=0.6)

    # Add percentage labels
    for bar, rate in zip(bars, success_rates):
        width = bar.get_width()
        ax.annotate(f'{rate:.1f}%',
                    xy=(width, bar.get_y() + bar.get_height() / 2),
                    xytext=(5, 0),
                    textcoords="offset points",
                    ha='left', va='center',
                    fontsize=12, fontweight='bold', color=COLORS['text'])

    ax.set_xlim(0, 110)
    ax.set_xlabel('Success Rate (%)', fontsize=12, fontweight='bold', color=COLORS['text'])
    ax.set_title('Smoothing Success Rates', fontsize=14, fontweight='bold', color=COLORS['text'], pad=20)
    ax.set_yticks(y)
    ax.set_yticklabels(names, fontsize=11, color=COLORS['text'])
    ax.grid(axis='x', linestyle='--', alpha=0.7, color=COLORS['grid'])
    ax.set_axisbelow(True)
    ax.axvline(100, color=COLORS['grid'], linestyle='-', linewidth=1)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, facecolor=COLORS['background'], edgecolor='none', bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def create_percentile_chart(results: Dict, output_path: str):
    """Create a chart showing percentile comparisons."""
    if not PLOTTING_AVAILABLE:
        return

    fig, ax = plt.subplots(figsize=(12, 6))
    fig.patch.set_facecolor(COLORS['background'])
    ax.set_facecolor(COLORS['background'])

    methods = ['cubic_spline', 'cubic_bezier', 'min_snap']
    percentiles = ['min', 'p25', 'median', 'p75', 'p95', 'max']
    percentile_labels = ['Min', 'P25', 'Median', 'P75', 'P95', 'Max']

    x = np.arange(len(percentiles))
    width = 0.25

    for i, method in enumerate(methods):
        stats = results[method]['stats']
        if stats['count'] > 0:
            values = [stats[p] for p in percentiles]
            offset = (i - 1) * width
            bars = ax.bar(x + offset, values, width, label=SMOOTHER_NAMES[method],
                          color=COLORS[method], edgecolor='white', linewidth=1)

    ax.set_ylabel('Time (ms)', fontsize=12, fontweight='bold', color=COLORS['text'])
    ax.set_title('Timing Percentiles Comparison', fontsize=14, fontweight='bold', color=COLORS['text'], pad=20)
    ax.set_xticks(x)
    ax.set_xticklabels(percentile_labels, fontsize=11, color=COLORS['text'])
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(axis='y', linestyle='--', alpha=0.7, color=COLORS['grid'])
    ax.set_axisbelow(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, facecolor=COLORS['background'], edgecolor='none', bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def create_speedup_chart(results: Dict, output_path: str):
    """Create a chart showing speedup relative to RRT planning."""
    if not PLOTTING_AVAILABLE:
        return

    fig, ax = plt.subplots(figsize=(10, 6))
    fig.patch.set_facecolor(COLORS['background'])
    ax.set_facecolor(COLORS['background'])

    rrt_mean = results['rrt']['stats']['mean']
    if rrt_mean <= 0:
        return

    methods = ['cubic_spline', 'cubic_bezier', 'min_snap']
    names = [SMOOTHER_NAMES[m] for m in methods]
    colors = [COLORS[m] for m in methods]

    speedups = []
    for method in methods:
        smoother_mean = results[method]['stats']['mean']
        if smoother_mean > 0:
            speedups.append(rrt_mean / smoother_mean)
        else:
            speedups.append(0)

    x = np.arange(len(names))
    bars = ax.bar(x, speedups, color=colors, edgecolor='white', linewidth=2)

    # Add value labels
    for bar, speedup in zip(bars, speedups):
        height = bar.get_height()
        if height > 0:
            ax.annotate(f'{speedup:.1f}x',
                        xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 5),
                        textcoords="offset points",
                        ha='center', va='bottom',
                        fontsize=12, fontweight='bold', color=COLORS['text'])

    ax.axhline(1, color=COLORS['rrt'], linestyle='--', linewidth=2, label='RRT baseline (1x)')
    ax.set_ylabel('Speedup Factor (x times faster than RRT)', fontsize=11, fontweight='bold', color=COLORS['text'])
    ax.set_title('Smoothing Speed Relative to RRT Planning', fontsize=14, fontweight='bold', color=COLORS['text'],
                 pad=20)
    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=11, color=COLORS['text'])
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(axis='y', linestyle='--', alpha=0.7, color=COLORS['grid'])
    ax.set_axisbelow(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, facecolor=COLORS['background'], edgecolor='none', bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def create_pipeline_time_chart(results: Dict, output_path: str):
    """Create a multi-panel pipeline time chart with appropriate scales for each method."""
    if not PLOTTING_AVAILABLE:
        return

    fig, axes = plt.subplots(1, 3, figsize=(16, 6))
    fig.patch.set_facecolor(COLORS['background'])

    rrt_mean = results['rrt']['stats']['mean']

    # Panel 1: RRT + Cubic Spline (fast, ~6ms scale)
    ax1 = axes[0]
    ax1.set_facecolor(COLORS['background'])

    cs_time = results['cubic_spline']['stats']['mean']
    total_cs = rrt_mean + cs_time

    bars1 = ax1.bar([0], [rrt_mean], color=COLORS['rrt'], edgecolor='white', linewidth=2, label='RRT Planning')
    bars2 = ax1.bar([0], [cs_time], bottom=[rrt_mean], color=COLORS['cubic_spline'], edgecolor='white', linewidth=2,
                    label='Cubic Spline')

    ax1.annotate(f'{total_cs:.2f}ms', xy=(0, total_cs), xytext=(0, 5),
                 textcoords="offset points", ha='center', va='bottom',
                 fontsize=12, fontweight='bold', color=COLORS['text'])
    ax1.annotate(f'RRT: {rrt_mean:.2f}ms', xy=(0, rrt_mean / 2), ha='center', va='center',
                 fontsize=10, fontweight='bold', color='white')
    ax1.annotate(f'+{cs_time:.2f}ms', xy=(0, rrt_mean + cs_time / 2), ha='center', va='center',
                 fontsize=9, fontweight='bold', color='white')

    # Set y-axis limit with some padding for fast smoothers
    ax1.set_ylim(0, max(total_cs * 1.3, 10))
    ax1.set_ylabel('Time (ms)', fontsize=12, fontweight='bold', color=COLORS['text'])
    ax1.set_title('RRT + Cubic Spline', fontsize=13, fontweight='bold', color=COLORS['text'], pad=15)
    ax1.set_xticks([0])
    ax1.set_xticklabels([''])
    ax1.grid(axis='y', linestyle='--', alpha=0.7, color=COLORS['grid'])
    ax1.set_axisbelow(True)
    ax1.spines['top'].set_visible(False)
    ax1.spines['right'].set_visible(False)
    ax1.legend(loc='upper right', fontsize=9)

    # Add overhead percentage
    overhead_cs = (cs_time / rrt_mean) * 100
    ax1.text(0.5, 0.02, f'+{overhead_cs:.1f}% overhead', transform=ax1.transAxes,
             ha='center', fontsize=10, style='italic', color=COLORS['text'])

    # Panel 2: RRT + Cubic Bezier (fast, ~6ms scale)
    ax2 = axes[1]
    ax2.set_facecolor(COLORS['background'])

    cb_time = results['cubic_bezier']['stats']['mean']
    total_cb = rrt_mean + cb_time

    bars3 = ax2.bar([0], [rrt_mean], color=COLORS['rrt'], edgecolor='white', linewidth=2, label='RRT Planning')
    bars4 = ax2.bar([0], [cb_time], bottom=[rrt_mean], color=COLORS['cubic_bezier'], edgecolor='white', linewidth=2,
                    label='Cubic Bezier')

    ax2.annotate(f'{total_cb:.2f}ms', xy=(0, total_cb), xytext=(0, 5),
                 textcoords="offset points", ha='center', va='bottom',
                 fontsize=12, fontweight='bold', color=COLORS['text'])
    ax2.annotate(f'RRT: {rrt_mean:.2f}ms', xy=(0, rrt_mean / 2), ha='center', va='center',
                 fontsize=10, fontweight='bold', color='white')
    ax2.annotate(f'+{cb_time:.2f}ms', xy=(0, rrt_mean + cb_time / 2), ha='center', va='center',
                 fontsize=9, fontweight='bold', color='white')

    ax2.set_ylim(0, max(total_cb * 1.3, 10))
    ax2.set_ylabel('Time (ms)', fontsize=12, fontweight='bold', color=COLORS['text'])
    ax2.set_title('RRT + Cubic Bezier', fontsize=13, fontweight='bold', color=COLORS['text'], pad=15)
    ax2.set_xticks([0])
    ax2.set_xticklabels([''])
    ax2.grid(axis='y', linestyle='--', alpha=0.7, color=COLORS['grid'])
    ax2.set_axisbelow(True)
    ax2.spines['top'].set_visible(False)
    ax2.spines['right'].set_visible(False)
    ax2.legend(loc='upper right', fontsize=9)

    overhead_cb = (cb_time / rrt_mean) * 100
    ax2.text(0.5, 0.02, f'+{overhead_cb:.1f}% overhead', transform=ax2.transAxes,
             ha='center', fontsize=10, style='italic', color=COLORS['text'])

    # Panel 3: RRT + Minimum Snap (slow, ~180ms scale)
    ax3 = axes[2]
    ax3.set_facecolor(COLORS['background'])

    ms_time = results['min_snap']['stats']['mean']
    total_ms = rrt_mean + ms_time

    bars5 = ax3.bar([0], [rrt_mean], color=COLORS['rrt'], edgecolor='white', linewidth=2, label='RRT Planning')
    bars6 = ax3.bar([0], [ms_time], bottom=[rrt_mean], color=COLORS['min_snap'], edgecolor='white', linewidth=2,
                    label='Minimum Snap')

    ax3.annotate(f'{total_ms:.1f}ms', xy=(0, total_ms), xytext=(0, 5),
                 textcoords="offset points", ha='center', va='bottom',
                 fontsize=12, fontweight='bold', color=COLORS['text'])
    ax3.annotate(f'RRT: {rrt_mean:.2f}ms', xy=(0, rrt_mean / 2), ha='center', va='center',
                 fontsize=8, fontweight='bold', color='white')
    ax3.annotate(f'+{ms_time:.1f}ms', xy=(0, rrt_mean + ms_time / 2), ha='center', va='center',
                 fontsize=10, fontweight='bold', color='white')

    ax3.set_ylim(0, total_ms * 1.2)
    ax3.set_ylabel('Time (ms)', fontsize=12, fontweight='bold', color=COLORS['text'])
    ax3.set_title('RRT + Minimum Snap', fontsize=13, fontweight='bold', color=COLORS['text'], pad=15)
    ax3.set_xticks([0])
    ax3.set_xticklabels([''])
    ax3.grid(axis='y', linestyle='--', alpha=0.7, color=COLORS['grid'])
    ax3.set_axisbelow(True)
    ax3.spines['top'].set_visible(False)
    ax3.spines['right'].set_visible(False)
    ax3.legend(loc='upper right', fontsize=9)

    overhead_ms = (ms_time / rrt_mean) * 100
    ax3.text(0.5, 0.02, f'+{overhead_ms:.0f}% overhead', transform=ax3.transAxes,
             ha='center', fontsize=10, style='italic', color=COLORS['text'])

    # Add note about different scales
    fig.text(0.5, 0.01, '⚠ Note: Y-axes use different scales to show appropriate resolution for each method',
             ha='center', fontsize=10, style='italic', color='#7f8c8d')

    plt.suptitle('Total Pipeline Time (RRT + Smoothing)', fontsize=14, fontweight='bold', color=COLORS['text'], y=0.98)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(output_path, dpi=150, facecolor=COLORS['background'], edgecolor='none', bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def create_summary_dashboard(results: Dict, raw_data: Dict, output_path: str):
    """Create a high-quality summary dashboard with statistics table and pipeline time charts."""
    if not PLOTTING_AVAILABLE:
        return

    # High quality figure settings
    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['font.size'] = 11
    plt.rcParams['axes.linewidth'] = 1.2
    plt.rcParams['grid.linewidth'] = 0.8

    fig = plt.figure(figsize=(18, 14), dpi=200)
    fig.patch.set_facecolor('#ffffff')

    # Title
    fig.suptitle('Smoothing Benchmark Results', fontsize=22, fontweight='bold',
                 color='#1a1a2e', y=0.97)

    # Create grid: table on top (larger), 3 pipeline charts on bottom
    gs = fig.add_gridspec(2, 3, height_ratios=[1.6, 1], hspace=0.3, wspace=0.25,
                          left=0.06, right=0.94, top=0.90, bottom=0.08)

    # =========================================================================
    # 1. Main Statistics Table (top row, spans all 3 columns)
    # =========================================================================
    ax_table = fig.add_subplot(gs[0, :])
    ax_table.set_facecolor('#ffffff')
    ax_table.axis('off')

    # Build comprehensive table data
    table_data = [
        ['Metric', 'RRT Planning', 'Cubic Spline', 'Cubic Bezier', 'Minimum Snap'],
        ['Count', f"{results['rrt']['stats']['count']:,}",
         f"{results['cubic_spline']['stats']['count']:,}",
         f"{results['cubic_bezier']['stats']['count']:,}",
         f"{results['min_snap']['stats']['count']:,}"],
        ['Mean (ms)', f"{results['rrt']['stats']['mean']:.3f}",
         f"{results['cubic_spline']['stats']['mean']:.3f}",
         f"{results['cubic_bezier']['stats']['mean']:.3f}",
         f"{results['min_snap']['stats']['mean']:.3f}"],
        ['Median (ms)', f"{results['rrt']['stats']['median']:.3f}",
         f"{results['cubic_spline']['stats']['median']:.3f}",
         f"{results['cubic_bezier']['stats']['median']:.3f}",
         f"{results['min_snap']['stats']['median']:.3f}"],
        ['Std Dev (ms)', f"{results['rrt']['stats']['std']:.3f}",
         f"{results['cubic_spline']['stats']['std']:.3f}",
         f"{results['cubic_bezier']['stats']['std']:.3f}",
         f"{results['min_snap']['stats']['std']:.3f}"],
        ['Min (ms)', f"{results['rrt']['stats']['min']:.3f}",
         f"{results['cubic_spline']['stats']['min']:.3f}",
         f"{results['cubic_bezier']['stats']['min']:.3f}",
         f"{results['min_snap']['stats']['min']:.3f}"],
        ['Max (ms)', f"{results['rrt']['stats']['max']:.3f}",
         f"{results['cubic_spline']['stats']['max']:.3f}",
         f"{results['cubic_bezier']['stats']['max']:.3f}",
         f"{results['min_snap']['stats']['max']:.3f}"],
        ['P25 (ms)', f"{results['rrt']['stats']['p25']:.3f}",
         f"{results['cubic_spline']['stats']['p25']:.3f}",
         f"{results['cubic_bezier']['stats']['p25']:.3f}",
         f"{results['min_snap']['stats']['p25']:.3f}"],
        ['P75 (ms)', f"{results['rrt']['stats']['p75']:.3f}",
         f"{results['cubic_spline']['stats']['p75']:.3f}",
         f"{results['cubic_bezier']['stats']['p75']:.3f}",
         f"{results['min_snap']['stats']['p75']:.3f}"],
        ['P95 (ms)', f"{results['rrt']['stats']['p95']:.3f}",
         f"{results['cubic_spline']['stats']['p95']:.3f}",
         f"{results['cubic_bezier']['stats']['p95']:.3f}",
         f"{results['min_snap']['stats']['p95']:.3f}"],
        ['P99 (ms)', f"{results['rrt']['stats']['p99']:.3f}",
         f"{results['cubic_spline']['stats']['p99']:.3f}",
         f"{results['cubic_bezier']['stats']['p99']:.3f}",
         f"{results['min_snap']['stats']['p99']:.3f}"],
    ]

    # Create table with better styling
    table = ax_table.table(cellText=table_data, loc='center', cellLoc='center',
                           colWidths=[0.14, 0.17, 0.17, 0.17, 0.17])
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    table.scale(1.2, 2.2)

    # Style header row with method colors
    header_colors = ['#2c3e50', COLORS['rrt'], COLORS['cubic_spline'],
                     COLORS['cubic_bezier'], COLORS['min_snap']]
    for j, color in enumerate(header_colors):
        table[(0, j)].set_facecolor(color)
        table[(0, j)].set_text_props(color='white', fontweight='bold', fontsize=13)
        table[(0, j)].set_height(0.06)

    # Style data rows with alternating colors and proper formatting
    for i in range(1, len(table_data)):
        for j in range(5):
            if i % 2 == 0:
                table[(i, j)].set_facecolor('#f8f9fa')
            else:
                table[(i, j)].set_facecolor('#ffffff')
            table[(i, j)].set_text_props(fontsize=11)
            # Bold the metric column
            if j == 0:
                table[(i, j)].set_text_props(fontweight='bold', fontsize=11)

    ax_table.set_title('Timing Statistics', fontsize=16, fontweight='bold',
                       color='#1a1a2e', pad=25, loc='center')

    # =========================================================================
    # 2. Pipeline Time Charts (bottom row, 3 separate panels)
    # =========================================================================
    rrt_mean = results['rrt']['stats']['mean']

    # Define chart configurations
    chart_configs = [
        ('cubic_spline', 'RRT + Cubic Spline', COLORS['cubic_spline']),
        ('cubic_bezier', 'RRT + Cubic Bezier', COLORS['cubic_bezier']),
        ('min_snap', 'RRT + Minimum Snap', COLORS['min_snap']),
    ]

    for idx, (method_key, title, color) in enumerate(chart_configs):
        ax = fig.add_subplot(gs[1, idx])
        ax.set_facecolor('#ffffff')

        smoother_time = results[method_key]['stats']['mean']
        total_time = rrt_mean + smoother_time
        overhead_pct = (smoother_time / rrt_mean) * 100

        # Create stacked bar
        bar_width = 0.5
        bars1 = ax.bar([0], [rrt_mean], bar_width, color=COLORS['rrt'],
                       edgecolor='white', linewidth=2, label='RRT Planning')
        bars2 = ax.bar([0], [smoother_time], bar_width, bottom=[rrt_mean],
                       color=color, edgecolor='white', linewidth=2, label='Smoothing')

        # Add value annotations inside bars
        # RRT portion
        if rrt_mean > total_time * 0.15:  # Only show if bar is tall enough
            ax.annotate(f'{rrt_mean:.2f}ms', xy=(0, rrt_mean / 2), ha='center', va='center',
                        fontsize=11, fontweight='bold', color='white')

        # Smoother portion
        if smoother_time > total_time * 0.08:
            ax.annotate(f'{smoother_time:.2f}ms', xy=(0, rrt_mean + smoother_time / 2),
                        ha='center', va='center', fontsize=10, fontweight='bold', color='white')

        # Total time label on top
        ax.annotate(f'Total: {total_time:.2f}ms', xy=(0, total_time), xytext=(0, 8),
                    textcoords="offset points", ha='center', va='bottom',
                    fontsize=13, fontweight='bold', color='#1a1a2e',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='#f0f0f0', edgecolor='none'))

        # Overhead annotation
        ax.text(0.5, -0.08, f'+{overhead_pct:.1f}% overhead', transform=ax.transAxes,
                ha='center', fontsize=10, style='italic', color='#666666')

        # Set appropriate y-axis limits based on method
        if method_key == 'min_snap':
            ax.set_ylim(0, total_time * 1.25)
        else:
            # For fast smoothers, use consistent scale
            max_fast_total = rrt_mean + max(results['cubic_spline']['stats']['mean'],
                                            results['cubic_bezier']['stats']['mean'])
            ax.set_ylim(0, max(max_fast_total * 1.4, 10))

        ax.set_ylabel('Time (ms)', fontsize=12, fontweight='bold', color='#1a1a2e')
        ax.set_title(title, fontsize=14, fontweight='bold', color='#1a1a2e', pad=15)
        ax.set_xticks([])
        ax.grid(axis='y', linestyle='--', alpha=0.5, color='#cccccc')
        ax.set_axisbelow(True)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_color('#cccccc')
        ax.spines['bottom'].set_color('#cccccc')

        # Legend
        ax.legend(loc='upper right', fontsize=9, framealpha=0.9)

    # Add note about different scales
    fig.text(0.5, 0.02, '⚠ Note: Y-axes use different scales to show appropriate resolution for each method',
             ha='center', fontsize=11, style='italic', color='#888888')

    plt.savefig(output_path, dpi=200, facecolor='#ffffff', edgecolor='none', bbox_inches='tight')
    plt.close()

    # Reset rcParams
    plt.rcParams.update(plt.rcParamsDefault)

    print(f"Saved: {output_path}")


def generate_all_plots(filepath: str, results: Dict, output_dir: str):
    """Generate all visualization plots."""
    if not PLOTTING_AVAILABLE:
        print("Plotting not available. Install matplotlib: pip install matplotlib")
        return

    # Load raw data for histograms/box plots
    raw_data = load_raw_timing_data(filepath)

    base_name = Path(filepath).stem

    print("\nGenerating visualizations...")

    # Generate individual plots (only the essential ones)
    create_timing_box_plot(raw_data, os.path.join(output_dir, f"{base_name}_timing_boxplot.png"))
    create_histogram_grid(raw_data, os.path.join(output_dir, f"{base_name}_histograms.png"))

    # Generate summary dashboard (contains table + pipeline time charts)
    create_summary_dashboard(results, raw_data, os.path.join(output_dir, f"{base_name}_dashboard.png"))

    print(f"\nAll plots saved to: {output_dir}")


def load_raw_timing_data(filepath: str) -> Dict[str, List[float]]:
    """Load raw timing data from CSV for histograms/box plots."""
    rows = load_csv(filepath)

    raw_data = {
        'rrt_times': [],
        'cubic_spline_times': [],
        'cubic_bezier_times': [],
        'min_snap_times': [],
    }

    for row in rows:
        rrt_time = float(row.get('rrt_time_ms', 0))
        if rrt_time > 0:
            raw_data['rrt_times'].append(rrt_time)

        if int(row.get('cubic_spline_success', 0)) == 1:
            time_val = float(row.get('cubic_spline_time_ms', 0))
            if time_val > 0:
                raw_data['cubic_spline_times'].append(time_val)

        if int(row.get('cubic_bezier_success', 0)) == 1:
            time_val = float(row.get('cubic_bezier_time_ms', 0))
            if time_val > 0:
                raw_data['cubic_bezier_times'].append(time_val)

        if int(row.get('min_snap_success', 0)) == 1:
            time_val = float(row.get('min_snap_time_ms', 0))
            if time_val > 0:
                raw_data['min_snap_times'].append(time_val)

    return raw_data


def main():
    parser = argparse.ArgumentParser(
        description="Analyze smoothing benchmark results",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Examples:
  %(prog)s                              # Analyze most recent results + generate plots
  %(prog)s path/to/smoothing_*.csv      # Specific file
  %(prog)s --all                        # Analyze all result files
  %(prog)s --no-plot                    # Skip visualization generation
  %(prog)s --save                       # Also save analysis to text file

Default results directory: {DEFAULT_RESULTS_DIR}
"""
    )
    parser.add_argument(
        'input_csv',
        nargs='?',
        default=None,
        help="Path to smoothing benchmark CSV file (default: most recent)"
    )
    parser.add_argument(
        '--results-dir',
        default=DEFAULT_RESULTS_DIR,
        help=f"Directory to search for results (default: {DEFAULT_RESULTS_DIR})"
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help="Analyze all result files in the results directory"
    )
    parser.add_argument(
        '--no-plot',
        action='store_true',
        help="Disable visualization plot generation"
    )
    parser.add_argument(
        '--save',
        action='store_true',
        help="Save analysis to a text file"
    )
    parser.add_argument(
        '-q', '--quiet',
        action='store_true',
        help="Only show comparison table"
    )

    args = parser.parse_args()

    if args.all:
        # Analyze all files
        files = find_all_results(args.results_dir)
        if not files:
            print(f"No results found in: {args.results_dir}")
            sys.exit(1)

        print(f"Found {len(files)} result files\n")

        for filepath in files:
            print("\n" + "=" * 80)
            results = analyze_file(filepath)
            if results:
                if args.quiet:
                    print(f"File: {filepath}")
                    print_comparison_table(results)
                else:
                    print_summary(results)

                if args.save:
                    output_path = filepath.replace('.csv', '_analysis.txt')
                    save_analysis(results, output_path)

                if not args.no_plot:
                    output_dir = os.path.dirname(filepath)
                    generate_all_plots(filepath, results, output_dir)
    else:
        # Analyze single file
        if args.input_csv is None:
            args.input_csv = find_most_recent_results(args.results_dir)
            if args.input_csv is None:
                print(f"No results found in: {args.results_dir}")
                sys.exit(1)
            print(f"Using most recent results: {args.input_csv}\n")

        if not os.path.exists(args.input_csv):
            print(f"Error: File not found: {args.input_csv}")
            sys.exit(1)

        results = analyze_file(args.input_csv)

        if results:
            if args.quiet:
                print_comparison_table(results)
            else:
                print_summary(results)

            if args.save:
                output_path = args.input_csv.replace('.csv', '_analysis.txt')
                save_analysis(results, output_path)

            if not args.no_plot:
                output_dir = os.path.dirname(args.input_csv) or '.'
                generate_all_plots(args.input_csv, results, output_dir)


if __name__ == "__main__":
    main()