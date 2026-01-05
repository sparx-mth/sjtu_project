#!/usr/bin/env python3
"""
Smoothing Benchmark Runner
==========================
Processes RRT path planning results and benchmarks different smoothing algorithms.

Input: RRT benchmark JSON file (from rrt_first_benchmark)
Output: CSV with timing for each path and each smoothing method

Usage:
    python run_smoothing_benchmark.py                           # Use most recent RRT results
    python run_smoothing_benchmark.py path/to/rrt_first_*.json  # Specific file
    python run_smoothing_benchmark.py -o results/               # Custom output dir
    python run_smoothing_benchmark.py --iterations 10           # Multiple iterations
"""

import argparse
import glob
import json
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Optional, Tuple
import csv

# Add smoothers to path
sys.path.insert(0, str(Path(__file__).parent))

from smoothers.cubic_spline_smoother import CubicSplineSmoother
from smoothers.cubic_bezier_smoother import CubicBezierSmoother

try:
    from smoothers.min_snap_smoother import MinSnapSmoother, MINSNAP_AVAILABLE
except ImportError:
    MINSNAP_AVAILABLE = False
    MinSnapSmoother = None

# Default paths
DEFAULT_RRT_RESULTS_DIR = "/home/nadavc/PycharmProjects/sjtu_project/benchmark/rrt_benchmark/results"
DEFAULT_OUTPUT_DIR = "/home/nadavc/PycharmProjects/sjtu_project/benchmark/smoothing_benchmark/results"


def find_most_recent_rrt_file(results_dir: str) -> Optional[str]:
    """
    Find the most recent RRT results JSON file in the given directory.

    Looks for files matching 'rrt_first_*.json' pattern and returns
    the one with the most recent modification time.

    Args:
        results_dir: Directory to search for RRT result files

    Returns:
        Path to most recent file, or None if no files found
    """
    pattern = os.path.join(results_dir, "rrt_first_*.json")
    files = glob.glob(pattern)

    if not files:
        return None

    # Sort by modification time, most recent first
    files.sort(key=os.path.getmtime, reverse=True)
    return files[0]


@dataclass
class SmootherTiming:
    """Timing results for a single smoother on a single path."""
    smoother_name: str
    time_ms: float
    success: bool
    error_message: str = ""
    trajectory_length: float = 0.0  # Resulting trajectory length


@dataclass
class PathResult:
    """Complete benchmark result for a single path."""
    pair_id: int
    iteration_id: int
    rrt_planning_time_ms: float
    path_length: float
    num_waypoints: int
    cubic_spline: SmootherTiming = None
    cubic_bezier: SmootherTiming = None
    min_snap: SmootherTiming = None


@dataclass
class BenchmarkSession:
    """Complete benchmark session data."""
    input_file: str
    timestamp: str
    num_paths: int
    smoothing_iterations: int  # How many times to run each smoother for averaging
    results: List[PathResult] = field(default_factory=list)

    # Summary statistics
    total_paths_processed: int = 0
    cubic_spline_success_count: int = 0
    cubic_bezier_success_count: int = 0
    min_snap_success_count: int = 0


def time_smoother(smoother_class, waypoints_x: List[float], waypoints_y: List[float],
                  iterations: int = 1, **kwargs) -> SmootherTiming:
    """
    Time a smoother on given waypoints.

    Args:
        smoother_class: The smoother class to instantiate
        waypoints_x, waypoints_y: Path waypoints
        iterations: Number of times to run for averaging
        **kwargs: Additional arguments for the smoother

    Returns:
        SmootherTiming with averaged time
    """
    smoother_name = smoother_class.__name__

    if len(waypoints_x) < 2:
        return SmootherTiming(
            smoother_name=smoother_name,
            time_ms=0.0,
            success=False,
            error_message="Path has fewer than 2 waypoints"
        )

    times = []
    trajectory = None

    for i in range(iterations):
        try:
            start_time = time.perf_counter()
            trajectory = smoother_class(waypoints_x, waypoints_y, **kwargs)
            end_time = time.perf_counter()
            times.append((end_time - start_time) * 1000)  # Convert to ms
        except Exception as e:
            return SmootherTiming(
                smoother_name=smoother_name,
                time_ms=0.0,
                success=False,
                error_message=str(e)
            )

    avg_time = sum(times) / len(times)
    traj_length = trajectory.total_length if trajectory else 0.0

    return SmootherTiming(
        smoother_name=smoother_name,
        time_ms=avg_time,
        success=True,
        trajectory_length=traj_length
    )


def process_path(path_x: List[float], path_y: List[float],
                 pair_id: int, iteration_id: int, rrt_time_ms: float,
                 path_length: float, smoothing_iterations: int = 1) -> PathResult:
    """
    Process a single path with all smoothers.

    Args:
        path_x, path_y: Path waypoints
        pair_id: The point pair ID
        iteration_id: The iteration ID within that pair
        rrt_time_ms: Original RRT planning time
        path_length: Original path length
        smoothing_iterations: How many times to run each smoother

    Returns:
        PathResult with timing for all smoothers
    """
    result = PathResult(
        pair_id=pair_id,
        iteration_id=iteration_id,
        rrt_planning_time_ms=rrt_time_ms,
        path_length=path_length,
        num_waypoints=len(path_x)
    )

    # Cubic Spline
    result.cubic_spline = time_smoother(
        CubicSplineSmoother, path_x, path_y,
        iterations=smoothing_iterations
    )

    # Cubic Bezier (Hermite)
    result.cubic_bezier = time_smoother(
        CubicBezierSmoother, path_x, path_y,
        iterations=smoothing_iterations
    )

    # Minimum Snap (if available)
    if MINSNAP_AVAILABLE and MinSnapSmoother is not None:
        result.min_snap = time_smoother(
            MinSnapSmoother, path_x, path_y,
            iterations=smoothing_iterations
        )
    else:
        result.min_snap = SmootherTiming(
            smoother_name="MinSnapSmoother",
            time_ms=0.0,
            success=False,
            error_message="minsnap-trajectories not installed"
        )

    return result


def load_rrt_results(json_path: str) -> Dict:
    """Load RRT benchmark results from JSON file."""
    with open(json_path, 'r') as f:
        return json.load(f)


def run_benchmark(json_path: str, smoothing_iterations: int = 1,
                  verbose: bool = True) -> BenchmarkSession:
    """
    Run the full smoothing benchmark on RRT results.

    Args:
        json_path: Path to RRT benchmark JSON file
        smoothing_iterations: How many times to run each smoother for averaging
        verbose: Print progress

    Returns:
        BenchmarkSession with all results
    """
    if verbose:
        print(f"Loading RRT results from: {json_path}")

    data = load_rrt_results(json_path)

    session = BenchmarkSession(
        input_file=json_path,
        timestamp=datetime.now().strftime("%Y%m%d_%H%M%S"),
        num_paths=0,
        smoothing_iterations=smoothing_iterations
    )

    # Count total paths
    total_paths = sum(
        len([it for it in pr['iterations'] if it['success']])
        for pr in data.get('pair_results', [])
    )
    session.num_paths = total_paths

    if verbose:
        print(f"Found {total_paths} successful paths to process")
        print(f"Smoothing iterations per path: {smoothing_iterations}")
        if not MINSNAP_AVAILABLE:
            print("Warning: minsnap-trajectories not installed, skipping MinSnap")
        print()

    processed = 0

    for pair_result in data.get('pair_results', []):
        pair_id = pair_result['pair_id']

        for iteration in pair_result['iterations']:
            if not iteration['success']:
                continue

            iteration_id = iteration['iteration_id']
            path_x = iteration.get('path_x', [])
            path_y = iteration.get('path_y', [])
            rrt_time = iteration.get('planning_time_ms', 0)
            path_length = iteration.get('path_length', 0)

            if len(path_x) < 2 or len(path_y) < 2:
                continue

            result = process_path(
                path_x, path_y,
                pair_id, iteration_id,
                rrt_time, path_length,
                smoothing_iterations
            )

            session.results.append(result)
            session.total_paths_processed += 1

            if result.cubic_spline and result.cubic_spline.success:
                session.cubic_spline_success_count += 1
            if result.cubic_bezier and result.cubic_bezier.success:
                session.cubic_bezier_success_count += 1
            if result.min_snap and result.min_snap.success:
                session.min_snap_success_count += 1

            processed += 1
            if verbose and processed % 50 == 0:
                print(f"  Processed {processed}/{total_paths} paths...")

    if verbose:
        print(f"\nCompleted processing {session.total_paths_processed} paths")

    return session


def save_csv(session: BenchmarkSession, output_path: str):
    """
    Save benchmark results to CSV file.

    Format:
    pair_id, iteration_id, rrt_time_ms, num_waypoints, path_length,
    cubic_spline_time_ms, cubic_spline_success,
    cubic_bezier_time_ms, cubic_bezier_success,
    min_snap_time_ms, min_snap_success
    """
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)

        # Header
        writer.writerow([
            'pair_id', 'iteration_id', 'rrt_time_ms', 'num_waypoints', 'path_length',
            'cubic_spline_time_ms', 'cubic_spline_success', 'cubic_spline_traj_length',
            'cubic_bezier_time_ms', 'cubic_bezier_success', 'cubic_bezier_traj_length',
            'min_snap_time_ms', 'min_snap_success', 'min_snap_traj_length'
        ])

        # Data rows
        for result in session.results:
            row = [
                result.pair_id,
                result.iteration_id,
                f"{result.rrt_planning_time_ms:.3f}",
                result.num_waypoints,
                f"{result.path_length:.4f}",
            ]

            # Cubic Spline
            if result.cubic_spline:
                row.extend([
                    f"{result.cubic_spline.time_ms:.4f}",
                    1 if result.cubic_spline.success else 0,
                    f"{result.cubic_spline.trajectory_length:.4f}"
                ])
            else:
                row.extend(["0", 0, "0"])

            # Cubic Bezier
            if result.cubic_bezier:
                row.extend([
                    f"{result.cubic_bezier.time_ms:.4f}",
                    1 if result.cubic_bezier.success else 0,
                    f"{result.cubic_bezier.trajectory_length:.4f}"
                ])
            else:
                row.extend(["0", 0, "0"])

            # Min Snap
            if result.min_snap:
                row.extend([
                    f"{result.min_snap.time_ms:.4f}",
                    1 if result.min_snap.success else 0,
                    f"{result.min_snap.trajectory_length:.4f}"
                ])
            else:
                row.extend(["0", 0, "0"])

            writer.writerow(row)

    print(f"Saved CSV: {output_path}")


def save_json(session: BenchmarkSession, output_path: str):
    """Save benchmark results to JSON file."""

    def result_to_dict(r: PathResult) -> dict:
        d = {
            'pair_id': r.pair_id,
            'iteration_id': r.iteration_id,
            'rrt_planning_time_ms': r.rrt_planning_time_ms,
            'path_length': r.path_length,
            'num_waypoints': r.num_waypoints,
        }

        for name, timing in [('cubic_spline', r.cubic_spline),
                             ('cubic_bezier', r.cubic_bezier),
                             ('min_snap', r.min_snap)]:
            if timing:
                d[name] = {
                    'time_ms': timing.time_ms,
                    'success': timing.success,
                    'trajectory_length': timing.trajectory_length,
                    'error_message': timing.error_message if not timing.success else ""
                }
            else:
                d[name] = {'time_ms': 0, 'success': False, 'error_message': 'Not run'}

        return d

    output = {
        'benchmark_type': 'smoothing_benchmark',
        'input_file': session.input_file,
        'timestamp': session.timestamp,
        'smoothing_iterations': session.smoothing_iterations,
        'summary': {
            'total_paths': session.num_paths,
            'paths_processed': session.total_paths_processed,
            'cubic_spline_success_count': session.cubic_spline_success_count,
            'cubic_bezier_success_count': session.cubic_bezier_success_count,
            'min_snap_success_count': session.min_snap_success_count,
        },
        'results': [result_to_dict(r) for r in session.results]
    }

    with open(output_path, 'w') as f:
        json.dump(output, f, indent=2)

    print(f"Saved JSON: {output_path}")


def save_summary(session: BenchmarkSession, output_path: str):
    """Save human-readable summary."""

    # Compute statistics
    spline_times = [r.cubic_spline.time_ms for r in session.results if r.cubic_spline and r.cubic_spline.success]
    bezier_times = [r.cubic_bezier.time_ms for r in session.results if r.cubic_bezier and r.cubic_bezier.success]
    minsnap_times = [r.min_snap.time_ms for r in session.results if r.min_snap and r.min_snap.success]
    rrt_times = [r.rrt_planning_time_ms for r in session.results]

    def stats(values):
        if not values:
            return "N/A"
        import statistics
        mean = statistics.mean(values)
        std = statistics.stdev(values) if len(values) > 1 else 0
        return f"{mean:.3f} ± {std:.3f} ms (min={min(values):.3f}, max={max(values):.3f})"

    with open(output_path, 'w') as f:
        f.write("=" * 60 + "\n")
        f.write("SMOOTHING BENCHMARK SUMMARY\n")
        f.write("=" * 60 + "\n\n")

        f.write(f"Input file: {session.input_file}\n")
        f.write(f"Timestamp: {session.timestamp}\n")
        f.write(f"Smoothing iterations per path: {session.smoothing_iterations}\n\n")

        f.write("PATHS PROCESSED:\n")
        f.write(f"  Total paths: {session.num_paths}\n")
        f.write(f"  Successfully processed: {session.total_paths_processed}\n\n")

        f.write("SUCCESS RATES:\n")
        total = session.total_paths_processed
        f.write(f"  Cubic Spline: {session.cubic_spline_success_count}/{total} ")
        f.write(f"({100 * session.cubic_spline_success_count / total:.1f}%)\n" if total > 0 else "\n")
        f.write(f"  Cubic Bezier: {session.cubic_bezier_success_count}/{total} ")
        f.write(f"({100 * session.cubic_bezier_success_count / total:.1f}%)\n" if total > 0 else "\n")
        f.write(f"  Minimum Snap: {session.min_snap_success_count}/{total} ")
        f.write(f"({100 * session.min_snap_success_count / total:.1f}%)\n\n" if total > 0 else "\n\n")

        f.write("TIMING STATISTICS:\n")
        f.write(f"  RRT Planning:  {stats(rrt_times)}\n")
        f.write(f"  Cubic Spline:  {stats(spline_times)}\n")
        f.write(f"  Cubic Bezier:  {stats(bezier_times)}\n")
        f.write(f"  Minimum Snap:  {stats(minsnap_times)}\n")

    print(f"Saved summary: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Benchmark trajectory smoothing algorithms on RRT paths",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Examples:
  %(prog)s                              # Use most recent RRT results
  %(prog)s path/to/rrt_first_*.json     # Specific file
  %(prog)s -o results/ --iterations 10  # Custom output, multiple iterations

Default paths:
  RRT results: {DEFAULT_RRT_RESULTS_DIR}
  Output:      {DEFAULT_OUTPUT_DIR}
"""
    )
    parser.add_argument(
        'input_json',
        nargs='?',
        default=None,
        help="Path to RRT benchmark JSON file (default: most recent in RRT results dir)"
    )
    parser.add_argument(
        '-o', '--output-dir',
        default=DEFAULT_OUTPUT_DIR,
        help=f"Output directory for results (default: {DEFAULT_OUTPUT_DIR})"
    )
    parser.add_argument(
        '-i', '--iterations',
        type=int,
        default=1,
        help="Number of smoothing iterations per path for averaging (default: 1)"
    )
    parser.add_argument(
        '-q', '--quiet',
        action='store_true',
        help="Suppress progress output"
    )
    parser.add_argument(
        '--rrt-dir',
        default=DEFAULT_RRT_RESULTS_DIR,
        help=f"Directory to search for RRT results (default: {DEFAULT_RRT_RESULTS_DIR})"
    )

    args = parser.parse_args()

    # Determine input file
    if args.input_json is None:
        # Find most recent RRT results file
        args.input_json = find_most_recent_rrt_file(args.rrt_dir)
        if args.input_json is None:
            print(f"Error: No RRT results found in: {args.rrt_dir}")
            print(f"Looking for files matching: rrt_first_*.json")
            sys.exit(1)
        if not args.quiet:
            print(f"Using most recent RRT results: {args.input_json}")

    if not os.path.exists(args.input_json):
        print(f"Error: Input file not found: {args.input_json}")
        sys.exit(1)

    # Create output directory if needed
    os.makedirs(args.output_dir, exist_ok=True)

    # Run benchmark
    session = run_benchmark(
        args.input_json,
        smoothing_iterations=args.iterations,
        verbose=not args.quiet
    )

    # Generate output filename
    input_basename = Path(args.input_json).stem
    output_base = os.path.join(args.output_dir, f"smoothing_{input_basename}_{session.timestamp}")

    # Save results
    save_csv(session, output_base + ".csv")
    save_json(session, output_base + ".json")
    save_summary(session, output_base + ".txt")

    if not args.quiet:
        print(f"\nResults saved to:")
        print(f"  {output_base}.csv")
        print(f"  {output_base}.json")
        print(f"  {output_base}.txt")


if __name__ == "__main__":
    main()