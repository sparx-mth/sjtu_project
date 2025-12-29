#!/usr/bin/env python3
"""
smoothing_benchmark.py
----------------------
Benchmark the trajectory smoothing separately from RRT*.

Usage:
    python3 smoothing_benchmark.py --input results/benchmark_XXXX.json
    python3 smoothing_benchmark.py --synthetic --num 100
"""

import sys
import json
import time
import argparse
import math
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
from scipy.interpolate import CubicSpline


@dataclass
class TrajectoryPoint:
    x: float
    y: float
    vx: float
    vy: float
    curvature: float
    s: float


class SmoothTrajectory:
    """Cubic spline trajectory smoother (same as your trajectory_smoother.py)."""

    def __init__(self, waypoints_x: List[float], waypoints_y: List[float],
                 min_spacing: float = 0.05):
        # Filter close points
        xs, ys = self._filter(waypoints_x, waypoints_y, min_spacing)
        if len(xs) < 2:
            raise ValueError("Need at least 2 points")

        self._xs = np.array(xs)
        self._ys = np.array(ys)
        self._s = self._arc_length(self._xs, self._ys)
        self.total_length = self._s[-1]

        bc = 'natural' if len(xs) >= 4 else 'clamped'
        self._spline_x = CubicSpline(self._s, self._xs, bc_type=bc)
        self._spline_y = CubicSpline(self._s, self._ys, bc_type=bc)

    @staticmethod
    def _filter(xs, ys, min_spacing):
        if not xs:
            return [], []
        fx, fy = [xs[0]], [ys[0]]
        for x, y in zip(xs[1:], ys[1:]):
            if math.hypot(x - fx[-1], y - fy[-1]) >= min_spacing:
                fx.append(x)
                fy.append(y)
        if len(xs) > 1 and math.hypot(xs[-1] - fx[-1], ys[-1] - fy[-1]) >= min_spacing * 0.5:
            fx.append(xs[-1])
            fy.append(ys[-1])
        return fx, fy

    @staticmethod
    def _arc_length(xs, ys):
        dx = np.diff(xs)
        dy = np.diff(ys)
        ds = np.sqrt(dx ** 2 + dy ** 2)
        s = np.zeros(len(xs))
        s[1:] = np.cumsum(ds)
        return s

    def get_point(self, s: float) -> TrajectoryPoint:
        s = max(0, min(s, self.total_length))
        x = float(self._spline_x(s))
        y = float(self._spline_y(s))
        dx = float(self._spline_x(s, 1))
        dy = float(self._spline_y(s, 1))
        speed = math.hypot(dx, dy)

        if speed > 1e-6:
            vx, vy = dx / speed, dy / speed
            ddx = float(self._spline_x(s, 2))
            ddy = float(self._spline_y(s, 2))
            curvature = abs(dx * ddy - dy * ddx) / (speed ** 3)
        else:
            vx, vy, curvature = 1.0, 0.0, 0.0

        return TrajectoryPoint(x, y, vx, vy, curvature, s)

    def sample(self, spacing: float = 0.1) -> List[TrajectoryPoint]:
        points = []
        s = 0.0
        while s <= self.total_length:
            points.append(self.get_point(s))
            s += spacing
        if points and points[-1].s < self.total_length - 0.01:
            points.append(self.get_point(self.total_length))
        return points


def benchmark_smooth(wx: List[float], wy: List[float], sample_spacing: float = 0.1):
    """Benchmark smoothing of a single path."""
    input_length = sum(math.hypot(wx[i] - wx[i - 1], wy[i] - wy[i - 1])
                       for i in range(1, len(wx)))

    t0 = time.perf_counter()
    try:
        traj = SmoothTrajectory(wx, wy)
        points = traj.sample(sample_spacing)
        t1 = time.perf_counter()

        curvatures = [p.curvature for p in points]
        return {
            'success': True,
            'input_points': len(wx),
            'input_length': input_length,
            'smooth_length': traj.total_length,
            'output_points': len(points),
            'time_ms': (t1 - t0) * 1000,
            'max_curvature': max(curvatures) if curvatures else 0,
            'mean_curvature': np.mean(curvatures) if curvatures else 0,
        }
    except Exception as e:
        t1 = time.perf_counter()
        return {
            'success': False,
            'input_points': len(wx),
            'input_length': input_length,
            'time_ms': (t1 - t0) * 1000,
            'error': str(e)
        }


def generate_synthetic_path(num_points: int, path_type: str = 'random',
                            rng=None) -> Tuple[List[float], List[float]]:
    """Generate synthetic test path."""
    if rng is None:
        rng = np.random.default_rng()

    noise = 0.1

    if path_type == 'straight':
        t = np.linspace(0, 10, num_points)
        x = t + rng.normal(0, noise, num_points)
        y = t * 0.5 + rng.normal(0, noise, num_points)
    elif path_type == 'curved':
        t = np.linspace(0, np.pi, num_points)
        x = 5 * np.cos(t) + rng.normal(0, noise, num_points)
        y = 5 * np.sin(t) + rng.normal(0, noise, num_points)
    elif path_type == 'zigzag':
        t = np.linspace(0, 10, num_points)
        x = t + rng.normal(0, noise, num_points)
        y = 2 * np.sin(t * 2) + rng.normal(0, noise, num_points)
    else:  # random
        x, y = [0.0], [0.0]
        for _ in range(num_points - 1):
            angle = rng.uniform(0, 2 * np.pi)
            step = rng.uniform(0.5, 2.0)
            x.append(x[-1] + step * np.cos(angle))
            y.append(y[-1] + step * np.sin(angle))
        x = np.array(x) + rng.normal(0, noise, num_points)
        y = np.array(y) + rng.normal(0, noise, num_points)

    return list(x), list(y)


def main():
    parser = argparse.ArgumentParser(description='Benchmark trajectory smoothing')
    parser.add_argument('--input', '-i', help='Benchmark JSON to extract paths from')
    parser.add_argument('--synthetic', '-s', action='store_true',
                        help='Generate synthetic paths')
    parser.add_argument('--num', '-n', type=int, default=100,
                        help='Number of paths (synthetic mode)')
    parser.add_argument('--type', '-t', default='mixed',
                        choices=['random', 'straight', 'curved', 'zigzag', 'mixed'])
    args = parser.parse_args()

    paths = []

    if args.synthetic:
        print(f"Generating {args.num} synthetic paths...")
        rng = np.random.default_rng(42)
        types = ['random', 'straight', 'curved', 'zigzag']

        for i in range(args.num):
            num_pts = rng.integers(5, 50)
            ptype = types[i % 4] if args.type == 'mixed' else args.type
            wx, wy = generate_synthetic_path(num_pts, ptype, rng)
            paths.append((wx, wy))

    elif args.input:
        print(f"Loading paths from {args.input}...")
        with open(args.input) as f:
            data = json.load(f)

        # Generate synthetic paths based on characteristics
        for pair in data.get('pair_results', []):
            for it in pair.get('iteration_results', []):
                if it.get('success'):
                    # We don't have waypoints in JSON, generate similar synthetic
                    n = it.get('num_improvements', 10) + 5
                    wx, wy = generate_synthetic_path(n, 'curved')
                    paths.append((wx, wy))

    else:
        print("Error: Specify --input or --synthetic")
        sys.exit(1)

    # Run benchmark
    print(f"\nBenchmarking {len(paths)} paths...\n")

    results = []
    for i, (wx, wy) in enumerate(paths):
        if (i + 1) % 20 == 0:
            print(f"  {i + 1}/{len(paths)}")
        results.append(benchmark_smooth(wx, wy))

    # Statistics
    successful = [r for r in results if r['success']]

    print("\n" + "=" * 50)
    print("SMOOTHING BENCHMARK RESULTS")
    print("=" * 50)
    print(f"Total: {len(results)}")
    print(f"Success: {len(successful)} ({100 * len(successful) / len(results):.1f}%)")

    if successful:
        times = [r['time_ms'] for r in successful]
        print(f"\nTiming:")
        print(f"  Mean: {np.mean(times):.3f}ms")
        print(f"  Std: {np.std(times):.3f}ms")
        print(f"  Min: {np.min(times):.3f}ms")
        print(f"  Max: {np.max(times):.3f}ms")

        lengths = [r['smooth_length'] for r in successful]
        print(f"\nPath Length:")
        print(f"  Mean: {np.mean(lengths):.2f}m")

        curvs = [r['max_curvature'] for r in successful]
        print(f"\nMax Curvature:")
        print(f"  Mean: {np.mean(curvs):.4f}")
        print(f"  Max: {np.max(curvs):.4f}")

    print("=" * 50)


if __name__ == '__main__':
    main()