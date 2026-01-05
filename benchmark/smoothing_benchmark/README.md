# Smoothing Benchmark

Benchmarks different trajectory smoothing algorithms on RRT path planning results.

## Structure

```
smoothing_benchmark/
├── smoothers/
│   ├── __init__.py
│   ├── cubic_spline_smoother.py    # Cubic spline interpolation
│   ├── cubic_bezier_smoother.py    # Cubic Hermite with heading continuity
│   └── min_snap_smoother.py        # Minimum-snap optimization
├── run_smoothing_benchmark.py       # Main benchmark runner
├── results/                         # Output directory
└── README.md
```

## Default Paths

```
RRT Results:  /home/nadavc/PycharmProjects/sjtu_project/benchmark/rrt_benchmark/results
Output:       /home/nadavc/PycharmProjects/sjtu_project/benchmark/smoothing_benchmark/results
```

## Requirements

```bash
# Core dependencies
pip install numpy scipy

# For minimum-snap smoothing (optional)
pip install minsnap-trajectories
```

## Usage

### Basic Usage (Automatic)

```bash
# Process the most recent RRT results automatically
python run_smoothing_benchmark.py

# With multiple smoothing iterations for averaging
python run_smoothing_benchmark.py --iterations 10
```

### Manual File Selection

```bash
# Specify a specific RRT results file
python run_smoothing_benchmark.py path/to/rrt_first_*.json

# Custom output directory
python run_smoothing_benchmark.py -o custom_results/

# Change the RRT results search directory
python run_smoothing_benchmark.py --rrt-dir /path/to/rrt/results/
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `input_json` | RRT benchmark JSON file (optional) | Most recent in RRT results dir |
| `-o, --output-dir` | Output directory | `.../smoothing_benchmark/results` |
| `-i, --iterations` | Smoothing iterations per path | 1 |
| `--rrt-dir` | Directory to search for RRT results | `.../rrt_benchmark/results` |
| `-q, --quiet` | Suppress progress output | false |

## Output Files

The benchmark generates three output files:

### CSV File (`smoothing_*.csv`)

Contains timing data for each path:

| Column | Description |
|--------|-------------|
| `pair_id` | Point pair index |
| `iteration_id` | Iteration within the pair |
| `rrt_time_ms` | RRT planning time (from input) |
| `num_waypoints` | Number of waypoints in path |
| `path_length` | Original path length (meters) |
| `cubic_spline_time_ms` | Cubic spline smoothing time |
| `cubic_spline_success` | 1 if successful, 0 otherwise |
| `cubic_spline_traj_length` | Resulting trajectory length |
| `cubic_bezier_time_ms` | Cubic Bezier smoothing time |
| `cubic_bezier_success` | 1 if successful, 0 otherwise |
| `cubic_bezier_traj_length` | Resulting trajectory length |
| `min_snap_time_ms` | Minimum snap smoothing time |
| `min_snap_success` | 1 if successful, 0 otherwise |
| `min_snap_traj_length` | Resulting trajectory length |

### JSON File (`smoothing_*.json`)

Complete results with metadata:

```json
{
  "benchmark_type": "smoothing_benchmark",
  "input_file": "path/to/rrt_first_*.json",
  "timestamp": "20240115_143052",
  "smoothing_iterations": 10,
  "summary": {
    "total_paths": 2000,
    "paths_processed": 1950,
    "cubic_spline_success_count": 1950,
    "cubic_bezier_success_count": 1950,
    "min_snap_success_count": 1920
  },
  "results": [...]
}
```

### Summary File (`smoothing_*.txt`)

Human-readable summary with statistics:

```
============================================================
SMOOTHING BENCHMARK SUMMARY
============================================================

Input file: path/to/rrt_first_*.json
Timestamp: 20240115_143052
Smoothing iterations per path: 10

PATHS PROCESSED:
  Total paths: 2000
  Successfully processed: 1950

SUCCESS RATES:
  Cubic Spline: 1950/1950 (100.0%)
  Cubic Bezier: 1950/1950 (100.0%)
  Minimum Snap: 1920/1950 (98.5%)

TIMING STATISTICS:
  RRT Planning:  45.230 ± 12.450 ms (min=5.120, max=125.670)
  Cubic Spline:   0.234 ± 0.056 ms (min=0.089, max=0.567)
  Cubic Bezier:   0.456 ± 0.123 ms (min=0.234, max=0.890)
  Minimum Snap:  12.345 ± 3.456 ms (min=5.678, max=45.678)
```

## Individual Smoother Usage

You can also use the smoothers directly in your code:

```python
from smoothers import CubicSplineSmoother, CubicBezierSmoother

# Example waypoints from RRT
waypoints_x = [0, 1, 2, 3, 4, 5]
waypoints_y = [0, 0.5, 0, 0.5, 0, 0.5]

# Cubic Spline
spline_traj = CubicSplineSmoother(waypoints_x, waypoints_y)
print(f"Spline length: {spline_traj.total_length:.2f}m")

# Cubic Bezier (Hermite)
bezier_traj = CubicBezierSmoother(waypoints_x, waypoints_y)
print(f"Bezier length: {bezier_traj.total_length:.2f}m")

# Sample points along trajectory
for point in spline_traj.sample_trajectory(spacing=0.5):
    print(f"  s={point.s:.1f}: ({point.x:.2f}, {point.y:.2f})")
```

## Smoother Algorithms

### Cubic Spline (cubic_spline_smoother.py)

- Standard cubic spline interpolation
- C2 continuity (smooth position, velocity, acceleration)
- Arc-length parameterization for uniform sampling
- Fastest option

### Cubic Bezier/Hermite (cubic_bezier_smoother.py)

- Cubic Hermite spline with tangent control
- G1 continuity with heading awareness
- Smooth heading transitions at waypoints
- Good balance of speed and quality

### Minimum Snap (min_snap_smoother.py)

- Optimal for quadrotor drones
- Minimizes 4th derivative (snap) of trajectory
- Provides full state: position, velocity, acceleration
- Requires `minsnap-trajectories` package
- Slowest but highest quality for dynamic systems

## Integration with RRT Benchmark

Typical workflow:

```bash
# 1. Run RRT path planning benchmark
cd /home/nadavc/PycharmProjects/sjtu_project/benchmark/rrt_benchmark
./build/rrt_first_benchmark -m map.yaml -k 20 -i 100

# 2. Run smoothing benchmark (automatically uses most recent RRT results)
cd /home/nadavc/PycharmProjects/sjtu_project/benchmark/smoothing_benchmark
python run_smoothing_benchmark.py --iterations 5

# 3. Analyze combined results
# (Results are in ./results/ directory)
```

## Notes

- If `minsnap-trajectories` is not installed, the benchmark will skip minimum snap smoothing and report it as failed
- Paths with fewer than 2 waypoints are skipped
- The benchmark measures pure smoothing time (excludes I/O and path loading)
- Use `--iterations > 1` for more accurate timing (reduces noise from system variability)