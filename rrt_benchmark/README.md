# RRT Path Planning Benchmark Suite

Standalone benchmark suite for RRT path planning, containing two benchmarks:

1. **RRT* Optimization Benchmark** - Measures path improvement over time using RRT*/InformedRRT*/BIT*
2. **RRT First-Solution Benchmark** - Measures time to find first valid path using regular RRT

## Structure

```
rrt_benchmark/
├── include/
│   ├── rrt_benchmark.h          # RRT* benchmark header
│   └── rrt_first_solution.h     # RRT first-solution header
├── src/
│   ├── rrt_benchmark.cpp        # RRT* implementation
│   ├── main.cpp                 # RRT* entry point
│   ├── rrt_first_solution.cpp   # RRT first-solution implementation
│   └── rrt_first_solution_main.cpp  # RRT first-solution entry point
├── scripts/
│   ├── analyze.py               # RRT* plots
│   ├── visualize_first_routes.py    # RRT first-solution plots
│   └── smoothing_benchmark.py
├── CMakeLists.txt
└── README.md
```

## Requirements

### C++ Dependencies

```bash
# Ubuntu/Debian
sudo apt-get install \
    libompl-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    cmake \
    build-essential

# Or on other systems, install:
# - OMPL (Open Motion Planning Library)
# - OpenCV
# - yaml-cpp
# - Boost
```

### Python Dependencies

```bash
pip3 install numpy scipy matplotlib pyyaml
```

## Build

```bash
cd rrt_benchmark
mkdir build && cd build
cmake ..
make
```

This builds two executables:
- `rrt_benchmark` - RRT* optimization benchmark
- `rrt_first_benchmark` - RRT first-solution benchmark

---

## Benchmark 1: RRT* Optimization Benchmark

Measures how RRT*/InformedRRT*/BIT* improves path quality over time.

### Usage

```bash
# Basic run with BIT* (default algorithm)
./build/rrt_benchmark -m /path/to/map.yaml

# Quick test
./build/rrt_benchmark -m map.yaml -p 5 -i 20

# Use different algorithms
./build/rrt_benchmark -m map.yaml -a RRTstar -p 5 -i 20
./build/rrt_benchmark -m map.yaml -a InformedRRTstar -p 5 -i 20
./build/rrt_benchmark -m map.yaml -a BITstar -p 5 -i 20

# Full options
./build/rrt_benchmark -m map.yaml -a BITstar -p 20 -i 100 -d 10 -t 5.0 -o results/
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `-m, --map` | Map YAML file (required) | - |
| `-o, --output` | Output directory | results |
| `-a, --algo` | Algorithm: RRTstar, InformedRRTstar, BITstar | BITstar |
| `-p, --pairs` | Number of point pairs | 20 |
| `-i, --iterations` | Iterations per pair | 100 |
| `-d, --distance` | Min distance between points (m) | 5.0 |
| `-t, --timeout` | Planning timeout (s) | 3.0 |
| `-s, --seed` | Random seed | random |
| `-q, --quiet` | Less output | false |

### Analyze Results

```bash
python3 scripts/analyze.py results/benchmark_XXXX.json
```

### What It Measures

1. **First Solution Time** - How long until a valid path is found
2. **Path Improvement** - How RRT* optimizes the path over time
3. **Final Path Quality** - Length compared to air distance
4. **Success Rate** - Percentage of successful planning attempts

### Available Algorithms

| Algorithm | Description |
|-----------|-------------|
| **RRTstar** | Standard RRT* with clearance optimization. Good baseline. |
| **InformedRRTstar** | RRT* with informed sampling using ellipsoidal heuristic. Faster convergence after finding initial solution. |
| **BITstar** | Batch Informed Trees. Combines best of RRT* and graph-based planners. Often fastest convergence. |

**Note:** InformedRRTstar and BITstar use path length optimization (required for informed sampling heuristics). RRTstar uses clearance-weighted optimization.

### Output Files

```
results/
├── benchmark_YYYYMMDD_HHMMSS.json   # Complete data
├── benchmark_YYYYMMDD_HHMMSS.txt    # Summary
├── improvement_over_time.png        # Main plot
├── first_solution_dist.png
├── quality_metrics.png
└── per_pair_summary.png
```

---

## Benchmark 2: RRT First-Solution Benchmark

Measures time to find the **first valid path** using regular RRT (no optimization).

### Usage

```bash
# Basic run
./build/rrt_first_benchmark -m /path/to/map.yaml

# Quick test
  

# Full benchmark: K=20 pairs, I=100 iterations, M=30m min distance
./build/rrt_first_benchmark -m map.yaml -k 20 -i 100 -d 30

# With reproducible seed
./build/rrt_first_benchmark -m map.yaml -k 20 -i 100 -d 30 -t 4 -s 42
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `-m, --map` | Map YAML file | maps/hospital_map_cropped.yaml |
| `-o, --output` | Output directory | results |
| `-k, --pairs` | Number of point pairs (K) | 20 |
| `-i, --iterations` | Iterations per pair (I) | 100 |
| `-d, --distance` | Min air distance between points (M) in meters | 30.0 |
| `-t, --timeout` | Planning timeout (skip if no solution) | 4.0 |
| `-g, --goal-bias` | RRT goal bias | 0.05 |
| `-r, --range` | RRT step range (0 = auto) | 0 |
| `-s, --seed` | Random seed | random |
| `-q, --quiet` | Less output | false |

### Visualize Results

```bash
python3 scripts/visualize_first_routes.py results/rrt_first_XXXX.json
```

### What It Measures

1. **Planning Time** - Time for RRT to find first valid path
2. **Route Generation** - Complete processed route (smoothed + interpolated)
3. **Path Length** - Final path length in meters
4. **Success Rate** - Paths found within timeout

### Output Files

```
results/
├── rrt_first_YYYYMMDD_HHMMSS.json   # Complete data (all routes)
├── rrt_first_YYYYMMDD_HHMMSS.txt    # Summary
├── rrt_first_YYYYMMDD_HHMMSS.csv    # For analysis
├── rrt_first_*_overview.png         # All routes on map
├── rrt_first_*_pair01.png           # Individual pair routes
├── rrt_first_*_histograms.png       # Time/length distributions
└── rrt_first_*_comparison.png       # Per-pair comparison
```

### Output Data Format

Each iteration stores the complete route:
```json
{
  "iteration_id": 0,
  "success": true,
  "planning_time_ms": 45.23,
  "path_length": 42.567,
  "path_x": [x0, x1, x2, ...],
  "path_y": [y0, y1, y2, ...]
}
```

---

## Comparison of Benchmarks

| Feature | RRT* Benchmark | RRT First-Solution |
|---------|---------------|-------------------|
| Algorithm | RRT* / BIT* / InformedRRT* | Regular RRT |
| Goal | Measure optimization over time | Measure time to first path |
| Timeout behavior | Continue optimizing | Skip pair |
| Output paths | First + Final | First only (complete route) |
| Use case | Path quality study | Planning speed study |

---

## Map Format

Standard ROS map format (YAML + image):

```yaml
image: map.png
resolution: 0.05  # meters per pixel
origin: [-10.0, -10.0, 0.0]  # x, y, yaw
```

Image: Grayscale PNG where white (>250) = free, dark = obstacle.

## Running on Multiple Machines

```bash
# Machine 1
./rrt_benchmark -m map.yaml -s 1 -o results_m1/

# Machine 2
./rrt_benchmark -m map.yaml -s 2 -o results_m2/

# Combine and analyze later
```

## Comparing Algorithms

```bash
# Run same scenario with different algorithms
./rrt_benchmark -m map.yaml -a RRTstar -s 42 -p 10 -i 50 -o results_rrt/
./rrt_benchmark -m map.yaml -a InformedRRTstar -s 42 -p 10 -i 50 -o results_irrt/
./rrt_benchmark -m map.yaml -a BITstar -s 42 -p 10 -i 50 -o results_bit/

# Analyze each
python3 scripts/analyze.py results_rrt/benchmark_*.json
python3 scripts/analyze.py results_irrt/benchmark_*.json
python3 scripts/analyze.py results_bit/benchmark_*.json
```