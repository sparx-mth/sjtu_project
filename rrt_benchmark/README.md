# RRT* Path Planning Benchmark

Standalone benchmark suite for RRT* path planning with clearance optimization.

## Structure

```
rrt_benchmark/
├── include/
│   └── rrt_benchmark.h      # Header file
├── src/
│   ├── rrt_benchmark.cpp    # Implementation
│   └── main.cpp             # Entry point
├── scripts/
│   ├── analyze.py           # Generate plots
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
pip3 install numpy scipy matplotlib
```

## Build

```bash
cd rrt_benchmark
mkdir build && cd build
cmake ..
make
```

## Usage

### Run Benchmark

```bash
# Basic run with BIT* (default algorithm)
cd rrt_benchmark
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
# Generate plots
python3 scripts/analyze.py results/benchmark_XXXX.json

# Without display (server mode)
python3 scripts/analyze.py results/benchmark_XXXX.json --no-show
```

## Output

### Files

```
results/
├── benchmark_YYYYMMDD_HHMMSS.json   # Complete data
├── benchmark_YYYYMMDD_HHMMSS.txt    # Summary
├── improvement_over_time.png        # Main plot
├── first_solution_dist.png
├── quality_metrics.png
└── per_pair_summary.png
```

### Key Plots

1. **improvement_over_time.png** - Path distance vs time showing RRT* optimization
2. **first_solution_dist.png** - Distribution of first solution times
3. **quality_metrics.png** - Path lengths, improvements, efficiency
4. **per_pair_summary.png** - Statistics per point pair

## What It Measures

1. **First Solution Time** - How long until a valid path is found
2. **Path Improvement** - How RRT* optimizes the path over time
3. **Final Path Quality** - Length compared to air distance
4. **Success Rate** - Percentage of successful planning attempts

## Available Algorithms

| Algorithm | Description |
|-----------|-------------|
| **RRTstar** | Standard RRT* with clearance optimization. Good baseline. |
| **InformedRRTstar** | RRT* with informed sampling using ellipsoidal heuristic. Faster convergence after finding initial solution. |
| **BITstar** | Batch Informed Trees. Combines best of RRT* and graph-based planners. Often fastest convergence. |

**Note:** InformedRRTstar and BITstar use path length optimization (required for informed sampling heuristics). RRTstar uses clearance-weighted optimization.

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