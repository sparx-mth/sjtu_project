/**
 * rrt_benchmark.h
 * ---------------
 * RRT* path planning benchmark - data structures and class declarations.
 *
 * This is a standalone benchmarking tool (no ROS dependencies).
 */

#pragma once

#include <string>
#include <vector>
#include <utility>
#include <random>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace rrt_bench {

// =============================================================================
// Configuration
// =============================================================================

struct Config {
    int safety_margin = 10;              // Pixels to dilate obstacles
    double planning_timeout = 3.0;       // Seconds
    double clearance_weight = 5.0;       // Weight for clearance cost
    double min_clearance_smooth = 15.0;  // Min clearance to allow smoothing
    double interpolation_spacing = 3.0;  // Meters between interpolated points
    double snapshot_interval_ms = 50.0;  // How often to record solutions
    std::string planner_type = "BITstar";  // Options: "RRTstar", "InformedRRTstar", "BITstar"
};

// =============================================================================
// Result Structures
// =============================================================================

/** A snapshot of a solution at a specific time */
struct Snapshot {
    double time_ms;          // Time since planning started
    double path_length;      // Path length in meters
    int num_raw_points;      // Points from RRT*
    int num_smoothed_points; // Points after smoothing
    int num_final_points;    // Points after interpolation
};

/** Result of a single planning run */
struct PlanResult {
    bool success = false;
    std::string message;

    double total_time_ms = 0;
    double first_solution_time_ms = 0;
    double first_solution_length = 0;
    double final_path_length = 0;

    std::vector<Snapshot> snapshots;

    // First solution path (world coords)
    std::vector<double> first_path_x;
    std::vector<double> first_path_y;

    // Final path (world coords)
    std::vector<double> path_x;
    std::vector<double> path_y;
};

/** Result for one point pair (multiple iterations) */
struct PairResult {
    int pair_id;
    int start_x, start_y;  // Grid coords
    int goal_x, goal_y;
    double start_world_x, start_world_y;
    double goal_world_x, goal_world_y;
    double air_distance;

    std::vector<PlanResult> iterations;

    // Statistics (computed after all iterations)
    double success_rate = 0;
    double mean_first_time_ms = 0;
    double std_first_time_ms = 0;
    double mean_final_length = 0;
    double std_final_length = 0;
    double mean_improvement_pct = 0;

    // Example paths from first successful iteration (for visualization)
    std::vector<double> example_first_path_x;
    std::vector<double> example_first_path_y;
    std::vector<double> example_final_path_x;
    std::vector<double> example_final_path_y;
};

/** Full benchmark session */
struct Session {
    std::string timestamp;
    std::string map_file;
    int num_pairs;
    int iterations_per_pair;
    double min_distance;
    Config config;

    std::vector<PairResult> results;

    // Global stats
    int total_runs = 0;
    int total_successes = 0;
    double success_rate = 0;
    double mean_first_time_ms = 0;
    double mean_final_length = 0;
    double duration_seconds = 0;
};

// =============================================================================
// Map Handler - Loads and processes occupancy grid
// =============================================================================

class Map {
public:
    bool load(const std::string& yaml_path, int safety_margin = 10);

    bool isValid(double x, double y) const;
    double getClearance(double x, double y) const;

    std::pair<int, int> worldToGrid(double wx, double wy) const;
    std::pair<double, double> gridToWorld(int gx, int gy) const;

    std::pair<int, int> randomFreePoint(std::mt19937& rng) const;
    std::pair<std::pair<int,int>, std::pair<int,int>>
        randomPointPair(double min_dist_meters, std::mt19937& rng) const;

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }

private:
    cv::Mat occupancy_;      // Binary occupancy (0=free, 255=obstacle)
    cv::Mat distance_map_;   // Distance to nearest obstacle
    int width_ = 0, height_ = 0;
    double resolution_ = 0;
    double origin_x_ = 0, origin_y_ = 0;
};

// =============================================================================
// RRT* Planner with benchmarking support
// =============================================================================

class Planner {
public:
    Planner(Map* map, const Config& config = Config());

    PlanResult plan(int start_x, int start_y, int goal_x, int goal_y);

private:
    // Smoothing: remove redundant points, keep tight corners
    // Uses OMPL's checkMotion (same as original pipeline)
    std::vector<std::pair<double,double>> smooth(
        const std::vector<std::pair<double,double>>& path,
        const ob::SpaceInformationPtr& si);

    // Add points at regular intervals
    void interpolate(std::vector<double>& x, std::vector<double>& y, double spacing);

    // Compute path length
    double pathLength(const std::vector<double>& x, const std::vector<double>& y) const;

    Map* map_;
    Config config_;
};

// =============================================================================
// Benchmark Runner
// =============================================================================

class Runner {
public:
    Runner(const std::string& map_path, const Config& config = Config());

    void setSeed(unsigned int seed);

    Session run(int num_pairs, int iterations, double min_distance, bool verbose = true);

    void saveJson(const Session& session, const std::string& filepath);
    void saveSummary(const Session& session, const std::string& filepath);

private:
    PairResult runPair(int id, int sx, int sy, int gx, int gy,
                       int iterations, bool verbose);
    void computeStats(PairResult& pr);
    void computeGlobalStats(Session& session);

    Map map_;
    Config config_;
    std::unique_ptr<Planner> planner_;
    std::mt19937 rng_;
};

// =============================================================================
// Utilities
// =============================================================================

std::string getTimestamp();
double mean(const std::vector<double>& v);
double stddev(const std::vector<double>& v);

} // namespace rrt_bench