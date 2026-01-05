/**
 * rrt_first_solution.h
 * --------------------
 * RRT path planning benchmark - first solution only (no optimization).
 *
 * This benchmark uses regular RRT (not RRT*) to measure:
 * - Time to find the first valid path
 * - Path characteristics after the standard pipeline (smooth + interpolate)
 *
 * No ROS dependencies - standalone benchmarking tool.
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
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace rrt_first {

// =============================================================================
// Configuration
// =============================================================================

struct Config {
    int safety_margin = 10;              // Pixels to dilate obstacles
    double planning_timeout = 4.0;       // Seconds (skip if no solution)
    double min_clearance_smooth = 15.0;  // Min clearance to allow smoothing
    double interpolation_spacing = 3.0;  // Meters between interpolated points
    double rrt_range = 0.0;              // RRT step size (0 = auto)
    double rrt_goal_bias = 0.05;         // Goal bias for RRT (default 5%)
};

// =============================================================================
// Result Structures
// =============================================================================

/** Result of a single RRT planning run */
struct IterationResult {
    int iteration_id;
    bool success = false;
    std::string message;

    double planning_time_ms = 0;         // Time for RRT to find solution
    double total_time_ms = 0;            // Including post-processing

    // Path metrics
    int num_raw_points = 0;              // Points from RRT
    int num_smoothed_points = 0;         // After smoothing
    int num_final_points = 0;            // After interpolation
    double path_length = 0;              // Final path length in meters

    // Complete processed route (world coordinates)
    std::vector<double> path_x;
    std::vector<double> path_y;
};

/** Result for one point pair (multiple iterations) */
struct PairResult {
    int pair_id;
    
    // Grid coordinates
    int start_x, start_y;
    int goal_x, goal_y;
    
    // World coordinates
    double start_world_x, start_world_y;
    double goal_world_x, goal_world_y;
    double air_distance;                 // Euclidean distance in meters

    // All iteration results
    std::vector<IterationResult> iterations;

    // Statistics (computed after all iterations)
    int num_success = 0;
    double success_rate = 0;
    double mean_planning_time_ms = 0;
    double std_planning_time_ms = 0;
    double mean_path_length = 0;
    double std_path_length = 0;
    double min_path_length = 0;
    double max_path_length = 0;

    // First successful route (for visualization)
    std::vector<double> example_path_x;
    std::vector<double> example_path_y;
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

    // Global statistics
    int total_runs = 0;
    int total_successes = 0;
    double success_rate = 0;
    double mean_planning_time_ms = 0;
    double mean_path_length = 0;
    double duration_seconds = 0;
};

// =============================================================================
// Map Handler
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
    cv::Mat occupancy_;
    cv::Mat distance_map_;
    int width_ = 0, height_ = 0;
    double resolution_ = 0;
    double origin_x_ = 0, origin_y_ = 0;
};

// =============================================================================
// RRT Planner (first solution only)
// =============================================================================

class Planner {
public:
    Planner(Map* map, const Config& config = Config());

    IterationResult plan(int iteration_id, int start_x, int start_y, int goal_x, int goal_y);

private:
    // Adaptive smoothing: remove redundant points, keep tight corners
    std::vector<std::pair<double,double>> smooth(
        const std::vector<std::pair<double,double>>& path,
        const ob::SpaceInformationPtr& si);

    // Add points at regular intervals
    void interpolate(std::vector<double>& x, std::vector<double>& y, double spacing);

    // Compute path length
    double pathLength(const std::vector<double>& x, const std::vector<double>& y) const;

    // Validate path doesn't cross obstacles
    bool validatePath(const std::vector<double>& x, const std::vector<double>& y) const;

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
    void saveCsv(const Session& session, const std::string& filepath);

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

} // namespace rrt_first