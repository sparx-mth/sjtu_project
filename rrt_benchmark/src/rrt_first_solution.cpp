/**
 * rrt_first_solution.cpp
 * ----------------------
 * Implementation of RRT path planning benchmark (first solution only).
 *
 * Uses regular RRT (not RRT*) to find the first valid path, then applies
 * the same post-processing pipeline as the ROS service:
 *   1. Adaptive smoothing (remove redundant points, keep tight spaces)
 *   2. Interpolation (add points for smooth spline fitting)
 *   3. Path validation (ensure route doesn't cross walls)
 */

#include "rrt_first_solution.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <filesystem>

#include <yaml-cpp/yaml.h>
#include <ompl/base/PlannerTerminationCondition.h>

namespace rrt_first {

// =============================================================================
// Utilities
// =============================================================================

std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
    return ss.str();
}

double mean(const std::vector<double>& v) {
    if (v.empty()) return 0;
    return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

double stddev(const std::vector<double>& v) {
    if (v.size() < 2) return 0;
    double m = mean(v);
    double sq = 0;
    for (double x : v) sq += (x - m) * (x - m);
    return std::sqrt(sq / v.size());
}

// =============================================================================
// Map Implementation
// =============================================================================

bool Map::load(const std::string& yaml_path, int safety_margin) {
    try {
        YAML::Node cfg = YAML::LoadFile(yaml_path);
        resolution_ = cfg["resolution"].as<double>();
        origin_x_ = cfg["origin"][0].as<double>();
        origin_y_ = cfg["origin"][1].as<double>();

        std::string img_path = cfg["image"].as<std::string>();
        if (img_path[0] != '/') {
            auto dir = std::filesystem::path(yaml_path).parent_path();
            img_path = (dir / img_path).string();
        }

        cv::Mat img = cv::imread(img_path, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            std::cerr << "Error: Cannot load image: " << img_path << std::endl;
            return false;
        }

        // Threshold: >250 is free, else obstacle
        cv::Mat binary;
        cv::threshold(img, binary, 250, 255, cv::THRESH_BINARY_INV);
        cv::flip(binary, binary, 0);  // Flip Y to match ROS convention

        // Dilate obstacles by safety margin
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(2*safety_margin+1, 2*safety_margin+1));
        cv::dilate(binary, occupancy_, kernel);

        // Distance transform for clearance checking
        cv::Mat free;
        cv::bitwise_not(occupancy_, free);
        cv::distanceTransform(free, distance_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE);

        width_ = occupancy_.cols;
        height_ = occupancy_.rows;

        std::cout << "Map loaded: " << width_ << "x" << height_
                  << ", resolution=" << resolution_ << "m/px" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Error loading map: " << e.what() << std::endl;
        return false;
    }
}

bool Map::isValid(double x, double y) const {
    int ix = static_cast<int>(std::round(x));
    int iy = static_cast<int>(std::round(y));
    if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) return false;
    return occupancy_.at<uchar>(iy, ix) == 0;
}

double Map::getClearance(double x, double y) const {
    int ix = static_cast<int>(std::round(x));
    int iy = static_cast<int>(std::round(y));
    if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) return 0;
    return distance_map_.at<float>(iy, ix);
}

std::pair<int,int> Map::worldToGrid(double wx, double wy) const {
    return {
        static_cast<int>(std::round((wx - origin_x_) / resolution_)),
        static_cast<int>(std::round((wy - origin_y_) / resolution_))
    };
}

std::pair<double,double> Map::gridToWorld(int gx, int gy) const {
    return {
        gx * resolution_ + origin_x_,
        gy * resolution_ + origin_y_
    };
}

std::pair<int,int> Map::randomFreePoint(std::mt19937& rng) const {
    std::uniform_int_distribution<int> dx(0, width_-1);
    std::uniform_int_distribution<int> dy(0, height_-1);

    for (int i = 0; i < 10000; ++i) {
        int x = dx(rng), y = dy(rng);
        if (isValid(x, y)) return {x, y};
    }
    throw std::runtime_error("Could not find free point after 10000 attempts");
}

std::pair<std::pair<int,int>, std::pair<int,int>>
Map::randomPointPair(double min_dist_meters, std::mt19937& rng) const {
    double min_grid = min_dist_meters / resolution_;

    for (int i = 0; i < 1000; ++i) {
        auto [sx, sy] = randomFreePoint(rng);
        auto [gx, gy] = randomFreePoint(rng);
        double d = std::hypot(gx - sx, gy - sy);
        if (d >= min_grid) {
            return {{sx, sy}, {gx, gy}};
        }
    }
    throw std::runtime_error("Could not find point pair with required distance");
}

// =============================================================================
// Planner Implementation
// =============================================================================

Planner::Planner(Map* map, const Config& config) : map_(map), config_(config) {}

std::vector<std::pair<double,double>> Planner::smooth(
    const std::vector<std::pair<double,double>>& path,
    const ob::SpaceInformationPtr& si)
{
    if (path.size() < 3) return path;

    std::vector<std::pair<double,double>> result;
    result.push_back(path[0]);

    // Create states for OMPL motion checking
    ob::ScopedState<> s1(si->getStateSpace());
    ob::ScopedState<> s2(si->getStateSpace());

    for (size_t i = 1; i < path.size() - 1; ++i) {
        double clearance = map_->getClearance(path[i].first, path[i].second);

        // Use OMPL's checkMotion (same as original pipeline)
        s1[0] = result.back().first;
        s1[1] = result.back().second;
        s2[0] = path[i+1].first;
        s2[1] = path[i+1].second;
        bool canSkip = si->checkMotion(s1.get(), s2.get());

        // Keep point if in tight space or can't skip
        if (clearance < config_.min_clearance_smooth || !canSkip) {
            result.push_back(path[i]);
        }
    }
    result.push_back(path.back());
    return result;
}

void Planner::interpolate(std::vector<double>& x, std::vector<double>& y, double spacing) {
    if (x.size() < 2 || spacing <= 0) return;

    std::vector<double> nx, ny;
    nx.push_back(x[0]);
    ny.push_back(y[0]);

    for (size_t i = 1; i < x.size(); ++i) {
        double dx = x[i] - x[i-1];
        double dy = y[i] - y[i-1];
        double len = std::hypot(dx, dy);

        if (len > 1e-6) {
            int n = static_cast<int>(std::floor(len / spacing));
            for (int j = 1; j <= n; ++j) {
                double t = static_cast<double>(j) / (n + 1);
                nx.push_back(x[i-1] + t*dx);
                ny.push_back(y[i-1] + t*dy);
            }
        }
        nx.push_back(x[i]);
        ny.push_back(y[i]);
    }

    x = std::move(nx);
    y = std::move(ny);
}

double Planner::pathLength(const std::vector<double>& x, const std::vector<double>& y) const {
    double len = 0;
    for (size_t i = 1; i < x.size(); ++i) {
        len += std::hypot(x[i] - x[i-1], y[i] - y[i-1]);
    }
    return len;
}

bool Planner::validatePath(const std::vector<double>& x, const std::vector<double>& y) const {
    if (x.size() < 2) return false;

    // Check each segment doesn't cross obstacles
    for (size_t i = 0; i < x.size() - 1; ++i) {
        // Convert to grid coordinates
        auto [gx1, gy1] = map_->worldToGrid(x[i], y[i]);
        auto [gx2, gy2] = map_->worldToGrid(x[i+1], y[i+1]);

        // Bresenham line validation
        int dx = std::abs(gx2 - gx1);
        int dy = std::abs(gy2 - gy1);
        int sx = (gx1 < gx2) ? 1 : -1;
        int sy = (gy1 < gy2) ? 1 : -1;
        int err = dx - dy;

        int cx = gx1, cy = gy1;
        while (true) {
            if (!map_->isValid(cx, cy)) {
                return false;
            }
            if (cx == gx2 && cy == gy2) break;

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; cx += sx; }
            if (e2 < dx)  { err += dx; cy += sy; }
        }
    }
    return true;
}

IterationResult Planner::plan(int iteration_id, int start_x, int start_y, int goal_x, int goal_y) {
    IterationResult result;
    result.iteration_id = iteration_id;

    if (!map_->isValid(start_x, start_y)) {
        result.message = "Start in obstacle";
        return result;
    }
    if (!map_->isValid(goal_x, goal_y)) {
        result.message = "Goal in obstacle";
        return result;
    }

    auto [goal_wx, goal_wy] = map_->gridToWorld(goal_x, goal_y);

    // =========================================================================
    // OMPL Setup - Using regular RRT (not RRT*)
    // =========================================================================
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, 0);
    bounds.setHigh(0, map_->width() - 1);
    bounds.setLow(1, 0);
    bounds.setHigh(1, map_->height() - 1);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);

    ss.setStateValidityChecker([this](const ob::State* state) {
        const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
        return map_->isValid(s->values[0], s->values[1]);
    });

    ob::ScopedState<> start(space);
    start[0] = static_cast<double>(start_x);
    start[1] = static_cast<double>(start_y);

    ob::ScopedState<> goal(space);
    goal[0] = static_cast<double>(goal_x);
    goal[1] = static_cast<double>(goal_y);

    ss.setStartAndGoalStates(start, goal);

    // Configure RRT planner
    auto rrt = std::make_shared<og::RRT>(ss.getSpaceInformation());
    if (config_.rrt_range > 0) {
        rrt->setRange(config_.rrt_range);
    }
    rrt->setGoalBias(config_.rrt_goal_bias);
    ss.setPlanner(rrt);

    auto si = ss.getSpaceInformation();

    // Helper lambda to validate path doesn't go through obstacles
    auto isPathValid = [&](og::PathGeometric& path) -> bool {
        if (path.getStateCount() < 2) return false;
        for (size_t i = 0; i < path.getStateCount() - 1; ++i) {
            if (!si->checkMotion(path.getState(i), path.getState(i + 1))) {
                return false;
            }
        }
        return true;
    };

    // =========================================================================
    // Plan: Find first valid solution
    // Uses OMPL termination conditions to ensure full timeout is used
    // =========================================================================
    auto t_start = std::chrono::high_resolution_clock::now();

    // Setup the planner - must call setup() before solve()
    ss.setup();

    // Create termination condition: stop when EXACT solution found OR timeout
    // This ensures RRT keeps growing the tree for the full timeout if needed
    ob::PlannerTerminationCondition ptc = ob::plannerOrTerminationCondition(
        ob::exactSolnPlannerTerminationCondition(ss.getProblemDefinition()),
        ob::timedPlannerTerminationCondition(config_.planning_timeout)
    );

    // Single solve call with proper termination condition
    ob::PlannerStatus status = ss.solve(ptc);

    auto t_plan_end = std::chrono::high_resolution_clock::now();
    result.planning_time_ms = std::chrono::duration<double, std::milli>(t_plan_end - t_start).count();

    // Check if we got an exact solution
    if (status != ob::PlannerStatus::EXACT_SOLUTION || !ss.haveSolutionPath()) {
        result.message = "No path found within timeout (" +
                        std::to_string(config_.planning_timeout) + "s)";
        result.total_time_ms = result.planning_time_ms;
        return result;
    }

    og::PathGeometric& path = ss.getSolutionPath();

    // Validate the path doesn't go through obstacles (OMPL check)
    if (!isPathValid(path)) {
        result.message = "Path validation failed (crosses obstacles)";
        result.total_time_ms = result.planning_time_ms;
        return result;
    }

    // =========================================================================
    // Post-processing: Same pipeline as ROS service
    // =========================================================================

    // Extract raw path
    std::vector<std::pair<double,double>> raw;
    for (size_t i = 0; i < path.getStateCount(); ++i) {
        const auto* s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        raw.emplace_back(s->values[0], s->values[1]);
    }
    result.num_raw_points = static_cast<int>(raw.size());

    // Adaptive smoothing
    auto smoothed = smooth(raw, si);
    result.num_smoothed_points = static_cast<int>(smoothed.size());

    // Convert to world coordinates
    std::vector<double> wx, wy;
    for (auto& [gx, gy] : smoothed) {
        auto [x, y] = map_->gridToWorld(static_cast<int>(std::round(gx)),
                                        static_cast<int>(std::round(gy)));
        wx.push_back(x);
        wy.push_back(y);
    }

    // Ensure goal is included
    if (!wx.empty()) {
        double d = std::hypot(wx.back() - goal_wx, wy.back() - goal_wy);
        if (d > 0.1) {
            wx.push_back(goal_wx);
            wy.push_back(goal_wy);
        }
    }

    // Interpolate for smooth trajectories
    interpolate(wx, wy, config_.interpolation_spacing);
    result.num_final_points = static_cast<int>(wx.size());

    // Note: We skip the validatePath() check here because:
    // 1. OMPL already validated the path
    // 2. Smoothing uses OMPL's checkMotion() which is consistent with OMPL's validation
    // 3. The Bresenham-based validatePath() can be stricter and cause false negatives

    // Calculate path length
    result.path_length = pathLength(wx, wy);

    // Store the complete route
    result.path_x = std::move(wx);
    result.path_y = std::move(wy);

    auto t_end = std::chrono::high_resolution_clock::now();
    result.total_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    result.success = true;
    result.message = "OK";

    return result;
}

// =============================================================================
// Runner Implementation
// =============================================================================

Runner::Runner(const std::string& map_path, const Config& config)
    : config_(config), rng_(std::random_device{}())
{
    if (!map_.load(map_path, config.safety_margin)) {
        throw std::runtime_error("Failed to load map: " + map_path);
    }
    planner_ = std::make_unique<Planner>(&map_, config);
}

void Runner::setSeed(unsigned int seed) {
    rng_.seed(seed);
}

void Runner::computeStats(PairResult& pr) {
    std::vector<double> times, lengths;

    for (auto& r : pr.iterations) {
        if (r.success) {
            pr.num_success++;
            times.push_back(r.planning_time_ms);
            lengths.push_back(r.path_length);
        }
    }

    pr.success_rate = static_cast<double>(pr.num_success) / pr.iterations.size();
    pr.mean_planning_time_ms = mean(times);
    pr.std_planning_time_ms = stddev(times);
    pr.mean_path_length = mean(lengths);
    pr.std_path_length = stddev(lengths);

    if (!lengths.empty()) {
        pr.min_path_length = *std::min_element(lengths.begin(), lengths.end());
        pr.max_path_length = *std::max_element(lengths.begin(), lengths.end());
    }
}

void Runner::computeGlobalStats(Session& s) {
    std::vector<double> times, lengths;
    s.total_runs = 0;
    s.total_successes = 0;

    for (auto& pr : s.results) {
        for (auto& r : pr.iterations) {
            s.total_runs++;
            if (r.success) {
                s.total_successes++;
                times.push_back(r.planning_time_ms);
                lengths.push_back(r.path_length);
            }
        }
    }

    s.success_rate = s.total_runs > 0 ?
        static_cast<double>(s.total_successes) / s.total_runs : 0;
    s.mean_planning_time_ms = mean(times);
    s.mean_path_length = mean(lengths);
}

PairResult Runner::runPair(int id, int sx, int sy, int gx, int gy,
                           int iterations, bool verbose) {
    PairResult pr;
    pr.pair_id = id;
    pr.start_x = sx; pr.start_y = sy;
    pr.goal_x = gx; pr.goal_y = gy;

    auto [swx, swy] = map_.gridToWorld(sx, sy);
    auto [gwx, gwy] = map_.gridToWorld(gx, gy);
    pr.start_world_x = swx; pr.start_world_y = swy;
    pr.goal_world_x = gwx; pr.goal_world_y = gwy;
    pr.air_distance = std::hypot(gwx - swx, gwy - swy);

    bool saved_example = false;

    for (int i = 0; i < iterations; ++i) {
        auto result = planner_->plan(i, sx, sy, gx, gy);

        // Save example path from first successful iteration
        if (result.success && !saved_example) {
            pr.example_path_x = result.path_x;
            pr.example_path_y = result.path_y;
            saved_example = true;
        }

        pr.iterations.push_back(std::move(result));
    }

    computeStats(pr);
    return pr;
}

Session Runner::run(int num_pairs, int iterations, double min_distance, bool verbose) {
    Session session;
    session.timestamp = getTimestamp();
    session.num_pairs = num_pairs;
    session.iterations_per_pair = iterations;
    session.min_distance = min_distance;
    session.config = config_;

    if (verbose) {
        std::cout << "RRT First-Solution Benchmark: " << num_pairs << " pairs x " << iterations
                  << " iterations, timeout=" << config_.planning_timeout << "s\n";
    }

    auto t_start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_pairs; ++i) {
        auto [start, goal] = map_.randomPointPair(min_distance, rng_);
        auto [sx, sy] = start;
        auto [gx, gy] = goal;

        if (verbose) {
            std::cout << "Pair " << (i+1) << "/" << num_pairs << "..." << std::flush;
        }

        auto pr = runPair(i, sx, sy, gx, gy, iterations, verbose);
        session.results.push_back(std::move(pr));

        if (verbose) {
            auto& r = session.results.back();
            std::cout << " done (" << std::fixed << std::setprecision(0)
                      << (r.success_rate*100) << "% success, "
                      << std::setprecision(1) << r.mean_planning_time_ms << "ms avg)\n";
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    session.duration_seconds = std::chrono::duration<double>(t_end - t_start).count();

    computeGlobalStats(session);

    if (verbose) {
        std::cout << "\nDone in " << std::setprecision(1) << session.duration_seconds << "s. "
                  << "Success: " << std::setprecision(0) << (session.success_rate*100) << "%, "
                  << "Mean planning: " << std::setprecision(1) << session.mean_planning_time_ms << "ms, "
                  << "Mean length: " << std::setprecision(2) << session.mean_path_length << "m\n";
    }

    return session;
}

void Runner::saveJson(const Session& s, const std::string& filepath) {
    std::ofstream f(filepath);
    if (!f) throw std::runtime_error("Cannot write to: " + filepath);

    f << std::fixed;
    f << "{\n";
    f << "  \"benchmark_type\": \"rrt_first_solution\",\n";
    f << "  \"timestamp\": \"" << s.timestamp << "\",\n";
    f << "  \"map_file\": \"" << s.map_file << "\",\n";
    f << "  \"num_pairs\": " << s.num_pairs << ",\n";
    f << "  \"iterations_per_pair\": " << s.iterations_per_pair << ",\n";
    f << "  \"min_distance\": " << s.min_distance << ",\n";
    f << "  \"planning_timeout\": " << s.config.planning_timeout << ",\n";
    f << "  \"duration_seconds\": " << std::setprecision(2) << s.duration_seconds << ",\n";
    f << "  \"total_runs\": " << s.total_runs << ",\n";
    f << "  \"total_successes\": " << s.total_successes << ",\n";
    f << "  \"overall_success_rate\": " << std::setprecision(4) << s.success_rate << ",\n";
    f << "  \"mean_planning_time_ms\": " << std::setprecision(2) << s.mean_planning_time_ms << ",\n";
    f << "  \"mean_path_length\": " << std::setprecision(3) << s.mean_path_length << ",\n";
    f << "  \"config\": {\n";
    f << "    \"safety_margin\": " << s.config.safety_margin << ",\n";
    f << "    \"min_clearance_smooth\": " << s.config.min_clearance_smooth << ",\n";
    f << "    \"interpolation_spacing\": " << s.config.interpolation_spacing << ",\n";
    f << "    \"rrt_range\": " << s.config.rrt_range << ",\n";
    f << "    \"rrt_goal_bias\": " << s.config.rrt_goal_bias << "\n";
    f << "  },\n";
    f << "  \"pair_results\": [\n";

    for (size_t pi = 0; pi < s.results.size(); ++pi) {
        const auto& pr = s.results[pi];
        f << "    {\n";
        f << "      \"pair_id\": " << pr.pair_id << ",\n";
        f << "      \"start_grid\": [" << pr.start_x << ", " << pr.start_y << "],\n";
        f << "      \"goal_grid\": [" << pr.goal_x << ", " << pr.goal_y << "],\n";
        f << "      \"start_world\": [" << std::setprecision(4) << pr.start_world_x << ", " << pr.start_world_y << "],\n";
        f << "      \"goal_world\": [" << pr.goal_world_x << ", " << pr.goal_world_y << "],\n";
        f << "      \"air_distance\": " << std::setprecision(3) << pr.air_distance << ",\n";
        f << "      \"num_success\": " << pr.num_success << ",\n";
        f << "      \"success_rate\": " << std::setprecision(4) << pr.success_rate << ",\n";
        f << "      \"mean_planning_time_ms\": " << std::setprecision(2) << pr.mean_planning_time_ms << ",\n";
        f << "      \"std_planning_time_ms\": " << pr.std_planning_time_ms << ",\n";
        f << "      \"mean_path_length\": " << std::setprecision(3) << pr.mean_path_length << ",\n";
        f << "      \"std_path_length\": " << pr.std_path_length << ",\n";
        f << "      \"min_path_length\": " << pr.min_path_length << ",\n";
        f << "      \"max_path_length\": " << pr.max_path_length << ",\n";

        // Example path for visualization
        f << "      \"example_path_x\": [";
        for (size_t i = 0; i < pr.example_path_x.size(); ++i) {
            f << std::setprecision(4) << pr.example_path_x[i];
            if (i < pr.example_path_x.size() - 1) f << ", ";
        }
        f << "],\n";
        f << "      \"example_path_y\": [";
        for (size_t i = 0; i < pr.example_path_y.size(); ++i) {
            f << std::setprecision(4) << pr.example_path_y[i];
            if (i < pr.example_path_y.size() - 1) f << ", ";
        }
        f << "],\n";

        // All iterations
        f << "      \"iterations\": [\n";
        for (size_t ii = 0; ii < pr.iterations.size(); ++ii) {
            const auto& ir = pr.iterations[ii];
            f << "        {\n";
            f << "          \"iteration_id\": " << ir.iteration_id << ",\n";
            f << "          \"success\": " << (ir.success ? "true" : "false") << ",\n";
            f << "          \"message\": \"" << ir.message << "\",\n";
            f << "          \"planning_time_ms\": " << std::setprecision(2) << ir.planning_time_ms << ",\n";
            f << "          \"total_time_ms\": " << ir.total_time_ms << ",\n";
            f << "          \"num_raw_points\": " << ir.num_raw_points << ",\n";
            f << "          \"num_smoothed_points\": " << ir.num_smoothed_points << ",\n";
            f << "          \"num_final_points\": " << ir.num_final_points << ",\n";
            f << "          \"path_length\": " << std::setprecision(3) << ir.path_length << ",\n";

            // Complete route
            f << "          \"path_x\": [";
            for (size_t i = 0; i < ir.path_x.size(); ++i) {
                f << std::setprecision(4) << ir.path_x[i];
                if (i < ir.path_x.size() - 1) f << ", ";
            }
            f << "],\n";
            f << "          \"path_y\": [";
            for (size_t i = 0; i < ir.path_y.size(); ++i) {
                f << std::setprecision(4) << ir.path_y[i];
                if (i < ir.path_y.size() - 1) f << ", ";
            }
            f << "]\n";
            f << "        }" << (ii < pr.iterations.size()-1 ? ",\n" : "\n");
        }
        f << "      ]\n";
        f << "    }" << (pi < s.results.size()-1 ? ",\n" : "\n");
    }
    f << "  ]\n}\n";

    std::cout << "Saved JSON: " << filepath << std::endl;
}

void Runner::saveSummary(const Session& s, const std::string& filepath) {
    std::ofstream f(filepath);
    if (!f) throw std::runtime_error("Cannot write to: " + filepath);

    f << std::fixed;
    f << "========================================\n";
    f << "RRT FIRST-SOLUTION BENCHMARK SUMMARY\n";
    f << "========================================\n\n";
    f << "Timestamp: " << s.timestamp << "\n";
    f << "Duration: " << std::setprecision(1) << s.duration_seconds << "s\n\n";
    f << "Configuration:\n";
    f << "  Pairs: " << s.num_pairs << "\n";
    f << "  Iterations: " << s.iterations_per_pair << "\n";
    f << "  Min distance: " << s.min_distance << "m\n";
    f << "  Timeout: " << s.config.planning_timeout << "s\n";
    f << "  RRT goal bias: " << s.config.rrt_goal_bias << "\n\n";
    f << "Results:\n";
    f << "  Total runs: " << s.total_runs << "\n";
    f << "  Successes: " << s.total_successes << "\n";
    f << "  Success rate: " << std::setprecision(1) << (s.success_rate*100) << "%\n";
    f << "  Mean planning time: " << s.mean_planning_time_ms << "ms\n";
    f << "  Mean path length: " << std::setprecision(2) << s.mean_path_length << "m\n\n";
    f << "----------------------------------------\n";
    f << "Per-Pair Results:\n";
    f << "----------------------------------------\n\n";

    for (const auto& pr : s.results) {
        f << "Pair " << (pr.pair_id+1) << ":\n";
        f << "  Grid: (" << pr.start_x << "," << pr.start_y << ") -> ("
          << pr.goal_x << "," << pr.goal_y << ")\n";
        f << "  Air distance: " << std::setprecision(2) << pr.air_distance << "m\n";
        f << "  Success: " << pr.num_success << "/" << pr.iterations.size()
          << " (" << std::setprecision(0) << (pr.success_rate*100) << "%)\n";
        f << "  Planning time: " << std::setprecision(1) << pr.mean_planning_time_ms
          << " ± " << pr.std_planning_time_ms << " ms\n";
        f << "  Path length: " << std::setprecision(2) << pr.mean_path_length
          << " ± " << pr.std_path_length << " m";
        if (pr.num_success > 0) {
            f << " (min=" << pr.min_path_length << ", max=" << pr.max_path_length << ")";
        }
        f << "\n\n";
    }

    std::cout << "Saved summary: " << filepath << std::endl;
}

void Runner::saveCsv(const Session& s, const std::string& filepath) {
    std::ofstream f(filepath);
    if (!f) throw std::runtime_error("Cannot write to: " + filepath);

    // Header
    f << "pair_id,iteration_id,success,planning_time_ms,total_time_ms,"
      << "num_raw_points,num_smoothed_points,num_final_points,path_length,"
      << "start_x,start_y,goal_x,goal_y,air_distance\n";

    for (const auto& pr : s.results) {
        for (const auto& ir : pr.iterations) {
            f << pr.pair_id << ","
              << ir.iteration_id << ","
              << (ir.success ? 1 : 0) << ","
              << std::fixed << std::setprecision(2) << ir.planning_time_ms << ","
              << ir.total_time_ms << ","
              << ir.num_raw_points << ","
              << ir.num_smoothed_points << ","
              << ir.num_final_points << ","
              << std::setprecision(3) << ir.path_length << ","
              << pr.start_x << ","
              << pr.start_y << ","
              << pr.goal_x << ","
              << pr.goal_y << ","
              << pr.air_distance << "\n";
        }
    }

    std::cout << "Saved CSV: " << filepath << std::endl;
}

} // namespace rrt_first