/**
 * rrt_benchmark.cpp
 * -----------------
 * Implementation of RRT* path planning benchmark.
 */

#include "rrt_benchmark.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <filesystem>

#include <yaml-cpp/yaml.h>

namespace rrt_bench {

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

        // Dilate obstacles
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(2*safety_margin+1, 2*safety_margin+1));
        cv::dilate(binary, occupancy_, kernel);

        // Distance transform
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
    throw std::runtime_error("Could not find free point");
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
// Clearance Objective for OMPL
// =============================================================================

class ClearanceObjective : public ob::StateCostIntegralObjective {
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si, const Map* map, double weight)
        : ob::StateCostIntegralObjective(si, true), map_(map), weight_(weight) {}

    ob::Cost stateCost(const ob::State* s) const override {
        auto* rs = s->as<ob::RealVectorStateSpace::StateType>();
        double c = map_->getClearance(rs->values[0], rs->values[1]);
        return ob::Cost(weight_ / (c + 1.0));
    }
private:
    const Map* map_;
    double weight_;
};

// =============================================================================
// Planner Implementation
// =============================================================================

Planner::Planner(Map* map, const Config& config) : map_(map), config_(config) {
    // OMPL setup is done per-plan call (same as original pipeline)
}

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

PlanResult Planner::plan(int start_x, int start_y, int goal_x, int goal_y) {
    PlanResult result;

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
    // OMPL Setup - Same as original pipeline (using SimpleSetup)
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

    ss.setOptimizationObjective(
        std::make_shared<ClearanceObjective>(ss.getSpaceInformation(), map_, config_.clearance_weight));

    ob::ScopedState<> start(space);
    start[0] = static_cast<double>(start_x);
    start[1] = static_cast<double>(start_y);

    ob::ScopedState<> goal(space);
    goal[0] = static_cast<double>(goal_x);
    goal[1] = static_cast<double>(goal_y);

    ss.setStartAndGoalStates(start, goal);
    ss.setPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));

    // =========================================================================
    // Two-phase planning: fine polling for first solution, coarse for improvements
    // =========================================================================
    auto t_start = std::chrono::high_resolution_clock::now();

    // Storage for solution snapshots
    struct SolutionSnapshot {
        double time_ms;
        double path_length;
        std::vector<double> path_x;
        std::vector<double> path_y;
        int num_raw_points;
        int num_smoothed_points;
    };
    std::vector<SolutionSnapshot> snapshots;

    auto si = ss.getSpaceInformation();

    // Helper lambda to validate that a path doesn't go through obstacles
    auto isPathValid = [&](og::PathGeometric& path) -> bool {
        if (path.getStateCount() < 2) return false;

        for (size_t i = 0; i < path.getStateCount() - 1; ++i) {
            // Check if motion between consecutive states is valid
            if (!si->checkMotion(path.getState(i), path.getState(i + 1))) {
                return false;
            }
        }
        return true;
    };

    // Helper lambda to extract and process current solution
    auto extractSolution = [&]() -> SolutionSnapshot {
        og::PathGeometric& current_path = ss.getSolutionPath();

        // Extract raw path from OMPL (validated path)
        std::vector<std::pair<double,double>> raw;
        for (size_t i = 0; i < current_path.getStateCount(); ++i) {
            const auto* s = current_path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
            raw.emplace_back(s->values[0], s->values[1]);
        }
        int n_raw = raw.size();

        // Apply smoothing
        auto smoothed = smooth(raw, si);
        int n_smooth = smoothed.size();

        // Convert to world coordinates
        std::vector<double> wx, wy;
        for (auto& [gx, gy] : smoothed) {
            auto [x, y] = map_->gridToWorld(std::round(gx), std::round(gy));
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

        // Interpolate
        interpolate(wx, wy, config_.interpolation_spacing);

        auto t_now = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t_now - t_start).count();
        double len = pathLength(wx, wy);

        return {elapsed_ms, len, std::move(wx), std::move(wy), n_raw, n_smooth};
    };

    // -------------------------------------------------------------------------
    // Phase 1: Fine polling (1ms) until first VALID solution is found
    // -------------------------------------------------------------------------
    const double fine_poll = 0.001;  // 1ms
    double elapsed_sec = 0;

    while (elapsed_sec < config_.planning_timeout) {
        ob::PlannerStatus status = ss.solve(fine_poll);

        auto t_now = std::chrono::high_resolution_clock::now();
        elapsed_sec = std::chrono::duration<double>(t_now - t_start).count();

        // Only accept EXACT solutions that pass validation
        if (status == ob::PlannerStatus::EXACT_SOLUTION && ss.haveSolutionPath()) {
            og::PathGeometric& path = ss.getSolutionPath();

            // Verify the path is actually valid (doesn't go through walls)
            if (isPathValid(path)) {
                // Found first valid solution - extract it
                auto first_snap = extractSolution();

                result.success = true;
                result.first_solution_time_ms = first_snap.time_ms;
                result.first_solution_length = first_snap.path_length;
                result.first_path_x = first_snap.path_x;
                result.first_path_y = first_snap.path_y;
                result.path_x = first_snap.path_x;
                result.path_y = first_snap.path_y;
                result.final_path_length = first_snap.path_length;

                snapshots.push_back(std::move(first_snap));
                break;  // Move to phase 2
            }
        }
    }

    // -------------------------------------------------------------------------
    // Phase 2: Coarse polling (50ms) for path optimization
    // -------------------------------------------------------------------------
    if (result.success) {
        const double coarse_poll = config_.snapshot_interval_ms / 1000.0;  // 50ms default
        double remaining = config_.planning_timeout - elapsed_sec;

        while (remaining > 0) {
            double dt = std::min(coarse_poll, remaining);
            ob::PlannerStatus status = ss.solve(dt);

            // Check for improved valid solution
            if (status == ob::PlannerStatus::EXACT_SOLUTION && ss.haveSolutionPath()) {
                auto snap = extractSolution();

                // Only record if path improved meaningfully
                if (snap.path_length < snapshots.back().path_length * 0.999) {
                    snapshots.push_back(snap);
                }

                // Always update final path
                result.path_x = std::move(snap.path_x);
                result.path_y = std::move(snap.path_y);
                result.final_path_length = snap.path_length;
            }

            remaining -= dt;
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    result.total_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // Build result snapshots
    for (const auto& snap : snapshots) {
        result.snapshots.push_back({
            snap.time_ms,
            snap.path_length,
            snap.num_raw_points,
            snap.num_smoothed_points,
            static_cast<int>(snap.path_x.size())
        });
    }

    result.message = result.success ? "OK" : "No path found";

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
    std::vector<double> times, lengths, imps;
    int succ = 0;

    for (auto& r : pr.iterations) {
        if (r.success) {
            succ++;
            times.push_back(r.first_solution_time_ms);
            lengths.push_back(r.final_path_length);
            if (r.first_solution_length > 0) {
                imps.push_back(100.0 * (r.first_solution_length - r.final_path_length)
                               / r.first_solution_length);
            }
        }
    }

    pr.success_rate = static_cast<double>(succ) / pr.iterations.size();
    pr.mean_first_time_ms = mean(times);
    pr.std_first_time_ms = stddev(times);
    pr.mean_final_length = mean(lengths);
    pr.std_final_length = stddev(lengths);
    pr.mean_improvement_pct = mean(imps);
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
                times.push_back(r.first_solution_time_ms);
                lengths.push_back(r.final_path_length);
            }
        }
    }

    s.success_rate = s.total_runs > 0 ?
        static_cast<double>(s.total_successes) / s.total_runs : 0;
    s.mean_first_time_ms = mean(times);
    s.mean_final_length = mean(lengths);
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
        auto result = planner_->plan(sx, sy, gx, gy);

        // Save example paths from first successful iteration
        if (result.success && !saved_example) {
            pr.example_first_path_x = result.first_path_x;
            pr.example_first_path_y = result.first_path_y;
            pr.example_final_path_x = result.path_x;
            pr.example_final_path_y = result.path_y;
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
        std::cout << "RRT* Benchmark: " << num_pairs << " pairs x " << iterations
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
                      << std::setprecision(1) << r.mean_first_time_ms << "ms avg)\n";
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    session.duration_seconds = std::chrono::duration<double>(t_end - t_start).count();

    computeGlobalStats(session);

    if (verbose) {
        std::cout << "\nDone in " << std::setprecision(1) << session.duration_seconds << "s. "
                  << "Success: " << std::setprecision(0) << (session.success_rate*100) << "%, "
                  << "Mean first: " << std::setprecision(1) << session.mean_first_time_ms << "ms, "
                  << "Mean length: " << std::setprecision(2) << session.mean_final_length << "m\n";
    }

    return session;
}

void Runner::saveJson(const Session& s, const std::string& filepath) {
    std::ofstream f(filepath);
    if (!f) throw std::runtime_error("Cannot write to: " + filepath);

    f << std::fixed;
    f << "{\n";
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
    f << "  \"mean_first_solution_time_ms\": " << std::setprecision(2) << s.mean_first_time_ms << ",\n";
    f << "  \"mean_final_path_length\": " << std::setprecision(3) << s.mean_final_length << ",\n";
    f << "  \"config\": {\n";
    f << "    \"safety_margin\": " << s.config.safety_margin << ",\n";
    f << "    \"clearance_weight\": " << s.config.clearance_weight << ",\n";
    f << "    \"min_clearance_smooth\": " << s.config.min_clearance_smooth << ",\n";
    f << "    \"interpolation_spacing\": " << s.config.interpolation_spacing << "\n";
    f << "  },\n";
    f << "  \"pair_results\": [\n";

    for (size_t pi = 0; pi < s.results.size(); ++pi) {
        const auto& pr = s.results[pi];
        f << "    {\n";
        f << "      \"pair_id\": " << pr.pair_id << ",\n";
        f << "      \"start_grid\": [" << pr.start_x << ", " << pr.start_y << "],\n";
        f << "      \"goal_grid\": [" << pr.goal_x << ", " << pr.goal_y << "],\n";
        f << "      \"air_distance_world\": " << std::setprecision(3) << pr.air_distance << ",\n";
        f << "      \"success_rate\": " << std::setprecision(4) << pr.success_rate << ",\n";
        f << "      \"mean_first_solution_time_ms\": " << std::setprecision(2) << pr.mean_first_time_ms << ",\n";
        f << "      \"std_first_solution_time_ms\": " << pr.std_first_time_ms << ",\n";
        f << "      \"mean_final_length\": " << std::setprecision(3) << pr.mean_final_length << ",\n";
        f << "      \"std_final_length\": " << pr.std_final_length << ",\n";
        f << "      \"mean_improvement_percent\": " << std::setprecision(2) << pr.mean_improvement_pct << ",\n";

        // Example first path (from first successful iteration)
        f << "      \"example_first_path_x\": [";
        for (size_t i = 0; i < pr.example_first_path_x.size(); ++i) {
            f << std::setprecision(4) << pr.example_first_path_x[i];
            if (i < pr.example_first_path_x.size() - 1) f << ", ";
        }
        f << "],\n";
        f << "      \"example_first_path_y\": [";
        for (size_t i = 0; i < pr.example_first_path_y.size(); ++i) {
            f << std::setprecision(4) << pr.example_first_path_y[i];
            if (i < pr.example_first_path_y.size() - 1) f << ", ";
        }
        f << "],\n";

        // Example final path
        f << "      \"example_final_path_x\": [";
        for (size_t i = 0; i < pr.example_final_path_x.size(); ++i) {
            f << std::setprecision(4) << pr.example_final_path_x[i];
            if (i < pr.example_final_path_x.size() - 1) f << ", ";
        }
        f << "],\n";
        f << "      \"example_final_path_y\": [";
        for (size_t i = 0; i < pr.example_final_path_y.size(); ++i) {
            f << std::setprecision(4) << pr.example_final_path_y[i];
            if (i < pr.example_final_path_y.size() - 1) f << ", ";
        }
        f << "],\n";

        f << "      \"iteration_results\": [\n";

        for (size_t ii = 0; ii < pr.iterations.size(); ++ii) {
            const auto& ir = pr.iterations[ii];
            double imp = (ir.success && ir.first_solution_length > 0) ?
                100.0 * (ir.first_solution_length - ir.final_path_length) / ir.first_solution_length : 0;

            f << "        {\n";
            f << "          \"success\": " << (ir.success ? "true" : "false") << ",\n";
            f << "          \"total_time_ms\": " << std::setprecision(2) << ir.total_time_ms << ",\n";
            f << "          \"first_solution_time_ms\": " << ir.first_solution_time_ms << ",\n";
            f << "          \"first_solution_length\": " << std::setprecision(3) << ir.first_solution_length << ",\n";
            f << "          \"final_path_length\": " << ir.final_path_length << ",\n";
            f << "          \"improvement_percent\": " << std::setprecision(2) << imp << ",\n";
            f << "          \"num_improvements\": " << ir.snapshots.size() << ",\n";
            f << "          \"improvement_timeline\": [\n";

            for (size_t si = 0; si < ir.snapshots.size(); ++si) {
                const auto& snap = ir.snapshots[si];
                f << "            {\"timestamp_ms\": " << std::setprecision(2) << snap.time_ms
                  << ", \"path_length\": " << std::setprecision(3) << snap.path_length << "}";
                f << (si < ir.snapshots.size()-1 ? ",\n" : "\n");
            }
            f << "          ]\n";
            f << "        }" << (ii < pr.iterations.size()-1 ? ",\n" : "\n");
        }
        f << "      ]\n";
        f << "    }" << (pi < s.results.size()-1 ? ",\n" : "\n");
    }
    f << "  ]\n}\n";

    std::cout << "Saved: " << filepath << std::endl;
}

void Runner::saveSummary(const Session& s, const std::string& filepath) {
    std::ofstream f(filepath);
    if (!f) throw std::runtime_error("Cannot write to: " + filepath);

    f << std::fixed;
    f << "========================================\n";
    f << "RRT* BENCHMARK SUMMARY\n";
    f << "========================================\n\n";
    f << "Timestamp: " << s.timestamp << "\n";
    f << "Duration: " << std::setprecision(1) << s.duration_seconds << "s\n\n";
    f << "Configuration:\n";
    f << "  Pairs: " << s.num_pairs << "\n";
    f << "  Iterations: " << s.iterations_per_pair << "\n";
    f << "  Min distance: " << s.min_distance << "m\n";
    f << "  Timeout: " << s.config.planning_timeout << "s\n\n";
    f << "Results:\n";
    f << "  Total runs: " << s.total_runs << "\n";
    f << "  Successes: " << s.total_successes << "\n";
    f << "  Success rate: " << std::setprecision(1) << (s.success_rate*100) << "%\n";
    f << "  Mean first solution: " << s.mean_first_time_ms << "ms\n";
    f << "  Mean path length: " << std::setprecision(2) << s.mean_final_length << "m\n\n";
    f << "----------------------------------------\n";
    f << "Per-Pair Results:\n";
    f << "----------------------------------------\n\n";

    for (const auto& pr : s.results) {
        f << "Pair " << (pr.pair_id+1) << ":\n";
        f << "  (" << pr.start_x << "," << pr.start_y << ") -> ("
          << pr.goal_x << "," << pr.goal_y << ")\n";
        f << "  Air: " << std::setprecision(2) << pr.air_distance << "m\n";
        f << "  Success: " << std::setprecision(0) << (pr.success_rate*100) << "%\n";
        f << "  First solution: " << std::setprecision(1) << pr.mean_first_time_ms
          << " ± " << pr.std_first_time_ms << " ms\n";
        f << "  Final length: " << std::setprecision(2) << pr.mean_final_length
          << " ± " << pr.std_final_length << " m\n";
        f << "  Improvement: " << std::setprecision(1) << pr.mean_improvement_pct << "%\n\n";
    }

    std::cout << "Saved: " << filepath << std::endl;
}

} // namespace rrt_bench