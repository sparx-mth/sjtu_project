/**
 * rrt_planner_service.cpp
 * -----------------------
 * RRT* path planning with clearance optimization.
 *
 * Pipeline:
 *   1. RRT* planning with clearance cost
 *   2. Adaptive smoothing (remove redundant points, keep tight spaces)
 *   3. Interpolation (add points for smooth spline fitting)
 *   4. Convert to world coordinates + compute velocities
 */

#include <rclcpp/rclcpp.hpp>
#include <autonomous_system/srv/plan_path.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class RRTPlannerService : public rclcpp::Node
{
public:
    RRTPlannerService() : Node("rrt_planner_service")
    {
        declare_parameter("map_yaml", "/root/sjtu_project/sjtu_drone/maps/hospital_map_cropped.yaml");
        declare_parameter("safety_margin", 10);
        declare_parameter("planning_timeout", 3.0);
        declare_parameter("desired_speed", 0.4);
        declare_parameter("clearance_weight", 10.0);
        declare_parameter("min_clearance_for_smooth", 15.0);
        declare_parameter("interpolation_spacing", 2.0);  // meters between interpolated points

        std::string map_path = get_parameter("map_yaml").as_string();
        safety_margin_ = get_parameter("safety_margin").as_int();
        planning_timeout_ = get_parameter("planning_timeout").as_double();
        desired_speed_ = get_parameter("desired_speed").as_double();
        clearance_weight_ = get_parameter("clearance_weight").as_double();
        min_clearance_smooth_ = get_parameter("min_clearance_for_smooth").as_double();
        interpolation_spacing_ = get_parameter("interpolation_spacing").as_double();

        load_map(map_path);

        service_ = create_service<autonomous_system::srv::PlanPath>(
            "/plan_path_rrt",
            std::bind(&RRTPlannerService::plan_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "RRT Planner ready (clearance=%.1f, interp=%.2fm)",
                    clearance_weight_, interpolation_spacing_);
    }

private:
    void load_map(const std::string& yaml_path)
    {
        YAML::Node config = YAML::LoadFile(yaml_path);
        resolution_ = config["resolution"].as<double>();
        origin_x_ = config["origin"][0].as<double>();
        origin_y_ = config["origin"][1].as<double>();

        std::string img_path = config["image"].as<std::string>();
        if (img_path[0] != '/') {
            img_path = yaml_path.substr(0, yaml_path.find_last_of('/') + 1) + img_path;
        }

        cv::Mat img = cv::imread(img_path, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            RCLCPP_ERROR(get_logger(), "Failed to load map: %s", img_path.c_str());
            return;
        }

        cv::Mat binary;
        cv::threshold(img, binary, 250, 255, cv::THRESH_BINARY_INV);
        cv::flip(binary, binary, 0);

        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(2 * safety_margin_ + 1, 2 * safety_margin_ + 1));
        cv::dilate(binary, map_data_, kernel);

        cv::Mat free_space;
        cv::bitwise_not(map_data_, free_space);
        cv::distanceTransform(free_space, distance_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE);

        width_ = map_data_.cols;
        height_ = map_data_.rows;

        RCLCPP_INFO(get_logger(), "Map: %dx%d, resolution: %.3f", width_, height_, resolution_);
    }

    bool is_valid(double x, double y) const
    {
        int gx = static_cast<int>(std::round(x));
        int gy = static_cast<int>(std::round(y));
        if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return false;
        return map_data_.at<uchar>(gy, gx) == 0;
    }

    double get_clearance(double x, double y) const
    {
        int gx = static_cast<int>(std::round(x));
        int gy = static_cast<int>(std::round(y));
        if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return 0.0;
        return distance_map_.at<float>(gy, gx);
    }

    std::pair<int, int> world_to_map(double wx, double wy) const
    {
        return {static_cast<int>(std::round((wx - origin_x_) / resolution_)),
                static_cast<int>(std::round((wy - origin_y_) / resolution_))};
    }

    std::pair<double, double> map_to_world(int gx, int gy) const
    {
        return {gx * resolution_ + origin_x_, gy * resolution_ + origin_y_};
    }

    // =========================================================================
    // Interpolate path to have points at regular spacing (in world meters)
    // =========================================================================
    void interpolate_path(
        std::vector<double>& wx, std::vector<double>& wy,
        double spacing) const
    {
        if (wx.size() < 2 || spacing <= 0) return;

        std::vector<double> new_wx, new_wy;
        new_wx.push_back(wx[0]);
        new_wy.push_back(wy[0]);

        for (size_t i = 1; i < wx.size(); ++i) {
            double x0 = wx[i - 1], y0 = wy[i - 1];
            double x1 = wx[i], y1 = wy[i];
            double dx = x1 - x0, dy = y1 - y0;
            double seg_len = std::hypot(dx, dy);

            if (seg_len < 1e-6) continue;

            // Number of intermediate points
            int n_points = static_cast<int>(std::floor(seg_len / spacing));

            // Add intermediate points
            for (int j = 1; j <= n_points; ++j) {
                double t = static_cast<double>(j) / (n_points + 1);
                new_wx.push_back(x0 + t * dx);
                new_wy.push_back(y0 + t * dy);
            }

            // Add endpoint
            new_wx.push_back(x1);
            new_wy.push_back(y1);
        }

        wx = std::move(new_wx);
        wy = std::move(new_wy);
    }

    void compute_velocities(
        const std::vector<double>& wx, const std::vector<double>& wy,
        std::vector<double>& vx, std::vector<double>& vy, double speed) const
    {
        size_t n = wx.size();
        vx.resize(n);
        vy.resize(n);

        for (size_t i = 0; i < n; ++i) {
            double dx = (i < n - 1) ? wx[i + 1] - wx[i] : (n > 1 ? wx[i] - wx[i - 1] : 0.0);
            double dy = (i < n - 1) ? wy[i + 1] - wy[i] : (n > 1 ? wy[i] - wy[i - 1] : 0.0);
            double mag = std::hypot(dx, dy);
            vx[i] = (mag > 1e-6) ? (dx / mag) * speed : 0.0;
            vy[i] = (mag > 1e-6) ? (dy / mag) * speed : 0.0;
        }

        // Zero velocity at goal
        if (n > 0) {
            vx[n - 1] = 0.0;
            vy[n - 1] = 0.0;
        }
    }

    // Clearance optimization objective
    class ClearanceObjective : public ob::StateCostIntegralObjective
    {
    public:
        ClearanceObjective(const ob::SpaceInformationPtr& si,
                          const RRTPlannerService* planner, double weight)
            : ob::StateCostIntegralObjective(si, true), planner_(planner), weight_(weight) {}

        ob::Cost stateCost(const ob::State* s) const override
        {
            const auto* st = s->as<ob::RealVectorStateSpace::StateType>();
            double clearance = planner_->get_clearance(st->values[0], st->values[1]);
            return ob::Cost(weight_ / (clearance + 1.0));
        }

    private:
        const RRTPlannerService* planner_;
        double weight_;
    };

    void plan_callback(
        const std::shared_ptr<autonomous_system::srv::PlanPath::Request> request,
        std::shared_ptr<autonomous_system::srv::PlanPath::Response> response)
    {
        auto [start_gx, start_gy] = world_to_map(request->start_x, request->start_y);
        auto [goal_gx, goal_gy] = world_to_map(request->goal_x, request->goal_y);

        RCLCPP_INFO(get_logger(), "Planning: (%.2f,%.2f) -> (%.2f,%.2f)",
                    request->start_x, request->start_y, request->goal_x, request->goal_y);

        if (!is_valid(start_gx, start_gy)) {
            response->success = false;
            response->message = "Start in obstacle";
            return;
        }
        if (!is_valid(goal_gx, goal_gy)) {
            response->success = false;
            response->message = "Goal in obstacle";
            return;
        }

        // OMPL setup
        auto space = std::make_shared<ob::RealVectorStateSpace>(2);
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, 0);
        bounds.setHigh(0, width_ - 1);
        bounds.setLow(1, 0);
        bounds.setHigh(1, height_ - 1);
        space->setBounds(bounds);

        og::SimpleSetup ss(space);

        ss.setStateValidityChecker([this](const ob::State* state) {
            const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
            return is_valid(s->values[0], s->values[1]);
        });

        ss.setOptimizationObjective(
            std::make_shared<ClearanceObjective>(ss.getSpaceInformation(), this, clearance_weight_));

        ob::ScopedState<> start(space);
        start[0] = static_cast<double>(start_gx);
        start[1] = static_cast<double>(start_gy);

        ob::ScopedState<> goal(space);
        goal[0] = static_cast<double>(goal_gx);
        goal[1] = static_cast<double>(goal_gy);

        ss.setStartAndGoalStates(start, goal);
        ss.setPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));

        ob::PlannerStatus solved = ss.solve(planning_timeout_);

        if (!solved) {
            response->success = false;
            response->message = "No path found";
            RCLCPP_WARN(get_logger(), "No path found");
            return;
        }

        og::PathGeometric& path = ss.getSolutionPath();
        auto& si = ss.getSpaceInformation();

        // =====================================================================
        // Step 1: Adaptive smoothing (remove redundant, keep tight spaces)
        // =====================================================================
        std::vector<ob::State*> smoothed;
        smoothed.push_back(si->cloneState(path.getState(0)));

        for (size_t i = 1; i < path.getStateCount() - 1; ++i) {
            const auto* curr = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
            double clearance = get_clearance(curr->values[0], curr->values[1]);
            bool can_skip = si->checkMotion(smoothed.back(), path.getState(i + 1));

            if (clearance < min_clearance_smooth_ || !can_skip) {
                smoothed.push_back(si->cloneState(path.getState(i)));
            }
        }
        smoothed.push_back(si->cloneState(path.getState(path.getStateCount() - 1)));

        // =====================================================================
        // Step 2: Convert to world coordinates
        // =====================================================================
        std::vector<double> world_x, world_y;

        for (auto* s : smoothed) {
            const auto* st = s->as<ob::RealVectorStateSpace::StateType>();

            if (!is_valid(st->values[0], st->values[1])) {
                RCLCPP_ERROR(get_logger(), "Invalid waypoint in smoothed path!");
                for (auto* state : smoothed) si->freeState(state);
                response->success = false;
                response->message = "Path validation failed";
                return;
            }

            int mgx = static_cast<int>(std::round(st->values[0]));
            int mgy = static_cast<int>(std::round(st->values[1]));
            auto [wx, wy] = map_to_world(mgx, mgy);
            world_x.push_back(wx);
            world_y.push_back(wy);

            si->freeState(s);
        }

        // Ensure goal is included
        double dist_to_goal = std::hypot(
            world_x.back() - request->goal_x,
            world_y.back() - request->goal_y);
        if (dist_to_goal > 0.1) {
            world_x.push_back(request->goal_x);
            world_y.push_back(request->goal_y);
        }

        size_t before_interp = world_x.size();

        // =====================================================================
        // Step 3: Interpolate for smooth spline fitting
        // =====================================================================
        interpolate_path(world_x, world_y, interpolation_spacing_);

        // =====================================================================
        // Step 4: Compute velocities and fill response
        // =====================================================================
        response->waypoints_x = world_x;
        response->waypoints_y = world_y;

        compute_velocities(
            response->waypoints_x, response->waypoints_y,
            response->velocities_x, response->velocities_y,
            desired_speed_);

        response->success = true;
        response->message = "Path: " + std::to_string(before_interp) + " -> " +
                           std::to_string(world_x.size()) + " pts (interpolated)";

        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    cv::Mat map_data_, distance_map_;
    int width_, height_, safety_margin_;
    double resolution_, origin_x_, origin_y_;
    double planning_timeout_, desired_speed_, clearance_weight_;
    double min_clearance_smooth_, interpolation_spacing_;
    rclcpp::Service<autonomous_system::srv::PlanPath>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTPlannerService>());
    rclcpp::shutdown();
    return 0;
}