/**
 * rrt_planner_service.cpp
 * -----------------------
 * RRT* path planning with clearance optimization (prefers middle of hallways/doors).
 * Uses distance transform for clearance costs.
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
        declare_parameter("clearance_weight",5.0);

        std::string map_path = get_parameter("map_yaml").as_string();
        safety_margin_ = get_parameter("safety_margin").as_int();
        planning_timeout_ = get_parameter("planning_timeout").as_double();
        desired_speed_ = get_parameter("desired_speed").as_double();
        clearance_weight_ = get_parameter("clearance_weight").as_double();

        load_map(map_path);

        service_ = create_service<autonomous_system::srv::PlanPath>(
            "/plan_path_rrt",
            std::bind(&RRTPlannerService::plan_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "RRT Planner Service ready (clearance_weight=%.1f)", clearance_weight_);
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
        cv::threshold(img, binary, 250, 255, cv::THRESH_BINARY_INV);  // 0=free, 255=obstacle
        cv::flip(binary, binary, 0);

        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(2 * safety_margin_ + 1, 2 * safety_margin_ + 1));
        cv::dilate(binary, map_data_, kernel);  // map_data_: 0=free, 255=obstacle

        // Distance transform: distance to nearest obstacle for each free cell
        cv::Mat free_space;
        cv::bitwise_not(map_data_, free_space);  // 255=free, 0=obstacle
        cv::distanceTransform(free_space, distance_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE);

        width_ = map_data_.cols;
        height_ = map_data_.rows;

        double max_clearance;
        cv::minMaxLoc(distance_map_, nullptr, &max_clearance);
        RCLCPP_INFO(get_logger(), "Map: %dx%d, max_clearance: %.1f px", width_, height_, max_clearance);
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

    void compute_velocities(const std::vector<double>& wx, const std::vector<double>& wy,
                           std::vector<double>& vx, std::vector<double>& vy, double speed)
    {
        size_t n = wx.size();
        vx.resize(n); vy.resize(n);

        for (size_t i = 0; i < n; ++i) {
            double dx = (i < n-1) ? wx[i+1] - wx[i] : (n > 1 ? wx[i] - wx[i-1] : 0.0);
            double dy = (i < n-1) ? wy[i+1] - wy[i] : (n > 1 ? wy[i] - wy[i-1] : 0.0);
            double mag = std::hypot(dx, dy);
            vx[i] = (mag > 1e-6) ? (dx / mag) * speed : 0.0;
            vy[i] = (mag > 1e-6) ? (dy / mag) * speed : 0.0;
        }
        if (n > 0) { vx[n-1] = vy[n-1] = 0.0; }
    }

    // Clearance optimization objective
    class ClearanceObjective : public ob::StateCostIntegralObjective
    {
    public:
        ClearanceObjective(const ob::SpaceInformationPtr& si, const RRTPlannerService* planner, double weight)
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

        // OMPL setup (same structure as original)
        auto space = std::make_shared<ob::RealVectorStateSpace>(2);
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, 0); bounds.setHigh(0, width_ - 1);
        bounds.setLow(1, 0); bounds.setHigh(1, height_ - 1);
        space->setBounds(bounds);

        og::SimpleSetup ss(space);

        // Validity checker - MUST reject obstacles
        ss.setStateValidityChecker([this](const ob::State* state) {
            const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
            return is_valid(s->values[0], s->values[1]);
        });

        // Clearance-based optimization
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

        if (solved) {
            // NOTE: Removed simplifySolution() - it can cut through obstacles!
            og::PathGeometric& path = ss.getSolutionPath();

            double path_length = path.length();
            int num_points = std::max(10, static_cast<int>(path_length / 5.0));
            path.interpolate(num_points);

            // Verify path validity before returning
            for (size_t i = 0; i < path.getStateCount(); ++i) {
                const auto* s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
                if (!is_valid(s->values[0], s->values[1])) {
                    RCLCPP_ERROR(get_logger(), "Invalid waypoint at index %zu! Rejecting path.", i);
                    response->success = false;
                    response->message = "Path validation failed - waypoint in obstacle";
                    return;
                }
                int mgx = static_cast<int>(std::round(s->values[0]));
                int mgy = static_cast<int>(std::round(s->values[1]));
                auto [wx, wy] = map_to_world(mgx, mgy);
                response->waypoints_x.push_back(wx);
                response->waypoints_y.push_back(wy);
            }

            // Ensure goal included
            if (!response->waypoints_x.empty()) {
                double dist = std::hypot(response->waypoints_x.back() - request->goal_x,
                                        response->waypoints_y.back() - request->goal_y);
                if (dist > 0.3) {
                    response->waypoints_x.push_back(request->goal_x);
                    response->waypoints_y.push_back(request->goal_y);
                }
            }

            compute_velocities(response->waypoints_x, response->waypoints_y,
                             response->velocities_x, response->velocities_y, desired_speed_);

            response->success = true;
            response->message = "Path found: " + std::to_string(response->waypoints_x.size()) + " waypoints";
            RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        } else {
            response->success = false;
            response->message = "No path found";
            RCLCPP_WARN(get_logger(), "No path found");
        }
    }

    cv::Mat map_data_, distance_map_;
    int width_, height_, safety_margin_;
    double resolution_, origin_x_, origin_y_;
    double planning_timeout_, desired_speed_, clearance_weight_;
    rclcpp::Service<autonomous_system::srv::PlanPath>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTPlannerService>());
    rclcpp::shutdown();
    return 0;
}