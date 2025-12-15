/**
 * rrt_planner_service.cpp
 * -----------------------
 * Minimal RRT* path planning service using OMPL.
 * ONLY does planning - returns waypoints. No navigation.
 */

#include <rclcpp/rclcpp.hpp>
#include <autonomous_system/srv/plan_path.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

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

        std::string map_path = get_parameter("map_yaml").as_string();
        safety_margin_ = get_parameter("safety_margin").as_int();
        planning_timeout_ = get_parameter("planning_timeout").as_double();

        load_map(map_path);

        service_ = create_service<autonomous_system::srv::PlanPath>(
            "/plan_path_rrt",
            std::bind(&RRTPlannerService::plan_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "RRT Planner Service ready on /plan_path_rrt");
    }

private:
    void load_map(const std::string& yaml_path)
    {
        YAML::Node config = YAML::LoadFile(yaml_path);
        resolution_ = config["resolution"].as<double>();

        auto origin = config["origin"];
        origin_x_ = origin[0].as<double>();
        origin_y_ = origin[1].as<double>();

        std::string img_path = config["image"].as<std::string>();
        if (img_path[0] != '/') {
            size_t pos = yaml_path.find_last_of('/');
            img_path = yaml_path.substr(0, pos + 1) + img_path;
        }

        cv::Mat img = cv::imread(img_path, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            RCLCPP_ERROR(get_logger(), "Failed to load map: %s", img_path.c_str());
            return;
        }

        cv::Mat binary;
        cv::threshold(img, binary, 250, 1, cv::THRESH_BINARY_INV);
        cv::flip(binary, binary, 0);

        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(2 * safety_margin_ + 1, 2 * safety_margin_ + 1));
        cv::dilate(binary, map_data_, kernel);

        width_ = map_data_.cols;
        height_ = map_data_.rows;

        RCLCPP_INFO(get_logger(), "Map loaded: %dx%d, resolution: %.3f", width_, height_, resolution_);
    }

    bool is_valid(double x, double y)
    {
        int gx = static_cast<int>(std::round(x));
        int gy = static_cast<int>(std::round(y));
        if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return false;
        return map_data_.at<uchar>(gy, gx) == 0;
    }

    std::pair<int, int> world_to_map(double wx, double wy)
    {
        int gx = static_cast<int>(std::round((wx - origin_x_) / resolution_));
        int gy = static_cast<int>(std::round((wy - origin_y_) / resolution_));
        return {gx, gy};
    }

    std::pair<double, double> map_to_world(int gx, int gy)
    {
        double wx = gx * resolution_ + origin_x_;
        double wy = gy * resolution_ + origin_y_;
        return {wx, wy};
    }

    void plan_callback(
        const std::shared_ptr<autonomous_system::srv::PlanPath::Request> request,
        std::shared_ptr<autonomous_system::srv::PlanPath::Response> response)
    {
        auto [start_gx, start_gy] = world_to_map(request->start_x, request->start_y);
        auto [goal_gx, goal_gy] = world_to_map(request->goal_x, request->goal_y);

        RCLCPP_INFO(get_logger(), "Planning: (%.2f, %.2f) -> (%.2f, %.2f)",
                    request->start_x, request->start_y, request->goal_x, request->goal_y);

        if (!is_valid(start_gx, start_gy)) {
            response->success = false;
            response->message = "Start position in obstacle";
            return;
        }
        if (!is_valid(goal_gx, goal_gy)) {
            response->success = false;
            response->message = "Goal position in obstacle";
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
            ss.simplifySolution();
            og::PathGeometric& path = ss.getSolutionPath();

            // Interpolate
            double path_length = path.length();
            int num_points = std::max(10, static_cast<int>(path_length / 5.0));
            path.interpolate(num_points);

            // Convert to world coordinates
            for (size_t i = 0; i < path.getStateCount(); ++i) {
                const auto* s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
                int mgx = static_cast<int>(std::round(s->values[0]));
                int mgy = static_cast<int>(std::round(s->values[1]));
                auto [wx, wy] = map_to_world(mgx, mgy);
                response->waypoints_x.push_back(wx);
                response->waypoints_y.push_back(wy);
            }

            // Ensure goal is included
            if (!response->waypoints_x.empty()) {
                double last_x = response->waypoints_x.back();
                double last_y = response->waypoints_y.back();
                double dist = std::hypot(last_x - request->goal_x, last_y - request->goal_y);
                if (dist > 0.3) {
                    response->waypoints_x.push_back(request->goal_x);
                    response->waypoints_y.push_back(request->goal_y);
                }
            }

            response->success = true;
            response->message = "Path found with " + std::to_string(response->waypoints_x.size()) + " waypoints";
            RCLCPP_INFO(get_logger(), "Path found: %zu waypoints", response->waypoints_x.size());
        } else {
            response->success = false;
            response->message = "No path found";
            RCLCPP_WARN(get_logger(), "No path found");
        }
    }

    cv::Mat map_data_;
    int width_, height_, safety_margin_;
    double resolution_, origin_x_, origin_y_;
    double planning_timeout_;

    rclcpp::Service<autonomous_system::srv::PlanPath>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTPlannerService>());
    rclcpp::shutdown();
    return 0;
}