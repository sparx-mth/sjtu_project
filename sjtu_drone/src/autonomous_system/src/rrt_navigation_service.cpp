/**
 * rrt_navigation_service.cpp
 * --------------------------
 * Minimal RRT* navigation service using OMPL for 2D occupancy grid.
 * Same interface as Python version - works with meta_agent_node.py
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <autonomous_system/srv/navigate_to_pose.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class RRTNavigationService : public rclcpp::Node
{
public:
    RRTNavigationService() : Node("rrt_navigation_service")
    {
        // Parameters
        declare_parameter("map_yaml", "/root/sjtu_project/sjtu_drone/maps/hospital_map_cropped.yaml");
        declare_parameter("safety_margin", 12);
        declare_parameter("cruise_speed", 0.4);
        declare_parameter("waypoint_tolerance", 0.2);

        std::string map_path = get_parameter("map_yaml").as_string();
        safety_margin_ = get_parameter("safety_margin").as_int();
        cruise_speed_ = get_parameter("cruise_speed").as_double();
        tolerance_ = get_parameter("waypoint_tolerance").as_double();

        // Load map
        load_map(map_path);

        // Use separate callback group for subscription (allows concurrent execution)
        auto sub_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = sub_cb_group;

        // ROS interfaces
        pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/simple_drone/gt_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                current_pose_ = *msg;
                has_pose_ = true;
            }, sub_options);

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);

        service_ = create_service<autonomous_system::srv::NavigateToPose>(
            "/navigate_rrt",
            std::bind(&RRTNavigationService::navigate_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "RRT Navigation Service ready on /navigate_rrt");
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

        // Binary: 0=free, 1=obstacle (only white=255 is free)
        cv::Mat binary;
        cv::threshold(img, binary, 250, 1, cv::THRESH_BINARY_INV);
        cv::flip(binary, binary, 0);  // Flip for bottom-left origin

        // Inflate obstacles
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(2 * safety_margin_ + 1, 2 * safety_margin_ + 1));
        cv::dilate(binary, map_data_, kernel);

        width_ = map_data_.cols;
        height_ = map_data_.rows;

        RCLCPP_INFO(get_logger(), "Map loaded: %dx%d", width_, height_);
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

    std::vector<std::pair<double, double>> plan(double sx, double sy, double gx, double gy)
    {
        std::vector<std::pair<double, double>> waypoints;

        auto [start_gx, start_gy] = world_to_map(sx, sy);
        auto [goal_gx, goal_gy] = world_to_map(gx, gy);

        if (!is_valid(start_gx, start_gy) || !is_valid(goal_gx, goal_gy)) {
            RCLCPP_ERROR(get_logger(), "Start or goal in obstacle!");
            return waypoints;
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

        // Validity checker
        ss.setStateValidityChecker([this](const ob::State* state) {
            const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
            return is_valid(s->values[0], s->values[1]);
        });

        // Start and goal
        ob::ScopedState<> start(space);
        start[0] = start_gx;
        start[1] = start_gy;

        ob::ScopedState<> goal(space);
        goal[0] = goal_gx;
        goal[1] = goal_gy;

        ss.setStartAndGoalStates(start, goal);

        // Use RRT*
        ss.setPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));

        // Solve
        ob::PlannerStatus solved = ss.solve(3.0);

        if (solved) {
            ss.simplifySolution();
            og::PathGeometric& path = ss.getSolutionPath();
            path.interpolate();

            for (size_t i = 0; i < path.getStateCount(); ++i) {
                const auto* s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
                int mgx = static_cast<int>(std::round(s->values[0]));
                int mgy = static_cast<int>(std::round(s->values[1]));
                auto [wx, wy] = map_to_world(mgx, mgy);
                waypoints.push_back({wx, wy});
            }
            RCLCPP_INFO(get_logger(), "Path found: %zu waypoints", waypoints.size());
        } else {
            RCLCPP_WARN(get_logger(), "No path found");
        }

        return waypoints;
    }

    geometry_msgs::msg::Pose get_current_pose()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return current_pose_;
    }

    void navigate_callback(
        const std::shared_ptr<autonomous_system::srv::NavigateToPose::Request> request,
        std::shared_ptr<autonomous_system::srv::NavigateToPose::Response> response)
    {
        if (!has_pose_) {
            response->success = false;
            response->message = "No pose data";
            return;
        }

        double target_z = request->z > 0 ? request->z : 1.5;
        auto pose = get_current_pose();

        RCLCPP_INFO(get_logger(), "Planning to (%.2f, %.2f)", request->x, request->y);

        auto waypoints = plan(pose.position.x, pose.position.y, request->x, request->y);

        if (waypoints.empty()) {
            response->success = false;
            response->message = "No path found";
            return;
        }

        // Follow waypoints
        for (size_t i = 0; i < waypoints.size(); ++i) {
            auto [wx, wy] = waypoints[i];
            RCLCPP_INFO(get_logger(), "Waypoint %zu/%zu: (%.2f, %.2f)",
                        i + 1, waypoints.size(), wx, wy);

            while (rclcpp::ok()) {
                pose = get_current_pose();
                double dx = wx - pose.position.x;
                double dy = wy - pose.position.y;
                double dz = target_z - pose.position.z;
                double dist = std::hypot(dx, dy);

                if (dist < tolerance_) break;

                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = cruise_speed_ * dx / dist;
                cmd.linear.y = cruise_speed_ * dy / dist;
                cmd.linear.z = std::abs(dz) > 0.1 ? 0.3 * dz : 0.0;
                cmd_pub_->publish(cmd);

                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

        // Stop
        cmd_pub_->publish(geometry_msgs::msg::Twist());

        response->success = true;
        response->message = "Goal reached";
    }

    // Map data
    cv::Mat map_data_;
    int width_, height_, safety_margin_;
    double resolution_, origin_x_, origin_y_;

    // Navigation
    double cruise_speed_, tolerance_;
    geometry_msgs::msg::Pose current_pose_;
    bool has_pose_ = false;
    std::mutex pose_mutex_;

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Service<autonomous_system::srv::NavigateToPose>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Use MultiThreadedExecutor to allow subscription callbacks during service
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<RRTNavigationService>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}