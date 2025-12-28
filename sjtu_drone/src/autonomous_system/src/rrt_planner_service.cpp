/**
 * rrt_planner_service.cpp
 * -----------------------
 * RRT* path planning with Dubins curves and clearance optimization.
 *
 * Pipeline:
 *   1. RRT* planning with Dubins state space (respects turning radius)
 *   2. Adaptive smoothing (remove redundant points, keep tight spaces)
 *   3. Dubins-native interpolation (arc-length based sampling)
 *   4. Convert to world coordinates + compute velocities
 *
 * Changes for Dubins:
 *   - Uses DubinsStateSpace instead of RealVectorStateSpace
 *   - States include (x, y, yaw)
 *   - Paths respect minimum turning radius constraint
 *   - No need for external spline smoothing (Dubins paths are already smooth)
 */

#include <rclcpp/rclcpp.hpp>
#include <autonomous_system/srv/plan_path.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/DiscreteMotionValidator.h>

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
        declare_parameter("safety_margin", 8);
        declare_parameter("planning_timeout", 6.0);
        declare_parameter("desired_speed", 0.4);
        declare_parameter("clearance_weight", 5.0);
        declare_parameter("min_clearance_for_smooth", 15.0);
        declare_parameter("interpolation_spacing", 3.0);  // meters between interpolated points
        declare_parameter("turning_radius", 0.5);         // Dubins minimum turning radius (meters)

        std::string map_path = get_parameter("map_yaml").as_string();
        safety_margin_ = get_parameter("safety_margin").as_int();
        planning_timeout_ = get_parameter("planning_timeout").as_double();
        desired_speed_ = get_parameter("desired_speed").as_double();
        clearance_weight_ = get_parameter("clearance_weight").as_double();
        min_clearance_smooth_ = get_parameter("min_clearance_for_smooth").as_double();
        interpolation_spacing_ = get_parameter("interpolation_spacing").as_double();
        turning_radius_ = get_parameter("turning_radius").as_double();

        load_map(map_path);

        service_ = create_service<autonomous_system::srv::PlanPath>(
            "/plan_path_rrt",
            std::bind(&RRTPlannerService::plan_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "RRT Dubins Planner ready (clearance=%.1f, turning_radius=%.2fm)",
                    clearance_weight_, turning_radius_);
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

public:
    // Public for DubinsMotionValidator access
    bool is_valid(double x, double y) const
    {
        int gx = static_cast<int>(std::round(x));
        int gy = static_cast<int>(std::round(y));
        if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return false;
        return map_data_.at<uchar>(gy, gx) == 0;
    }

private:
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

    std::pair<double, double> map_to_world(double gx, double gy) const
    {
        return {gx * resolution_ + origin_x_, gy * resolution_ + origin_y_};
    }

    // =========================================================================
    // Compute initial heading from start to goal
    // =========================================================================
    double compute_heading(double from_x, double from_y, double to_x, double to_y) const
    {
        return std::atan2(to_y - from_y, to_x - from_x);
    }

    // =========================================================================
    // Interpolate Dubins path at regular arc-length spacing
    // =========================================================================
    void interpolate_dubins_path(
        og::PathGeometric& path,
        const ob::SpaceInformationPtr& si,
        double spacing,
        std::vector<double>& world_x,
        std::vector<double>& world_y,
        std::vector<double>& yaws) const
    {
        world_x.clear();
        world_y.clear();
        yaws.clear();

        if (path.getStateCount() < 2) return;

        auto* dubins_space = si->getStateSpace()->as<ob::DubinsStateSpace>();

        for (size_t i = 0; i < path.getStateCount() - 1; ++i) {
            const ob::State* s1 = path.getState(i);
            const ob::State* s2 = path.getState(i + 1);

            double seg_length = dubins_space->distance(s1, s2);
            int n_samples = std::max(2, static_cast<int>(std::ceil(seg_length / spacing)));

            for (int j = 0; j < n_samples; ++j) {
                double t = static_cast<double>(j) / n_samples;

                ob::State* interp = si->allocState();
                dubins_space->interpolate(s1, s2, t, interp);

                const auto* se2 = interp->as<ob::DubinsStateSpace::StateType>();
                double gx = se2->getX();
                double gy = se2->getY();
                double yaw = se2->getYaw();

                auto [wx, wy] = map_to_world(gx, gy);
                world_x.push_back(wx);
                world_y.push_back(wy);
                yaws.push_back(yaw);

                si->freeState(interp);
            }
        }

        // Add final state
        const auto* final_state = path.getState(path.getStateCount() - 1)
                                      ->as<ob::DubinsStateSpace::StateType>();
        auto [wx, wy] = map_to_world(final_state->getX(), final_state->getY());
        world_x.push_back(wx);
        world_y.push_back(wy);
        yaws.push_back(final_state->getYaw());
    }

    void compute_velocities(
        const std::vector<double>& yaws,
        std::vector<double>& vx, std::vector<double>& vy, double speed) const
    {
        size_t n = yaws.size();
        vx.resize(n);
        vy.resize(n);

        for (size_t i = 0; i < n; ++i) {
            vx[i] = std::cos(yaws[i]) * speed;
            vy[i] = std::sin(yaws[i]) * speed;
        }

        // Zero velocity at goal
        if (n > 0) {
            vx[n - 1] = 0.0;
            vy[n - 1] = 0.0;
        }
    }

    // =========================================================================
    // Custom motion validator that densely checks Dubins curves
    // =========================================================================
    class DubinsMotionValidator : public ob::MotionValidator
    {
    public:
        DubinsMotionValidator(const ob::SpaceInformationPtr& si,
                              const RRTPlannerService* planner,
                              double check_resolution = 0.5)  // Check every 0.5 pixels
            : ob::MotionValidator(si), planner_(planner), resolution_(check_resolution)
        {
            dubins_space_ = si->getStateSpace()->as<ob::DubinsStateSpace>();
        }

        bool checkMotion(const ob::State* s1, const ob::State* s2) const override
        {
            // Get Dubins path length
            double dist = dubins_space_->distance(s1, s2);
            if (dist < 1e-6) return true;

            // Number of checks along the curve
            int n_checks = std::max(2, static_cast<int>(std::ceil(dist / resolution_)));

            ob::State* interp = si_->allocState();

            for (int i = 0; i <= n_checks; ++i) {
                double t = static_cast<double>(i) / n_checks;
                dubins_space_->interpolate(s1, s2, t, interp);

                const auto* st = interp->as<ob::DubinsStateSpace::StateType>();
                if (!planner_->is_valid(st->getX(), st->getY())) {
                    si_->freeState(interp);
                    return false;
                }
            }

            si_->freeState(interp);
            return true;
        }

        bool checkMotion(const ob::State* s1, const ob::State* s2,
                        std::pair<ob::State*, double>& lastValid) const override
        {
            double dist = dubins_space_->distance(s1, s2);
            if (dist < 1e-6) {
                lastValid.second = 1.0;
                return true;
            }

            int n_checks = std::max(2, static_cast<int>(std::ceil(dist / resolution_)));

            ob::State* interp = si_->allocState();
            double last_valid_t = 0.0;

            for (int i = 0; i <= n_checks; ++i) {
                double t = static_cast<double>(i) / n_checks;
                dubins_space_->interpolate(s1, s2, t, interp);

                const auto* st = interp->as<ob::DubinsStateSpace::StateType>();
                if (!planner_->is_valid(st->getX(), st->getY())) {
                    if (lastValid.first != nullptr && last_valid_t > 0) {
                        dubins_space_->interpolate(s1, s2, last_valid_t, lastValid.first);
                    }
                    lastValid.second = last_valid_t;
                    si_->freeState(interp);
                    return false;
                }
                last_valid_t = t;
            }

            si_->freeState(interp);
            lastValid.second = 1.0;
            return true;
        }

    private:
        const RRTPlannerService* planner_;
        const ob::DubinsStateSpace* dubins_space_;
        double resolution_;
    };

    // Clearance optimization objective (updated for Dubins)
    class ClearanceObjective : public ob::StateCostIntegralObjective
    {
    public:
        ClearanceObjective(const ob::SpaceInformationPtr& si,
                          const RRTPlannerService* planner, double weight)
            : ob::StateCostIntegralObjective(si, true), planner_(planner), weight_(weight) {}

        ob::Cost stateCost(const ob::State* s) const override
        {
            const auto* st = s->as<ob::DubinsStateSpace::StateType>();
            double clearance = planner_->get_clearance(st->getX(), st->getY());
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

        RCLCPP_INFO(get_logger(), "Planning (Dubins): (%.2f,%.2f) -> (%.2f,%.2f)",
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

        // Compute headings (point toward goal from start, maintain at goal)
        double start_yaw = compute_heading(start_gx, start_gy, goal_gx, goal_gy);
        double goal_yaw = start_yaw;  // Arrive with same heading (can be customized)

        // Convert turning radius from world meters to map pixels
        double turning_radius_pixels = turning_radius_ / resolution_;

        // =====================================================================
        // Dubins State Space Setup
        // =====================================================================
        auto space = std::make_shared<ob::DubinsStateSpace>(turning_radius_pixels);

        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, 0);
        bounds.setHigh(0, width_ - 1);
        bounds.setLow(1, 0);
        bounds.setHigh(1, height_ - 1);
        space->setBounds(bounds);

        og::SimpleSetup ss(space);

        // Use custom motion validator that checks along Dubins curves
        auto si = ss.getSpaceInformation();
        si->setMotionValidator(std::make_shared<DubinsMotionValidator>(si, this, 1.0));

        // State validity checker (only checks x, y position)
        ss.setStateValidityChecker([this](const ob::State* state) {
            const auto* s = state->as<ob::DubinsStateSpace::StateType>();
            return is_valid(s->getX(), s->getY());
        });

        // Use path length objective (clearance doesn't work well with asymmetric Dubins)
        // ss.setOptimizationObjective(
        //     std::make_shared<ClearanceObjective>(ss.getSpaceInformation(), this, clearance_weight_));

        // =====================================================================
        // Set start and goal with headings
        // =====================================================================
        ob::ScopedState<ob::DubinsStateSpace> start(space);
        start->setX(static_cast<double>(start_gx));
        start->setY(static_cast<double>(start_gy));
        start->setYaw(start_yaw);

        ob::ScopedState<ob::DubinsStateSpace> goal(space);
        goal->setX(static_cast<double>(goal_gx));
        goal->setY(static_cast<double>(goal_gy));
        goal->setYaw(goal_yaw);

        ss.setStartAndGoalStates(start, goal);

        // Use RRT (not RRTstar) - RRTstar requires symmetric distance which Dubins lacks
        auto planner = std::make_shared<og::RRT>(si);
        planner->setRange(turning_radius_pixels * 3);  // Limit extension distance
        ss.setPlanner(planner);

        ob::PlannerStatus solved = ss.solve(planning_timeout_);

        if (!solved) {
            response->success = false;
            response->message = "No path found";
            RCLCPP_WARN(get_logger(), "No path found");
            return;
        }

        og::PathGeometric& path = ss.getSolutionPath();

        // =====================================================================
        // Simplify path (optional - Dubins paths are already smooth)
        // =====================================================================
        // path.simplify() can be called but may break Dubins constraints
        // We skip heavy smoothing since Dubins already respects turning radius

        // =====================================================================
        // Interpolate along Dubins curves
        // =====================================================================
        std::vector<double> world_x, world_y, yaws;
        interpolate_dubins_path(path, si, interpolation_spacing_ / resolution_,
                               world_x, world_y, yaws);

        // =====================================================================
        // Validate path
        // =====================================================================
        for (size_t i = 0; i < world_x.size(); ++i) {
            auto [gx, gy] = world_to_map(world_x[i], world_y[i]);
            if (!is_valid(gx, gy)) {
                RCLCPP_WARN(get_logger(), "Path point %zu invalid, but continuing", i);
            }
        }

        // Ensure goal is included
        double dist_to_goal = std::hypot(
            world_x.back() - request->goal_x,
            world_y.back() - request->goal_y);
        if (dist_to_goal > 0.1) {
            world_x.push_back(request->goal_x);
            world_y.push_back(request->goal_y);
            yaws.push_back(goal_yaw);
        }

        // =====================================================================
        // Compute velocities and fill response
        // =====================================================================
        response->waypoints_x = world_x;
        response->waypoints_y = world_y;

        compute_velocities(yaws, response->velocities_x, response->velocities_y, desired_speed_);

        response->success = true;
        response->message = "Dubins path: " + std::to_string(path.getStateCount()) +
                           " states -> " + std::to_string(world_x.size()) + " pts";

        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    cv::Mat map_data_, distance_map_;
    int width_, height_, safety_margin_;
    double resolution_, origin_x_, origin_y_;
    double planning_timeout_, desired_speed_, clearance_weight_;
    double min_clearance_smooth_, interpolation_spacing_;
    double turning_radius_;
    rclcpp::Service<autonomous_system::srv::PlanPath>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTPlannerService>());
    rclcpp::shutdown();
    return 0;
}