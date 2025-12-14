#!/usr/bin/env python3
"""
RRT Navigation Service Node
---------------------------
ROS2 service that uses RRT* for path planning and navigates the drone.
Minimal implementation compatible with meta_agent_node.py.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from autonomous_system.srv import NavigateToPose
from autonomous_system.planning.rrt_planner import RRTStarPlanner

import math
import time


class RRTNavigationService(Node):
    """ROS2 service node for RRT*-based navigation."""

    def __init__(self):
        super().__init__("rrt_navigation_service")

        # Parameters
        self.declare_parameter("map_yaml", "/root/sjtu_project/sjtu_drone/maps/hospital_map_cropped.yaml")
        self.declare_parameter("safety_margin", 10)
        self.declare_parameter("cruise_speed", 0.5)
        self.declare_parameter("waypoint_tolerance", 0.3)

        map_path = self.get_parameter("map_yaml").value
        margin = self.get_parameter("safety_margin").value

        # Initialize planner
        self.planner = RRTStarPlanner(map_path, safety_margin=margin)
        self.speed = self.get_parameter("cruise_speed").value
        self.tolerance = self.get_parameter("waypoint_tolerance").value

        # Drone state
        self.current_pose = None

        # ROS2 interfaces
        self.pose_sub = self.create_subscription(Pose, "/simple_drone/gt_pose", self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)
        self.srv = self.create_service(NavigateToPose, "/navigate_rrt", self.navigate_cb)

        self.get_logger().info("RRT Navigation Service ready on /navigate_rrt")

    def pose_cb(self, msg: Pose):
        self.current_pose = msg

    def navigate_cb(self, request, response):
        """Service callback: navigate to (x, y, z) using RRT*."""
        goal = (request.x, request.y)
        target_z = request.z if request.z > 0 else 1.5

        if self.current_pose is None:
            response.success = False
            response.message = "No pose data available"
            return response

        start = (self.current_pose.position.x, self.current_pose.position.y)
        self.get_logger().info(f"Planning RRT* path: {start} -> {goal}")

        # Plan path
        waypoints = self.planner.plan_world(start, goal, timeout=3.0)

        if not waypoints:
            response.success = False
            response.message = "RRT* could not find a path"
            return response

        self.get_logger().info(f"Following {len(waypoints)} waypoints")

        # Follow waypoints
        success = self.follow_path(waypoints, target_z)

        response.success = success
        response.message = "Goal reached" if success else "Navigation failed"
        return response

    def follow_path(self, waypoints, target_z: float) -> bool:
        """Simple waypoint follower."""
        for i, (wx, wy) in enumerate(waypoints):
            self.get_logger().info(f"Waypoint {i+1}/{len(waypoints)}: ({wx:.2f}, {wy:.2f})")

            while rclpy.ok():
                if self.current_pose is None:
                    time.sleep(0.1)
                    continue

                dx = wx - self.current_pose.position.x
                dy = wy - self.current_pose.position.y
                dz = target_z - self.current_pose.position.z
                dist = math.hypot(dx, dy)

                if dist < self.tolerance:
                    break

                # Compute velocity command
                cmd = Twist()
                cmd.linear.x = self.speed * dx / dist
                cmd.linear.y = self.speed * dy / dist
                cmd.linear.z = 0.3 * dz if abs(dz) > 0.1 else 0.0
                self.cmd_pub.publish(cmd)

                time.sleep(0.05)

        # Stop at goal
        self.cmd_pub.publish(Twist())
        return True


def main():
    rclpy.init()
    node = RRTNavigationService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()