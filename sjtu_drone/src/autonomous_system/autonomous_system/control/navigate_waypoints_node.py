#!/usr/bin/env python3
"""
Waypoint Follower Node
----------------------
Listens for a path published as a PoseArray on /planned_path
and flies through all received waypoints sequentially.

This node does NOT generate waypoints.
It only executes them using the shared WaypointController.

Typical workflow:
    - A* planner publishes PoseArray to /planned_path
    - This node receives it
    - It calls controller.goto(x, y, z) for each waypoint
"""

import rclpy
from geometry_msgs.msg import PoseArray
from autonomous_system.control.waypoint_controller import WaypointController


class WaypointFollower(WaypointController):
    """
    A node that follows externally provided waypoints.

    Inherits:
        WaypointController for movement

    Subscribes:
        /planned_path : geometry_msgs/PoseArray

    Behavior:
        - On path reception: extract (x,y) waypoints
        - Sequentially navigate to each waypoint
        - Log progress and completion
    """

    def __init__(self):
        super().__init__('waypoint_follower')

        # subscribe to planner output
        self.sub = self.create_subscription(
            PoseArray,
            '/planned_path',
            self.path_cb,
            10
        )

        self.path = []
        self.get_logger().info("Waiting for /planned_path...")

    # -------------------------------------------------------------------------

    def path_cb(self, msg: PoseArray):
        """Receive a path and begin navigation."""
        self.path = [(p.position.x, p.position.y) for p in msg.poses]
        self.get_logger().info(f"Received path with {len(self.path)} waypoints.")
        self.follow_path()

    # -------------------------------------------------------------------------

    def follow_path(self):
        """Navigate sequentially through all waypoints in self.path."""
        for (x, y) in self.path:
            self.get_logger().info(f"→ Next waypoint: ({x:.2f}, {y:.2f})")
            self.goto(x, y)
        self.get_logger().info("All waypoints completed ✓")


# -------------------------------------------------------------------------

def main():
    """Initialize and spin the WaypointFollower node."""
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
