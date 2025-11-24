#!/usr/bin/env python3
"""
Meta-Agent (Simple Version)
---------------------------
A high-level agent that interacts with the NavigationAgentService.

For now:
    - Reads a target (x,y,z) from the user
    - Sends a request to /navigate_to_pose
    - Waits for the result
    - Prints success/failure

Later this Meta-Agent can be upgraded into:
    - Behavior tree
    - Mission planner
    - RL/MCTS agent
    - Multi-agent coordinator
"""

import rclpy
from rclpy.node import Node
from autonomous_system.srv import NavigateToPose


class MetaAgent(Node):
    """
    A simple high-level agent that commands the navigation service.

    Responsibilities:
        - Ask user for a target point
        - Call /navigate_to_pose service
        - Wait for result
        - Print outcome

    Future responsibilities will include:
        - Decision making
        - Task sequencing
        - Room scanning, door navigation, exploration, etc.
    """

    def __init__(self):
        super().__init__("meta_agent")

        # Client to the navigation service
        self.cli = self.create_client(NavigateToPose, "/navigate_to_pose")

        self.get_logger().info("Waiting for /navigate_to_pose service...")

        # Wait until service appears
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available yet, waiting...")

        self.get_logger().info("Meta-Agent ready.")

        # Start interactive loop
        self.main_loop()

    # ---------------------------------------------------------------------

    def main_loop(self):
        """Main interactive text loop for now."""
        while rclpy.ok():
            try:
                x = float(input("Enter target X (meters): "))
                y = float(input("Enter target Y (meters): "))
                z = float(input("Enter target Z (meters): "))
            except ValueError:
                print("Invalid input. Enter numeric values.")
                continue

            print(f"Sending navigation request to ({x:.2f}, {y:.2f}, {z:.2f}) ...")
            success, message = self.navigate_to(x, y, z)

            print(f"\n=== RESULT ===")
            print(f"Success: {success}")
            print(f"Message: {message}\n")

    # ---------------------------------------------------------------------

    def navigate_to(self, x: float, y: float, z: float):
        """
        Send a navigation request to the NavigationAgentService.

        Returns:
            (success: bool, message: str)
        """
        req = NavigateToPose.Request()
        req.x = x
        req.y = y
        req.z = z

        future = self.cli.call_async(req)

        # Wait for answer
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return False, "Service call failed or interrupted."

        return future.result().success, future.result().message


# ---------------------------------------------------------------------

def main():
    rclpy.init()
    node = MetaAgent()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
