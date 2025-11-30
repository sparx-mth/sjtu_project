#!/usr/bin/env python3
"""
Meta-Agent (Multi-Agent Version)
---------------------------------
A high-level agent that interacts with multiple navigation services.

Available agents:
    1. NavigationAgentService - Navigate to specific coordinates
    2. DoorwayTraversalAgent - Find and traverse the nearest door

For now:
    - Asks user which agent to activate
    - Collects required inputs
    - Sends request to the appropriate service
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
    A high-level agent that commands multiple navigation services.

    Available Services:
        - /navigate_to_pose : Navigate to specific world coordinates
        - /traverse_doorway : Find and traverse the nearest door

    Responsibilities:
        - Present menu of available agents
        - Collect required inputs from user
        - Call appropriate service
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
        self.nav_client = self.create_client(NavigateToPose, "/navigate_to_pose")

        # Client to the doorway traversal service
        self.door_client = self.create_client(NavigateToPose, "/traverse_doorway")

        self.get_logger().info("Waiting for services...")

        # Wait for navigation service
        self.get_logger().info("  Waiting for /navigate_to_pose...")
        while not self.nav_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("    Service not available yet, waiting...")

        # Wait for doorway traversal service
        self.get_logger().info("  Waiting for /traverse_doorway...")
        while not self.door_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("    Service not available yet, waiting...")

        self.get_logger().info("Meta-Agent ready. All services available.")

        # Start interactive loop
        self.main_loop()

    # ---------------------------------------------------------------------

    def print_menu(self):
        """Print the available agents menu."""
        print("\n" + "=" * 50)
        print("           AUTONOMOUS DRONE CONTROL")
        print("=" * 50)
        print("  1. Navigate to coordinates (x, y, z)")
        print("  2. Traverse nearest doorway")
        print("  3. Exit")
        print("=" * 50)

    def main_loop(self):
        """Main interactive text loop."""
        while rclpy.ok():
            self.print_menu()

            try:
                choice = input("Select agent (1-3): ").strip()
            except EOFError:
                break

            if choice == "1":
                self.handle_navigation()
            elif choice == "2":
                self.handle_doorway_traversal()
            elif choice == "3":
                print("Exiting Meta-Agent...")
                break
            else:
                print("Invalid choice. Please enter 1, 2, or 3.")

    # ---------------------------------------------------------------------

    def handle_navigation(self):
        """Handle navigation to specific coordinates."""
        print("\n--- Navigate to Coordinates ---")
        try:
            x = float(input("Enter target X (meters): "))
            y = float(input("Enter target Y (meters): "))
            z = float(input("Enter target Z (meters) [default=1.5]: ") or "1.5")
            z = float(z)
        except ValueError:
            print("Invalid input. Enter numeric values.")
            return

        print(f"\nSending navigation request to ({x:.2f}, {y:.2f}, {z:.2f}) ...")
        success, message = self.call_navigation_service(x, y, z)

        self.print_result(success, message)

    def handle_doorway_traversal(self):
        """Handle doorway traversal request."""
        print("\n--- Traverse Nearest Doorway ---")
        print("The drone will find and traverse the nearest door.")

        confirm = input("Proceed? (y/n) [default=y]: ").strip().lower() or "y"
        if confirm != "y":
            print("Cancelled.")
            return

        # Get current altitude preference
        try:
            z = float(input("Enter cruise altitude (meters) [default=1.5]: ") or "1.5")
        except ValueError:
            z = 1.5

        print(f"\nSending doorway traversal request (altitude={z:.2f}m) ...")
        success, message = self.call_doorway_service(z)

        self.print_result(success, message)

    # ---------------------------------------------------------------------

    def call_navigation_service(self, x: float, y: float, z: float):
        """
        Send a navigation request to the NavigationAgentService.

        Returns:
            (success: bool, message: str)
        """
        req = NavigateToPose.Request()
        req.x = x
        req.y = y
        req.z = z

        future = self.nav_client.call_async(req)

        # Wait for answer
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return False, "Service call failed or interrupted."

        return future.result().success, future.result().message

    def call_doorway_service(self, z: float = 1.5):
        """
        Send a traversal request to the DoorwayTraversalAgent.

        The x, y coordinates are ignored by the service - it finds
        the nearest door automatically.

        Returns:
            (success: bool, message: str)
        """
        req = NavigateToPose.Request()
        req.x = 0.0  # Ignored - agent finds nearest door
        req.y = 0.0  # Ignored - agent finds nearest door
        req.z = z    # Cruise altitude

        future = self.door_client.call_async(req)

        # Wait for answer
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return False, "Service call failed or interrupted."

        return future.result().success, future.result().message

    # ---------------------------------------------------------------------

    def print_result(self, success: bool, message: str):
        """Print the result of an operation."""
        print("\n" + "=" * 50)
        print("                   RESULT")
        print("=" * 50)
        if success:
            print(f"  ✓ SUCCESS: {message}")
        else:
            print(f"  ✗ FAILED: {message}")
        print("=" * 50)


# ---------------------------------------------------------------------

def main():
    rclpy.init()
    try:
        node = MetaAgent()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()