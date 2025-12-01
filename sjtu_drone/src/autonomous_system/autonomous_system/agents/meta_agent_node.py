#!/usr/bin/env python3
"""
Meta-Agent (Multi-Agent Version)
---------------------------------
A high-level agent that interacts with multiple navigation services.

Available agents:
    1. NavigationAgentService - Navigate to specific coordinates
    2. DoorwayTraversalAgent - Find and traverse the nearest door
    3. FrontierExplorationService - Autonomous frontier exploration
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
        - /explore_frontiers : Autonomous frontier-based exploration
    """

    def __init__(self):
        super().__init__("meta_agent")

        # Client to the navigation service
        self.nav_client = self.create_client(NavigateToPose, "/navigate_to_pose")

        # Client to the doorway traversal service
        self.door_client = self.create_client(NavigateToPose, "/traverse_doorway")

        # Client to the frontier exploration service
        self.explore_client = self.create_client(NavigateToPose, "/explore_frontiers")

        self.get_logger().info("Waiting for services...")

        # Wait for navigation service
        self.get_logger().info("  Waiting for /navigate_to_pose...")
        while not self.nav_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("    Service not available yet, waiting...")

        # Check for optional services (don't block on them)
        self.door_available = self.door_client.wait_for_service(timeout_sec=2.0)
        if self.door_available:
            self.get_logger().info("  /traverse_doorway available")
        else:
            self.get_logger().info("  /traverse_doorway not available (optional)")

        self.explore_available = self.explore_client.wait_for_service(timeout_sec=2.0)
        if self.explore_available:
            self.get_logger().info("  /explore_frontiers available")
        else:
            self.get_logger().info("  /explore_frontiers not available (optional)")

        self.get_logger().info("Meta-Agent ready.")

        # Start interactive loop
        self.main_loop()

    # ---------------------------------------------------------------------

    def print_menu(self):
        """Print the available agents menu."""
        print("\n" + "=" * 50)
        print("           AUTONOMOUS DRONE CONTROL")
        print("=" * 50)
        print("  1. Navigate to coordinates (x, y, z)")
        if self.door_available:
            print("  2. Traverse nearest doorway")
        else:
            print("  2. [Unavailable] Traverse nearest doorway")
        if self.explore_available:
            print("  3. Frontier exploration (autonomous)")
        else:
            print("  3. [Unavailable] Frontier exploration")
        print("  4. Exit")
        print("=" * 50)

    def main_loop(self):
        """Main interactive text loop."""
        while rclpy.ok():
            self.print_menu()

            try:
                choice = input("Select agent (1-4): ").strip()
            except EOFError:
                break

            if choice == "1":
                self.handle_navigation()
            elif choice == "2":
                if self.door_available:
                    self.handle_doorway_traversal()
                else:
                    print("Doorway traversal service not available.")
            elif choice == "3":
                if self.explore_available:
                    self.handle_frontier_exploration()
                else:
                    print("Frontier exploration service not available.")
            elif choice == "4":
                print("Exiting Meta-Agent...")
                break
            else:
                print("Invalid choice. Please enter 1-4.")

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

        try:
            z = float(input("Enter cruise altitude (meters) [default=1.5]: ") or "1.5")
        except ValueError:
            z = 1.5

        print(f"\nSending doorway traversal request (altitude={z:.2f}m) ...")
        success, message = self.call_doorway_service(z)

        self.print_result(success, message)

    def handle_frontier_exploration(self):
        """Handle frontier exploration request."""
        print("\n--- Frontier Exploration ---")
        print("The drone will autonomously explore unknown areas.")
        print("It will navigate to frontiers (boundaries between known and unknown).")

        confirm = input("Proceed? (y/n) [default=y]: ").strip().lower() or "y"
        if confirm != "y":
            print("Cancelled.")
            return

        try:
            z = float(input("Enter cruise altitude (meters) [default=1.5]: ") or "1.5")
        except ValueError:
            z = 1.5

        print(f"\nStarting frontier exploration (altitude={z:.2f}m) ...")
        print("This may take a while. Press Ctrl+C to interrupt.")
        success, message = self.call_exploration_service(z)

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

        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return False, "Service call failed or interrupted."

        return future.result().success, future.result().message

    def call_doorway_service(self, z: float = 1.5):
        """
        Send a traversal request to the DoorwayTraversalAgent.

        Returns:
            (success: bool, message: str)
        """
        req = NavigateToPose.Request()
        req.x = 0.0
        req.y = 0.0
        req.z = z

        future = self.door_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return False, "Service call failed or interrupted."

        return future.result().success, future.result().message

    def call_exploration_service(self, z: float = 1.5):
        """
        Send an exploration request to the FrontierExplorationService.

        Returns:
            (success: bool, message: str)
        """
        req = NavigateToPose.Request()
        req.x = 0.0  # Ignored
        req.y = 0.0  # Ignored
        req.z = z    # Cruise altitude

        future = self.explore_client.call_async(req)

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
            print(f"  SUCCESS: {message}")
        else:
            print(f"  FAILED: {message}")
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