#!/usr/bin/env python3
"""
Meta-Agent (Multi-Agent Version)
---------------------------------
A high-level agent that interacts with multiple navigation services.

Available agents:
    1. NavigationAgentService - Navigate to specific coordinates (A*)
    2. DoorwayTraversalAgent - Find and traverse the nearest door
    3. FrontierExplorationService - Autonomous frontier exploration
    4. TurnRightAgent - Rotate 90° clockwise
    5. TurnLeftAgent - Rotate 90° counterclockwise
    6. MoveForwardAgent - Move forward 1 meter
    7. RRT Navigation - Navigate using RRT* planner (point-by-point)
    8. RRT Smooth Navigation - Navigate using RRT* with smooth trajectory
"""

import rclpy
from rclpy.node import Node
from autonomous_system.srv import NavigateToPose


class MetaAgent(Node):
    """
    A high-level agent that commands multiple navigation services.
    """

    def __init__(self):
        super().__init__("meta_agent")

        # Service clients
        self.nav_client = self.create_client(NavigateToPose, "/navigate_to_pose")
        self.door_client = self.create_client(NavigateToPose, "/traverse_doorway")
        self.explore_client = self.create_client(NavigateToPose, "/explore_frontiers")
        self.turn_right_client = self.create_client(NavigateToPose, "/turn_right")
        self.turn_left_client = self.create_client(NavigateToPose, "/turn_left")
        self.move_forward_client = self.create_client(NavigateToPose, "/move_forward")
        self.rrt_client = self.create_client(NavigateToPose, "/navigate_rrt")
        self.rrt_smooth_client = self.create_client(NavigateToPose, "/navigate_rrt_smooth")

        self.get_logger().info("Waiting for services...")

        # Wait for navigation service
        self.get_logger().info("  Waiting for /navigate_to_pose...")
        while not self.nav_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("    Service not available yet, waiting...")

        # Check for optional services (don't block on them)
        self.door_available = self.door_client.wait_for_service(timeout_sec=2.0)
        self.explore_available = self.explore_client.wait_for_service(timeout_sec=2.0)
        self.turn_right_available = self.turn_right_client.wait_for_service(timeout_sec=2.0)
        self.turn_left_available = self.turn_left_client.wait_for_service(timeout_sec=2.0)
        self.move_forward_available = self.move_forward_client.wait_for_service(timeout_sec=2.0)
        self.rrt_available = self.rrt_client.wait_for_service(timeout_sec=2.0)
        self.rrt_smooth_available = self.rrt_smooth_client.wait_for_service(timeout_sec=2.0)

        for name, avail in [("/traverse_doorway", self.door_available),
                            ("/explore_frontiers", self.explore_available),
                            ("/turn_right", self.turn_right_available),
                            ("/turn_left", self.turn_left_available),
                            ("/move_forward", self.move_forward_available),
                            ("/navigate_rrt", self.rrt_available),
                            ("/navigate_rrt_smooth", self.rrt_smooth_available)]:
            status = "available" if avail else "not available (optional)"
            self.get_logger().info(f"  {name} {status}")

        self.get_logger().info("Meta-Agent ready.")
        self.main_loop()

    # ---------------------------------------------------------------------

    def print_menu(self):
        """Print the available agents menu."""
        print("\n" + "=" * 50)
        print("           AUTONOMOUS DRONE CONTROL")
        print("=" * 50)
        print("  1. Navigate to coordinates (A*)")
        print(f"  2. {'Traverse nearest doorway' if self.door_available else '[Unavailable] Traverse nearest doorway'}")
        print(f"  3. {'Frontier exploration (autonomous)' if self.explore_available else '[Unavailable] Frontier exploration'}")
        print(f"  4. {'Turn right (90° clockwise)' if self.turn_right_available else '[Unavailable] Turn right'}")
        print(f"  5. {'Turn left (90° counter-clockwise)' if self.turn_left_available else '[Unavailable] Turn left'}")
        print(f"  6. {'Move forward (1 meter)' if self.move_forward_available else '[Unavailable] Move forward'}")
        print(f"  7. {'Navigate via RRT* (point-by-point)' if self.rrt_available else '[Unavailable] Navigate via RRT*'}")
        print(f"  8. {'Navigate via RRT* (smooth)' if self.rrt_smooth_available else '[Unavailable] Navigate via RRT* (smooth)'}")
        print("  9. Exit")
        print("=" * 50)

    def main_loop(self):
        """Main interactive text loop."""
        while rclpy.ok():
            self.print_menu()

            try:
                choice = input("Select agent (1-9): ").strip()
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
                if self.turn_right_available:
                    self.handle_turn_right()
                else:
                    print("Turn right service not available.")
            elif choice == "5":
                if self.turn_left_available:
                    self.handle_turn_left()
                else:
                    print("Turn left service not available.")
            elif choice == "6":
                if self.move_forward_available:
                    self.handle_move_forward()
                else:
                    print("Move forward service not available.")
            elif choice == "7":
                if self.rrt_available:
                    self.handle_rrt_navigation()
                else:
                    print("RRT navigation service not available.")
            elif choice == "8":
                if self.rrt_smooth_available:
                    self.handle_rrt_smooth_navigation()
                else:
                    print("RRT smooth navigation service not available.")
            elif choice == "9":
                print("Exiting Meta-Agent...")
                break
            else:
                print("Invalid choice. Please enter 1-9.")

    # ---------------------------------------------------------------------

    def handle_navigation(self):
        """Handle navigation to specific coordinates."""
        print("\n--- Navigate to Coordinates (A*) ---")
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

    def handle_rrt_navigation(self):
        """Handle RRT* navigation to specific coordinates (point-by-point)."""
        print("\n--- Navigate via RRT* (point-by-point) ---")
        try:
            x = float(input("Enter target X (meters): "))
            y = float(input("Enter target Y (meters): "))
            z = float(input("Enter target Z (meters) [default=1.5]: ") or "1.5")
            z = float(z)
        except ValueError:
            print("Invalid input. Enter numeric values.")
            return

        print(f"\nSending RRT* navigation request to ({x:.2f}, {y:.2f}, {z:.2f}) ...")
        success, message = self.call_rrt_service(x, y, z)
        self.print_result(success, message)

    def handle_rrt_smooth_navigation(self):
        """Handle RRT* smooth navigation to specific coordinates."""
        print("\n--- Navigate via RRT* (smooth trajectory) ---")
        print("Uses cubic spline + Pure Pursuit for smooth, continuous flight.")
        try:
            x = float(input("Enter target X (meters): "))
            y = float(input("Enter target Y (meters): "))
            z = float(input("Enter target Z (meters) [default=1.5]: ") or "1.5")
            z = float(z)
        except ValueError:
            print("Invalid input. Enter numeric values.")
            return

        print(f"\nSending smooth RRT* navigation request to ({x:.2f}, {y:.2f}, {z:.2f}) ...")
        success, message = self.call_rrt_smooth_service(x, y, z)
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

        confirm = input("Proceed? (y/n) [default=y]: ").strip().lower() or "y"
        if confirm != "y":
            print("Cancelled.")
            return

        try:
            z = float(input("Enter cruise altitude (meters) [default=1.5]: ") or "1.5")
        except ValueError:
            z = 1.5

        print(f"\nStarting frontier exploration (altitude={z:.2f}m) ...")
        success, message = self.call_exploration_service(z)
        self.print_result(success, message)

    def handle_turn_right(self):
        """Handle turn right request."""
        print("\n--- Turn Right (90° clockwise) ---")
        success, message = self.call_turn_service(self.turn_right_client)
        self.print_result(success, message)

    def handle_turn_left(self):
        """Handle turn left request."""
        print("\n--- Turn Left (90° counter-clockwise) ---")
        success, message = self.call_turn_service(self.turn_left_client)
        self.print_result(success, message)

    def handle_move_forward(self):
        """Handle move forward request."""
        print("\n--- Move Forward (1 meter) ---")
        success, message = self.call_simple_service(self.move_forward_client)
        self.print_result(success, message)

    # ---------------------------------------------------------------------

    def call_navigation_service(self, x: float, y: float, z: float):
        """Send a navigation request."""
        req = NavigateToPose.Request()
        req.x = x
        req.y = y
        req.z = z

        future = self.nav_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return False, "Service call failed or interrupted."
        return future.result().success, future.result().message

    def call_rrt_service(self, x: float, y: float, z: float):
        """Send an RRT* navigation request (point-by-point)."""
        req = NavigateToPose.Request()
        req.x = x
        req.y = y
        req.z = z

        future = self.rrt_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return False, "Service call failed or interrupted."
        return future.result().success, future.result().message

    def call_rrt_smooth_service(self, x: float, y: float, z: float):
        """Send an RRT* smooth navigation request."""
        req = NavigateToPose.Request()
        req.x = x
        req.y = y
        req.z = z

        future = self.rrt_smooth_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return False, "Service call failed or interrupted."
        return future.result().success, future.result().message

    def call_doorway_service(self, z: float = 1.5):
        """Send a doorway traversal request."""
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
        """Send a frontier exploration request."""
        req = NavigateToPose.Request()
        req.x = 0.0
        req.y = 0.0
        req.z = z

        future = self.explore_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return False, "Service call failed or interrupted."
        return future.result().success, future.result().message

    def call_turn_service(self, client):
        """Send a turn request."""
        req = NavigateToPose.Request()
        req.x = 0.0
        req.y = 0.0
        req.z = 0.0

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return False, "Service call failed or interrupted."
        return future.result().success, future.result().message

    def call_simple_service(self, client):
        """Send a simple service request (no parameters)."""
        req = NavigateToPose.Request()
        req.x = 0.0
        req.y = 0.0
        req.z = 0.0

        future = client.call_async(req)
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