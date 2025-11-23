#!/usr/bin/env python3
"""
Manual Navigation Node
----------------------
A simple interactive node that allows the user to enter
target coordinates manually. Uses the shared WaypointController.

Workflow:
    - User enters (x, y)
    - Node calls controller.goto(x, y)
"""

import rclpy
from autonomous_system.control.waypoint_controller import WaypointController


def main():
    """Run manual navigation loop."""
    rclpy.init()
    ctrl = WaypointController('manual_navigation')

    print("\nManual Navigator Ready.")
    print("Enter target coordinates (X, Y). Press Ctrl+C to exit.\n")

    while rclpy.ok():
        try:
            tx = float(input("Enter X: "))
            ty = float(input("Enter Y: "))
        except ValueError:
            print("Invalid input. Please enter numbers.")
            continue

        ctrl.goto(tx, ty)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
