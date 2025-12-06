#!/usr/bin/env python3
"""Turn Left Agent - Rotates drone 90° counterclockwise. Service: /turn_left"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from autonomous_system.srv import NavigateToPose
from autonomous_system.control.yaw_controller import YawController


class TurnLeftAgent(Node):
    def __init__(self):
        super().__init__("turn_left_agent")
        self.yaw_ctrl = YawController("turn_left_yaw")
        self.srv = self.create_service(
            NavigateToPose, "/turn_left", self.handle_request,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("TurnLeftAgent ready (90° counterclockwise)")

    def handle_request(self, request, response):
        self.get_logger().info("Executing 90° left turn...")
        response.success = self.yaw_ctrl.rotate(-90)
        response.message = "Turn left complete" if response.success else "Turn failed"
        return response


def main():
    rclpy.init()
    node = TurnLeftAgent()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(node.yaw_ctrl)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()