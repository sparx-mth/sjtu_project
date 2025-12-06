#!/usr/bin/env python3
"""Turn Right Agent - Rotates drone 90° clockwise. Service: /turn_right"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from autonomous_system.srv import NavigateToPose
from autonomous_system.control.yaw_controller import YawController


class TurnRightAgent(Node):
    def __init__(self):
        super().__init__("turn_right_agent")
        self.yaw_ctrl = YawController("turn_right_yaw")
        self.srv = self.create_service(
            NavigateToPose, "/turn_right", self.handle_request,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("TurnRightAgent ready (90° clockwise)")

    def handle_request(self, request, response):
        self.get_logger().info("Executing 90° right turn...")
        response.success = self.yaw_ctrl.rotate(90)
        response.message = "Turn right complete" if response.success else "Turn failed"
        return response


def main():
    rclpy.init()
    node = TurnRightAgent()
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