#!/usr/bin/env python3
"""Move Forward Agent - Moves drone forward exactly 1 meter. Service: /move_forward"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from autonomous_system.srv import NavigateToPose
from autonomous_system.control.forward_controller import ForwardController


class MoveForwardAgent(Node):
    def __init__(self):
        super().__init__("move_forward_agent")
        self.fwd_ctrl = ForwardController("move_forward_ctrl")
        self.srv = self.create_service(
            NavigateToPose, "/move_forward", self.handle_request,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("MoveForwardAgent ready (1m forward)")

    def handle_request(self, request, response):
        self.get_logger().info("Executing 1m forward move...")
        response.success = self.fwd_ctrl.move(1.0)
        response.message = "Move forward complete" if response.success else "Move failed"
        return response


def main():
    rclpy.init()
    node = MoveForwardAgent()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(node.fwd_ctrl)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()