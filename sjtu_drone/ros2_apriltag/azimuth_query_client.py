#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from builtin_interfaces.msg import Time as TimeMsg
from your_pkg_name.srv import GetAzimuthAtTime


class AzimuthQueryClient(Node):
    def __init__(self):
        super().__init__("azimuth_query_client")

        # Create service client
        self.client = self.create_client(GetAzimuthAtTime, "get_azimuth_at_time")

        # Wait for service
        self.get_logger().info("Waiting for service 'get_azimuth_at_time'...")
        self.client.wait_for_service()

        # Example: use the current time as the timestamp
        req_time = self.get_clock().now()

        # Convert to ROS message
        stamp_msg = TimeMsg()
        stamp_msg.sec = req_time.seconds_nanoseconds()[0]
        stamp_msg.nanosec = req_time.seconds_nanoseconds()[1]

        # Build service request
        request = GetAzimuthAtTime.Request()
        request.stamp = stamp_msg

        # Call service async
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            rclpy.shutdown()
            return

        if not response.success:
            self.get_logger().warn(
                f"No valid azimuth found. Closest time diff = {response.time_diff:.3f}s"
            )
        else:
            self.get_logger().info(
                f"Azimuth at requested time: {response.azimuth:.2f}° "
                f"(dt = {response.time_diff:.3f}s)"
            )

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AzimuthQueryClient()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
