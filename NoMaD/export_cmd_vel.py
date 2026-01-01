# export_cmd_vel.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import csv

class CmdVelExporter(Node):
    def __init__(self):
        super().__init__("cmd_vel_exporter")
        self.sub = self.create_subscription(
            Twist,
            "/simple_drone/cmd_vel",
            self.cb,
            10
        )

        self.file = open("cmd_vel.csv", "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["time", "v", "w"])

    def cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.writer.writerow([t, msg.linear.x, msg.angular.z])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = CmdVelExporter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
