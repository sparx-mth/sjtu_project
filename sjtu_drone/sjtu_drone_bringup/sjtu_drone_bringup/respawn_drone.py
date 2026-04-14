#!/usr/bin/env python3
import sys
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose


def main():
    if len(sys.argv) < 6:
        print("Usage: respawn_drone.py <urdf_file> <name> <namespace> <x> <y> [z]")
        sys.exit(1)

    urdf_file = sys.argv[1]
    name = sys.argv[2]
    namespace = sys.argv[3]
    x = float(sys.argv[4])
    y = float(sys.argv[5])
    z = float(sys.argv[6]) if len(sys.argv) > 6 else 2.0

    with open(urdf_file, "r", encoding="utf-8") as f:
        xml = f.read()

    rclpy.init()
    node = rclpy.create_node("respawn_drone_helper")

    delete_cli = node.create_client(DeleteEntity, "/delete_entity")
    spawn_cli = node.create_client(SpawnEntity, "/spawn_entity")

    while not delete_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("waiting for /delete_entity...")
    while not spawn_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("waiting for /spawn_entity...")

    delete_req = DeleteEntity.Request()
    delete_req.name = name
    delete_future = delete_cli.call_async(delete_req)
    rclpy.spin_until_future_complete(node, delete_future)

    if delete_future.result() is not None:
        node.get_logger().info(
            f"Delete result: {delete_future.result().success} - {delete_future.result().status_message}"
        )
    else:
        node.get_logger().error(f"Delete failed: {delete_future.exception()}")

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0

    spawn_req = SpawnEntity.Request()
    spawn_req.name = name
    spawn_req.xml = xml
    spawn_req.robot_namespace = namespace
    spawn_req.initial_pose = pose
    spawn_req.reference_frame = "world"

    spawn_future = spawn_cli.call_async(spawn_req)
    rclpy.spin_until_future_complete(node, spawn_future)

    if spawn_future.result() is not None:
        node.get_logger().info(
            f"Spawn result: {spawn_future.result().success} - {spawn_future.result().status_message}"
        )
    else:
        node.get_logger().error(f"Spawn failed: {spawn_future.exception()}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()