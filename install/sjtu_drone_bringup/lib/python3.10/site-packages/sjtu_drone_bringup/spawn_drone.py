#!/usr/bin/env python3
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_drone')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    content = sys.argv[1]
    namespace = sys.argv[2]

    # SAFER INITIAL POSITION - Inside the hospital
    initial_pose = Pose()
    initial_pose.position.x = 1.0    # Center of the world
    initial_pose.position.y = 1.0    # Center of the world
    initial_pose.position.z = 2.0    # 1 meter above ground

    # Orientation (quaternion) - default is no rotation
    initial_pose.orientation.x = 0.0
    initial_pose.orientation.y = 0.0
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 1.0

    req = SpawnEntity.Request()
    req.name = namespace
    req.xml = content
    req.robot_namespace = namespace
    req.reference_frame = "world"
    req.initial_pose = initial_pose

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            f'Spawned at ({initial_pose.position.x}, {initial_pose.position.y}, {initial_pose.position.z}) - ' +
            f'Result: {future.result().success} - {future.result().status_message}')
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()