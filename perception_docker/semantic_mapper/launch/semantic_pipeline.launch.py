"""
semantic_pipeline.launch.py — FALCON BEV → scene graph.

Launches the ROS 2 perception node (and optionally RViz). The node
subscribes to nav_msgs/OccupancyGrid published by the FALCON-side
`bev_publisher` at /falcon/bev_2d. All geometry (bbox, resolution,
origin) comes from the grid header itself, so it's not configurable
here — tune it on the FALCON side, in gazebo_exploration.launch.

Override any knob on the command line, e.g.:
    ros2 launch semantic_mapper semantic_pipeline.launch.py \\
        door_cut_m:=0.8 start_rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('bev_topic',
            default_value='/falcon/bev_2d'),
        DeclareLaunchArgument('world_frame', default_value='world'),

        # Door cutting / linking / discovery radii (all metres).
        DeclareLaunchArgument('door_cut_m',          default_value='0.60'),
        DeclareLaunchArgument('door_match_radius_m', default_value='0.90'),
        DeclareLaunchArgument('door_discover_m',     default_value='0.30'),
        DeclareLaunchArgument('min_room_cells',      default_value='40'),
        DeclareLaunchArgument('room_iou_threshold',  default_value='0.15'),
        DeclareLaunchArgument('tick_rate',           default_value='2.0'),

        DeclareLaunchArgument('start_rviz', default_value='false'),
    ]

    mapper = Node(
        package='semantic_mapper',
        executable='semantic_mapper_node',
        name='semantic_mapper',
        output='screen',
        parameters=[{
            k: LaunchConfiguration(k) for k in (
                'bev_topic', 'world_frame',
                'door_cut_m', 'door_match_radius_m', 'door_discover_m',
                'min_room_cells', 'room_iou_threshold', 'tick_rate',
            )
        }],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('semantic_mapper'),
            'rviz', 'semantic_mapper.rviz'])],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        output='screen',
    )

    return LaunchDescription(args + [mapper, rviz])