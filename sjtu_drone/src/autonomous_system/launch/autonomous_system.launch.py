#!/usr/bin/env python3
"""
Launch file for the autonomous drone system.
Starts:
    - NavigationAgentService
    - DoorwayTraversalAgent
    - Optionally: MetaAgent (can be enabled/disabled)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Navigation agent (service-based waypoint + A*)
        Node(
            package='autonomous_system',
            executable='navigation_agent_service',
            name='navigation_agent_service',
            output='screen'
        ),

        # Doorway traversal agent (finds and traverses nearest door)
        Node(
            package='autonomous_system',
            executable='doorway_traversal_agent',
            name='doorway_traversal_agent',
            output='screen'
        ),

        # Optional: Run the Meta-Agent
        # (uncomment to enable user-driven missions)
        Node(
            package='autonomous_system',
            executable='meta_agent',
            name='meta_agent',
            output='screen'
        ),

        # Map visualization tool
        Node(
            package='autonomous_system',
            executable='show_drone_map',
            name='show_drone_map',
            output='screen'
        ),
    ])