#!/usr/bin/env python3
"""
Launch file for the autonomous drone system.
Starts:
    - NavigationAgentService
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

        # Optional: Run the Meta-Agent
        # (uncomment to enable user-driven missions)
        Node(
            package='autonomous_system',
            executable='meta_agent',
            name='meta_agent',
            output='screen'
        ),
    ])
