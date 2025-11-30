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

        # Map visualization with exploration (fog of war)
        # Use 'show_drone_map' for original, 'show_drone_map_exploration' for fog of war
        Node(
            package='autonomous_system',
            executable='show_drone_map_exploration',
            name='show_drone_map_exploration',
            output='screen',
            parameters=[{
                'exploration_radius': 300,  # pixels
                # 'exploration_radius_meters': 3.0,  # alternative: specify in meters
            }]
        ),
    ])