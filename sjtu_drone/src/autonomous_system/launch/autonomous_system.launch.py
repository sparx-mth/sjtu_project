#!/usr/bin/env python3
"""
Launch file for the autonomous drone system.
Starts:
    - NavigationAgentService
    - DoorwayTraversalAgent
    - FrontierExplorationService
    - RoomExplorationAgent
    - MetaAgent
    - Map visualization
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

        # Frontier exploration agent (explores map using frontiers)
        Node(
            package='autonomous_system',
            executable='frontier_exploration_service',
            name='frontier_exploration_service',
            output='screen',
            parameters=[{
                'exploration_radius': 3.0,  # meters
                'exploration_timeout': 60.0,  # seconds
            }]
        ),

        # Room exploration agent (explores current room without crossing doors)
        Node(
            package='autonomous_system',
            executable='room_exploration_agent',
            name='room_exploration_agent',
            output='screen',
            parameters=[{
                'cruise_altitude': 1.5,
                'exploration_timeout': 120.0,
                'min_door_width': 0.6,
                'max_door_width': 2.0,
            }]
        ),

        # Meta-Agent (user-driven mission control)
        Node(
            package='autonomous_system',
            executable='meta_agent',
            name='meta_agent',
            output='screen'
        ),

        # Map visualization with exploration (fog of war)
        Node(
            package='autonomous_system',
            executable='show_drone_map_exploration',
            name='show_drone_map_exploration',
            output='screen',
            parameters=[{
                'exploration_radius': 300,  # pixels
            }]
        ),
    ])