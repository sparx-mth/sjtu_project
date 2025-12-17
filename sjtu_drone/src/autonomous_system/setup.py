from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'autonomous_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nadav',
    maintainer_email='you@example.com',
    description='Autonomous drone navigation agents using ROS2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'navigation_agent_service = autonomous_system.agents.navigation_agent_service_node:main',
            'doorway_traversal_agent = autonomous_system.agents.doorway_traversal_agent:main',
            'frontier_exploration_service = autonomous_system.agents.frontier_exploration_service_node:main',
            # 'room_exploration_agent = autonomous_system.agents.room_exploration_agent:main',
            'meta_agent = autonomous_system.agents.meta_agent_node:main',
            'turn_right_agent = autonomous_system.agents.turn_right_agent_node:main',
            'turn_left_agent = autonomous_system.agents.turn_left_agent_node:main',
            'move_forward_agent = autonomous_system.agents.move_forward_agent_node:main',
            'rrt_navigation_agent = autonomous_system.agents.rrt_navigation_agent:main',
            'show_drone_map = autonomous_system.mapping.show_drone_map:main',
            'show_drone_map_exploration = autonomous_system.mapping.show_drone_map_with_exploration:main',
            'route_map_viewer = autonomous_system.mapping.route_map_viewer:main',
        ],
    },
)