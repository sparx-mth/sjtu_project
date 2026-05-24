cmake_minimum_required(VERSION 3.0.2)
project(falcon_adapter)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs sensor_msgs std_msgs tf quadrotor_msgs)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/falcon_adapter.py
  scripts/falcon_playback.py
  scripts/cmd_to_vel.py
  scripts/bev_publisher.py
  scripts/sensor_gate.py
  scripts/astar_planner.py
  scripts/waypoint_follower.py
  scripts/voxel_reset_watcher.py
  scripts/pose_adapter.py
  scripts/sim_adapter.py
  scripts/visual_servoing_controller.py
  scripts/nav_geom.py
  scripts/navdp_client.py
  scripts/navdp_click.py
  scripts/pixel_goal_tracker.py
  scripts/trajectory_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)