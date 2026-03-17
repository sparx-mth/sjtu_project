cmake_minimum_required(VERSION 3.0.2)
project(falcon_adapter)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs sensor_msgs std_msgs tf)
catkin_package()
catkin_install_python(PROGRAMS scripts/falcon_adapter.py
                     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
