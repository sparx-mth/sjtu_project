#!/bin/bash
# FALCON Docker Entrypoint
# Sources ROS Noetic + the catkin workspace on every container launch.

set -e

source /opt/ros/noetic/setup.bash

if [ -f /catkin_ws/devel/setup.bash ]; then
    source /catkin_ws/devel/setup.bash
fi

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

exec "$@"