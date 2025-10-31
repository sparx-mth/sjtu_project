#!/bin/bash

ROS_DISTRO=humble
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
IMAGE_NAME="sjtu_drone:humble_ros2"

if [ -z "$1" ]; then
  echo "Usage: $0 <world_file>"
  echo "Example: $0 office_cpr.world"
  exit 1
fi

WORLD_NAME=$1
WORLD_PATH="/root/drone_workspace/gazebo_models_worlds_collection/worlds/${WORLD_NAME}"

xhost +local:docker
docker run \
    -it --rm \
    --gpus all \
    -v ${XSOCK}:${XSOCK} \
    -v ${XAUTH}:${XAUTH} \
    -e DISPLAY=${DISPLAY} \
    -e XAUTHORITY=${XAUTH} \
    --env=QT_X11_NO_MITSHM=1 \
    --privileged \
    --net=host \
    -v $HOME/drone_workspace:/root/drone_workspace:rw \
    --name="sjtu_drone_${WORLD_NAME%.*}" \
    ${IMAGE_NAME} \
    bash -c "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/root/drone_workspace/gazebo_models_worlds_collection/models && \
             export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:\$GAZEBO_RESOURCE_PATH:/root/drone_workspace/gazebo_models_worlds_collection/worlds && \
             cd /root/drone_workspace && \
             colcon build && \
             source install/setup.bash && \
             ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py world:=${WORLD_PATH}"
xhost -local:docker
