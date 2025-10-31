#!/bin/bash

usage(){
    echo "Usage: $0 [-r <humble|iron|rolling>]"
    exit 1
}

ROS_DISTRO=${ROS_DISTRO:-"humble"}  # default is humble
while getopts "r:" opt; do
    case $opt in
        r)
            if [ $OPTARG != "humble" ] && [ $OPTARG != "iron" ] && [ $OPTARG != "rolling" ]; then
                echo "Invalid ROS distro: $OPTARG" >&2
                usage
            fi
            ROS_DISTRO=$OPTARG
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            ;;
    esac
done

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

# Use the local image we built instead of pulling from DockerHub
IMAGE_NAME="sjtu_drone:${ROS_DISTRO}"

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
    --name="sjtu_drone" \
    ${IMAGE_NAME}
xhost -local:docker
