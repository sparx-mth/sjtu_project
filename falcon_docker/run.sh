#!/bin/bash
# ============================================================
# run.sh — launch the FALCON container with GPU + X11 display
# Usage:
#   ./run.sh              → interactive shell
#   ./run.sh roslaunch …  → run a command directly
# ============================================================

IMAGE="falcon-ros:noetic"
CONTAINER="falcon"

# Allow the container to connect to the host's X server
xhost +local:docker 2>/dev/null || true

docker run -it --rm \
    --name "${CONTAINER}" \
    --gpus all \
    --env DISPLAY="${DISPLAY}" \
    --env QT_X11_NO_MITSHM=1 \
    --env NVIDIA_VISIBLE_DEVICES=all \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --network host \
    "${IMAGE}" \
    "$@"