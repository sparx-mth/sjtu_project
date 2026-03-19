#!/bin/bash
# ============================================================
# run_octa_maze.sh — FALCON built-in simulator (GPU required)
#
# Uses FALCON's own map_render + poscmd_2_odom pipeline.
# No external Gazebo — the GPU renders depth from STL meshes.
#
# Usage:
#   ./run_octa_maze.sh              → interactive shell
#   ./run_octa_maze.sh roslaunch …  → run a command directly
# ============================================================

IMAGE="falcon-ros:noetic"
CONTAINER="falcon-gpu"

xhost +local:docker 2>/dev/null || true

docker run -it --rm \
    --name "${CONTAINER}" \
    --gpus all \
    --env DISPLAY="${DISPLAY}" \
    --env QT_X11_NO_MITSHM=1 \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --network host \
    "${IMAGE}" \
    "${@:-bash}"