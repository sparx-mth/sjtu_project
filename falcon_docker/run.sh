#!/bin/bash
# ============================================================
# run.sh — launch the FALCON container (CPU-only) with X11
# Usage:
#   ./run.sh              → interactive shell
#   ./run.sh roslaunch …  → run a command directly
# ============================================================

IMAGE="falcon-ros:noetic"
CONTAINER="falcon"

# Allow the container to connect to the host's X server
xhost +local:docker 2>/dev/null || true

# NOTE: --gpus removed — Gazebo uses the GPU, FALCON runs on CPU
docker run -it --rm \
    --name "${CONTAINER}" \
    --env DISPLAY="${DISPLAY}" \
    --env QT_X11_NO_MITSHM=1 \
    --env CUDA_VISIBLE_DEVICES="" \
    --env NVIDIA_VISIBLE_DEVICES="void" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --network host \
    "${IMAGE}" \
    "${@:-bash}"