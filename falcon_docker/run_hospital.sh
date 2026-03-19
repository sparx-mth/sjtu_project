#!/bin/bash
# ============================================================
# run_hospital.sh — FALCON + external Gazebo drone (CPU-only)
#
# The GPU stays free for Gazebo on the host.
# FALCON receives depth images via ros1_bridge.
#
# Usage:
#   ./run_hospital.sh              → interactive shell
#   ./run_hospital.sh roslaunch …  → run a command directly
# ============================================================

IMAGE="falcon-ros:noetic"
CONTAINER="falcon"

xhost +local:docker 2>/dev/null || true

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