#!/bin/bash
# ============================================================
# falcon_docker/run_hospital.sh — FALCON + external Gazebo drone
#
# Equivalent to the "Terminal 3" docker run command in README.md.
# The GPU stays free for Gazebo on the host.
# FALCON receives depth images via ros1_bridge.
#
# Usage:
#   ./run_hospital.sh              → interactive shell
#   ./run_hospital.sh roslaunch …  → run a command directly
# ============================================================

IMAGE="falcon-ros:noetic"
CONTAINER="falcon"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

xhost +local:docker 2>/dev/null || true

docker run -it --rm \
    --name "${CONTAINER}" \
    --gpus all \
    --env DISPLAY="${DISPLAY}" \
    --env QT_X11_NO_MITSHM=1 \
    --env CUDA_VISIBLE_DEVICES="" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "${SCRIPT_DIR}/adapter/scripts/falcon_adapter.py:/catkin_ws/src/falcon_adapter/scripts/falcon_adapter.py" \
    --volume "${SCRIPT_DIR}/adapter/launch/gazebo_exploration.launch:/catkin_ws/src/falcon_adapter/launch/gazebo_exploration.launch" \
    --volume "${SCRIPT_DIR}/hospital.yaml:/catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/map/hospital.yaml" \
    --network host \
    "${IMAGE}" \
    "${@:-bash}"