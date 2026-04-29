#!/bin/bash
# ============================================================
# falcon_docker/run_hospital.sh — FALCON + external Gazebo drone
#
# v13: CPU-only build, but Gazebo and RViz still get GPU-accelerated
#      OpenGL rendering via the NVIDIA Container Toolkit.
#
# Key change vs v12:
#   * NVIDIA_DRIVER_CAPABILITIES=all (was unset). Without this, the
#     toolkit only exposes 'compute,utility', so OpenGL/EGL libs
#     are NOT mounted into the container and Gazebo silently falls
#     back to llvmpipe (software rendering). With 'all' (or
#     'graphics,display,compute,utility'), Gazebo and RViz use the
#     real GPU.
#   * --shm-size=2g for Gazebo / DDS shared-memory transport.
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
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --env NVIDIA_VISIBLE_DEVICES=all \
    --shm-size=2g \
    --ulimit nofile=65536:65536 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "${SCRIPT_DIR}/adapter/scripts/falcon_adapter.py:/catkin_ws/src/falcon_adapter/scripts/falcon_adapter.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/cmd_to_vel.py:/catkin_ws/src/falcon_adapter/scripts/cmd_to_vel.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/bev_publisher.py:/catkin_ws/src/falcon_adapter/scripts/bev_publisher.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/exploration_monitor.py:/catkin_ws/src/falcon_adapter/scripts/exploration_monitor.py" \
    --volume "${SCRIPT_DIR}/adapter/launch/gazebo_exploration.launch:/catkin_ws/src/falcon_adapter/launch/gazebo_exploration.launch" \
    --volume "${SCRIPT_DIR}/hospital.yaml:/catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/map/hospital.yaml" \
    --network host \
    "${IMAGE}" \
    "${@:-bash}"