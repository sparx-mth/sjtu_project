#!/bin/bash
# ============================================================
# falcon_docker/run_hospital.sh — FALCON + external Gazebo drone
#
# v14: adds completion_watcher.py + batch_runner.py mounts so you
#      can run a batch of N successful experiments back-to-back:
#
#        ./run_hospital.sh hospital                      # interactive
#        # then inside the container:
#        python3 /catkin_ws/src/falcon_adapter/scripts/batch_runner.py hospital 10 300
#
# v13 (preserved): NVIDIA_DRIVER_CAPABILITIES=all so Gazebo/RViz get
# real GPU OpenGL (otherwise toolkit only mounts compute libs and
# Gazebo silently falls back to llvmpipe). --shm-size=2g for
# Gazebo / DDS shared memory.
# ============================================================

IMAGE="falcon-ros:noetic"
CONTAINER="falcon"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# -----------------------------
# Environment name (defaults to hospital). Selects which <env>.yaml to mount
# into FALCON's exploration_manager config dir.
# Usage: ./run_hospital.sh [env_name] [extra docker CMD ...]
#   ./run_hospital.sh                 -> mounts hospital.yaml
#   ./run_hospital.sh playground      -> mounts playground.yaml
# -----------------------------
ENV_NAME="${1:-hospital}"
if [[ $# -ge 1 ]]; then shift; fi

if [[ ! -f "${SCRIPT_DIR}/${ENV_NAME}.yaml" ]]; then
  echo "[ERROR] Map config not found: ${SCRIPT_DIR}/${ENV_NAME}.yaml"
  echo "        Available configs:"
  ls -1 "${SCRIPT_DIR}"/*.yaml 2>/dev/null | xargs -n1 basename || true
  exit 1
fi
echo "[INFO] FALCON env: ${ENV_NAME}  (config: ${SCRIPT_DIR}/${ENV_NAME}.yaml)"

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
    --volume "${SCRIPT_DIR}/adapter/scripts/run_recorder.py:/catkin_ws/src/falcon_adapter/scripts/run_recorder.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/completion_watcher.py:/catkin_ws/src/falcon_adapter/scripts/completion_watcher.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/batch_runner.py:/catkin_ws/src/falcon_adapter/scripts/batch_runner.py" \
    --volume "${SCRIPT_DIR}/adapter/launch/gazebo_exploration.launch:/catkin_ws/src/falcon_adapter/launch/gazebo_exploration.launch" \
    --volume "${SCRIPT_DIR}/${ENV_NAME}.yaml:/catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/map/${ENV_NAME}.yaml" \
    --volume "${SCRIPT_DIR}/runs:/home/falcon/runs" \
    --network host \
    "${IMAGE}" \
    "${@:-bash}"