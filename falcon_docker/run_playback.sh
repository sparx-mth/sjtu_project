#!/bin/bash
# ============================================================
# falcon_docker/run_playback.sh — FALCON in dataset playback mode.
#
# Feeds FALCON from a recorded JSON + folder of .npy depth frames
# instead of from Gazebo. No simulator, no closed-loop control —
# just the voxel mapper building a map from your real-drone data.
#
# Usage:
#   ./run_playback.sh <map_name> <dataset_dir>  [extra docker CMD ...]
#
#   <map_name>      hospital | playground | ... (must have <map_name>.yaml here)
#   <dataset_dir>   absolute path on host to the folder containing
#                       poses.json
#                       frames/             (frame_*.jpg + frame_*.npy)
#
# Example:
#   ./run_playback.sh playground ~/recordings/run_001
#   # then inside the container:
#   roslaunch falcon_adapter playback_exploration.launch \
#       poses_json:=/data/poses.json \
#       frames_dir:=/data/frames \
#       map_name:=playground \
#       fx:=455.0 fy:=455.0 cx:=320.0 cy:=240.0 \
#       image_width:=640 image_height:=480 \
#       depth_scale:=1.0
# ============================================================

IMAGE="falcon-ros:noetic"
CONTAINER="falcon"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ENV_NAME="${1:-hospital}"
DATASET_DIR="${2:-}"
if [[ $# -ge 1 ]]; then shift; fi
if [[ $# -ge 1 ]]; then shift; fi

if [[ -z "${DATASET_DIR}" || ! -d "${DATASET_DIR}" ]]; then
  echo "[ERROR] Dataset dir not found. Usage: $0 <map_name> <dataset_dir>"
  exit 1
fi
if [[ ! -f "${SCRIPT_DIR}/${ENV_NAME}.yaml" ]]; then
  echo "[ERROR] Map config not found: ${SCRIPT_DIR}/${ENV_NAME}.yaml"
  exit 1
fi
echo "[INFO] FALCON env:    ${ENV_NAME}"
echo "[INFO] Dataset (host): ${DATASET_DIR}    -> /data inside container"

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
    --volume "${SCRIPT_DIR}/adapter/scripts/falcon_playback.py:/catkin_ws/src/falcon_adapter/scripts/falcon_playback.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/run_recorder.py:/catkin_ws/src/falcon_adapter/scripts/run_recorder.py" \
    --volume "${SCRIPT_DIR}/adapter/launch/playback_exploration.launch:/catkin_ws/src/falcon_adapter/launch/playback_exploration.launch" \
    --volume "${SCRIPT_DIR}/${ENV_NAME}.yaml:/catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/map/${ENV_NAME}.yaml" \
    --volume "${DATASET_DIR}:/data:ro" \
    --volume "${SCRIPT_DIR}/runs:/home/falcon/runs" \
    --network host \
    "${IMAGE}" \
    "${@:-bash}"