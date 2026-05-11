#!/bin/bash
# ============================================================
# falcon_docker/run_hospital.sh — FALCON + external Gazebo drone
#
# v17: adds mounts for the real-drone path:
#        - pose_adapter.py
#        - real_drone.launch
#      so `roslaunch falcon_adapter real_drone.launch ...` works
#      from inside the container without rebuilding the image.
#      All v16 mounts retained.
# ============================================================

IMAGE="falcon-ros:noetic"
CONTAINER="falcon"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ENV_NAME="${1:-hospital}"
if [[ $# -ge 1 ]]; then shift; fi

if [[ ! -f "${SCRIPT_DIR}/${ENV_NAME}.yaml" ]]; then
  echo "[ERROR] Map config not found: ${SCRIPT_DIR}/${ENV_NAME}.yaml"
  echo "        Available configs:"
  ls -1 "${SCRIPT_DIR}"/*.yaml 2>/dev/null | xargs -n1 basename || true
  exit 1
fi
echo "[INFO] FALCON env: ${ENV_NAME}  (config: ${SCRIPT_DIR}/${ENV_NAME}.yaml)"

# Auto-chmod +x on host so we don't lose 10 min wondering why nodes
# aren't found. Harmless if they were already executable.
chmod +x "${SCRIPT_DIR}"/adapter/scripts/*.py 2>/dev/null || true

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
    --volume "${SCRIPT_DIR}/adapter/scripts/respawn_drone.py:/catkin_ws/src/falcon_adapter/scripts/respawn_drone.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/sensor_gate.py:/catkin_ws/src/falcon_adapter/scripts/sensor_gate.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/astar_planner.py:/catkin_ws/src/falcon_adapter/scripts/astar_planner.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/waypoint_follower.py:/catkin_ws/src/falcon_adapter/scripts/waypoint_follower.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/voxel_reset_watcher.py:/catkin_ws/src/falcon_adapter/scripts/voxel_reset_watcher.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/bev_click_goal.py:/catkin_ws/src/falcon_adapter/scripts/bev_click_goal.py" \
    --volume "${SCRIPT_DIR}/adapter/scripts/pose_adapter.py:/catkin_ws/src/falcon_adapter/scripts/pose_adapter.py" \
    --volume /var/run/docker.sock:/var/run/docker.sock \
    --volume "${SCRIPT_DIR}/adapter/launch/gazebo_exploration.launch:/catkin_ws/src/falcon_adapter/launch/gazebo_exploration.launch" \
    --volume "${SCRIPT_DIR}/adapter/launch/gazebo_waypoint_nav.launch:/catkin_ws/src/falcon_adapter/launch/gazebo_waypoint_nav.launch" \
    --volume "${SCRIPT_DIR}/adapter/launch/real_drone.launch:/catkin_ws/src/falcon_adapter/launch/real_drone.launch" \
    --volume "${SCRIPT_DIR}/${ENV_NAME}.yaml:/catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/map/${ENV_NAME}.yaml" \
    --volume "${SCRIPT_DIR}/runs:/home/falcon/runs" \
    --network host \
    "${IMAGE}" \
    "${@:-bash}"