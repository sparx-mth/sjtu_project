#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
source /bridge_ws/install/setup.bash

# ROS1 side — needs to find roscore (on host or in another container)
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

# ROS2 side — must match your sim container's domain
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "════════════════════════════════════════"
echo "  ros1_bridge (Noetic <-> Foxy)"
echo "  ROS1 master : ${ROS_MASTER_URI}"
echo "  ROS2 domain : ${ROS_DOMAIN_ID}"
echo "  Bridge mode : ${BRIDGE_MODE:-dynamic}"
echo "════════════════════════════════════════"

# BRIDGE_MODE=dynamic  → bridges any topic both sides are active on (default, easiest)
# BRIDGE_MODE=static   → bridges only topics listed in /bridge_ws/bridge.yaml (more control)
if [[ "${BRIDGE_MODE:-dynamic}" == "static" && -f /bridge_ws/bridge.yaml ]]; then
    echo "[INFO] Using static parameter_bridge with /bridge_ws/bridge.yaml"
    exec ros2 run ros1_bridge parameter_bridge \
        --ros-args --params-file /bridge_ws/bridge.yaml
else
    echo "[INFO] Using dynamic_bridge — bridging all matched topics"
    exec ros2 run ros1_bridge dynamic_bridge \
        --bridge-all-2to1-topics \
        --bridge-all-1to2-topics
fi