#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
source /bridge_ws/install/setup.bash

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ── DDS selection ──────────────────────────────────────────────
# Default to CycloneDDS to match ROS2 Humble's default.
# Override with RMW_IMPLEMENTATION env var if your sim uses FastRTPS.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# If using FastRTPS, restrict discovery to localhost
if [[ "${RMW_IMPLEMENTATION}" == "rmw_fastrtps_cpp" && -f /fastdds_localhost.xml ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_localhost.xml
fi

# If using CycloneDDS, restrict discovery to localhost
if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" && -f /cyclonedds_localhost.xml ]]; then
    export CYCLONEDDS_URI=file:///cyclonedds_localhost.xml
fi

echo "════════════════════════════════════════"
echo "  ros1_bridge (Noetic <-> Foxy)"
echo "  ROS1 master : ${ROS_MASTER_URI}"
echo "  ROS2 domain : ${ROS_DOMAIN_ID}"
echo "  RMW         : ${RMW_IMPLEMENTATION}"
echo "  Bridge mode : ${BRIDGE_MODE:-dynamic}"
echo "════════════════════════════════════════"

if [[ "${BRIDGE_MODE:-dynamic}" == "static" && -f /bridge_ws/bridge.yaml ]]; then
    exec ros2 run ros1_bridge parameter_bridge \
        --ros-args --params-file /bridge_ws/bridge.yaml
else
    exec ros2 run ros1_bridge dynamic_bridge \
        --bridge-all-2to1-topics \
        --bridge-all-1to2-topics
fi