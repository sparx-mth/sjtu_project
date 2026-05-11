#!/bin/bash
set -e

# Base settings
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"
LOGFILE="${LOGFILE:-/tmp/bridge.log}"

# Redirect output to log file
: > "${LOGFILE}" 2>/dev/null || true
exec > >(tee -a "${LOGFILE}") 2> >(tee -a "${LOGFILE}" >&2)

# Source ROS1
source /opt/ros/noetic/setup.bash

echo "================================================"
echo "  ros1_bridge - Simple Dynamic Bridge"
echo "  ROS1 master : ${ROS_MASTER_URI}"
echo "  ROS2 domain : ${ROS_DOMAIN_ID}"
echo "  Log file    : ${LOGFILE}"
echo "================================================"

# Wait for roscore
echo "[bridge] Waiting for roscore..."
until timeout 2 rostopic list >/dev/null 2>&1; do
    sleep 1
done
echo "[bridge] roscore reachable."

# Source ROS2 and bridge workspace
source /opt/ros/foxy/setup.bash 2>/dev/null
source /bridge_ws/install/setup.bash 2>/dev/null

# Run bridge in a loop
while true; do
    echo "[bridge] Starting dynamic_bridge --bridge-all-topics..."
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
    echo "[bridge] bridge exited. Restarting in 3s..."
    sleep 3
done