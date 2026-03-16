#!/bin/bash
IMAGE="ros1_bridge:noetic-foxy"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[INFO] Image '${IMAGE}' not found — building (takes ~10 min first time)..."
    docker build -t "${IMAGE}" "${SCRIPT_DIR}"
fi

# ROS_DOMAIN_ID must match your sim container (your run.sh uses 20)
docker run -it --rm \
    --net=host \
    --name="ros1_bridge" \
    -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
    -e ROS_IP="${ROS_IP:-127.0.0.1}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}" \
    -e RMW_IMPLEMENTATION="rmw_fastrtps_cpp" \
    -e BRIDGE_MODE="${BRIDGE_MODE:-dynamic}" \
    "${IMAGE}"