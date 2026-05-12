#!/bin/bash
set -e

# ============================================================
# data_publisher_docker/run_publisher.sh
#
# Replays recorded pose + depth on ROS2 topics so the bridge can
# forward them to the ROS1 FALCON stack. Stand-in for a live drone
# during testing.
#
# Domain & RMW are matched to run_bridge.sh (domain 5, FastDDS).
# ============================================================

IMAGE="data_publisher:jazzy"
CONTAINER="data_publisher"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Host data directory. The Python script expects:
#   /data/xtend_rectified_depth_take_*/depth_npy/*.npy
#   /data/estimated_trajectory_xtend_rectified_depth_take_*.json
# Override with:  DATA_DIR=/path/to/recordings ./run_publisher.sh
DATA_DIR="${DATA_DIR:-${HOME}/Desktop}"

# Build image if missing
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[INFO] Image '${IMAGE}' not found — building..."
    docker build -t "${IMAGE}" "${SCRIPT_DIR}"
fi

docker rm -f "${CONTAINER}" 2>/dev/null || true

echo "================================================"
echo "  Launching ${CONTAINER}"
echo "  Domain : 5"
echo "  RMW    : rmw_fastrtps_cpp"
echo "  Data   : ${DATA_DIR}  ->  /data"
echo "================================================"

docker run -it --rm \
    --net=host \
    --name="${CONTAINER}" \
    -e ROS_DOMAIN_ID=5 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -v "${DATA_DIR}:/data:ro" \
    "${IMAGE}"