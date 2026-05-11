#!/bin/bash
set -e

IMAGE="ros1_bridge:noetic-foxy"
CONTAINER="ros1_bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Build image if missing
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[INFO] Image '${IMAGE}' not found - building..."
    docker build -t "${IMAGE}" "${SCRIPT_DIR}"
fi

# Prepare host log file
LOGFILE_HOST="${SCRIPT_DIR}/bridge.log"
touch "${LOGFILE_HOST}"
chmod 666 "${LOGFILE_HOST}"

docker rm -f "${CONTAINER}" 2>/dev/null || true

echo "================================================"
echo "  Launching ${CONTAINER}"
echo "  Domain : ${ROS_DOMAIN_ID:-5}"
echo "  Log    : ${LOGFILE_HOST}"
echo "================================================"

docker run -it --rm \
    --net=host \
    --privileged \
    --name="${CONTAINER}" \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e ROS_DOMAIN_ID="5" \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e LOGFILE="/tmp/bridge.log" \
    -v "${SCRIPT_DIR}/entrypoint.sh:/entrypoint.sh:ro" \
    -v "${LOGFILE_HOST}:/tmp/bridge.log:rw" \
    --entrypoint /entrypoint.sh \
    "${IMAGE}"