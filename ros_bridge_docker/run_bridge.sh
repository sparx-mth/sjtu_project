#!/bin/bash
# ============================================================
# ros_bridge_docker/run_bridge.sh
#
# Launch ros1_bridge in lazy-dynamic mode.
# Prereq: roscore must already be running in its own container.
# ============================================================
set -eo pipefail

IMAGE="ros1_bridge:noetic-foxy"
CONTAINER="ros1_bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Bump kernel UDP receive buffer for sustained depth streams.
# Needs sudo. Silent if unavailable — CycloneDDS will log a
# warning about the requested buffer being capped.
if command -v sudo >/dev/null 2>&1; then
    sudo sysctl -w net.core.rmem_max=26214400    >/dev/null 2>&1 || true
    sudo sysctl -w net.core.rmem_default=26214400 >/dev/null 2>&1 || true
fi

# ── Build image if missing ────────────────────────────────────
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[INFO] Image '${IMAGE}' not found — building (~10 min first time)..."
    docker build -t "${IMAGE}" "${SCRIPT_DIR}"
fi

# ── Host-side log file so we can see crashes even with --rm ──
LOGFILE_HOST="${SCRIPT_DIR}/bridge.log"
: > "${LOGFILE_HOST}"
chmod 666 "${LOGFILE_HOST}" 2>/dev/null || true

docker rm -f "${CONTAINER}" 2>/dev/null || true

echo "════════════════════════════════════════════════"
echo "  Launching ${CONTAINER}"
echo "  Domain : ${ROS_DOMAIN_ID:-20}"
echo "  RMW    : ${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
echo "  Log    : ${LOGFILE_HOST}"
echo "════════════════════════════════════════════════"

docker run -it --rm \
    --net=host \
    --name="${CONTAINER}" \
    -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    -e LOGFILE="/tmp/bridge.log" \
    -v "${SCRIPT_DIR}/entrypoint.sh:/entrypoint.sh:ro" \
    -v "${LOGFILE_HOST}:/tmp/bridge.log:rw" \
    --entrypoint /entrypoint.sh \
    "${IMAGE}"