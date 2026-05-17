#!/bin/bash
# ============================================================
# ros_bridge_docker/run_bridge_sim.sh
#
# Launches ros1_bridge configured for the Gazebo sjtu_drone sim.
# Sister script to run_bridge.sh (which is, and stays, the real
# drone path). Use whichever matches what you're actually running:
#
#   ./run_bridge.sh        # real drone   (unchanged)
#   ./run_bridge_sim.sh    # Gazebo sim
#
# Sim-specific differences vs the real-drone script:
#   - bridge_sim.yaml      (sim topics only)
#   - rmw_cyclonedds_cpp   (Gazebo sjtu_drone's default RMW)
#   - ROS_DOMAIN_ID=20     (legacy sim domain; override with env)
#   - sysctl rmem_max bumped to 25 MB so sustained depth streams
#     don't drop samples under CycloneDDS
#   - no FastDDS profile (only the real-drone setup needs it)
# ============================================================
set -e

IMAGE="ros1_bridge:noetic-foxy"
CONTAINER="ros1_bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Sim defaults — env vars still win for one-off overrides.
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
BRIDGE_YAML_HOST="${SCRIPT_DIR}/bridge_sim.yaml"

# ── Build image if missing ───────────────────────────────────
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[INFO] Image '${IMAGE}' not found — building..."
    docker build -t "${IMAGE}" "${SCRIPT_DIR}"
fi

# ── Sanity check ─────────────────────────────────────────────
if [ ! -f "${BRIDGE_YAML_HOST}" ]; then
    echo "[ERROR] ${BRIDGE_YAML_HOST} not found"
    echo "        parameter_bridge needs it to know which topics + QoS"
    exit 1
fi

# ── Bump kernel UDP receive buffer for sustained depth ───────
# CycloneDDS asks for 10 MB receive buffers; sustained depth
# streams will start dropping samples without this. Silent if
# sudo isn't available — CycloneDDS will log a capped-buffer
# warning at startup but still work, just less robustly.
if command -v sudo >/dev/null 2>&1; then
    sudo sysctl -w net.core.rmem_max=26214400    >/dev/null 2>&1 || true
    sudo sysctl -w net.core.rmem_default=26214400 >/dev/null 2>&1 || true
fi

# ── Host log file ────────────────────────────────────────────
LOGFILE_HOST="${SCRIPT_DIR}/bridge_sim.log"
touch "${LOGFILE_HOST}"
chmod 666 "${LOGFILE_HOST}"

docker rm -f "${CONTAINER}" 2>/dev/null || true

echo "════════════════════════════════════════════════"
echo "  Launching ${CONTAINER}   mode=sim"
echo "  Domain   : ${ROS_DOMAIN_ID}"
echo "  RMW      : ${RMW_IMPLEMENTATION}"
echo "  Config   : ${BRIDGE_YAML_HOST}"
echo "  Log      : ${LOGFILE_HOST}"
echo "════════════════════════════════════════════════"

docker run -it --rm \
    --net=host \
    --ipc=host \
    --name="${CONTAINER}" \
    -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
    -e ROS_HOSTNAME="localhost" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
    -e BRIDGE_MODE="sim" \
    -e LOGFILE="/tmp/bridge.log" \
    -e BRIDGE_YAML="/bridge.yaml" \
    -v "${SCRIPT_DIR}/entrypoint.sh:/entrypoint.sh:ro" \
    -v "${BRIDGE_YAML_HOST}:/bridge.yaml:ro" \
    -v "${LOGFILE_HOST}:/tmp/bridge.log:rw" \
    -v /dev/shm:/dev/shm \
    --entrypoint /entrypoint.sh \
    "${IMAGE}" \
    "$@"