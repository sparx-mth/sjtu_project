#!/bin/bash
# ============================================================
# ros_bridge_docker/run_bridge.sh — ROS1 <-> ROS2 bridge
#
# Equivalent to the "Terminal 4" docker run command in README.md.
# Bypasses the container entrypoint to avoid the localhost DDS
# config, which can break multicast discovery on --net=host.
#
# Usage:
#   ./run_bridge.sh
# ============================================================

IMAGE="ros1_bridge:noetic-foxy"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[INFO] Image '${IMAGE}' not found — building (takes ~10 min first time)..."
    docker build -t "${IMAGE}" "${SCRIPT_DIR}"
fi

docker run -it --rm \
    --net=host \
    --name="ros1_bridge" \
    -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
    --entrypoint bash \
    "${IMAGE}" -c '
        source /opt/ros/noetic/setup.bash
        source /opt/ros/foxy/setup.bash
        source /bridge_ws/install/setup.bash
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export ROS_DOMAIN_ID='"${ROS_DOMAIN_ID:-20}"'

        # ── DDS config: match the sim side ──────────────────────
        # Use the built-in XML but override the NetworkInterfaceAddress
        # to auto-detect (the original "lo" is not multicast-capable).
        # Foxy CycloneDDS does not support SharedMemory, so no SHM tag.
        cat > /tmp/cyclonedds_bridge.xml <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
EOF
        export CYCLONEDDS_URI=file:///tmp/cyclonedds_bridge.xml

        echo "════════════════════════════════════════"
        echo "  ros1_bridge (Noetic <-> Foxy)"
        echo "  ROS1 master : ${ROS_MASTER_URI}"
        echo "  ROS2 domain : ${ROS_DOMAIN_ID}"
        echo "  RMW         : ${RMW_IMPLEMENTATION}"
        echo "  DDS config  : ${CYCLONEDDS_URI}"
        echo "════════════════════════════════════════"

        while true; do
            echo "[bridge] Starting dynamic_bridge..."
            ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics --bridge-all-1to2-topics
            EXIT_CODE=$?
            echo "[bridge] Bridge exited (code ${EXIT_CODE}). Restarting in 3s..."
            sleep 3
        done
    '