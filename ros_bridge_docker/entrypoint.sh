#!/bin/bash
# ============================================================
# ros_bridge_docker/entrypoint.sh
#
# Dynamic ros1_bridge with lazy matching:
#   - only bridges topics where both sides have pub/sub interest
#   - no flooding of ROS2 with FALCON's /voxel_mapping/* etc.
#   - DDS interface pinned; receive buffers sized for sustained
#     1.2 MB-per-frame depth streams
#
# Output mirrored to /tmp/bridge.log (bind-mounted to host).
# ============================================================

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
LOGFILE="${LOGFILE:-/tmp/bridge.log}"

: > "${LOGFILE}" 2>/dev/null || true
exec > >(tee -a "${LOGFILE}") 2> >(tee -a "${LOGFILE}" >&2)

# ── Pin CycloneDDS to the default-route NIC ──────────────────
DDS_IFACE="$(ip -o -4 route show to default 2>/dev/null | awk '{print $5; exit}')"
if [[ -z "${DDS_IFACE}" ]]; then
    DDS_IFACE="$(ls /sys/class/net/ 2>/dev/null \
                  | grep -vE '^(lo|docker|br-|veth|tun|tap)' | head -1)"
fi

# ── Tuned CycloneDDS config ──────────────────────────────────
# MinimumSocketReceiveBufferSize asks the kernel for a 10 MB
# receive buffer per participant. Without this, UDP samples
# are dropped under sustained depth load and the reader can
# wedge after a few minutes of flight.
#
# The kernel caps this at net.core.rmem_max, which must be
# raised on the host (run_bridge.sh tries via sudo).
cat > /tmp/cyclonedds.xml <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <NetworkInterfaceAddress>${DDS_IFACE}</NetworkInterfaceAddress>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Internal>
      <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
    </Internal>
  </Domain>
</CycloneDDS>
EOF
export CYCLONEDDS_URI=file:///tmp/cyclonedds.xml

# ── Source ROS1 first so rostopic is definitely on PATH ──────
source /opt/ros/noetic/setup.bash

echo "════════════════════════════════════════════════"
echo "  ros1_bridge — dynamic, lazy matching"
echo "  ROS1 master : ${ROS_MASTER_URI}"
echo "  ROS2 domain : ${ROS_DOMAIN_ID}"
echo "  DDS iface   : ${DDS_IFACE:-<arbitrary>}"
echo "  RMW         : ${RMW_IMPLEMENTATION}"
echo "  Log file    : ${LOGFILE}"
echo "════════════════════════════════════════════════"

# ── Wait for roscore ─────────────────────────────────────────
echo "[bridge] Waiting for roscore..."
for i in $(seq 1 60); do
    if timeout 2 rostopic list >/dev/null 2>&1; then
        echo "[bridge] roscore reachable (attempt ${i})."
        break
    fi
    sleep 1
    if [[ ${i} -eq 60 ]]; then
        echo "[bridge] ERROR: roscore unreachable."
        echo "[bridge] Check: docker ps | grep roscore"
        exit 1
    fi
done

# ── Source ROS2 + bridge workspace ───────────────────────────
source /opt/ros/foxy/setup.bash 2>/dev/null
source /bridge_ws/install/setup.bash 2>/dev/null

# ── Dynamic bridge (lazy matching) ───────────────────────────
# Without --bridge-all-*-topics, dynamic_bridge only bridges a
# topic once it sees a matching pub/sub pair on both sides.
# This prevents FALCON's 40+ internal topics from being
# uselessly forwarded to ROS2 and competing with depth for
# bridge bandwidth.
while true; do
    echo "[bridge] Starting dynamic_bridge (lazy)..."
    ros2 run ros1_bridge dynamic_bridge
    echo "[bridge] bridge exited. Restarting in 3s..."
    sleep 3
done