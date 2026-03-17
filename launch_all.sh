#!/bin/bash
# ============================================================
# launch_all.sh — Start the full exploration stack
#
# Containers started (in order):
#   1. Gazebo sim  (sjtu_drone, ROS2 Humble)
#   2. roscore     (inside bridge image)
#   3. ros1_bridge (Noetic <-> Foxy dynamic bridge)
#   4. FALCON      (exploration planner + adapter)
#
# All containers use --net=host so they share localhost.
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
WORLD="${1:-hospital.world}"

RED='\033[31m'; GREEN='\033[32m'; CYAN='\033[36m'; NC='\033[0m'
info()  { echo -e "${CYAN}[INFO]${NC} $1"; }
ok()    { echo -e "${GREEN}[ OK ]${NC} $1"; }
err()   { echo -e "${RED}[ERR ]${NC} $1"; }

echo ""
echo "════════════════════════════════════════════════"
echo "  FALCON + Gazebo Exploration Stack"
echo "  World    : ${WORLD}"
echo "  Domain ID: ${DOMAIN_ID}"
echo "════════════════════════════════════════════════"
echo ""

# ── 0. Cleanup ────────────────────────────────────────────────
info "Stopping leftover containers..."
docker stop falcon ros1_bridge roscore 2>/dev/null || true
docker rm   falcon ros1_bridge roscore 2>/dev/null || true

# ── 1. Gazebo Sim ─────────────────────────────────────────────
info "Step 1/4: Starting Gazebo simulation..."
echo "  Run in a separate terminal:"
echo ""
echo "    cd ${SCRIPT_DIR}/sjtu_drone"
echo "    ./run.sh ${WORLD}"
echo ""
read -p "  Press ENTER when Gazebo is fully loaded..."
ok "Gazebo assumed ready."

# ── 2. roscore ────────────────────────────────────────────────
info "Step 2/4: Starting roscore..."
docker run -d --rm --net=host --name=roscore \
    --entrypoint bash ros1_bridge:noetic-foxy -c \
    "source /opt/ros/noetic/setup.bash && roscore" \
    >/dev/null

# Wait for roscore
for i in $(seq 1 15); do
    if docker logs roscore 2>&1 | grep -q "started core service"; then
        ok "roscore is ready."
        break
    fi
    sleep 1
    [[ $i -eq 15 ]] && { err "roscore did not start in time."; exit 1; }
done

# ── 3. Bridge ─────────────────────────────────────────────────
info "Step 3/4: Starting ROS1<->ROS2 bridge..."

# Build bridge image if needed
if ! docker image inspect ros1_bridge:noetic-foxy >/dev/null 2>&1; then
    info "Building bridge image (first time ~10 min)..."
    docker build -t ros1_bridge:noetic-foxy "${SCRIPT_DIR}/ros_bridge_docker"
fi

docker run -d --rm --net=host --name=ros1_bridge \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e ROS_IP="127.0.0.1" \
    -e ROS_DOMAIN_ID="${DOMAIN_ID}" \
    -e RMW_IMPLEMENTATION="rmw_fastrtps_cpp" \
    -e BRIDGE_MODE="dynamic" \
    ros1_bridge:noetic-foxy \
    >/dev/null

sleep 3
if docker ps --format '{{.Names}}' | grep -q ros1_bridge; then
    ok "Bridge is running."
else
    err "Bridge failed to start. Check: docker logs ros1_bridge"
    exit 1
fi

# ── 4. FALCON ─────────────────────────────────────────────────
info "Step 4/4: Starting FALCON container..."
echo ""
echo "  The FALCON container will open an interactive shell."
echo "  Inside it, run:"
echo ""
echo "    # Terminal 1 — RViz"
echo "    roslaunch exploration_manager rviz.launch"
echo ""
echo "    # Terminal 2 — Adapter (open with: docker exec -it falcon bash)"
echo "    roslaunch falcon_adapter gazebo_exploration.launch"
echo ""
echo "    # Terminal 3 — Exploration planner"
echo "    roslaunch exploration_manager exploration.launch map_name:=hospital"
echo ""
echo "════════════════════════════════════════════════"
echo ""

# Build FALCON image if needed
if ! docker image inspect falcon-ros:noetic >/dev/null 2>&1; then
    info "Building FALCON image (first time 30-60 min)..."
    docker build --build-arg CUDA_ARCH=120 \
        -t falcon-ros:noetic "${SCRIPT_DIR}/falcon_docker"
fi

xhost +local:docker 2>/dev/null || true

docker run -it --rm \
    --name falcon \
    --gpus all \
    --env DISPLAY="${DISPLAY}" \
    --env QT_X11_NO_MITSHM=1 \
    --env NVIDIA_VISIBLE_DEVICES=all \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --network host \
    falcon-ros:noetic \
    bash
