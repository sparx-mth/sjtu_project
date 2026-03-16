#!/bin/bash
# verify_bridge.sh

PASS=0
FAIL=0
WARN=0

green()  { echo -e "\033[32m✅ $1\033[0m"; }
red()    { echo -e "\033[31m❌ $1\033[0m"; }
yellow() { echo -e "\033[33m⚠️  $1\033[0m"; }
info()   { echo -e "\033[36m── $1\033[0m"; }

echo ""
echo "════════════════════════════════════════"
echo "  Bridge Verification"
echo "════════════════════════════════════════"

# ── Check containers are running ─────────────────────────────────
info "Checking containers..."

for name in roscore ros1_bridge; do
  if docker ps --format '{{.Names}}' | grep -q "^${name}$"; then
    green "Container '${name}' is running"
    ((PASS++))
  else
    red "Container '${name}' is NOT running — start it first"
    ((FAIL++))
  fi
done

SIM_CONTAINER=$(docker ps --format '{{.Names}}' | grep -v ros1_bridge | grep -v roscore | head -1)
if [[ -n "${SIM_CONTAINER}" ]]; then
  green "Sim container found: '${SIM_CONTAINER}'"
  ((PASS++))
else
  red "No sim container found — is your ROS2 sim running?"
  ((FAIL++))
fi

# ── Check ROS2 topics visible from bridge ────────────────────────
echo ""
info "ROS2 topics visible from bridge (Domain ID=20)..."
ROS2_TOPICS=$(docker exec ros1_bridge bash -c \
  "source /opt/ros/foxy/setup.bash && \
   source /bridge_ws/install/setup.bash && \
   ROS_DOMAIN_ID=20 ros2 topic list 2>/dev/null" \
  | grep -v "^/parameter_events$" \
  | grep -v "^/rosout$" \
  | grep -v "ROS_DISTRO")

TOPIC_COUNT=$(echo "${ROS2_TOPICS}" | grep -c "^/" || true)

if [[ ${TOPIC_COUNT} -gt 3 ]]; then
  green "ROS2 sim topics visible (${TOPIC_COUNT} topics):"
  echo "${ROS2_TOPICS}" | sed 's/^/     /'
  ((PASS++))
else
  red "Too few ROS2 topics (${TOPIC_COUNT}) — takeoff the drone first:"
  echo "     docker exec -it ${SIM_CONTAINER} bash -c \\"
  echo "       \"source /opt/ros/humble/setup.bash && \\"
  echo "        ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty {} --once\""
  ((FAIL++))
fi

# ── Check cmd_vel specifically ────────────────────────────────────
echo ""
info "Checking for key FALCON topics..."
for topic in cmd_vel gt_pose odom; do
  FOUND=$(echo "${ROS2_TOPICS}" | grep "${topic}" | head -1)
  if [[ -n "${FOUND}" ]]; then
    green "Found: ${FOUND}"
    ((PASS++))
  else
    yellow "${topic} not found (may appear after takeoff)"
    ((WARN++))
  fi
done

# ── ROS1 side note ───────────────────────────────────────────────
echo ""
info "ROS1 bridged topics..."
ROS1_TOPICS=$(docker exec ros1_bridge bash -c \
  "source /opt/ros/noetic/setup.bash && \
   rostopic list 2>/dev/null" | grep -v "^/rosout")

if [[ -n "${ROS1_TOPICS}" ]]; then
  green "ROS1 topics bridged:"
  echo "${ROS1_TOPICS}" | sed 's/^/     /'
  ((PASS++))
else
  yellow "No ROS1 topics yet — this is NORMAL at this stage."
  echo "     ROS1 topics appear automatically once FALCON"
  echo "     is running and subscribes to them."
  ((WARN++))
fi

# ── Summary ──────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo "  Results: ${PASS} passed, ${FAIL} failed, ${WARN} warnings"
if [[ ${FAIL} -eq 0 ]]; then
  echo -e "\033[32m  ✅ Bridge is working! Ready for FALCON.\033[0m"
  if [[ ${WARN} -gt 0 ]]; then
    echo -e "\033[33m  ⚠️  Warnings are expected — see notes above.\033[0m"
  fi
else
  echo -e "\033[31m  ❌ Fix the failed checks above before proceeding.\033[0m"
fi
echo "════════════════════════════════════════"
echo ""