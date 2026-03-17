#!/bin/bash
# ros_bridge_docker/verify_bridge.sh
PASS=0; FAIL=0; WARN=0
green()  { echo -e "\033[32m  OK  $1\033[0m"; }
red()    { echo -e "\033[31m  FAIL $1\033[0m"; }
yellow() { echo -e "\033[33m  WARN $1\033[0m"; }

echo ""
echo "════════════════════════════════════════"
echo "  Bridge Verification"
echo "════════════════════════════════════════"

# Check containers
for c in roscore ros1_bridge; do
  if docker ps --format '{{.Names}}' | grep -q "^${c}$"; then
    green "'${c}' running"; ((PASS++))
  else
    red "'${c}' NOT running"; ((FAIL++))
  fi
done

SIM=$(docker ps --format '{{.Names}}' | grep -v ros1_bridge | grep -v roscore | grep -v falcon | head -1)
if [[ -n "${SIM}" ]]; then
  green "Sim container: '${SIM}'"; ((PASS++))
else
  red "No sim container found"; ((FAIL++))
fi

# ROS2 topics
echo ""
echo "── ROS2 topics (Domain 20) ──"
R2=$(docker exec ros1_bridge bash -c \
  "source /opt/ros/foxy/setup.bash && source /bridge_ws/install/setup.bash && \
   ROS_DOMAIN_ID=20 ros2 topic list 2>/dev/null" \
  | grep -v "^/parameter_events$" | grep -v "^/rosout$" | grep -v "ROS_DISTRO")
TC=$(echo "${R2}" | grep -c "^/" || true)

if [[ ${TC} -gt 3 ]]; then
  green "${TC} ROS2 topics visible"; ((PASS++))
  echo "${R2}" | sed 's/^/     /'
else
  red "Only ${TC} ROS2 topics — is the sim running?"; ((FAIL++))
fi

# Key topics
echo ""
echo "── Key topics ──"
for t in cmd_vel gt_pose depth; do
  if echo "${R2}" | grep -q "${t}"; then
    green "Found: $(echo "${R2}" | grep "${t}" | head -1)"; ((PASS++))
  else
    yellow "${t} not found yet"; ((WARN++))
  fi
done

# ROS1 side
echo ""
echo "── ROS1 bridged topics ──"
R1=$(docker exec ros1_bridge bash -c \
  "source /opt/ros/noetic/setup.bash && rostopic list 2>/dev/null" \
  | grep -v "^/rosout")
if [[ -n "${R1}" ]]; then
  green "ROS1 topics:"; echo "${R1}" | sed 's/^/     /'; ((PASS++))
else
  yellow "No ROS1 topics yet (normal before FALCON starts)"; ((WARN++))
fi

echo ""
echo "════════════════════════════════════════"
echo "  ${PASS} passed, ${FAIL} failed, ${WARN} warnings"
[[ ${FAIL} -eq 0 ]] && echo -e "\033[32m  Bridge OK\033[0m" || echo -e "\033[31m  Fix errors above\033[0m"
echo "════════════════════════════════════════"
