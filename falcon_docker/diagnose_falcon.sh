#!/bin/bash
# ============================================================
# diagnose_falcon.sh
# Run this on the HOST while all 4 containers are up:
#   sjtu_drone_hospital, roscore, ros1_bridge, falcon
#
# Usage: chmod +x diagnose_falcon.sh && ./diagnose_falcon.sh
# ============================================================
set -o pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS=0
WARN=0
FAIL=0

pass()  { echo -e "  ${GREEN}[PASS]${NC} $1"; ((PASS++)); }
warn()  { echo -e "  ${YELLOW}[WARN]${NC} $1"; ((WARN++)); }
fail()  { echo -e "  ${RED}[FAIL]${NC} $1"; ((FAIL++)); }
header(){ echo -e "\n${CYAN}═══ $1 ═══${NC}"; }

# Helper: run command inside a container
in_falcon() { docker exec falcon bash -c "source /catkin_ws/devel/setup.bash 2>/dev/null; $1" 2>&1; }
in_bridge() { docker exec ros1_bridge bash -c "source /opt/ros/foxy/setup.bash; source /bridge_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export ROS_DOMAIN_ID=20; $1" 2>&1; }
in_sim()    {
  # Find sim container (name varies)
  SIM_NAME=$(docker ps --format '{{.Names}}' | grep -E 'sjtu_drone' | head -1)
  if [[ -z "$SIM_NAME" ]]; then
    echo "SIM_CONTAINER_NOT_FOUND"
    return 1
  fi
  docker exec "$SIM_NAME" bash -c "source /opt/ros/humble/setup.bash 2>/dev/null; source /root/*/install/setup.bash 2>/dev/null; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export ROS_DOMAIN_ID=20; $1" 2>&1
}

# ==============================================================
header "1. CONTAINER STATUS"
# ==============================================================

for name in roscore ros1_bridge falcon; do
  if docker ps --format '{{.Names}}' | grep -qx "$name"; then
    pass "$name is running"
  else
    fail "$name is NOT running"
  fi
done

SIM_NAME=$(docker ps --format '{{.Names}}' | grep -E 'sjtu_drone' | head -1)
if [[ -n "$SIM_NAME" ]]; then
  pass "Sim container ($SIM_NAME) is running"
else
  fail "No sjtu_drone container found"
fi

# ==============================================================
header "2. DDS & DOMAIN ID (sim side)"
# ==============================================================

if [[ -n "$SIM_NAME" ]]; then
  SIM_RMW=$(docker exec "$SIM_NAME" bash -c 'echo $RMW_IMPLEMENTATION' 2>&1)
  SIM_DOM=$(docker exec "$SIM_NAME" bash -c 'echo $ROS_DOMAIN_ID' 2>&1)
  echo "  Sim RMW_IMPLEMENTATION = $SIM_RMW"
  echo "  Sim ROS_DOMAIN_ID      = $SIM_DOM"
  if [[ "$SIM_RMW" == *"cyclonedds"* ]]; then
    pass "Sim uses CycloneDDS"
  else
    fail "Sim NOT using CycloneDDS (got: $SIM_RMW). Bridge requires CycloneDDS."
  fi
  if [[ "$SIM_DOM" == "20" ]]; then
    pass "Sim Domain ID = 20"
  else
    warn "Sim Domain ID = $SIM_DOM (bridge expects 20)"
  fi
fi

# ==============================================================
header "3. ROS2 TOPICS (seen by bridge)"
# ==============================================================

BRIDGE_TOPICS=$(in_bridge "ros2 topic list 2>/dev/null")
echo "$BRIDGE_TOPICS" | head -20
echo "  ---"

for t in /simple_drone/gt_pose /simple_drone/cmd_vel "/simple_drone/front_depth/depth/image_raw"; do
  if echo "$BRIDGE_TOPICS" | grep -qF "$t"; then
    pass "Bridge sees $t"
  else
    fail "Bridge does NOT see $t"
  fi
done

# ==============================================================
header "4. ROS1 TOPICS (inside FALCON container)"
# ==============================================================

FALCON_TOPICS=$(in_falcon "rostopic list 2>/dev/null")
echo "$FALCON_TOPICS" | head -30
echo "  ---"

for t in /odom_world /map_ros/depth /map_ros/pose /simple_drone/cmd_vel /simple_drone/gt_pose /planning/pos_cmd; do
  if echo "$FALCON_TOPICS" | grep -qF "$t"; then
    pass "FALCON sees $t"
  else
    fail "FALCON does NOT see $t (check bridge + adapter)"
  fi
done

# ==============================================================
header "5. TOPIC RATES"
# ==============================================================

echo "  Checking /odom_world rate (expect ~30 Hz, NOT 800+ Hz)..."
ODOM_HZ=$(in_falcon "timeout 4 rostopic hz /odom_world 2>&1 | tail -1")
echo "  → $ODOM_HZ"
ODOM_RATE=$(echo "$ODOM_HZ" | grep -oP 'average rate: \K[0-9.]+' || echo "0")
if (( $(echo "$ODOM_RATE > 100" | bc -l 2>/dev/null || echo 0) )); then
  fail "/odom_world rate is ${ODOM_RATE} Hz — WAY too fast! Velocity estimation will be noisy."
  echo -e "       ${YELLOW}→ The gt_pose callback fires too often. Throttle it or use a timer.${NC}"
elif (( $(echo "$ODOM_RATE > 5" | bc -l 2>/dev/null || echo 0) )); then
  pass "/odom_world rate ~${ODOM_RATE} Hz"
else
  warn "/odom_world rate is ${ODOM_RATE} Hz — too low or no data"
fi

echo ""
echo "  Checking /map_ros/depth rate (expect 10-30 Hz)..."
DEPTH_HZ=$(in_falcon "timeout 4 rostopic hz /map_ros/depth 2>&1 | tail -1")
echo "  → $DEPTH_HZ"
DEPTH_RATE=$(echo "$DEPTH_HZ" | grep -oP 'average rate: \K[0-9.]+' || echo "0")
if (( $(echo "$DEPTH_RATE > 5" | bc -l 2>/dev/null || echo 0) )); then
  pass "/map_ros/depth rate ~${DEPTH_RATE} Hz"
else
  fail "/map_ros/depth rate is ${DEPTH_RATE} Hz — too low or no data. No map possible."
fi

echo ""
echo "  Checking /map_ros/pose rate..."
POSE_HZ=$(in_falcon "timeout 4 rostopic hz /map_ros/pose 2>&1 | tail -1")
echo "  → $POSE_HZ"

# ==============================================================
header "6. DEPTH IMAGE FORMAT (critical for point cloud)"
# ==============================================================

echo "  Grabbing one depth message..."
DEPTH_INFO=$(in_falcon "timeout 5 rostopic echo /map_ros/depth -n 1 --noarr 2>&1")
DEPTH_ENC=$(echo "$DEPTH_INFO" | grep 'encoding:' | head -1 | awk '{print $2}')
DEPTH_W=$(echo "$DEPTH_INFO"   | grep 'width:'    | head -1 | awk '{print $2}')
DEPTH_H=$(echo "$DEPTH_INFO"   | grep 'height:'   | head -1 | awk '{print $2}')
DEPTH_FRAME=$(echo "$DEPTH_INFO" | grep 'frame_id:' | head -1 | awk '{print $2}')

echo "  encoding : $DEPTH_ENC"
echo "  size     : ${DEPTH_W}x${DEPTH_H}"
echo "  frame_id : $DEPTH_FRAME"

if [[ "$DEPTH_ENC" == "32FC1" ]]; then
  pass "Depth encoding is 32FC1 (float meters)"
  warn "FALCON's voxel_mapping may expect 16UC1 (mm). Check voxel_mapping.yaml 'depth_scale' param."
elif [[ "$DEPTH_ENC" == "16UC1" ]]; then
  pass "Depth encoding is 16UC1 (typical for FALCON)"
else
  warn "Unexpected depth encoding: $DEPTH_ENC"
fi

# ==============================================================
header "7. GT_POSE RAW VALUES (drone position)"
# ==============================================================

echo "  Grabbing one gt_pose message from the drone..."
GT_POSE=$(in_falcon "timeout 5 rostopic echo /simple_drone/gt_pose -n 1 2>&1")
echo "$GT_POSE" | head -15
GT_Z=$(echo "$GT_POSE" | grep -A3 'position:' | grep 'z:' | head -1 | awk '{print $2}')
echo "  → Drone Z = $GT_Z"
if (( $(echo "${GT_Z:-0} > 0.3" | bc -l 2>/dev/null || echo 0) )); then
  pass "Drone appears airborne (z=$GT_Z)"
else
  warn "Drone z=$GT_Z — may not be airborne yet"
fi

# ==============================================================
header "8. CMD_VEL ANALYSIS (why drone flies sideways)"
# ==============================================================

echo "  Grabbing cmd_vel..."
CMD_VEL=$(in_falcon "timeout 5 rostopic echo /simple_drone/cmd_vel -n 1 2>&1")
echo "$CMD_VEL"
VX=$(echo "$CMD_VEL" | sed -n '/^linear:/,/^angular:/p' | grep 'x:' | head -1 | awk '{print $2}')
VY=$(echo "$CMD_VEL" | sed -n '/^linear:/,/^angular:/p' | grep 'y:' | head -1 | awk '{print $2}')
VZ=$(echo "$CMD_VEL" | sed -n '/^linear:/,/^angular:/p' | grep 'z:' | head -1 | awk '{print $2}')

echo ""
echo "  linear.x=$VX  linear.y=$VY  linear.z=$VZ"

# Check if Y >> X (sideways flight)
ABS_VX=$(echo "${VX:-0}" | awk '{print ($1<0)?-$1:$1}')
ABS_VY=$(echo "${VY:-0}" | awk '{print ($1<0)?-$1:$1}')
if (( $(echo "$ABS_VY > $ABS_VX * 2" | bc -l 2>/dev/null || echo 0) )); then
  fail "|linear.y| >> |linear.x| → drone is flying SIDEWAYS"
  echo -e "       ${YELLOW}→ The _world_to_body() transform in falcon_adapter.py is likely wrong.${NC}"
  echo -e "       ${YELLOW}→ sjtu_drone's cmd_vel likely expects WORLD-frame velocities, not body-frame.${NC}"
  echo -e "       ${YELLOW}→ FIX: Remove the _world_to_body() call. Send world-frame vxy directly.${NC}"
fi

# ==============================================================
header "9. TF TREE (critical for FALCON point cloud)"
# ==============================================================

echo "  Checking TF frames published by the adapter..."
TF_FRAMES=$(in_falcon "timeout 3 rostopic echo /tf -n 20 2>&1" | grep 'frame_id\|child_frame_id' | sort -u)
echo "$TF_FRAMES"

if echo "$TF_FRAMES" | grep -q 'world'; then
  pass "TF has 'world' frame"
else
  warn "TF missing 'world' frame"
fi
if echo "$TF_FRAMES" | grep -q 'body'; then
  pass "TF has 'body' frame"
else
  warn "TF missing 'body' frame"
fi
if echo "$TF_FRAMES" | grep -q 'camera'; then
  pass "TF has 'camera' frame"
else
  fail "TF missing 'camera' frame — depth won't project into map"
fi

# ==============================================================
header "10. FALCON EXPLORATION NODE STATUS"
# ==============================================================

echo "  Checking if exploration_node is running..."
EXPL=$(in_falcon "rosnode list 2>/dev/null" | grep -i explor)
if [[ -n "$EXPL" ]]; then
  pass "exploration_node is running: $EXPL"
else
  fail "exploration_node NOT found in rosnode list"
fi

echo "  Checking if traj_server is running..."
TRAJ=$(in_falcon "rosnode list 2>/dev/null" | grep -i traj)
if [[ -n "$TRAJ" ]]; then
  pass "traj_server is running: $TRAJ"
else
  fail "traj_server NOT found"
fi

echo "  Checking if falcon_adapter is running..."
ADAPT=$(in_falcon "rosnode list 2>/dev/null" | grep -i adapter)
if [[ -n "$ADAPT" ]]; then
  pass "falcon_adapter is running: $ADAPT"
else
  fail "falcon_adapter NOT found"
fi

# ==============================================================
header "11. FALCON LOG MESSAGES (errors/warnings)"
# ==============================================================

echo "  Checking rosout for errors from FALCON nodes..."
ERRORS=$(in_falcon "timeout 3 rostopic echo /rosout -n 50 2>&1" | grep -iE 'error|fail|no odom|no depth|cannot' | head -10)
if [[ -n "$ERRORS" ]]; then
  warn "Recent error messages from FALCON:"
  echo "$ERRORS"
else
  pass "No obvious errors in recent rosout"
fi

# ==============================================================
header "12. VOXEL MAPPING SUBSCRIPTIONS"
# ==============================================================

echo "  Checking what topics exploration_node subscribes to..."
EXPL_NODE=$(in_falcon "rosnode list 2>/dev/null" | grep -i explor | head -1)
if [[ -n "$EXPL_NODE" ]]; then
  SUBS=$(in_falcon "rosnode info $EXPL_NODE 2>/dev/null" | sed -n '/Subscriptions:/,/^$/p' | head -20)
  echo "$SUBS"

  # Check if it's actually subscribed to the right depth topic
  if echo "$SUBS" | grep -q "map_ros/depth"; then
    pass "exploration_node subscribed to /map_ros/depth"
  elif echo "$SUBS" | grep -q "depth"; then
    warn "exploration_node subscribed to a depth topic but NOT /map_ros/depth"
    echo "$SUBS" | grep depth
  else
    fail "exploration_node NOT subscribed to any depth topic!"
  fi
fi

# ==============================================================
header "13. CHECKING sjtu_drone CMD_VEL FRAME CONVENTION"
# ==============================================================

echo "  Testing: Does sjtu_drone interpret cmd_vel in world or body frame?"
echo "  Current drone yaw (from gt_pose):"
GT_ORI=$(in_falcon "timeout 5 rostopic echo /simple_drone/gt_pose -n 1 2>&1")
ORI_X=$(echo "$GT_ORI" | sed -n '/orientation:/,/^[a-z]/p' | grep 'x:' | head -1 | awk '{print $2}')
ORI_Y=$(echo "$GT_ORI" | sed -n '/orientation:/,/^[a-z]/p' | grep 'y:' | head -1 | awk '{print $2}')
ORI_Z=$(echo "$GT_ORI" | sed -n '/orientation:/,/^[a-z]/p' | grep 'z:' | head -1 | awk '{print $2}')
ORI_W=$(echo "$GT_ORI" | sed -n '/orientation:/,/^[a-z]/p' | grep 'w:' | head -1 | awk '{print $2}')
echo "  Quaternion: ($ORI_X, $ORI_Y, $ORI_Z, $ORI_W)"
echo ""
echo "  If yaw ≠ 0 but linear.y >> linear.x in cmd_vel,"
echo "  the world-to-body transform is causing the sideways flight."
echo ""
echo "  Compute yaw with: yaw = 2*atan2(z,w) ≈ $(python3 -c "
import math
try:
    z,w = float('${ORI_Z:-0}'), float('${ORI_W:-1}')
    yaw = 2*math.atan2(z,w)
    print(f'{math.degrees(yaw):.1f} degrees ({yaw:.3f} rad)')
except: print('(could not compute)')
" 2>/dev/null)"

# ==============================================================
header "SUMMARY"
# ==============================================================

echo ""
echo -e "  ${GREEN}PASS: $PASS${NC}  ${YELLOW}WARN: $WARN${NC}  ${RED}FAIL: $FAIL${NC}"
echo ""

if (( FAIL > 0 )); then
  echo -e "${RED}═══ LIKELY ROOT CAUSES ═══${NC}"
  echo ""
  echo "1. DRONE FLIES SIDEWAYS (not forward):"
  echo "   The falcon_adapter.py applies _world_to_body() before publishing cmd_vel."
  echo "   sjtu_drone's cmd_vel plugin most likely expects WORLD-FRAME velocities."
  echo "   The extra rotation sends the drone sideways instead of forward."
  echo "   FIX: In falcon_adapter.py control_loop(), replace:"
  echo "     vx_body, vy_body = self._world_to_body(vxy[0], vxy[1], yaw)"
  echo "     cmd.linear.x = vx_body"
  echo "     cmd.linear.y = vy_body"
  echo "   WITH:"
  echo "     cmd.linear.x = vxy[0]"
  echo "     cmd.linear.y = vxy[1]"
  echo ""
  echo "2. ODOM RATE ~800 Hz (should be ~30 Hz):"
  echo "   The gt_pose callback fires for every message from the bridge."
  echo "   At 800 Hz, finite-difference velocity has extreme noise."
  echo "   FIX: Throttle in gt_pose_cb — skip if dt < 0.02 (50 Hz max)."
  echo ""
  echo "3. NO POINT CLOUD / NO MAP:"
  echo "   Check these in order:"
  echo "   a) Is depth encoding correct? FALCON likely expects 32FC1 (meters)."
  echo "      Check voxel_mapping.yaml for 'depth_scale' or 'depth_filter' params."
  echo "   b) Does the T_b_c in hospital.yaml match the actual camera orientation?"
  echo "      The current T_b_c assumes a standard optical frame (Z=forward)."
  echo "      If the sjtu_drone depth camera has a different orientation, T_b_c is wrong."
  echo "   c) Is the depth image resolution what FALCON expects?"
  echo "      Check voxel_mapping.yaml for image width/height params."
  echo "   d) Run inside falcon container:"
  echo "        rosparam get /voxel_mapping   (see all voxel mapping params)"
  echo "        rostopic echo /map_ros/depth --noarr -n 1  (check encoding, size, frame)"
  echo ""
fi