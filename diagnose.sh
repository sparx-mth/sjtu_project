#!/bin/bash
# ============================================================
# diagnose.sh — Full pipeline diagnostic for FALCON + sjtu_drone
#
# Run from the host with all 4 containers running.
# Each test is independent — prints PASS/FAIL/INFO.
#
# Usage:
#   bash diagnose.sh          # run all tests
#   bash diagnose.sh 14       # run only test 14
#   bash diagnose.sh 10 20    # run tests 10 through 20
# ============================================================

set -o pipefail

GREEN='\033[32m'
RED='\033[31m'
YELLOW='\033[33m'
CYAN='\033[36m'
BOLD='\033[1m'
NC='\033[0m'

PASS=0; FAIL=0; WARN=0; INFO=0
RESULTS=()

pass()  { echo -e "  ${GREEN}PASS${NC}  $1"; ((PASS++)); RESULTS+=("PASS: $1"); }
fail()  { echo -e "  ${RED}FAIL${NC}  $1"; ((FAIL++)); RESULTS+=("FAIL: $1"); }
warn()  { echo -e "  ${YELLOW}WARN${NC}  $1"; ((WARN++)); RESULTS+=("WARN: $1"); }
info()  { echo -e "  ${CYAN}INFO${NC}  $1"; ((INFO++)); RESULTS+=("INFO: $1"); }
header(){ echo -e "\n${BOLD}═══ $1 ═══${NC}"; }

# ── Helper: run command in a container ──
# Usage: in_container <name> <bash_command>
in_container() {
    local name="$1"; shift
    docker exec "$name" bash -c "$*" 2>&1
}

# Source strings for each container
# IMPORTANT: sim checks must use CycloneDDS + matching domain to see topics
SIM_ROS2_ENV="source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=20 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
ROS2_SRC="source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null"
BRIDGE_ROS1="source /opt/ros/noetic/setup.bash"
BRIDGE_ROS2="source /opt/ros/foxy/setup.bash && source /bridge_ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=20"
FALCON_SRC="source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash 2>/dev/null"

# ── Detect sim container name ──
SIM_CONTAINER=$(docker ps --format '{{.Names}}' | grep -v ros1_bridge | grep -v roscore | grep -v falcon | head -1)

# ── Range filter ──
RUN_FROM=${1:-1}
RUN_TO=${2:-999}
TEST_NUM=0
should_run() {
    ((TEST_NUM++))
    [[ $TEST_NUM -ge $RUN_FROM && $TEST_NUM -le $RUN_TO ]]
}

echo ""
echo "════════════════════════════════════════════════════════"
echo "  FALCON Pipeline Diagnostics"
echo "  $(date)"
echo "════════════════════════════════════════════════════════"

# ══════════════════════════════════════════════════════════════
# SECTION 1: CONTAINERS
# ══════════════════════════════════════════════════════════════
header "1. CONTAINERS"

if should_run; then
    # Test: containers running
    ALL_OK=true
    for c in roscore ros1_bridge falcon; do
        if docker ps --format '{{.Names}}' | grep -q "^${c}$"; then
            pass "Container '${c}' is running"
        else
            fail "Container '${c}' is NOT running"
            ALL_OK=false
        fi
    done
    if [[ -n "$SIM_CONTAINER" ]]; then
        pass "Sim container found: '${SIM_CONTAINER}'"
    else
        fail "No sim container found (expected sjtu_drone or similar)"
        ALL_OK=false
    fi
    if [[ "$ALL_OK" == false ]]; then
        echo -e "  ${RED}Cannot continue without all containers. Fix and re-run.${NC}"
    fi
fi

# ══════════════════════════════════════════════════════════════
# SECTION 2: GAZEBO / ROS2 SIDE
# ══════════════════════════════════════════════════════════════
header "2. GAZEBO (ROS2) — depth camera publishing"

if should_run; then
    # Test: ROS2 depth image topic exists
    R2_TOPICS=$(in_container "${SIM_CONTAINER}" "${SIM_ROS2_ENV} && ros2 topic list 2>/dev/null" | grep front_depth || true)
    if echo "$R2_TOPICS" | grep -q "depth/image_raw"; then
        pass "ROS2 topic exists: .../front_depth/depth/image_raw"
    else
        fail "ROS2 topic .../front_depth/depth/image_raw NOT found"
        echo "       All front_depth topics: ${R2_TOPICS:-none}"
    fi
fi

if should_run; then
    # Test: ROS2 depth camera_info topic exists
    if echo "$R2_TOPICS" | grep -q "depth/camera_info"; then
        pass "ROS2 topic exists: .../front_depth/depth/camera_info"
    else
        fail "ROS2 topic .../front_depth/depth/camera_info NOT found — depth plugin may not publish it"
    fi
fi

if should_run; then
    # Test: ROS2 depth image is actually publishing (hz > 0)
    HZ=$(in_container "${SIM_CONTAINER}" "${SIM_ROS2_ENV} && timeout 4 ros2 topic hz /simple_drone/front_depth/depth/image_raw 2>&1" | grep "average rate" | tail -1 || true)
    if [[ -n "$HZ" ]]; then
        RATE=$(echo "$HZ" | grep -oP '[\d.]+' | head -1)
        pass "ROS2 depth image publishing at ${RATE} Hz"
    else
        fail "ROS2 depth image has 0 Hz — camera not producing frames"
    fi
fi

if should_run; then
    # Test: ROS2 camera_info is publishing
    HZ=$(in_container "${SIM_CONTAINER}" "${SIM_ROS2_ENV} && timeout 4 ros2 topic hz /simple_drone/front_depth/depth/camera_info 2>&1" | grep "average rate" | tail -1 || true)
    if [[ -n "$HZ" ]]; then
        RATE=$(echo "$HZ" | grep -oP '[\d.]+' | head -1)
        pass "ROS2 camera_info publishing at ${RATE} Hz"
    else
        fail "ROS2 camera_info has 0 Hz — Gazebo plugin not publishing intrinsics"
    fi
fi

if should_run; then
    # Test: ROS2 depth encoding
    ENC=$(in_container "${SIM_CONTAINER}" "${SIM_ROS2_ENV} && timeout 3 ros2 topic echo /simple_drone/front_depth/depth/image_raw --once --no-arr 2>&1" | grep "encoding:" | head -1 || true)
    if [[ -n "$ENC" ]]; then
        info "ROS2 depth encoding: $ENC"
        if echo "$ENC" | grep -q "32FC1"; then
            pass "Depth encoding is 32FC1 (meters, float32) — correct for FALCON"
        elif echo "$ENC" | grep -q "16UC1"; then
            warn "Depth encoding is 16UC1 (mm, uint16) — adapter may need conversion"
        else
            warn "Unexpected depth encoding — check if FALCON can handle it"
        fi
    else
        fail "Could not read depth encoding"
    fi
fi

if should_run; then
    # Test: ROS2 depth image dimensions
    DIMS=$(in_container "${SIM_CONTAINER}" "${SIM_ROS2_ENV} && timeout 3 ros2 topic echo /simple_drone/front_depth/depth/image_raw --once --no-arr 2>&1" | grep -E "^(width|height):" || true)
    if [[ -n "$DIMS" ]]; then
        W=$(echo "$DIMS" | grep "width:" | awk '{print $2}')
        H=$(echo "$DIMS" | grep "height:" | awk '{print $2}')
        info "ROS2 depth image size: ${W}x${H}"
        if [[ "$W" == "640" && "$H" == "360" ]]; then
            pass "Depth dimensions match hospital.yaml / launch config (640x360)"
        else
            warn "Depth dimensions ${W}x${H} don't match expected 640x360 — update launch intrinsics"
        fi
    else
        fail "Could not read depth dimensions"
    fi
fi

if should_run; then
    # Test: ROS2 camera_info intrinsics (K matrix)
    K_LINE=$(in_container "${SIM_CONTAINER}" "${SIM_ROS2_ENV} && timeout 3 ros2 topic echo /simple_drone/front_depth/depth/camera_info --once 2>&1" | grep -A1 "^k:" || true)
    if [[ -n "$K_LINE" ]]; then
        info "ROS2 CameraInfo K matrix:"
        echo "$K_LINE" | sed 's/^/       /'
    else
        warn "Could not read CameraInfo K matrix"
    fi
fi

if should_run; then
    # Test: ROS2 depth data is NOT all zeros (actual depth values exist)
    DEPTH_CHECK=$(in_container "${SIM_CONTAINER}" "
        ${SIM_ROS2_ENV} &&
        python3 -c \"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import sys

rclpy.init()
node = Node('depth_check')
msg = None
def cb(m):
    global msg; msg = m
sub = node.create_subscription(Image, '/simple_drone/front_depth/depth/image_raw', cb, 1)
for _ in range(50):
    rclpy.spin_once(node, timeout_sec=0.1)
    if msg: break
if msg is None:
    print('NO_MSG')
    sys.exit(0)
if msg.encoding == '32FC1':
    arr = np.frombuffer(msg.data, dtype=np.float32)
elif msg.encoding == '16UC1':
    arr = np.frombuffer(msg.data, dtype=np.uint16)
else:
    arr = np.frombuffer(msg.data, dtype=np.uint8)
valid = arr[(arr > 0) & np.isfinite(arr)]
total = len(arr)
nvalid = len(valid)
if nvalid > 0:
    print(f'VALID pixels={nvalid}/{total} min={valid.min():.3f} max={valid.max():.3f} mean={valid.mean():.3f}')
else:
    print(f'ALL_ZERO total={total}')
node.destroy_node()
rclpy.shutdown()
\" 2>&1" | tail -1)
    if echo "$DEPTH_CHECK" | grep -q "VALID"; then
        pass "Depth image contains valid data: $DEPTH_CHECK"
    elif echo "$DEPTH_CHECK" | grep -q "ALL_ZERO"; then
        fail "Depth image is ALL ZEROS — camera sees nothing (check clip range, world geometry)"
    elif echo "$DEPTH_CHECK" | grep -q "NO_MSG"; then
        fail "No depth message received in 5s"
    else
        warn "Depth data check inconclusive: $DEPTH_CHECK"
    fi
fi

# ══════════════════════════════════════════════════════════════
# SECTION 3: BRIDGE
# ══════════════════════════════════════════════════════════════
header "3. BRIDGE — topics forwarded to ROS1"

if should_run; then
    # Test: bridge sees ROS2 depth topics
    B_R2=$(in_container ros1_bridge "${BRIDGE_ROS2} && ros2 topic list 2>/dev/null" | grep front_depth || true)
    if echo "$B_R2" | grep -q "depth/image_raw"; then
        pass "Bridge sees ROS2 topic: .../depth/image_raw"
    else
        fail "Bridge does NOT see ROS2 depth/image_raw — DDS domain or RMW mismatch?"
    fi
fi

if should_run; then
    if echo "$B_R2" | grep -q "depth/camera_info"; then
        pass "Bridge sees ROS2 topic: .../depth/camera_info"
    else
        fail "Bridge does NOT see ROS2 depth/camera_info"
    fi
fi

if should_run; then
    # Test: ROS1 topics exist on bridge side
    B_R1=$(in_container ros1_bridge "${BRIDGE_ROS1} && rostopic list 2>/dev/null" | grep front_depth || true)
    if echo "$B_R1" | grep -q "depth/image_raw"; then
        pass "ROS1 topic bridged: .../depth/image_raw"
    else
        fail "ROS1 topic .../depth/image_raw NOT bridged — adapter may not be subscribed to it"
    fi
fi

if should_run; then
    if echo "$B_R1" | grep -q "depth/camera_info"; then
        pass "ROS1 topic bridged: .../depth/camera_info"
    else
        fail "ROS1 topic .../depth/camera_info NOT bridged — adapter may not subscribe to it"
    fi
fi

if should_run; then
    # Test: gt_pose bridged
    if in_container ros1_bridge "${BRIDGE_ROS1} && rostopic list 2>/dev/null" | grep -q "gt_pose"; then
        pass "ROS1 topic bridged: .../gt_pose"
    else
        fail "ROS1 topic gt_pose NOT bridged"
    fi
fi

# ══════════════════════════════════════════════════════════════
# SECTION 4: FALCON ADAPTER — inputs arriving
# ══════════════════════════════════════════════════════════════
header "4. FALCON ADAPTER — input topics"

if should_run; then
    # Test: /odom_world publishing
    HZ=$(in_container falcon "${FALCON_SRC} && timeout 4 rostopic hz /odom_world 2>&1" | grep "average rate" | tail -1 || true)
    if [[ -n "$HZ" ]]; then
        RATE=$(echo "$HZ" | grep -oP '[\d.]+' | head -1)
        pass "/odom_world publishing at ${RATE} Hz"
    else
        fail "/odom_world not publishing — adapter not receiving gt_pose"
    fi
fi

if should_run; then
    # Test: /map_ros/depth publishing
    HZ=$(in_container falcon "${FALCON_SRC} && timeout 4 rostopic hz /map_ros/depth 2>&1" | grep "average rate" | tail -1 || true)
    if [[ -n "$HZ" ]]; then
        RATE=$(echo "$HZ" | grep -oP '[\d.]+' | head -1)
        pass "/map_ros/depth publishing at ${RATE} Hz"
    else
        fail "/map_ros/depth not publishing — adapter not receiving depth images"
    fi
fi

if should_run; then
    # Test: /map_ros/depth/camera_info publishing
    HZ=$(in_container falcon "${FALCON_SRC} && timeout 4 rostopic hz /map_ros/depth/camera_info 2>&1" | grep "average rate" | tail -1 || true)
    if [[ -n "$HZ" ]]; then
        RATE=$(echo "$HZ" | grep -oP '[\d.]+' | head -1)
        pass "/map_ros/depth/camera_info publishing at ${RATE} Hz"
    else
        fail "/map_ros/depth/camera_info not publishing — THIS IS LIKELY YOUR PROBLEM"
        echo -e "       ${RED}Without CameraInfo, FALCON cannot back-project depth into 3D${NC}"
    fi
fi

if should_run; then
    # Test: /map_ros/pose publishing
    HZ=$(in_container falcon "${FALCON_SRC} && timeout 4 rostopic hz /map_ros/pose 2>&1" | grep "average rate" | tail -1 || true)
    if [[ -n "$HZ" ]]; then
        RATE=$(echo "$HZ" | grep -oP '[\d.]+' | head -1)
        pass "/map_ros/pose publishing at ${RATE} Hz"
    else
        fail "/map_ros/pose not publishing"
    fi
fi

# ══════════════════════════════════════════════════════════════
# SECTION 5: FALCON ADAPTER — output data quality
# ══════════════════════════════════════════════════════════════
header "5. FALCON ADAPTER — data quality"

if should_run; then
    # Test: depth encoding on FALCON side
    ENC=$(in_container falcon "${FALCON_SRC} && timeout 3 rostopic echo /map_ros/depth/encoding -n1 2>&1" | head -1 || true)
    if [[ -n "$ENC" ]]; then
        info "FALCON-side depth encoding: '$ENC'"
        if echo "$ENC" | grep -q "32FC1"; then
            pass "Depth is 32FC1 — correct"
        elif echo "$ENC" | grep -q "16UC1"; then
            fail "Depth is 16UC1 on FALCON side — needs conversion to 32FC1 in adapter"
        else
            warn "Unexpected encoding: $ENC"
        fi
    else
        warn "Could not read FALCON-side depth encoding"
    fi
fi

if should_run; then
    # Test: depth frame_id is correct
    FRAME=$(in_container falcon "${FALCON_SRC} && timeout 3 rostopic echo /map_ros/depth/header/frame_id -n1 2>&1" | head -1 || true)
    if [[ -n "$FRAME" ]]; then
        info "Depth frame_id: '$FRAME'"
        if echo "$FRAME" | grep -q "camera"; then
            pass "Depth frame_id is 'camera' — correct"
        else
            warn "Depth frame_id is '$FRAME' — expected 'camera'"
        fi
    else
        warn "Could not read depth frame_id"
    fi
fi

if should_run; then
    # Test: camera_info frame_id matches depth
    CI_FRAME=$(in_container falcon "${FALCON_SRC} && timeout 3 rostopic echo /map_ros/depth/camera_info/header/frame_id -n1 2>&1" | head -1 || true)
    if [[ -n "$CI_FRAME" ]]; then
        info "CameraInfo frame_id: '$CI_FRAME'"
        if [[ "$CI_FRAME" == "$FRAME" ]] || echo "$CI_FRAME" | grep -q "camera"; then
            pass "CameraInfo frame_id matches depth frame_id"
        else
            warn "CameraInfo frame_id '$CI_FRAME' doesn't match depth frame_id '$FRAME'"
        fi
    else
        warn "Could not read camera_info frame_id"
    fi
fi

if should_run; then
    # Test: camera_info K matrix has non-zero focal length
    K_CHECK=$(in_container falcon "${FALCON_SRC} && python3 -c \"
import rospy
from sensor_msgs.msg import CameraInfo
rospy.init_node('k_check', anonymous=True)
msg = rospy.wait_for_message('/map_ros/depth/camera_info', CameraInfo, timeout=5.0)
fx, fy = msg.K[0], msg.K[4]
cx, cy = msg.K[2], msg.K[5]
w, h = msg.width, msg.height
print(f'fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} w={w} h={h}')
\" 2>&1" | grep "fx=" || true)
    if [[ -n "$K_CHECK" ]]; then
        info "CameraInfo intrinsics: $K_CHECK"
        FX=$(echo "$K_CHECK" | grep -oP 'fx=[\d.]+' | cut -d= -f2)
        if [[ -n "$FX" ]] && (( $(echo "$FX > 0" | bc -l 2>/dev/null || echo 0) )); then
            pass "CameraInfo has valid focal length (fx=$FX)"
        else
            fail "CameraInfo fx=0 — intrinsics are empty, FALCON cannot back-project"
        fi
    else
        fail "Could not read CameraInfo intrinsics — topic may not exist"
    fi
fi

if should_run; then
    # Test: depth data arriving at FALCON is not all zeros
    DEPTH_F=$(in_container falcon "${FALCON_SRC} && python3 -c \"
import rospy
from sensor_msgs.msg import Image
import numpy as np
rospy.init_node('depth_val_check', anonymous=True)
msg = rospy.wait_for_message('/map_ros/depth', Image, timeout=5.0)
if msg.encoding == '32FC1':
    arr = np.frombuffer(msg.data, dtype=np.float32)
elif msg.encoding == '16UC1':
    arr = np.frombuffer(msg.data, dtype=np.uint16)
else:
    arr = np.frombuffer(msg.data, dtype=np.uint8)
valid = arr[(arr > 0) & np.isfinite(arr)]
total = len(arr)
nvalid = len(valid)
nan_count = np.count_nonzero(np.isnan(arr.astype(np.float32)))
inf_count = np.count_nonzero(np.isinf(arr.astype(np.float32)))
zero_count = np.count_nonzero(arr == 0)
if nvalid > 0:
    print(f'VALID px={nvalid}/{total} min={valid.min():.3f} max={valid.max():.3f} mean={valid.mean():.3f} zeros={zero_count} nan={nan_count} inf={inf_count}')
else:
    print(f'ALL_INVALID total={total} zeros={zero_count} nan={nan_count} inf={inf_count}')
\" 2>&1" | tail -1)
    if echo "$DEPTH_F" | grep -q "VALID"; then
        pass "FALCON-side depth has valid data: $DEPTH_F"
    elif echo "$DEPTH_F" | grep -q "ALL_INVALID"; then
        fail "FALCON-side depth is ALL INVALID: $DEPTH_F"
    else
        warn "FALCON depth data check: $DEPTH_F"
    fi
fi

if should_run; then
    # Test: timestamp sync — depth and pose stamps are close
    SYNC=$(in_container falcon "${FALCON_SRC} && python3 -c \"
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
rospy.init_node('sync_check', anonymous=True)
depth_t = None
pose_t = None
def dcb(m): global depth_t; depth_t = m.header.stamp
def pcb(m): global pose_t; pose_t = m.header.stamp
rospy.Subscriber('/map_ros/depth', Image, dcb)
rospy.Subscriber('/map_ros/pose', PoseStamped, pcb)
rospy.sleep(2.0)
if depth_t and pose_t:
    diff = abs((depth_t - pose_t).to_sec())
    print(f'SYNC diff={diff:.4f}s depth_t={depth_t.to_sec():.3f} pose_t={pose_t.to_sec():.3f}')
elif depth_t is None:
    print('NO_DEPTH')
elif pose_t is None:
    print('NO_POSE')
\" 2>&1" | grep -E "SYNC|NO_" | tail -1)
    if echo "$SYNC" | grep -q "SYNC"; then
        DIFF=$(echo "$SYNC" | grep -oP 'diff=[\d.]+' | cut -d= -f2)
        info "Depth-pose timestamp difference: ${DIFF}s"
        if (( $(echo "$DIFF < 0.1" | bc -l 2>/dev/null || echo 0) )); then
            pass "Timestamps are synchronized (diff=${DIFF}s < 0.1s)"
        else
            warn "Timestamps differ by ${DIFF}s — FALCON may reject frames (check timestamp_tolerance)"
        fi
    elif echo "$SYNC" | grep -q "NO_DEPTH"; then
        fail "No depth messages during sync check"
    elif echo "$SYNC" | grep -q "NO_POSE"; then
        fail "No pose messages during sync check"
    else
        warn "Sync check inconclusive: $SYNC"
    fi
fi

# ══════════════════════════════════════════════════════════════
# SECTION 6: TF TREE
# ══════════════════════════════════════════════════════════════
header "6. TF FRAMES"

if should_run; then
    # Test: world -> body transform exists
    TF_WB=$(in_container falcon "${FALCON_SRC} && timeout 3 rosrun tf tf_echo world body 2>&1" | grep "Translation" | head -1 || true)
    if [[ -n "$TF_WB" ]]; then
        pass "TF world -> body exists: $TF_WB"
    else
        fail "TF world -> body NOT found"
    fi
fi

if should_run; then
    # Test: body -> camera transform exists
    TF_BC=$(in_container falcon "${FALCON_SRC} && timeout 3 rosrun tf tf_echo body camera 2>&1" | grep "Translation" | head -1 || true)
    if [[ -n "$TF_BC" ]]; then
        pass "TF body -> camera exists: $TF_BC"
    else
        fail "TF body -> camera NOT found"
    fi
fi

# ══════════════════════════════════════════════════════════════
# SECTION 7: FALCON MAPPING PIPELINE
# ══════════════════════════════════════════════════════════════
header "7. FALCON MAPPING — is voxel_mapping processing depth?"

if should_run; then
    # Test: voxel_mapping node is running
    NODES=$(in_container falcon "${FALCON_SRC} && rosnode list 2>/dev/null" || true)
    if echo "$NODES" | grep -qi "voxel_mapping\|map_ros"; then
        pass "voxel_mapping node is running"
    else
        warn "Could not confirm voxel_mapping node — check rosnode list"
        echo "       Nodes found:"
        echo "$NODES" | sed 's/^/       /'
    fi
fi

if should_run; then
    # Test: what topics does FALCON subscribe to for depth?
    # Check what voxel_mapping actually subscribes to
    VSUBS=$(in_container falcon "${FALCON_SRC} && rosnode info \$(rosnode list 2>/dev/null | grep -i 'voxel_mapping\|map_ros' | head -1) 2>/dev/null" | grep -A50 "Subscriptions:" | grep -B0 -E "^\s*\*" | head -20 || true)
    if [[ -n "$VSUBS" ]]; then
        info "voxel_mapping subscriptions:"
        echo "$VSUBS" | sed 's/^/       /'
    else
        warn "Could not list voxel_mapping subscriptions"
    fi
fi

if should_run; then
    # Test: depth_pointcloud being produced (means depth IS being processed)
    HZ=$(in_container falcon "${FALCON_SRC} && timeout 6 rostopic hz /voxel_mapping/depth_pointcloud 2>&1" | grep "average rate" | tail -1 || true)
    if [[ -n "$HZ" ]]; then
        RATE=$(echo "$HZ" | grep -oP '[\d.]+' | head -1)
        pass "/voxel_mapping/depth_pointcloud at ${RATE} Hz — mapping IS processing depth"
    else
        fail "/voxel_mapping/depth_pointcloud has 0 Hz — mapping is NOT processing depth"
        echo -e "       ${RED}This confirms depth is arriving but FALCON is discarding it${NC}"
        echo -e "       ${RED}Check: CameraInfo, encoding, timestamp_tolerance, intrinsics${NC}"
    fi
fi

if should_run; then
    # Test: occupancy grid being published
    HZ=$(in_container falcon "${FALCON_SRC} && timeout 6 rostopic hz /voxel_mapping/occupancy_grid_map 2>&1" | grep "average rate" | tail -1 || true)
    if [[ -n "$HZ" ]]; then
        RATE=$(echo "$HZ" | grep -oP '[\d.]+' | head -1)
        pass "/voxel_mapping/occupancy_grid_map at ${RATE} Hz"
    else
        info "/voxel_mapping/occupancy_grid_map has 0 Hz (may only publish on change)"
    fi
fi

# ══════════════════════════════════════════════════════════════
# SECTION 8: FALCON EXPLORATION PIPELINE
# ══════════════════════════════════════════════════════════════
header "8. FALCON EXPLORATION — planner state"

if should_run; then
    # Test: FSM state
    FSM=$(in_container falcon "${FALCON_SRC} && timeout 3 rostopic echo /exploration_manager/state -n1 2>&1" | head -3 || true)
    if [[ -n "$FSM" ]]; then
        info "FSM state: $FSM"
    else
        info "Could not read FSM state (topic may not exist or planner not started)"
    fi
fi

if should_run; then
    # Test: frontiers being found
    HZ=$(in_container falcon "${FALCON_SRC} && timeout 6 rostopic hz /planning_vis/frontier_pcl 2>&1" | grep "average rate" | tail -1 || true)
    if [[ -n "$HZ" ]]; then
        RATE=$(echo "$HZ" | grep -oP '[\d.]+' | head -1)
        pass "Frontier detection at ${RATE} Hz"
    else
        info "No frontiers being published (expected if map is not building)"
    fi
fi

# ══════════════════════════════════════════════════════════════
# SECTION 9: CROSS-CHECK — topic subscriber counts
# ══════════════════════════════════════════════════════════════
header "9. SUBSCRIBER COUNTS — are topics connected end-to-end?"

if should_run; then
    # Check that FALCON's depth topic has subscribers (voxel_mapping should subscribe)
    DEPTH_INFO=$(in_container falcon "${FALCON_SRC} && rostopic info /map_ros/depth 2>/dev/null" || true)
    PUB_COUNT=$(echo "$DEPTH_INFO" | grep -c "falcon_adapter\|adapter" || echo 0)
    SUB_COUNT=$(echo "$DEPTH_INFO" | grep -A50 "Subscribers:" | grep -c "\*" || echo 0)
    info "/map_ros/depth — publishers: ~${PUB_COUNT}, subscribers: ~${SUB_COUNT}"
    if [[ "$SUB_COUNT" -gt 0 ]]; then
        pass "/map_ros/depth has subscribers (voxel_mapping is listening)"
    else
        fail "/map_ros/depth has NO subscribers — voxel_mapping may expect a different topic name"
    fi
fi

if should_run; then
    # Check camera_info subscriber count
    CI_INFO=$(in_container falcon "${FALCON_SRC} && rostopic info /map_ros/depth/camera_info 2>/dev/null" || true)
    CI_SUBS=$(echo "$CI_INFO" | grep -A50 "Subscribers:" | grep -c "\*" || echo 0)
    info "/map_ros/depth/camera_info — subscribers: ~${CI_SUBS}"
    if [[ "$CI_SUBS" -gt 0 ]]; then
        pass "/map_ros/depth/camera_info has subscribers"
    else
        warn "/map_ros/depth/camera_info has NO subscribers — FALCON may not use this topic"
        echo -e "       ${YELLOW}If FALCON reads intrinsics from launch params (fx/fy/cx/cy) instead of${NC}"
        echo -e "       ${YELLOW}CameraInfo topic, this is OK — but check that launch params are correct${NC}"
    fi
fi

# ══════════════════════════════════════════════════════════════
# SECTION 10: LAUNCH PARAM INTRINSICS CHECK
# ══════════════════════════════════════════════════════════════
header "10. LAUNCH PARAMS — camera intrinsics on parameter server"

if should_run; then
    # Check if FALCON reads intrinsics from rosparam rather than CameraInfo
    FX=$(in_container falcon "${FALCON_SRC} && rosparam get /voxel_mapping/fx 2>/dev/null" || echo "NOT_SET")
    FY=$(in_container falcon "${FALCON_SRC} && rosparam get /voxel_mapping/fy 2>/dev/null" || echo "NOT_SET")
    CX=$(in_container falcon "${FALCON_SRC} && rosparam get /voxel_mapping/cx 2>/dev/null" || echo "NOT_SET")
    CY=$(in_container falcon "${FALCON_SRC} && rosparam get /voxel_mapping/cy 2>/dev/null" || echo "NOT_SET")
    CW=$(in_container falcon "${FALCON_SRC} && rosparam get /voxel_mapping/cam_width 2>/dev/null" || echo "NOT_SET")
    CH=$(in_container falcon "${FALCON_SRC} && rosparam get /voxel_mapping/cam_height 2>/dev/null" || echo "NOT_SET")
    DSF=$(in_container falcon "${FALCON_SRC} && rosparam get /voxel_mapping/depth_scaling_factor 2>/dev/null" || echo "NOT_SET")

    info "Rosparam intrinsics: fx=$FX fy=$FY cx=$CX cy=$CY cam=${CW}x${CH} depth_scale=$DSF"

    if [[ "$FX" != "NOT_SET" && "$FX" != "0" && "$FX" != "0.0" ]]; then
        pass "FALCON has intrinsics on param server (fx=$FX) — may not need CameraInfo topic"
    else
        warn "No intrinsics on param server — FALCON must get them from CameraInfo topic"
    fi
fi

if should_run; then
    # Check timestamp_tolerance
    TT=$(in_container falcon "${FALCON_SRC} && rosparam get /transformer/timestamp_tolerance 2>/dev/null" || echo "NOT_SET")
    info "timestamp_tolerance = $TT"
    if [[ "$TT" == "NOT_SET" || "$TT" == "0.001" ]]; then
        warn "timestamp_tolerance is default (0.001s) — too tight for bridged topics, set to 0.05"
    else
        pass "timestamp_tolerance = $TT"
    fi
fi

if should_run; then
    # Check pose_topic_type
    PTT=$(in_container falcon "${FALCON_SRC} && rosparam get /transformer/pose_topic_type 2>/dev/null" || echo "NOT_SET")
    info "pose_topic_type = $PTT"
    if [[ "$PTT" == "pose" ]]; then
        pass "pose_topic_type = pose — matches adapter's PoseStamped output"
    elif [[ "$PTT" == "NOT_SET" ]]; then
        warn "pose_topic_type not set — may default to wrong type"
    else
        warn "pose_topic_type = $PTT — adapter publishes PoseStamped, expected 'pose'"
    fi
fi

# ══════════════════════════════════════════════════════════════
# SUMMARY
# ══════════════════════════════════════════════════════════════
echo ""
echo "════════════════════════════════════════════════════════"
echo "  SUMMARY: ${PASS} passed, ${FAIL} failed, ${WARN} warnings, ${INFO} info"
echo "════════════════════════════════════════════════════════"

if [[ $FAIL -gt 0 ]]; then
    echo -e "\n${RED}${BOLD}Failed tests:${NC}"
    for r in "${RESULTS[@]}"; do
        if [[ "$r" == FAIL* ]]; then
            echo -e "  ${RED}✗${NC} ${r#FAIL: }"
        fi
    done
fi

if [[ $WARN -gt 0 ]]; then
    echo -e "\n${YELLOW}${BOLD}Warnings:${NC}"
    for r in "${RESULTS[@]}"; do
        if [[ "$r" == WARN* ]]; then
            echo -e "  ${YELLOW}!${NC} ${r#WARN: }"
        fi
    done
fi

echo ""
if [[ $FAIL -eq 0 ]]; then
    echo -e "${GREEN}${BOLD}All critical tests passed.${NC}"
    if [[ $WARN -gt 0 ]]; then
        echo -e "${YELLOW}Review warnings above.${NC}"
    fi
else
    echo -e "${RED}${BOLD}Fix the failures above and re-run: bash diagnose.sh${NC}"
fi
echo ""