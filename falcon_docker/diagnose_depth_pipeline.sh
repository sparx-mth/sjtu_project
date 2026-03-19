#!/bin/bash
# ============================================================
# diagnose_depth_pipeline.sh
# Targeted diagnostic for WHY there's no point cloud / no map.
#
# Run on the HOST while all 4 containers are up.
# Usage: chmod +x diagnose_depth_pipeline.sh && ./diagnose_depth_pipeline.sh
# ============================================================
set -o pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
PASS=0; WARN=0; FAIL=0

pass()  { echo -e "  ${GREEN}[PASS]${NC} $1"; ((PASS++)); }
warn()  { echo -e "  ${YELLOW}[WARN]${NC} $1"; ((WARN++)); }
fail()  { echo -e "  ${RED}[FAIL]${NC} $1"; ((FAIL++)); }
header(){ echo -e "\n${CYAN}═══ $1 ═══${NC}"; }

in_falcon() { docker exec falcon bash -c "source /catkin_ws/devel/setup.bash 2>/dev/null; $1" 2>&1; }

# ==============================================================
header "1. ACTUAL DEPTH IMAGE vs CONFIGURED INTRINSICS"
# ==============================================================

echo "  Grabbing one depth message from /map_ros/depth..."
DEPTH_MSG=$(in_falcon "timeout 8 rostopic echo /map_ros/depth -n 1 --noarr 2>&1")

if echo "$DEPTH_MSG" | grep -q "width:"; then
    ACTUAL_W=$(echo "$DEPTH_MSG" | grep 'width:'  | head -1 | awk '{print $2}')
    ACTUAL_H=$(echo "$DEPTH_MSG" | grep 'height:' | head -1 | awk '{print $2}')
    ACTUAL_ENC=$(echo "$DEPTH_MSG" | grep 'encoding:' | head -1 | awk '{print $2}')
    ACTUAL_FRAME=$(echo "$DEPTH_MSG" | grep 'frame_id:' | head -1 | awk '{print $2}')
    echo "  Actual image:  ${ACTUAL_W} x ${ACTUAL_H}, encoding=${ACTUAL_ENC}, frame=${ACTUAL_FRAME}"
else
    fail "No depth message on /map_ros/depth within 8s"
    echo "  → The adapter may not be receiving depth from the bridge."
    echo "  → Check: rostopic hz /simple_drone/front_depth/depth/image_raw"
    echo ""
    echo "  Checking if raw depth exists on bridge side..."
    RAW=$(in_falcon "timeout 5 rostopic hz /simple_drone/front_depth/depth/image_raw 2>&1 | tail -1")
    echo "  → $RAW"
    if echo "$RAW" | grep -q "average rate:"; then
        fail "Raw depth IS flowing but adapter isn't republishing → adapter depth_cb issue"
    else
        fail "Raw depth NOT flowing → bridge not forwarding depth topic"
        echo "  → Common cause: The depth topic name in sjtu_drone differs from what the adapter subscribes to."
        echo "  → Run inside sim container:"
        echo "      ros2 topic list | grep depth"
        echo "  → Then update falcon_adapter.py to match the actual topic name."
    fi
    ACTUAL_W=0; ACTUAL_H=0
fi

# Now check what FALCON is configured for
echo ""
echo "  Reading FALCON's configured intrinsics from rosparam..."
CFG_W=$(in_falcon "rosparam get /voxel_mapping/cam_width 2>/dev/null" | tr -d '[:space:]')
CFG_H=$(in_falcon "rosparam get /voxel_mapping/cam_height 2>/dev/null" | tr -d '[:space:]')
CFG_FX=$(in_falcon "rosparam get /voxel_mapping/fx 2>/dev/null" | tr -d '[:space:]')
CFG_FY=$(in_falcon "rosparam get /voxel_mapping/fy 2>/dev/null" | tr -d '[:space:]')
CFG_CX=$(in_falcon "rosparam get /voxel_mapping/cx 2>/dev/null" | tr -d '[:space:]')
CFG_CY=$(in_falcon "rosparam get /voxel_mapping/cy 2>/dev/null" | tr -d '[:space:]')
CFG_SCALE=$(in_falcon "rosparam get /voxel_mapping/depth_scaling_factor 2>/dev/null" | tr -d '[:space:]')

echo "  Configured:    cam_width=${CFG_W}  cam_height=${CFG_H}"
echo "  Configured:    fx=${CFG_FX}  fy=${CFG_FY}  cx=${CFG_CX}  cy=${CFG_CY}"
echo "  Configured:    depth_scaling_factor=${CFG_SCALE}"

if [[ "$ACTUAL_W" != "0" && "$ACTUAL_H" != "0" ]]; then
    if [[ "$ACTUAL_W" != "$CFG_W" ]]; then
        fail "WIDTH MISMATCH: actual=${ACTUAL_W} vs configured=${CFG_W}"
    else
        pass "Width matches: ${ACTUAL_W}"
    fi

    if [[ "$ACTUAL_H" != "$CFG_H" ]]; then
        fail "HEIGHT MISMATCH: actual=${ACTUAL_H} vs configured=${CFG_H}"
        EXPECTED_CY=$(python3 -c "print(${ACTUAL_H}/2.0)" 2>/dev/null)
        echo -e "       ${YELLOW}→ cy should be ${EXPECTED_CY} (half of ${ACTUAL_H}), not ${CFG_CY}${NC}"
        echo -e "       ${YELLOW}→ cam_height should be ${ACTUAL_H}, not ${CFG_H}${NC}"
        echo -e "       ${RED}→ THIS ALONE CAN CAUSE ZERO POINT CLOUD${NC}"
    else
        pass "Height matches: ${ACTUAL_H}"
    fi

    # Verify encoding vs depth_scaling_factor
    if [[ "$ACTUAL_ENC" == "32FC1" && "$CFG_SCALE" == "1.0" ]]; then
        pass "Encoding 32FC1 with depth_scaling_factor=1.0 (correct)"
    elif [[ "$ACTUAL_ENC" == "16UC1" && "$CFG_SCALE" == "1000.0" ]]; then
        pass "Encoding 16UC1 with depth_scaling_factor=1000.0 (correct)"
    elif [[ "$ACTUAL_ENC" == "32FC1" && "$CFG_SCALE" != "1.0" ]]; then
        fail "Encoding is 32FC1 but depth_scaling_factor=${CFG_SCALE} (should be 1.0)"
    elif [[ "$ACTUAL_ENC" == "16UC1" && "$CFG_SCALE" != "1000.0" ]]; then
        fail "Encoding is 16UC1 but depth_scaling_factor=${CFG_SCALE} (should be 1000.0)"
    fi
fi

# ==============================================================
header "2. WHAT TOPICS DOES VOXEL_MAPPING ACTUALLY SUBSCRIBE TO?"
# ==============================================================

echo "  This reveals whether the <remap> tags actually worked."
echo ""

EXPL_NODE=$(in_falcon "rosnode list 2>/dev/null" | grep -i explor | head -1)
if [[ -n "$EXPL_NODE" ]]; then
    echo "  exploration_node = $EXPL_NODE"
    NODE_INFO=$(in_falcon "rosnode info $EXPL_NODE 2>/dev/null")

    echo ""
    echo "  ── Subscriptions ──"
    echo "$NODE_INFO" | sed -n '/Subscriptions:/,/^$/p' | head -30
    echo ""
    echo "  ── Publications ──"
    echo "$NODE_INFO" | sed -n '/Publications:/,/^$/p' | head -30

    # Check critical subscriptions
    echo ""
    echo "  ── Critical subscription checks ──"

    # Depth topic
    DEPTH_SUB=$(echo "$NODE_INFO" | sed -n '/Subscriptions:/,/^$/p' | grep -i "depth")
    if [[ -n "$DEPTH_SUB" ]]; then
        pass "exploration_node subscribes to a depth topic: $DEPTH_SUB"
        if echo "$DEPTH_SUB" | grep -q "/map_ros/depth"; then
            pass "  → It's /map_ros/depth (remap worked!)"
        else
            fail "  → NOT /map_ros/depth — remap may have failed!"
            echo -e "       ${YELLOW}→ The node subscribes to: $DEPTH_SUB${NC}"
            echo -e "       ${YELLOW}→ Fix: set the param directly instead of using <remap>${NC}"
        fi
    else
        fail "exploration_node does NOT subscribe to any depth topic!"
        echo -e "       ${YELLOW}→ Voxel mapping may not be initialized, or uses a param-based topic name.${NC}"
    fi

    # Pose topic
    POSE_SUB=$(echo "$NODE_INFO" | sed -n '/Subscriptions:/,/^$/p' | grep -i "pose\|sensor")
    if [[ -n "$POSE_SUB" ]]; then
        pass "exploration_node subscribes to a pose topic: $POSE_SUB"
    else
        warn "exploration_node doesn't seem to subscribe to a pose topic"
        echo "  → It may be using TF for pose instead (check TF section below)"
    fi

    # Odom topic
    ODOM_SUB=$(echo "$NODE_INFO" | sed -n '/Subscriptions:/,/^$/p' | grep -i "odom")
    if [[ -n "$ODOM_SUB" ]]; then
        pass "exploration_node subscribes to odom: $ODOM_SUB"
    else
        warn "exploration_node doesn't subscribe to an odom topic"
    fi
else
    fail "exploration_node not found in rosnode list"
fi

# ==============================================================
header "3. VOXEL_MAPPING PARAMETERS (depth filter & range)"
# ==============================================================

echo "  Checking all voxel_mapping params..."
VM_PARAMS=$(in_falcon "rosparam list 2>/dev/null | grep voxel_mapping")
echo "$VM_PARAMS" | head -30
echo ""

# Check key depth-related params
for p in depth_scaling_factor depth_filter_maxdist depth_filter_mindist \
         depth_filter_margin use_depth_filter k_depth_scaling_factor \
         depth_topic pose_topic cam_width cam_height fx fy cx cy \
         resolution mp_update_range; do
    VAL=$(in_falcon "rosparam get /voxel_mapping/$p 2>/dev/null")
    if [[ -n "$VAL" && "$VAL" != *"not set"* && "$VAL" != *"ERROR"* ]]; then
        echo "  /voxel_mapping/$p = $VAL"
    fi
done

# Check if there's an explicitly configured depth topic param
DEPTH_TOPIC_PARAM=$(in_falcon "rosparam get /voxel_mapping/depth_topic 2>/dev/null" | tr -d '[:space:]')
if [[ -n "$DEPTH_TOPIC_PARAM" && "$DEPTH_TOPIC_PARAM" != *"not set"* ]]; then
    echo ""
    echo -e "  ${CYAN}IMPORTANT: /voxel_mapping/depth_topic = $DEPTH_TOPIC_PARAM${NC}"
    if [[ "$DEPTH_TOPIC_PARAM" != "/map_ros/depth" ]]; then
        fail "depth_topic param does NOT point to /map_ros/depth"
        echo -e "       ${YELLOW}→ Voxel mapping subscribes via parameter, not via remap!${NC}"
        echo -e "       ${YELLOW}→ Fix: add to launch: <param name=\"/voxel_mapping/depth_topic\" value=\"/map_ros/depth\" />${NC}"
    fi
fi

POSE_TOPIC_PARAM=$(in_falcon "rosparam get /voxel_mapping/pose_topic 2>/dev/null" | tr -d '[:space:]')
if [[ -n "$POSE_TOPIC_PARAM" && "$POSE_TOPIC_PARAM" != *"not set"* ]]; then
    echo ""
    echo -e "  ${CYAN}IMPORTANT: /voxel_mapping/pose_topic = $POSE_TOPIC_PARAM${NC}"
fi

# Also check the "transformer" params
TRANS_POSE=$(in_falcon "rosparam get /transformer/sensor_pose_topic 2>/dev/null" | tr -d '[:space:]')
if [[ -n "$TRANS_POSE" && "$TRANS_POSE" != *"not set"* ]]; then
    echo -e "  ${CYAN}/transformer/sensor_pose_topic = $TRANS_POSE${NC}"
fi

# ==============================================================
header "4. DEPTH DATA SANITY CHECK (are values valid?)"
# ==============================================================

echo "  Checking if depth values are non-zero and in expected range..."
DEPTH_SAMPLE=$(in_falcon "timeout 5 python3 -c \"
import rospy, sys, numpy as np
from sensor_msgs.msg import Image

rospy.init_node('depth_check', anonymous=True)
msg = rospy.wait_for_message('/map_ros/depth', Image, timeout=5.0)

if msg.encoding == '32FC1':
    data = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
elif msg.encoding == '16UC1':
    data = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
else:
    print(f'Unknown encoding: {msg.encoding}')
    sys.exit(1)

valid = data[(data > 0.01) & (data < 100.0) & np.isfinite(data)]
total = data.size
nan_count = np.isnan(data).sum() if data.dtype == np.float32 else 0
inf_count = np.isinf(data).sum() if data.dtype == np.float32 else 0
zero_count = (data == 0).sum()

print(f'Image size: {msg.width}x{msg.height}')
print(f'Encoding: {msg.encoding}')
print(f'Total pixels: {total}')
print(f'Valid pixels (0.01-100m): {len(valid)} ({100*len(valid)/total:.1f}%)')
print(f'Zero pixels: {zero_count} ({100*zero_count/total:.1f}%)')
print(f'NaN pixels: {nan_count}')
print(f'Inf pixels: {inf_count}')
if len(valid) > 0:
    print(f'Min valid depth: {valid.min():.3f}')
    print(f'Max valid depth: {valid.max():.3f}')
    print(f'Mean valid depth: {valid.mean():.3f}')
else:
    print('NO VALID DEPTH DATA!')
\" 2>&1")

echo "$DEPTH_SAMPLE"

if echo "$DEPTH_SAMPLE" | grep -q "NO VALID DEPTH DATA"; then
    fail "Depth image has NO valid pixels → camera may not be seeing anything"
    echo -e "       ${YELLOW}→ Is the drone facing a wall too close, or is the camera clipping?${NC}"
    echo -e "       ${YELLOW}→ Check the depth camera's <clip><near> and <far> in the SDF.${NC}"
elif echo "$DEPTH_SAMPLE" | grep -q "Valid pixels"; then
    VALID_PCT=$(echo "$DEPTH_SAMPLE" | grep "Valid pixels" | grep -oP '\(([0-9.]+)%' | tr -d '(')
    if (( $(echo "${VALID_PCT:-0} > 5" | bc -l 2>/dev/null || echo 0) )); then
        pass "Depth data looks healthy (${VALID_PCT}% valid pixels)"
    else
        warn "Only ${VALID_PCT}% valid pixels — camera may have limited view"
    fi
fi

# ==============================================================
header "5. TF CHAIN: world → body → camera"
# ==============================================================

echo "  Checking if TF can resolve world → camera..."
TF_CHECK=$(in_falcon "timeout 5 python3 -c \"
import rospy, tf
rospy.init_node('tf_check', anonymous=True)
listener = tf.TransformListener()
rospy.sleep(2.0)  # wait for TF buffer to fill

frames = ['world', 'body', 'camera']
for f in frames:
    try:
        exists = listener.frameExists(f)
        print(f'Frame {f}: {\"EXISTS\" if exists else \"MISSING\"} ')
    except:
        print(f'Frame {f}: ERROR')

try:
    listener.waitForTransform('world', 'camera', rospy.Time(0), rospy.Duration(3.0))
    (trans, rot) = listener.lookupTransform('world', 'camera', rospy.Time(0))
    print(f'world→camera transform: t={[round(x,2) for x in trans]} r={[round(x,3) for x in rot]}')
    print('TF CHAIN OK')
except Exception as e:
    print(f'world→camera FAILED: {e}')
\" 2>&1")

echo "$TF_CHECK"

if echo "$TF_CHECK" | grep -q "TF CHAIN OK"; then
    pass "TF world→camera chain is working"
else
    fail "TF world→camera chain BROKEN"
    echo -e "       ${YELLOW}→ Without this, depth cannot be projected into the map${NC}"
fi

# ==============================================================
header "6. FALCON MAP OUTPUT (is anything being published?)"
# ==============================================================

echo "  Checking if voxel_mapping publishes any map/cloud topics..."
ALL_TOPICS=$(in_falcon "rostopic list 2>/dev/null")

for pattern in "voxel" "map" "cloud" "occup" "frontier" "grid"; do
    MATCHES=$(echo "$ALL_TOPICS" | grep -i "$pattern")
    if [[ -n "$MATCHES" ]]; then
        echo "  Topics matching '$pattern':"
        echo "$MATCHES" | sed 's/^/     /'
        # Check if any have data
        for t in $MATCHES; do
            HZ=$(in_falcon "timeout 3 rostopic hz $t 2>&1 | tail -1")
            RATE=$(echo "$HZ" | grep -oP 'average rate: \K[0-9.]+' || echo "0")
            if (( $(echo "$RATE > 0" | bc -l 2>/dev/null || echo 0) )); then
                pass "$t publishing at ${RATE} Hz"
            else
                warn "$t exists but no data flowing"
            fi
        done
    fi
done

# ==============================================================
header "7. ROSOUT ERRORS FROM VOXEL_MAPPING"
# ==============================================================

echo "  Checking rosout for depth/map/voxel errors..."
ERRORS=$(in_falcon "timeout 5 rostopic echo /rosout -n 100 2>&1" | \
    grep -B2 -iE 'depth|voxel|map|point.cloud|intrinsic|camera|no.data|filter|discard' | head -30)
if [[ -n "$ERRORS" ]]; then
    warn "Relevant log messages:"
    echo "$ERRORS"
else
    echo "  No depth/map errors in recent rosout (100 messages checked)"
fi

# ==============================================================
header "SUMMARY"
# ==============================================================

echo ""
echo -e "  ${GREEN}PASS: $PASS${NC}  ${YELLOW}WARN: $WARN${NC}  ${RED}FAIL: $FAIL${NC}"
echo ""

echo -e "${CYAN}═══ MOST LIKELY FIXES ═══${NC}"
echo ""
echo "1. CAMERA RESOLUTION MISMATCH:"
echo "   The SDF defines a 640x480 camera but the launch file tells FALCON 640x360."
echo "   Run this script → check section 1 for the ACTUAL image dimensions."
echo "   Then fix gazebo_exploration.launch:"
echo "     <param name=\"/voxel_mapping/cam_height\" value=\"ACTUAL_HEIGHT\" />"
echo "     <param name=\"/voxel_mapping/cy\" value=\"ACTUAL_HEIGHT / 2\" />"
echo ""
echo "2. TOPIC REMAP NOT WORKING:"
echo "   If section 2 shows exploration_node subscribes to the WRONG depth topic,"
echo "   the <remap> tag isn't reaching the internal subscriber."
echo "   Fix: Set the topic directly as a parameter in the launch file:"
echo "     <param name=\"/voxel_mapping/depth_topic\" value=\"/map_ros/depth\" />"
echo "     <param name=\"/transformer/sensor_pose_topic\" value=\"/map_ros/pose\" />"
echo ""
echo "3. DEPTH DATA EMPTY:"
echo "   If section 4 shows 0 valid pixels, the depth camera in the SDF may not"
echo "   be working. Check Gazebo → the camera should show a depth view."
echo ""