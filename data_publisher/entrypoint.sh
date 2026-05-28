#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash

pids=()
cleanup() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

# --- Office Jetson nodes that run as the REAL code (no drone / GPU needed) ---

# node 8: static TF  odom -> xtend_camera  (same args as the office launch)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom xtend_camera \
    >/tmp/static_tf.log 2>&1 & pids+=($!)

# node 4: demo-mode state manager (FALCON drives modes via /xtend/demo_mode_request)
python3 /app/demo_mode_manager.py --initial-mode "${INITIAL_MODE:-idle}" & pids+=($!)

# node 3: Twist -> /xtend/cmd_nav converter (verbatim office code)
python3 /app/xtend_twist_to_cmd_nav.py & pids+=($!)

# --- Outputs of nodes 1/2/6/7 (depth model + flow + integrator), replayed ---
#     from the stored data: /xtend/depth_m, /flow_depth/pose_est,
#     /xtend/camera_info, /xtend/bearing
echo "[entrypoint] launching data_publisher.py $*"
exec python3 /app/data_publisher.py "$@"