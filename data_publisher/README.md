# XTEND Office Replay — full Jetson graph (ROS 2 Humble, default DDS)

Reproduces the office Jetson pipeline (everything **except FALCON and its bridge**)
so you can connect your FALCON + ROS1 bridge to it and catch any incompatibility
(QoS, message type, encoding, frame_id, timestamp, latching, missing topic).

Matches the Jetson exactly: **ROS 2 Humble, default middleware (Fast DDS),
`ROS_DOMAIN_ID=5`** — no middleware override, no router, no extra config.

## What runs, and how faithfully

Your stored data is the pipeline's **outputs** (depth `.npy` + pose trajectory JSON),
not its **inputs** (no RGB). So:

Run as the REAL office code (pure ROS, no drone/GPU needed):
- **node 3** `xtend_twist_to_cmd_nav.py` — `/cmd_vel` → `/xtend/cmd_nav` (verbatim)
- **node 4** `demo_mode_manager.py` — the state machine (your code, `DemoMode` inlined)
- **node 8** static TF `odom → xtend_camera` (same args as the launch)

Replayed from the stored data (these nodes need RGB + TensorRT/CUDA + `sparx_agency`
and cannot be re-run from output-only data — node 1 RGB grab, node 2 DA3 depth,
node 6 optical-flow velocity, node 7 integrator). Their bridge-visible outputs are
reproduced with identical type / QoS / frame / stamp / rate:
- `/xtend/depth_m`, `/flow_depth/pose_est`, `/xtend/camera_info`, `/xtend/bearing`

Cannot be reproduced from your data (flagged honestly):
- `/xtend/rgb` — no RGB is stored. (Record RGB if FALCON needs it.)
- `/xtend/local_telemetry` — custom `sparx_agency` message; no definition/data.

## Topics seen by FALCON's bridge

| topic | type | dir | QoS |
|---|---|---|---|
| `/xtend/depth_m` | sensor_msgs/Image (16UC1 mm) | out | best_effort, volatile, depth=1 |
| `/flow_depth/pose_est` | geometry_msgs/PoseStamped | out | best_effort, volatile, depth=5 |
| `/xtend/camera_info` | sensor_msgs/CameraInfo | out | best_effort, volatile, depth=5 |
| `/xtend/bearing` | std_msgs/Float32 | out | reliable, volatile, depth=10 |
| `/xtend/demo_mode` | std_msgs/String | out | reliable, **transient_local** (latched) |
| `/xtend/cmd_nav` | std_msgs/String | out | reliable, depth=10 |
| `/xtend/demo_mode_request` | std_msgs/String | in | reliable, depth=10 |
| `/cmd_vel` | geometry_msgs/Twist | in | reliable, depth=10 |
| `/xtend/reset_odom` | std_msgs/Empty | out | reliable, depth=10 |
| TF `odom → xtend_camera` | tf2 static | out | — |

`/xtend/depth_m` and `/flow_depth/pose_est` for one frame share the **same
`header.stamp`** (the original capture time); pose_est is delivered slightly after
depth — exactly like the real pipeline.

## Files
- `data_publisher.py` — replays depth/pose/camera_info/bearing
- `demo_mode_manager.py` — state manager
- `xtend_twist_to_cmd_nav.py` — node 3, verbatim
- `Dockerfile`, `entrypoint.sh` — Humble, default DDS, launches the whole graph
- `run_publisher.sh` — build + run; **set parameters here**

## Data layout (mounted at /data)
```
<take>/depth_npy/*.npy              depth in METERS (float)
estimated_trajectory_<take>.json    [{"image": "...", "pose": {x,y,z,yaw}}, ...]
```
Default mount `~/Desktop`; change `DATA_DIR` in `run_publisher.sh`.

## Build & run
```bash
./run_publisher.sh        # builds if needed, runs the whole graph; Ctrl-C to stop
```

## Parameters (top of `run_publisher.sh`)
- `PUBLISH_RATE_HZ` (office = 10), `POSE_DELAY_MEAN`, `POSE_DELAY_STD`
- `DEPTH_ENCODING` — `16UC1` (mm, office default) or `32FC1` (meters)
- `CAMERA_INFO_YAML` — host path to the office calibration YAML; empty disables it
- `INITIAL_MODE`, `ROS_DOMAIN_ID` (must be 5), `DEPTH_DIR`, `JSON_PATH`

## Drive state / commands (what FALCON does)
```bash
ros2 topic pub --once /xtend/demo_mode_request std_msgs/msg/String "{data: turning}"
ros2 topic echo /xtend/demo_mode          # latched: current mode arrives immediately
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
ros2 topic echo /xtend/cmd_nav            # see the converted command
```

## Verify (same domain)
```bash
export ROS_DOMAIN_ID=5
ros2 topic list
ros2 topic hz /xtend/depth_m
ros2 topic hz /flow_depth/pose_est
```