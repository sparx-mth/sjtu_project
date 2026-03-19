# sjtu_drone + AWS RoboMaker Hospital World (ROS2 Humble)

This guide explains how to run the **SJTU Drone simulation** with the **AWS RoboMaker Hospital world** on ROS2 Humble using Docker.

---

## Setup Steps


### Clone SJTU drone repo
```bash
git clone https://github.com/sparx-mth/sjtu_project
```

### Build Docker image
```bash
cd /sjtu_project/sjtu_drone
docker build -t sjtu_drone_clean:humble_ros2 .
```


Run:
```bash
cd /sjtu_project/sjtu_drone
chmod +x run.sh
./run.sh --no-map hospital.world
```

---

## 📂 Folder Structure
```
/sjtu_project
 ├─ sjtu_drone/
 │   ├─ Dockerfile
 │   ├─ run.sh
 │   └─ hospital.world
 └─ aws-robomaker-hospital-world/
     └─ worlds/
         └─ hospital.world
     └─ models/
         └─ Apriltag36_11_00014
         └─ Apriltag36_11_00015


---
```

## 🆘 Notes
- GPU recommended: install NVIDIA container toolkit if available
- First launch may take time to download Gazebo models
- If paths fail, ensure run.sh mounts current folder into the container

---

🎉 **Simulation ready! Fly the drone inside the hospital world.**


# FALCON Exploration in Gazebo

Autonomous 3D exploration of a Gazebo world using the [FALCON](https://github.com/HKUST-Aerial-Robotics/FALCON) planner and [sjtu_drone](https://github.com/NovoG93/sjtu_drone) quadrotor.

FALCON (ROS1 Noetic) plans where to fly. The sjtu_drone in Gazebo (ROS2 Humble) executes the flight. A `ros1_bridge` connects the two.

---

## Architecture

```
┌─────────────────────┐    ┌──────────────────┐    ┌───────────────────────────┐
│  GAZEBO SIM         │    │  ROS1 <-> ROS2   │    │  FALCON                   │
│  (ROS2 Humble)      │<-->│  BRIDGE          │<-->│  (ROS1 Noetic, CPU only)  │
│                     │    │                  │    │                           │
│  sjtu_drone         │    │  Noetic + Foxy   │    │  exploration planner      │
│  hospital.world     │    │  dynamic_bridge  │    │  + falcon_adapter node    │
│  depth camera       │    │                  │    │  + RViz                   │
│                     │    │                  │    │                           │
│  --net=host         │    │  --net=host      │    │  --net=host               │
│  CycloneDDS         │    │  CycloneDDS      │    │  GPU for RViz OpenGL only │
│  Domain ID = 20     │    │  Domain ID = 20  │    │  CUDA disabled (CPU plan) │
└─────────────────────┘    └──────────────────┘    └───────────────────────────┘
```

### What Each Container Does

| Container | Image | Purpose |
|---|---|---|
| **sjtu_drone** | Your existing sim image | Runs Gazebo with the hospital world and a quadrotor. Publishes depth, pose, IMU. Listens to `cmd_vel`. |
| **roscore** | `ros1_bridge:noetic-foxy` | ROS1 master. Required before the bridge or FALCON can start. |
| **ros1_bridge** | `ros1_bridge:noetic-foxy` | Translates all ROS1 topics to ROS2 and vice versa via CycloneDDS on shared host network. |
| **falcon** | `falcon-ros:noetic` | Runs FALCON's exploration planner, RViz, and the `falcon_adapter` node. |

### Data Flow

```
Gazebo drone                    Bridge              FALCON
───────────                     ──────              ──────
gt_pose (Pose)        ──ROS2──> ──ROS1──> adapter ──> /odom_world (Odometry)
front_depth/depth/image_raw  ─ROS2─> ──ROS1──> adapter ──> /map_ros/depth (Image)
                                                      /map_ros/pose (PoseStamped)

                                          FALCON planner
                                              │
                                          /planning/pos_cmd
                                              │
cmd_vel (Twist)       <─ROS2── <─ROS1── adapter (PD controller + world→body transform)
takeoff (Empty)       <─ROS2── <─ROS1── adapter (on startup, with retry)
```

The `falcon_adapter` node (Python, runs inside the FALCON container) does four things:
1. Converts the drone's `gt_pose` into the `Odometry` + `PoseStamped` + TF that FALCON expects
2. Re-stamps depth images with the correct frame
3. Converts FALCON's position commands into velocity commands via a PD controller
4. Transforms world-frame velocities into body-frame before publishing to `cmd_vel`

---

## Project Structure

```
sjtu_project/
├── launch_all.sh               # One script to start everything
├── README.md                   # This file
│
├── sjtu_drone/                 # Gazebo sim (ROS2) — your existing code
│   └── run.sh
│
├── ros_bridge_docker/          # ROS1↔ROS2 bridge
│   ├── Dockerfile
│   ├── entrypoint.sh
│   ├── fastdds_localhost.xml
│   ├── run_bridge.sh
│   └── verify_bridge.sh
│
└── falcon_docker/              # FALCON planner + adapter
    ├── Dockerfile              # Builds FALCON + adapter in one image
    ├── docker-compose.yml
    ├── entrypoint.sh
    ├── run.sh
    ├── hospital.yaml           # Map config for the hospital world
    └── adapter/                # The glue code (catkin package)
        ├── CMakeLists.txt
        ├── package.xml
        ├── scripts/
        │   └── falcon_adapter.py
        └── launch/
            └── gazebo_exploration.launch
```

---

## Prerequisites

On the host machine:

```bash
# Docker
sudo apt install docker.io
sudo usermod -aG docker $USER   # log out & back in

# NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Set `CUDA_ARCH` in `falcon_docker/Dockerfile` (or `docker-compose.yml`) for your GPU:

| GPU | CUDA_ARCH |
|---|---|
| RTX 50xx (Blackwell) | `120` |
| RTX 40xx (Ada) | `89` |
| RTX 30xx (Ampere) | `86` |
| RTX 20xx (Turing) | `75` |

---

## One-Time Build

```bash
# 1. Bridge image (~10 min)
cd ros_bridge_docker
docker build -t ros1_bridge:noetic-foxy .

# 2. FALCON image (~30-60 min, Open3D compiles from source)
cd ../falcon_docker
docker build --build-arg CUDA_ARCH=120 -t falcon-ros:noetic .
```

> **Already built both images?** No rebuild needed.
> The fixed `falcon_adapter.py`, `gazebo_exploration.launch`, and `hospital.yaml`
> are volume-mounted at runtime. The bridge entrypoint is bypassed at runtime.
> CycloneDDS is already installed in the bridge image.

---

## Running the Exploration

You need **4 terminals**. Order matters — each step must complete before the next.

> **DDS critical note:** The sim (Humble) and bridge (Foxy) must both use
> **CycloneDDS**. FastRTPS versions between Humble and Foxy are incompatible
> and cannot discover each other. CycloneDDS works cross-version.

### Terminal 1 — Gazebo Simulation

The sim's `run.sh` must use CycloneDDS. In `sjtu_drone/run.sh`.
Then launch:

```bash
cd sjtu_drone
./run.sh --no-map hospital.world
```

Wait until you see `Gazebo is running` and `The drone plugin finished loading!`.

---

### Terminal 2 — roscore

```bash
docker run -d --rm --net=host --name=roscore \
  --entrypoint bash ros1_bridge:noetic-foxy -c \
  "source /opt/ros/noetic/setup.bash && roscore"
```

Wait 3 seconds.

---

### Terminal 3 — Bridge

The bridge bypasses its built-in entrypoint to avoid localhost DDS restrictions.
Both sim and bridge must use CycloneDDS on Domain ID 20.

```bash
docker run -it --rm --net=host --name=ros1_bridge \
  -e ROS_MASTER_URI="http://localhost:11311" \
  --entrypoint bash \
  ros1_bridge:noetic-foxy -c '
    source /opt/ros/noetic/setup.bash
    source /opt/ros/foxy/setup.bash
    source /bridge_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=20
    echo "Bridge starting (CycloneDDS, Domain 20)..."
    ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics --bridge-all-1to2-topics
  '
```

This should stay running. You should see `created 2to1 bridge for topic ...` messages.

**Verify the bridge can see the sim (new tab):**

```bash
docker exec ros1_bridge bash -c \
  "source /opt/ros/foxy/setup.bash && \
   source /bridge_ws/install/setup.bash && \
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
   ROS_DOMAIN_ID=20 ros2 topic list 2>/dev/null | head -20"
```

** you must see `/simple_drone/*` topics before continuing.**
If you only see `/parameter_events`, `/rosout`, `/rosout_agg`, the bridge
cannot discover the sim. Check that both use CycloneDDS and Domain ID 20.

---

### Terminal 4 — FALCON (RViz + Adapter)

```bash
cd falcon_docker
xhost +local:docker 2>/dev/null || true
```

```bash
docker run -it --rm \
    --name falcon \
    --gpus all \
    --env DISPLAY="${DISPLAY}" \
    --env QT_X11_NO_MITSHM=1 \
    --env CUDA_VISIBLE_DEVICES="" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume $(pwd)/adapter/scripts/falcon_adapter.py:/catkin_ws/src/falcon_adapter/scripts/falcon_adapter.py \
    --volume $(pwd)/adapter/launch/gazebo_exploration.launch:/catkin_ws/src/falcon_adapter/launch/gazebo_exploration.launch \
    --volume $(pwd)/hospital.yaml:/catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/map/hospital.yaml \
    --network host \
    falcon-ros:noetic
```

Notes on the flags:
- `--gpus all` — RViz needs OpenGL for rendering
- `CUDA_VISIBLE_DEVICES=""` — blocks CUDA so FALCON planner runs on CPU (GPU stays free for Gazebo)
- Three volume mounts — the fixed adapter code + hospital map config (no rebuild needed)

**Inside the container:**

```bash
roslaunch exploration_manager rviz.launch
```

```bash
docker exec -it falcon bash
roslaunch falcon_adapter gazebo_exploration.launch 
```

The first command (backgrounded with `&`) starts the exploration planner, trajectory
server, and adapter. The second opens the RViz window.

You should see:
- `[Adapter] Sending takeoff...` (retries until bridge is ready)
- `[Adapter] Drone is airborne` once the drone lifts off
- RViz window opens
- FALCON begins autonomous exploration and the map builds in RViz

> **Manual takeoff:** If you prefer to take off the drone yourself before
> starting FALCON, add `auto_takeoff:=false` to the roslaunch command.

> **Note:** Do NOT run `exploration_manager exploration.launch` separately —
> `gazebo_exploration.launch` already includes the exploration planner node.
---

### Shutting Down

```bash
# Stop in reverse order:
docker stop falcon
docker stop ros1_bridge
docker stop roscore
docker stop sjtu_drone_hospital
# Gazebo: Ctrl+C in Terminal 1
```

---

## Verification

After Terminals 1–4 are running, check connectivity in a new tab:

```bash
# 1. All containers running?
docker ps --format 'table {{.Names}}\t{{.Status}}' | grep -E 'sjtu_drone|roscore|ros1_bridge|falcon'

# 2. Bridge sees sim topics?
docker exec ros1_bridge bash -c \
  "source /opt/ros/foxy/setup.bash && \
   source /bridge_ws/install/setup.bash && \
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
   ROS_DOMAIN_ID=20 ros2 topic list 2>/dev/null | grep simple_drone | head -10"

# 3. Odom flowing inside FALCON? (should show ~30 Hz)
docker exec falcon bash -c \
  "source /catkin_ws/devel/setup.bash && timeout 3 rostopic hz /odom_world"

# 4. Depth flowing? (should show ~15 Hz)
docker exec falcon bash -c \
  "source /catkin_ws/devel/setup.bash && timeout 3 rostopic hz /map_ros/depth"

# 5. cmd_vel being sent? (non-zero after exploration starts)
docker exec falcon bash -c \
  "source /catkin_ws/devel/setup.bash && rostopic echo /simple_drone/cmd_vel -n 1"
```

---

## Tuning

The adapter's PD controller gains are in `gazebo_exploration.launch`:

| Parameter | Default | Effect |
|---|---|---|
| `kp_xy` | 1.5 | Position tracking tightness (XY) |
| `kd_xy` | 0.3 | Damping (XY) |
| `kp_z` | 1.5 | Position tracking (Z) |
| `max_vel_xy` | 1.0 m/s | Max horizontal speed |
| `max_vel_z` | 0.5 m/s | Max vertical speed |

If the drone oscillates, reduce `kp_xy`. If it's sluggish, increase it.

---

## Adding the Depth Camera to the Drone SDF

The sjtu_drone needs a forward-facing depth camera. Add this sensor inside the drone's SDF model (in the link that faces forward):

```xml
<sensor type="depth_camera" name="depth_camera">
  <update_rate>15</update_rate>
  <camera>
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <visualize>true</visualize>
  <topic>depth/image_raw</topic>
</sensor>
```

The adapter subscribes to `${drone_ns}/front_depth/depth/image_raw`. Make sure the topic name matches.

---

## Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| `Connection refused` in bridge | roscore not running | Start roscore first, wait 3s |
| Bridge exits immediately | Entrypoint loads localhost DDS config | Bypass entrypoint with `--entrypoint bash` (see Terminal 3) |
| No ROS2 topics in bridge | Wrong `ROS_DOMAIN_ID` | Use `20` everywhere (match sim's `run.sh`) |
| No ROS2 topics (domain OK) | DDS middleware mismatch | Both sim and bridge must use CycloneDDS. FastRTPS versions between Foxy and Humble are incompatible |
| `selected interface "lo" is not multicast-capable` | CycloneDDS restricted to localhost | Don't set `CYCLONEDDS_URI` — let it auto-discover on the host network |
| `No pose yet` in adapter | Bridge not forwarding topics | Verify bridge sees `/simple_drone/*` topics before starting FALCON |
| `hospital.yaml not found` | Missing volume mount | Add `-v $(pwd)/hospital.yaml:/catkin_ws/src/FALCON/.../hospital.yaml` |
| RViz black / GL errors | No GPU access | Need `--gpus all` for OpenGL. `CUDA_VISIBLE_DEVICES=""` blocks only CUDA, not OpenGL |
| Drone drifts / flies wrong way | World-frame vs body-frame cmd_vel | Use the fixed `falcon_adapter.py` with `_world_to_body()` transform |
| `[FSM] No odom` in FALCON | Adapter not running or bridge down | Start adapter first; check `rostopic hz /odom_world` |
| Drone doesn't move | `cmd_vel` not bridged | Check bridge is running; ensure drone is airborne |
| FALCON crashes on start | GPU arch mismatch | Check `CUDA_ARCH` matches your GPU |
| Gazebo stutters when FALCON runs | GPU contention | `CUDA_VISIBLE_DEVICES=""` in FALCON container keeps CUDA compute off |
| Depth topic empty | No depth camera in SDF | Add depth sensor (see section above) |
| `librmw_cyclonedds_cpp.so not found` in sim | CycloneDDS not installed in sim image | Add `apt-get install ros-humble-rmw-cyclonedds-cpp` to sim's `run.sh` |