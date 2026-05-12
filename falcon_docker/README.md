# falcon_adapter — Real-Drone Runtime

Runs the FALCON exploration stack on a real drone. Pose + depth in → velocity commands out.

---

## Repository layout

```
.
├── ros_bridge_docker/        ← ROS2 ↔ ROS1 bridge container
│   ├── Dockerfile
│   ├── run_bridge.sh         ← starts the bridge
│   └── entrypoint.sh
│
└── falcon_docker/            ← FALCON ROS1 stack container
    ├── run_hospital.sh       ← starts the falcon container (any map)
    ├── office.yaml           ← map config for your environment
    ├── hospital.yaml         ← (and any other maps you have)
    └── adapter/              ← scripts + launch files mounted into the container
```

Two directories, two responsibilities. The bridge container hosts `roscore` and tunnels ROS2 → ROS1. The falcon container runs the actual stack.

---

## 1. ROS2 Bridge — **only if your drone publishes on ROS2**

Skip this whole section if your drone already publishes on ROS1.

**Build (one-time):**

```bash
cd ros_bridge_docker
docker build -t ros1_bridge:noetic-foxy .
```

Or just run `./run_bridge.sh` — it auto-builds if the image is missing.

**Start roscore** (inside the bridge container):

```bash
docker run -d --rm --net=host --name=roscore \
  --entrypoint bash ros1_bridge:noetic-foxy -c \
  "source /opt/ros/noetic/setup.bash && roscore"
```

**Start the bridge:**

```bash
cd ros_bridge_docker
./run_bridge.sh
```

Verify it's up — these should appear in `rostopic list`:

```
/flow_depth/pose_est
/xtend/depth_m
```

---

## 2. Launch FALCON

From `falcon_docker/`:

```bash
cd falcon_docker
./run_hospital.sh office
```

The first arg is the map name; the script loads `<name>.yaml` from the same directory. It drops you into a bash shell inside the `falcon` container. Then:

```bash
source /catkin_ws/devel/setup.bash
roslaunch falcon_adapter real_drone.launch map_name:=office
```

> The script is named `run_hospital.sh` for historical reasons but accepts any env. As long as `office.yaml` sits next to it, `./run_hospital.sh office` will work.

---

## 3. Topics

**Inputs (provided by your drone, bridged from ROS2 if needed):**

| Topic | Type | Purpose |
|---|---|---|
| `/flow_depth/pose_est` | `geometry_msgs/PoseStamped` | localization |
| `/xtend/depth_m` | `sensor_msgs/Image` | depth |

**Outputs (your autopilot consumes these):**

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | velocity commands |
| `/takeoff` | `std_msgs/Empty` | takeoff trigger |

---

## 4. Open RViz

Open a new host terminal and shell into the falcon container:

```bash
docker exec -it falcon bash
```

Inside the container:

```bash
source /catkin_ws/devel/setup.bash
roslaunch exploration_manager rviz.launch
```

This loads a pre-configured RViz with the BEV map, planned path, and odometry already wired up — no manual display setup needed.

---

## 5. BEV click-to-goal

Same pattern — open another host terminal:

```bash
docker exec -it falcon bash
```

Inside:

```bash
source /catkin_ws/devel/setup.bash
rosrun falcon_adapter bev_click_goal.py
```

A 2D map window opens. **Left-click** anywhere to publish a goal — A* replans and the drone flies the new path.

---

## Cheat sheet

```bash
# Terminal 1 — roscore + bridge (ROS2 drones only)
cd ros_bridge_docker/
docker run -d --rm --net=host --name=roscore \
  --entrypoint bash ros1_bridge:noetic-foxy -c \
  "source /opt/ros/noetic/setup.bash && roscore"
./run_bridge.sh

# Terminal 2 — FALCON
cd falcon_docker/
./run_hospital.sh office
# (inside container)
source /catkin_ws/devel/setup.bash
roslaunch falcon_adapter real_drone.launch map_name:=office

# Terminal 3 — RViz
docker exec -it falcon bash -c \
  "source /catkin_ws/devel/setup.bash && roslaunch exploration_manager rviz.launch"

# Terminal 4 — BEV click-to-goal
docker exec -it falcon bash -c \
  "source /catkin_ws/devel/setup.bash && rosrun falcon_adapter bev_click_goal.py"
```