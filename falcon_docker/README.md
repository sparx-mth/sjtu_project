# falcon_adapter — Real-Drone Runtime

Runs the FALCON exploration stack on a real drone. Pose + depth in → velocity commands out.

> **Running the Gazebo warehouse instead?** This file covers the real drone.
> The simulated `small_warehouse` campaign is `run_warehouse.sh`, and its state,
> its open failure and its from-scratch bring-up are in
> [`RESUME.md`](RESUME.md) — read that one first.

---

## Repository layout

```
.
├── ros_bridge_docker/        ← ROS2 ↔ ROS1 bridge container
│   ├── Dockerfile
│   ├── run_bridge.sh         ← starts the bridge
│   └── entrypoint.sh
│
├── data_publisher_docker/    ← ROS2 publisher (recording only)
│   ├── Dockerfile
│   ├── data_publisher.py
│   └── run_publisher.sh      ← replays recorded pose + depth on ROS2
│
└── falcon_docker/            ← FALCON ROS1 stack container
    ├── run_hospital.sh       ← starts the falcon container (any map)
    ├── office.yaml           ← map config for your environment
    ├── hospital.yaml         ← (and any other maps you have)
    └── adapter/              ← scripts + launch files mounted into the container
```

Three directories, three responsibilities. The bridge container hosts `roscore` and tunnels ROS2 → ROS1. The data publisher container is a stand-in for a real drone (skip it in production). The falcon container runs the actual stack.

**Everything on `ROS_DOMAIN_ID=5` with `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`** — both already baked into `run_bridge.sh` and `run_publisher.sh`.

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

> Topics only appear on the ROS1 side once a ROS1 subscriber asks for them (`dynamic_bridge` is lazy). If `rostopic list` looks empty before FALCON is up, that's normal — start FALCON and they'll show up.

---

## 2. Data Publisher — **RECORDED DATA ONLY**

> **⚠️ Run this only when replaying a recording. Skip entirely with a real drone — the real drone publishes the same topics natively.**

For development/testing, `run_publisher.sh` replays a recorded run on the same ROS2 topics the real drone would use, so the rest of the stack can't tell the difference.

```bash
cd data_publisher_docker
DATA_DIR=~/Desktop ./run_publisher.sh
```

`DATA_DIR` should point to the folder containing the `xtend_rectified_depth_take_*/` directory and the matching `estimated_trajectory_*.json`. Defaults to `~/Desktop` if unset. Edit the paths in `data_publisher.py` if your filenames differ.

**Verify** from inside the bridge container that the topics are discoverable:

```bash
docker exec -it ros1_bridge bash -c \
  "source /opt/ros/foxy/setup.bash && ros2 topic list"
```

Should now show `/flow_depth/pose_est` and `/xtend/depth_m`.

---

## 3. Open RViz

First, start the FALCON container — it stays alive in the background and provides the workspace for RViz, the 2D map, and the FALCON launch below:

```bash
cd falcon_docker
./run_hospital.sh office
```

The first arg is the map name; the script loads `<name>.yaml` from the same directory. It drops you into a bash shell inside the `falcon` container — **leave this terminal open**. We'll come back to it in step 6.

Now in a **new** host terminal:

```bash
docker exec -it falcon bash
export DISPLAY=:0
source /catkin_ws/devel/setup.bash
roslaunch exploration_manager rviz.launch
```

This loads a pre-configured RViz with the BEV map, planned path, and odometry already wired up.

---

## 4. Open the 2D Map (BEV click-to-goal)

In another **new** host terminal:

```bash
docker exec -it falcon bash
export DISPLAY=:0
source /catkin_ws/devel/setup.bash
rosrun falcon_adapter bev_click_goal.py
```

A 2D map window opens. **Left-click** anywhere to publish a goal — A* replans and the drone flies the new path. The red arrow marks the drone's live pose.

---

## 5. NavDP click viewer (optional, standalone sanity check)

> Skip unless you're checking the NavDP path in isolation. This viewer doesn't fly the drone and doesn't replace the BEV goal flow. It only confirms that `click pixel → body-frame (gx, gy) → NavDP → trajectory` is wired up correctly.

Requires the NavDP HTTP server running on `127.0.0.1:8888` (override with `_port:=`).

In another **new** host terminal:

```bash
docker exec -it falcon bash
export DISPLAY=:0
source /catkin_ws/devel/setup.bash
rosrun falcon_adapter navdp_click.py
```

An RGB + depth window opens. **Hover** the depth panel to read the depth value at the cursor. **Left-click** on the RGB panel to set a pixel goal, then press **ENTER** to send it to NavDP — the returned trajectory appears in a second window. `r` clears, `q` quits.

Override topics or intrinsics on a real drone (defaults are the sjtu_drone front camera, 640×480, 90° HFOV):

```
_rgb_topic:=... _depth_topic:=...
_fx:=... _fy:=... _cx:=... _cy:=...
```

---

## 6. Launch FALCON

Go back to the terminal from step 3 (the one running `./run_hospital.sh`):

```bash
docker exec -it falcon bash
source /catkin_ws/devel/setup.bash
roslaunch falcon_adapter real_drone.launch map_name:=office
```

The state machine progresses `WAIT_POSE → TAKING_OFF → HOVER_SETTLE → WAIT_PATH`. RViz now shows the drone, the 2D map fills in, and a click in the BEV window sets a goal.

---

## 7. Topics

**Inputs (provided by your drone OR `run_publisher.sh`, bridged from ROS2):**

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

## Cheat sheet

```bash
# Terminal 1 — roscore + bridge (skip if drone publishes on ROS1)
cd ros_bridge_docker/
docker run -d --rm --net=host --name=roscore \
  --entrypoint bash ros1_bridge:noetic-foxy -c \
  "source /opt/ros/noetic/setup.bash && roscore"
./run_bridge.sh

# Terminal 2 — RECORDED DATA ONLY (skip if using a real drone)
cd data_publisher_docker/
DATA_DIR=~/Desktop ./run_publisher.sh

# Terminal 3 — start the falcon container (leave open, used again at the end)
cd falcon_docker/
./run_hospital.sh office

# Terminal 4 — RViz
docker exec -it falcon bash -c \
  "source /catkin_ws/devel/setup.bash && roslaunch exploration_manager rviz.launch"

# Terminal 5 — 2D map (BEV click-to-goal)
docker exec -it falcon bash -c \
  "source /catkin_ws/devel/setup.bash && rosrun falcon_adapter bev_click_goal.py _drone_ns:=''"

# Terminal 6 — NavDP click viewer (optional)
docker exec -it falcon bash -c \
  "source /catkin_ws/devel/setup.bash && rosrun falcon_adapter navdp_click.py"

# Terminal 3 (again) — launch FALCON
source /catkin_ws/devel/setup.bash
roslaunch falcon_adapter real_drone.launch map_name:=office
```