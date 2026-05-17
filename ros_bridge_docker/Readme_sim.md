# ROS1 ↔ ROS2 Bridge — Gazebo (sim) mode

Sister doc to `Readme.md` (which covers the real-drone path).
**Nothing in the real-drone path changes** when you switch to sim:
`bridge.yaml`, `run_bridge.sh`, the real-drone topic list — all
untouched. Sim mode lives in `bridge_sim.yaml` + `run_bridge_sim.sh`,
loaded by an explicit, separate script.

```
ros_bridge_docker/
├── bridge.yaml          ← REAL DRONE topics + QoS         (do not edit for sim)
├── run_bridge.sh        ← REAL DRONE runner               (do not edit for sim)
├── bridge_sim.yaml      ← SIM topics + QoS
├── run_bridge_sim.sh    ← SIM runner
└── (Dockerfile + entrypoint are shared)
```

Picking the right runner:

```bash
./run_bridge.sh        # real drone   (unchanged)
./run_bridge_sim.sh    # Gazebo sim
```

---

## What's bridged in sim mode

| Direction      | Topic                                  | Type                              | Who publishes        | Who consumes                          |
|----------------|----------------------------------------|-----------------------------------|----------------------|---------------------------------------|
| ROS2 → ROS1    | `/simple_drone/front_depth/depth/image_raw` | `sensor_msgs/Image`           | Gazebo sjtu_drone    | FALCON sensing                        |
| ROS2 → ROS1    | `/simple_drone/front_depth/depth/camera_info` | `sensor_msgs/CameraInfo`    | Gazebo sjtu_drone    | FALCON sensing                        |
| ROS2 → ROS1    | `/simple_drone/gt_pose`                | `geometry_msgs/Pose`              | Gazebo sjtu_drone    | falcon_adapter                        |
| ROS1 → ROS2    | `/simple_drone/cmd_vel`                | `geometry_msgs/Twist`             | cmd_to_vel / wpf     | Gazebo sjtu_drone                     |
| ROS1 → ROS2    | `/simple_drone/{takeoff, land, posctrl}` | `std_msgs/Empty` / `std_msgs/Bool` | cmd_to_vel / room_search | Gazebo sjtu_drone                 |
| ROS1 → ROS2    | `/odom_world`                          | `nav_msgs/Odometry`               | falcon_adapter       | **room_search arrival detection**     |
| ROS1 → ROS2    | `/map_ros/depth`                       | `sensor_msgs/Image` (32FC1)       | falcon_adapter       | semantic_mapper/object_mapper_node    |
| ROS1 → ROS2    | `/map_ros/pose`                        | `geometry_msgs/PoseStamped`       | falcon_adapter       | semantic_mapper/object_mapper_node    |
| ROS2 → ROS1    | `/waypoint_nav/goal`                   | `geometry_msgs/Point`             | **room_search**      | astar_planner                         |
| ROS2 → ROS1    | `/waypoint_follower/external_ctrl`     | `std_msgs/Bool` (transient_local) | **room_search**      | waypoint_follower                     |

Topics **not** bridged on purpose:
- `/tf`, `/tf_static`, `/clock` — falcon_adapter rebuilds the TF tree on wall clock; bridging the sim ones causes ~1.77e9-sec stamp delays in the ROS1 graph.
- `/simple_drone/front/image_raw` — Gazebo publishes the RGB camera in ROS2 directly; `yolo_detector` and `room_search` both subscribe in ROS2. No bridge needed.
- `/perception/{detections, objects}` and `/target_seen` — pure ROS2 between `semantic_mapper` nodes and the orchestrator.

---

## End-to-end Gazebo test bring-up

The order matters — each step waits for something the previous step produced.

### 0) (Once) build the container images

```bash
cd ros_bridge_docker  && docker build -t ros1_bridge:noetic-foxy .
cd ../sjtu_drone      && docker build -t sjtu_drone_nadav:humble_ros2 .
cd ../falcon_docker   && docker build -t falcon-ros:noetic .   # see Dockerfile
cd ../room_docker     && ./run_room_search.sh   # auto-builds room_search:humble
```

### 1) ROS1 master (one terminal)

The bridge waits on `rostopic list` to confirm the master is up. The
simplest way is to spin a `roscore` container from the bridge image:

```bash
docker run -it --rm --net=host --name=roscore \
    --entrypoint bash ros1_bridge:noetic-foxy \
    -c 'source /opt/ros/noetic/setup.bash && roscore'
```

Wait for `started core service [/rosout]`.

### 2) Bridge in **sim** mode (one terminal)

```bash
cd ros_bridge_docker
./run_bridge_sim.sh
```

Wait for the banner ending in `Starting parameter_bridge...`.
Until ROS2 publishers actually exist, you won't see any
"Passing message from ROS 2 …" lines — that's normal; they appear
once Gazebo + room_docker come up.

### 3) Gazebo sjtu_drone (one terminal)

```bash
cd sjtu_drone
./run.sh small_house     # or hospital / bookstore / playground / ...
```

Verify the drone is publishing in ROS2 (from your host or from a ROS2
container in `ROS_DOMAIN_ID=20`):

```bash
ros2 topic hz /simple_drone/gt_pose
ros2 topic hz /simple_drone/front/image_raw
```

### 4) FALCON in Gazebo mode (one terminal)

`run_hospital.sh` (despite the name) takes a map argument and works for
any of the worlds shipped with the project. To match the world you ran
in step 3:

```bash
cd falcon_docker
./run_hospital.sh small_house     # same env as step 3
# inside the container:
roslaunch falcon_adapter gazebo_waypoint_nav.launch
```

Verify falcon_adapter is publishing the ROS1-side topics the bridge
needs to forward:

```bash
# in another shell on the ROS1 host (or inside the falcon container)
rostopic hz /odom_world
rostopic hz /map_ros/depth
rostopic hz /map_ros/pose
```

You should also see corresponding entries appear in the bridge log:

```
Passing message from ROS 1 nav_msgs/Odometry to ROS 2 ...
Passing message from ROS 1 sensor_msgs/Image to ROS 2 ...
```

### 5) room_search (one terminal)

```bash
cd room_docker
./run_room_search.sh \
    ros2 launch room_search room_search.launch.py \
    target_object:=keyboard \
    room_center_x:=4.0 \
    room_center_y:=5.0
```

The orchestrator should immediately log `state=WAIT_INIT → NAV_TO_ROOM`
once it receives the first `/odom_world` over the bridge, and you'll
see the path show up in the FALCON RViz / BEV viewer.

---

## Smoke checks

If any of these is missing, the corresponding step is broken:

```bash
# ROS2 sees the drone + camera + RGB
ros2 topic list | grep -E '/simple_drone/(gt_pose|front)'

# ROS2 sees what's bridged from ROS1
ros2 topic list | grep -E '^(/odom_world|/map_ros)'

# ROS1 sees what's bridged from ROS2 (run inside the falcon container)
rostopic list | grep -E '^(/waypoint_nav|/waypoint_follower|/simple_drone)'

# Round-trip: orchestrator publishes a goal, planner gets it
ros2 topic pub /waypoint_nav/goal geometry_msgs/msg/Point '{x: 4.0, y: 5.0, z: 0.0}' -1
rostopic echo /waypoint_nav/goal -n 1     # in the falcon container

# external_ctrl is latched — late ROS1 subscribers should still see it
ros2 topic pub --once /waypoint_follower/external_ctrl std_msgs/msg/Bool '{data: false}'
rostopic echo /waypoint_follower/external_ctrl -n 1
```

---

## Common pitfalls

- **`ROS_DOMAIN_ID` mismatch.** The bridge sim runner sets `ROS_DOMAIN_ID=20`
  and the room_docker run script defaults to the same. If you've
  overridden it in the sjtu_drone container or your shell, the
  containers won't see each other's ROS2 topics. `echo $ROS_DOMAIN_ID`
  in each shell before bringing things up.
- **RMW mismatch.** sim uses `rmw_cyclonedds_cpp` end-to-end. If your
  sjtu_drone container has been edited to use `rmw_fastrtps_cpp`, the
  bridge won't see its publishers. Pick one and stick with it.
- **Stalled depth stream.** The depth topic has `reliability: best_effort`
  on both sides. If you change one to `reliable`, DDS silently refuses
  to deliver — that's the original bug that motivated this whole setup.
- **room_search refuses to publish cmd_vel.** Check
  `rostopic echo /waypoint_follower/external_ctrl` — if it never goes
  True, waypoint_follower keeps publishing zeros that overwrite
  room_search's commands. Investigate the bridge entry for
  `/waypoint_follower/external_ctrl`; transient_local is required
  there.
