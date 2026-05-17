# room_docker

ROS2 Humble container for the **room_search** task:

1. Auto-navigate to a point on the map (e.g. the centre of a room, `(4, 5)`).
2. Rotate in place while watching the YOLO detector for one given object
   (e.g. `keyboard`).
3. When the object is detected, stop rotating, drive to the object's
   world XY, and land on it.

The detection chain (`yolo_detector`, `object_mapper_node`,
`target_watcher_node`) comes from `perception_docker/semantic_mapper`
— this docker re-uses those nodes via a bind-mount, and adds **one new
node** plus a launch file:

| New | What |
|---|---|
| `room_search_orchestrator_node` | State machine: `WAIT_INIT → NAV_TO_ROOM → ROTATE_AND_SEARCH → APPROACH_TARGET → LAND → DONE`. |
| `launch/room_search.launch.py` | Brings up the orchestrator together with YOLO + object_mapper + target_watcher, pre-wired with the target string and a small open-vocab YOLO prompt that includes `keyboard`. |

There is also **one small change** in `falcon_docker`:
`waypoint_follower.py` now subscribes to `/waypoint_follower/external_ctrl`
(`std_msgs/Bool`, latched). While `True`, `waypoint_follower._publish_twist`
is a no-op, so the orchestrator can drive `/cmd_vel` directly during the
in-place rotation without its commands being overwritten by waypoint
follower zeros.

```
room_docker/
├── Dockerfile               ROS2 Humble + ultralytics + cv_bridge + cyclonedds
├── entrypoint.sh
├── cyclonedds.xml
├── run_room_search.sh       Bind-mounts semantic_mapper + room_search, builds, runs
└── room_search/             ament_python package
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/room_search
    ├── room_search/
    │   ├── __init__.py
    │   └── room_search_orchestrator_node.py
    └── launch/
        └── room_search.launch.py
```

---

## Quick start

```bash
# 1) sim + falcon + ros1_bridge already running, then:
cd room_docker
./run_room_search.sh \
    ros2 launch room_search room_search.launch.py \
    target_object:=keyboard \
    room_center_x:=4.0 \
    room_center_y:=5.0
```

---

## How it talks to the rest of the stack

### Inbound (this node subscribes)

| Topic                  | Type                       | Origin                                 |
|------------------------|----------------------------|----------------------------------------|
| `/odom_world`          | `nav_msgs/Odometry`        | `falcon_adapter` (ROS1) → bridge       |
| `/target_seen`         | `std_msgs/Bool`            | `target_watcher_node` (ROS2 native)    |
| `/target_seen/info`    | `std_msgs/String` (JSON)   | `target_watcher_node` (ROS2 native)    |
| `/perception/objects`  | `std_msgs/String` (JSON)   | `object_mapper_node` (ROS2 native)     |

The pose source is configurable via `pose_topic` / `pose_type`. Defaults
to `/odom_world` (Odometry). If your bridge doesn't carry that, set e.g.
`pose_topic:=/simple_drone/gt_pose pose_type:=pose`.

### Outbound (this node publishes)

| Topic                                   | Type                  | Consumer                       |
|-----------------------------------------|-----------------------|--------------------------------|
| `/waypoint_nav/goal`                    | `geometry_msgs/Point` | `astar_planner` (ROS1) → bridge|
| `/<drone_ns>/cmd_vel`                   | `geometry_msgs/Twist` | drone (ROS1) → bridge          |
| `/<drone_ns>/land`                      | `std_msgs/Empty`      | drone (ROS1) → bridge          |
| `/waypoint_follower/external_ctrl`      | `std_msgs/Bool` (latched) | `waypoint_follower` (ROS1) → bridge |

### Bridge config you'll likely need to add

`parameter_bridge` is interface-pinned in this project (see
`ros_bridge_docker/bridge.yaml`), so every bridged topic must be listed.
Add **at least** these entries — the orchestrator publishes from ROS2
and the consumers are in ROS1:

```yaml
  - topic: /waypoint_nav/goal
    type: geometry_msgs/msg/Point
    queue_size: 1
    qos: { history: keep_last, depth: 1, reliability: reliable, durability: volatile }

  - topic: /waypoint_follower/external_ctrl
    type: std_msgs/msg/Bool
    queue_size: 1
    qos: { history: keep_last, depth: 1, reliability: reliable, durability: transient_local }

  - topic: /simple_drone/land
    type: std_msgs/msg/Empty
    queue_size: 1
    qos: { history: keep_last, depth: 1, reliability: reliable, durability: volatile }
```

(For a real drone whose `cmd_vel` and `land` are at root, drop the
`/simple_drone` prefix and set `drone_ns:=""` on the orchestrator.)

The reverse direction — `/odom_world` (or whatever pose topic you pick)
ROS1 → ROS2 — should already be in your bridge yaml since
`perception_docker` consumes the same source.

---

## The state machine

```
                ┌─────────────────────────┐
                │       WAIT_INIT         │   wait for first pose msg
                └────────────┬────────────┘
                             ▼
                ┌─────────────────────────┐
                │      NAV_TO_ROOM        │   publish (cx, cy) to
                │                         │   /waypoint_nav/goal,
                │                         │   re-publish every
                │                         │   nav_goal_republish_s,
                │                         │   exit when d < nav_arr_r
                └────────────┬────────────┘
                             ▼
                ┌─────────────────────────┐
                │   ROTATE_AND_SEARCH     │   external_ctrl=True
                │                         │   spin at rotation_rate_rad_s,
                │                         │   exit on /target_seen=True
                │                         │   (with XY in target_seen/info)
                │                         │   give up after max_rotation_revs
                └────────────┬────────────┘
                             ▼
                ┌─────────────────────────┐
                │    APPROACH_TARGET      │   external_ctrl=False
                │                         │   publish target_xy to
                │                         │   /waypoint_nav/goal,
                │                         │   keep refining XY from
                │                         │   /perception/objects until
                │                         │   we're within lock_r,
                │                         │   exit when d < approach_r
                └────────────┬────────────┘
                             ▼
                ┌─────────────────────────┐
                │          LAND           │   publish land_burst_count ×
                │                         │   <drone_ns>/land Empty
                │                         │   at land_burst_hz
                └────────────┬────────────┘
                             ▼
                ┌─────────────────────────┐
                │          DONE           │
                └─────────────────────────┘
```

The orchestrator never overrides altitude or vy — those are still owned
by the sjtu_drone landing controller and by `waypoint_follower`'s
`linear.z ≡ 0, linear.y ≡ 0` invariants. The orchestrator only ever
commands `(linear.x, angular.z)` during the rotation phase and lets the
existing nav stack do the rest.

---

## Launch arguments

| Arg                       | Default              | What it does                                                                 |
|---------------------------|----------------------|------------------------------------------------------------------------------|
| `target_object`           | `keyboard`           | Goes to `target_watcher_node.target_object` AND to the orchestrator.         |
| `room_center_x`           | `4.0`                | First nav goal X (world frame, metres).                                      |
| `room_center_y`           | `5.0`                | First nav goal Y.                                                            |
| `drone_ns`                | `/simple_drone`      | Namespace prefix for `/cmd_vel` and `/land`. Set `""` for real-drone root.   |
| `pose_topic`              | `/odom_world`        | Drone pose source for arrival detection.                                     |
| `pose_type`               | `odometry`           | `odometry` / `pose_stamped` / `pose`.                                        |
| `nav_arrival_radius_m`    | `0.50`               | Acceptance circle for "we arrived at the room centre".                       |
| `rotation_rate_rad_s`     | `0.5`                | In-place yaw rate during search (+ is CCW).                                  |
| `max_rotation_revs`       | `2.0`                | Give up after this many full turns with no detection.                        |
| `approach_radius_m`       | `0.35`               | Acceptance circle for "we reached the target".                               |
| `approach_timeout_s`      | `90.0`               | Hard timeout in APPROACH; lands anyway when hit.                             |
| `yolo_model`              | `yolov8s-world.pt`   | YOLO-World checkpoint (downloaded by ultralytics on first run).              |
| `yolo_vocabulary`         | desk/room set (incl. `keyboard`) | YOLO-World prompts. Override with a Python-list literal.         |
| `start_yolo` / `start_object_mapper` / `start_target_watcher` | `true` | Set `false` if you already run perception_docker. |

---

## Tweaks you'll probably need

* **Pose source.** If your bridge carries `/<drone_ns>/gt_pose` (Pose)
  but not `/odom_world`, override:
  ```
  pose_topic:=/simple_drone/gt_pose  pose_type:=pose
  ```

* **YOLO vocabulary.** YOLO-World is more accurate with fewer, well-
  separated prompts. For a focused demo:
  ```
  yolo_vocabulary:="['keyboard','laptop','monitor','desk','chair']"
  ```

* **Rotation direction.** `rotation_rate_rad_s:=-0.5` to spin clockwise.

* **Drone won't go all the way to the target.** Bump
  `approach_radius_m` down (e.g. `0.20`) and lower the `astar_planner`
  inflation in `gazebo_waypoint_nav.launch` so the planner doesn't
  refuse cells adjacent to the (presumably non-obstacle) target.
