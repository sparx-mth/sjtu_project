# room_docker

ROS2 Humble container for the **room_search** task:

1. Auto-navigate to a point on the map (e.g. the centre of a room, `(4, 5)`)
   using the existing `falcon_docker` A* nav stack.
2. Rotate in place while watching the YOLO detector for one given object
   (e.g. `keyboard`).
3. When the object is detected, **close in on it using sparse
   Lucas-Kanade optical flow on the RGB stream** — no depth, no
   world-frame XY, no A*, no localisation, and no YOLO in the inner
   loop. YOLO seeds an initial bbox; Shi-Tomasi corners inside that
   bbox are then propagated frame-to-frame with `calcOpticalFlowPyrLK`
   at camera rate (~30 Hz, costs 2–3 ms/frame on Jetson AGX Orin CPU,
   no neural inference, no contrib OpenCV modules). The bounding rect
   of the still-matched corners IS the updated bbox — so scale is
   implicit, and the same area-fraction threshold as before triggers
   LAND. Any fresh YOLO match while in approach re-seeds the tracker
   to bound drift to the YOLO inter-arrival time (1 s default on
   Jetson, configurable down to "essentially off").

The detection chain (`yolo_detector`, `object_mapper_node`,
`target_watcher_node`) comes from `perception_docker/semantic_mapper`
— this docker re-uses those nodes via a bind-mount, and adds **one new
node** plus a launch file:

| New | What |
|---|---|
| `room_search_orchestrator_node` | State machine: `WAIT_INIT → NAV_TO_ROOM → ROTATE_AND_SEARCH → VISUAL_APPROACH → LAND → DONE`. |
| `launch/room_search.launch.py` | Brings up the orchestrator together with YOLO + object_mapper + target_watcher, pre-wired with the target string and a small open-vocab YOLO prompt that includes `keyboard`. |

There is also **one small change** in `falcon_docker`:
`waypoint_follower.py` now subscribes to `/waypoint_follower/external_ctrl`
(`std_msgs/Bool`, latched). While `True`, `waypoint_follower._publish_twist`
is a no-op, so the orchestrator can drive `/cmd_vel` directly during the
in-place rotation and visual close-in without its commands being
overwritten by waypoint follower zeros.

```
room_docker/
├── Dockerfile               ROS2 Humble + ultralytics + cv_bridge + cyclonedds  (x86_64)
├── Dockerfile.jetson        Same, on dustynv/ros:humble-pytorch-l4t base       (aarch64 / Jetson AGX Orin)
├── entrypoint.sh
├── cyclonedds.xml
├── run_room_search.sh       Auto-picks Dockerfile by arch; bind-mounts both packages, builds, runs
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

## Quick start (x86_64 with NVIDIA GPU)

```bash
# 1) sim + falcon + ros1_bridge already running, then:
cd room_docker
./run_room_search.sh \
    ros2 launch room_search room_search.launch.py \
    target_object:=keyboard \
    room_center_x:=4.0 \
    room_center_y:=5.0
```

## Quick start (Jetson AGX Orin, JetPack 6.x)

```bash
# 1) sim/drone + falcon + ros1_bridge already running on the same network, then:
cd room_docker
./run_room_search.sh \
    ros2 launch room_search room_search.launch.py \
    target_object:=keyboard \
    room_center_x:=4.0 \
    room_center_y:=5.0
```

The run script detects `aarch64` and switches to `Dockerfile.jetson` +
`--runtime nvidia` + `--ipc=host` automatically. Override the
detection with `ROOM_DOCKER_TARGET=jetson|x86`.

### Jetson notes

* **Base image.** `Dockerfile.jetson` uses
  `dustynv/ros:humble-pytorch-l4t-r36.4.0` as the default base. That
  tag ships ROS2 Humble + a Jetson-CUDA build of PyTorch built against
  the L4T runtime. Override for a different JetPack release with:
  ```bash
  docker build --build-arg BASE_IMAGE=dustynv/ros:humble-pytorch-l4t-r35.4.1 \
               -f Dockerfile.jetson -t room_search:humble-jetson room_docker
  ```
  `r36.*` → JetPack 6 (Jetson AGX Orin, native Humble).
  `r35.*` → JetPack 5 (Jetson AGX Xavier; Humble works but isn't the
  L4T-native distro for that JetPack).

* **PyTorch is NOT reinstalled** in the Jetson Dockerfile. PyPI has
  no Jetson-CUDA wheels, so a `pip install torch` would silently
  downgrade the base image's GPU build to a CPU-only wheel and YOLO
  would then run on the ARM cores at ~1/30th the throughput.
  `ultralytics` is installed with `--no-deps` for exactly the same
  reason; its non-torch deps are pulled in explicitly afterwards.

* **Slimmer dependency set on Jetson.** The x86 image installs
  `ompl`, `scikit-image`, `scipy`, `openai-clip`, and `transforms3d`
  to keep it usable as a drop-in for the full perception_docker
  pipeline. The Jetson image drops them because none of the three
  semantic_mapper nodes this launch actually runs
  (`yolo_detector`, `object_mapper_node`, `target_watcher_node`)
  import them, and `ompl` in particular has no aarch64 pip wheel.
  Re-add any of them in `Dockerfile.jetson` if you extend the launch.

* **First run downloads the YOLO-World checkpoint** (~360 MB). The
  run script bind-mounts `~/.cache/ultralytics` into the container so
  the download survives container restarts — useful when a Jetson is
  on a tethered link.

* **No code changes** in `room_search_orchestrator_node.py` or
  `room_search.launch.py` for the Jetson port: the orchestrator is
  pure Python + ROS2 messages and the launch just instantiates nodes
  by package name. Everything arch-specific lives in
  `Dockerfile.jetson` and `run_room_search.sh`.

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
                │   ROTATE_AND_SEARCH     │   external_ctrl = True
                │                         │   spin at rotation_rate_rad_s,
                │                         │   exit when target_seen AND
                │                         │   a matching detection is fresh
                │                         │   in /perception/detections.
                │                         │   give up after max_rotation_revs
                └────────────┬────────────┘
                             ▼
                ┌─────────────────────────┐
                │    VISUAL_APPROACH      │   external_ctrl STAYS True
                │   (RGB bbox only —      │   closed loop on:
                │    no depth, no XY)     │     * bbox cx       → yaw to centre
                │                         │     * bbox area frac → forward vx
                │                         │   exit when area_frac ≥ land_area_frac
                │                         │   or visual_giveup_s of lost
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

The orchestrator never overrides altitude or vy. Once the visual
approach reaches `visual_land_depth_m`, the sjtu_drone landing
controller does the actual descent (in response to `/<ns>/land Empty`).
The orchestrator only ever commands `(linear.x, angular.z)`.

### Why RGB-only after detection

Depth sensors fail in glare, on glass, on dark/low-texture surfaces,
and at very close range — all common when "landing on" a desk object.
The bbox itself is a clean, monotonic proxy for proximity: at distance
`d` the bbox area fraction grows roughly as `1/d²` (a flat target
viewed perpendicularly through a pinhole). We don't need an absolute
distance to know we're "as close as we can get" — we just need a
threshold on how much of the image the target is filling.

### Why detect-once / track-many

Running YOLO-World at 4 Hz on a Jetson AGX while perception, control,
ROS2, and the drone hardware loop all share the same SoC is wasteful
once we already have a bbox. The inner loop uses **sparse Lucas-Kanade
optical flow** to propagate the bbox between YOLO frames:

* YOLO runs at `yolo_min_dt` (1 Hz default on Jetson) just to acquire
  the initial bbox during `ROTATE_AND_SEARCH` and to opportunistically
  re-anchor the tracker if it drifts.
* The orchestrator subscribes to the raw RGB topic and on every frame:
  - converts to grayscale (cv_bridge + `cv2.cvtColor`),
  - runs `cv2.calcOpticalFlowPyrLK` on the Shi-Tomasi corners that
    were seeded inside the YOLO bbox,
  - drops corners with `status == 0` or outside the image,
  - takes the bounding rect of the survivors as the new bbox.
* If too few corners survive (`< track_min_matches`, default 8), the
  tracker is marked invalid and the drone hovers waiting for any
  fresh YOLO match to re-seed.

This is among the lightest robust trackers available — no neural
inference, no `opencv-contrib` modules, no CUDA needed. On a Jetson
AGX Orin CPU it costs ~2–3 ms per 640x360 frame, so a 30 Hz camera
stream is processed live with headroom to spare for the rest of the
ROS pipeline.

### Platform invariant

**Every published Twist must satisfy `(linear.x = 0) XOR (angular.z = 0)`**
plus `linear.y = linear.z = 0`. The real drone's flight controller
refuses commands that mix forward motion with yaw, so the orchestrator
emits **either** pure-yaw **or** pure-forward, never both. (The
ROS1-side `waypoint_follower.py` already documents and enforces the
same invariant.)

### Visual control law

Each `1 / visual_ctrl_hz` (default 20 Hz) reads the **tracked** bbox
(updated by the LK loop on every RGB frame, not by YOLO):

1. If the tracker is invalid (lost lock, no fresh seed yet, or RGB
   stream silent for more than `visual_lost_hover_s`), publish a zero
   Twist and start counting toward `visual_giveup_s`.
2. Compute
   ```
   x_off     = (bbox_cx - rgb_W/2) / (rgb_W/2)       # normalised, [-1, +1]
   area_frac = (bbox_w * bbox_h)   / (rgb_W * rgb_H) # proximity proxy
   ```
3. If `area_frac >= visual_land_area_frac`, transition to `LAND`.
4. Otherwise pick a sub-mode with a Schmitt trigger on `|x_off|`:
   - In `YAW` sub-mode, stay until `|x_off| < visual_yaw_deadband_exit`,
     then switch to `ADVANCE`.
   - In `ADVANCE` sub-mode, stay until `|x_off| > visual_yaw_deadband_enter`,
     then switch back to `YAW`.
   - On every sub-mode switch, publish one `(0, 0)` brake tick — gives
     the platform PID a beat to settle the previous axis before the new
     command starts.
5. Emit the sub-mode's Twist:
   - **`YAW`**: `(0, wz)` where `wz = -visual_kp_yaw * x_off`, saturated
     at `visual_max_yaw_rate`. Sign: ROS body-frame `+angular.z` yaws
     CCW, which shifts the camera content LEFTWARDS, so target right of
     centre means `wz < 0`.
   - **`ADVANCE`**: `(vx, 0)` with a linear ramp on bbox area:
     ```
     area_frac < slowdown_area_frac → vx = vx_max
     area_frac ≥ slowdown_area_frac
       → vx = vx_max * (land_area_frac - area_frac)
                       / (land_area_frac - slowdown_area_frac)
     ```

There's also a hard `visual_approach_timeout_s` (default 90 s): if the
bbox never reaches `visual_land_area_frac` — small target, wide FOV,
weird lighting — the orchestrator lands wherever it is at that point
rather than hovering forever.

---

## How it talks to the rest of the stack

### Inbound (this node subscribes)

| Topic                  | Type                              | Origin                                 |
|------------------------|-----------------------------------|----------------------------------------|
| `/odom_world`          | `nav_msgs/Odometry`               | `falcon_adapter` (ROS1) → bridge       |
| `/target_seen`         | `std_msgs/Bool`                   | `target_watcher_node` (ROS2 native)    |
| `/target_seen/info`    | `std_msgs/String` (JSON)          | `target_watcher_node` (ROS2 native)    |
| `/perception/detections` | `vision_msgs/Detection2DArray`  | `yolo_detector` (ROS2 native; seed + occasional re-seed only) |
| `/simple_drone/front/image_raw` | `sensor_msgs/Image`        | drone (ROS1) → bridge; consumed by the LK tracker on every frame |

The visual close-in does NOT subscribe to depth. The pose source is
configurable via `pose_topic` / `pose_type` (default `/odom_world` /
`odometry`). The RGB topic is configurable via `rgb_topic` (default
`/simple_drone/front/image_raw`). Both need to be bridged from ROS1
— see your `ros_bridge_docker/bridge.yaml`.

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
Add **at least** these entries for the ROS2 → ROS1 direction:

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

And for the ROS1 → ROS2 direction:

```yaml
  - topic: /odom_world
    type: nav_msgs/msg/Odometry
    queue_size: 10
    qos: { history: keep_last, depth: 10, reliability: reliable, durability: volatile }
```

(The RGB topic that YOLO consumes — typically `/simple_drone/front/image_raw`
— should also already be bridged for `perception_docker`.)

(For a real drone whose `cmd_vel` / `land` are at root, drop the
`/simple_drone` prefix and set `drone_ns:=""` on the orchestrator.)

---

## Launch arguments

### Mission

| Arg                       | Default              | What it does                                                                 |
|---------------------------|----------------------|------------------------------------------------------------------------------|
| `target_object`           | `keyboard`           | Goes to `target_watcher.target_object` AND to the orchestrator.              |
| `room_center_x` / `_y`    | `4.0` / `5.0`        | First nav goal (world frame, metres).                                        |
| `drone_ns`                | `/simple_drone`      | Namespace prefix for `/cmd_vel` and `/land`. Set `""` for real-drone root.   |
| `pose_topic` / `pose_type`| `/odom_world` / `odometry` | Drone pose source. Use `pose_type:=pose` for a bare `geometry_msgs/Pose`.|
| `detections_topic`        | `/perception/detections` | YOLO detections feed (the only stream the visual loop consumes).         |
| `nav_arrival_radius_m`    | `0.50`               | Acceptance circle for "we arrived at the room centre".                       |
| `rotation_rate_rad_s`     | `0.5`                | In-place yaw rate during search (+ is CCW).                                  |
| `max_rotation_revs`       | `2.0`                | Give up after this many full turns with no detection.                        |

### Visual close-in (RGB-only)

| Arg                          | Default | What it does                                                                  |
|------------------------------|---------|-------------------------------------------------------------------------------|
| `rgb_image_width` / `_height`| `640` / `360` | Bbox normalisation. sjtu_drone defaults; override for real cameras.     |
| `visual_kp_yaw`              | `0.9`   | P-gain mapping normalised x-offset → yaw rate.                                |
| `visual_max_yaw_rate`        | `0.6`   | rad/s saturation on the visual yaw output.                                    |
| `visual_yaw_deadband_enter`  | `0.20`  | `\|x_off\|` at which we switch from `ADVANCE` back to `YAW`.                  |
| `visual_yaw_deadband_exit`   | `0.08`  | `\|x_off\|` at which we switch from `YAW` to `ADVANCE`. Must be `< enter`.    |
| `visual_vx_max`              | `0.20`  | Max forward velocity during approach (m/s).                                   |
| `visual_slowdown_area_frac`  | `0.03`  | Bbox area / image area at which the linear vx ramp starts.                    |
| `visual_land_area_frac`      | `0.12`  | Bbox area / image area that triggers the `LAND` transition.                   |
| `visual_lost_hover_s`        | `0.6`   | Window after which a missing detection counts as "lost".                      |
| `visual_giveup_s`            | `15.0`  | After this long staying lost, transition to `GIVE_UP`.                        |
| `visual_approach_timeout_s`  | `90.0`  | Hard fallback: `LAND` after this long even if the area threshold isn't met.   |

### YOLO

| Arg                | Default               | What it does                                                                 |
|--------------------|-----------------------|------------------------------------------------------------------------------|
| `yolo_min_dt`      | `1.0`                 | YOLO is no longer in the inner loop — the LK tracker propagates the bbox at camera rate. 1 Hz is plenty to seed + occasionally re-anchor. Crank up (e.g. `5.0`) for tighter Jetson budgets; down (e.g. `0.25`) if drift is severe. |
| `yolo_vocabulary`  | desk/room set incl. `keyboard` | Python-list literal. Override for a tighter set.                    |
| `start_yolo` / `start_object_mapper` / `start_target_watcher` | `true` | Set `false` if you already run perception_docker. |

### Lucas-Kanade tracker (inner loop)

| Arg                          | Default | What it does                                                                |
|------------------------------|---------|-----------------------------------------------------------------------------|
| `track_max_corners`          | `80`    | Cap on Shi-Tomasi corners seeded per bbox. More corners = more robust but more LK work per frame. |
| `track_corner_quality`       | `0.05`  | Shi-Tomasi quality threshold. Lower = more (weaker) corners.                |
| `track_corner_min_dist`      | `5.0`   | Minimum separation between seeded corners (pixels).                          |
| `track_lk_win`               | `21`    | LK window size. Bigger = handles larger motion but more compute.            |
| `track_lk_levels`            | `3`     | Pyramid levels. 3 handles motion up to ~`win * 2^levels` ≈ 168 px / frame.  |
| `track_min_matches`          | `8`     | Below this many surviving corners, tracker is lost.                         |
| `track_re_seed_on_detection` | `true`  | Re-seed corners from YOLO bbox on every fresh match — bounds tracker drift. |
| `track_frame_buffer_len`     | `30`    | Recent-frame ring buffer used for stamp-matching the YOLO bbox to the right frame. |
| `track_seed_roi_margin`      | `0.10`  | Fraction of bbox W/H added when extracting the ROI for corner seeding (gives Shi-Tomasi context on object edges). |

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

* **Yaw sign inverted.** If the drone yaws *away* from the target
  during VISUAL_APPROACH, your camera convention is mirrored — flip the
  sign by setting `visual_kp_yaw:=-0.9` (or invert in the source).

* **Drone overshoots the target.** Lower `visual_vx_max` and/or raise
  `visual_slowdown_area_frac`. The ramp is linear in
  `(land - area) / (land - slowdown)`, so raising `slowdown_area_frac`
  starts the slow-down earlier (at a larger remaining distance).

* **Drone lands too early or too late.** Tune `visual_land_area_frac`.
  Geometry: for a flat target of area `A` viewed perpendicularly,
  area_frac ≈ `(A * fx²) / (W * H * d²)`. So halving the trigger
  area_frac roughly multiplies the trigger distance by √2. For a
  keyboard-sized target on the sjtu_drone front camera, `0.12` is
  ~25 cm; `0.05` is ~40 cm; `0.25` is ~17 cm. Measure once with your
  setup (look at the heartbeat line: it prints `area=…` per detection)
  and pick a value.

* **Bbox never gets big enough.** Wide-FOV cameras or small targets
  may saturate at a low `area_frac`. Lower `visual_land_area_frac`
  accordingly, or rely on `visual_approach_timeout_s` to LAND from
  whatever the closest stable hover was.
