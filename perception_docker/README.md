# perception_docker

ROS 2 Humble container for **P2 (scene-graph groundwork)** of the
LLM-guided drone search pipeline. It consumes FALCON's 3D voxel map
(bridged from ROS 1), projects it to 2D, cuts it at the hardcoded
doors, and publishes the rooms + Voronoi skeleton + a JSON scene
graph.

YOLO and LLM room-labelling are **not** in this container yet — the
scene-graph JSON carries `label: null` and `objects: []` stubs so they
can be bolted on later without schema changes.

```
sjtu_project/
├── sjtu_drone/           ← ROS 2 sim (Gazebo + drone)
├── ros_bridge_docker/    ← ROS 1 ↔ ROS 2 bridge
├── falcon_docker/        ← ROS 1 FALCON + adapter
└── perception_docker/    ← THIS (ROS 2: semantic mapper)
    ├── Dockerfile
    ├── cyclonedds.xml
    ├── entrypoint.sh
    ├── run_perception.sh
    └── semantic_mapper/  ← ament_python pkg (bind-mounted, not baked in)
```

## Build & run

```bash
cd perception_docker
chmod +x run_perception.sh

# First run builds the image (~5 min) + the ament_python package, then
# drops you in a shell. Subsequent runs are instant.
./run_perception.sh

# Straight to launch:
./run_perception.sh ros2 launch semantic_mapper semantic_pipeline.launch.py \
    start_rviz:=true
```

All three containers share `--net=host` + `ROS_DOMAIN_ID=20`, so topic
discovery is automatic.

**After editing code on the host, just re-run `./run_perception.sh`.**
The `semantic_mapper/` tree is bind-mounted and `colcon build
--symlink-install` runs inside the container on every start. No image
rebuild needed unless the `Dockerfile` itself changes.

## Topics

### Consumed — FALCON's voxel map, bridged from ROS 1

| Topic                             | Type                       | Notes              |
|-----------------------------------|----------------------------|--------------------|
| `/voxel_mapping/occupancy_all`    | sensor_msgs/PointCloud2    | Occupied voxels    |
| `/voxel_mapping/free_all`         | sensor_msgs/PointCloud2    | Free voxels (opt.) |

If your FALCON fork uses different names, pass them via the launch
args `occ_topic:=…` / `free_topic:=…`. Check with `rostopic list |
grep -iE '(voxel|occup|free)'` inside the FALCON container.

### Produced

| Topic                    | Type                            | QoS              |
|--------------------------|---------------------------------|------------------|
| `/scene_graph/bev`       | nav_msgs/OccupancyGrid          | transient-local  |
| `/scene_graph/markers`   | visualization_msgs/MarkerArray  | reliable         |
| `/scene_graph`           | std_msgs/String (JSON)          | transient-local  |

Transient-local means RViz and future oracle / RPT\* nodes get the
current state the moment they subscribe.

## Tuning

Edit via launch args:

- `bev_resolution` — 0.15 m default. Drop to 0.10 m for tight
  hallways; raise for speed.
- `door_wall_m` — radius of the wall disc stamped at each door.
  Default 0.60 m (≈ ½ a door width). If two rooms share one colour
  across a door in RViz, bump it; if a single room is split into two
  colours, drop it.
- `z_slab_min` / `z_slab_max` — vertical band of FALCON voxels that
  count toward the BEV. Defaults 0.30 / 1.80 m.
- `min_room_cells` — drops specks (default 80, ≈ 1.8 m² at 0.15 m).
- `corridor_thresh_m` — rooms with median clearance below this get
  tagged `corridor` instead of `room`. Default 1.20 m.
- `bbox_xmin/xmax/ymin/ymax` — the BEV window. Defaults frame the
  hospital world.

See `semantic_mapper/README.md` for the debugging workflow when
segmentation looks off.

## Bridging from FALCON

`ros_bridge_docker/run_bridge.sh` already bridges 1→2 and 2→1
topics. The semantic mapper only needs 1→2 for the voxel point-clouds;
no changes there.

Future nodes (oracle, RPT\*) will live in this same container and can
talk to:

- `/scene_graph`, `/scene_graph/markers`, `/scene_graph/bev` — native ROS 2.
- `/falcon/connectivity_graph` — FALCON → bridge → ROS 2.
- `/falcon/set_bbox` — ROS 2 → bridge → FALCON (for per-room
  exploration bounds).