# perception_docker

Dedicated ROS2 Humble container for **P2 (MORE-style scene graph)** and
**P3 (YOLOv8 detector)** of the LLM-guided drone search pipeline.

Runs alongside the sim container (`sjtu_drone`) and the ROS1↔ROS2 bridge
(`ros_bridge_docker`). All three share `--net=host` and `ROS_DOMAIN_ID=20`,
so topic discovery is automatic.

```
sjtu_project/
├── sjtu_drone/            ← ROS2 sim (Gazebo + drone)
├── ros_bridge_docker/     ← ROS1 ↔ ROS2 bridge
├── falcon_docker/         ← ROS1 FALCON + adapter (upstream of bridge)
└── perception_docker/     ← THIS (ROS2: YOLO + semantic mapper)
    ├── Dockerfile
    ├── cyclonedds.xml
    ├── entrypoint.sh
    ├── run_perception.sh
    └── semantic_mapper/   ← ament_python package (mounted into container)
```

## Build & run

```bash
cd perception_docker
chmod +x run_perception.sh

# First run builds the image (~5 min) and the ament_python package,
# then drops you in a shell:
./run_perception.sh

# Or go straight to launch:
./run_perception.sh ros2 launch semantic_mapper semantic_pipeline.launch.py \
    target_text:=apple \
    bbox_xmin:=-25.0 bbox_ymin:=-25.0 bbox_xmax:=25.0 bbox_ymax:=25.0 \
    door_sigma_m:=0.45 door_cut_thresh:=0.6 \
    start_rviz:=true
```

## Topics

### Consumes (from sjtu_drone, ROS2 side)

| Topic                                            | Type                             |
|--------------------------------------------------|----------------------------------|
| `<drone_ns>/front/image_raw`                     | sensor_msgs/Image                |
| `<drone_ns>/front_depth/depth/image_raw`         | sensor_msgs/Image (32FC1)        |
| `<drone_ns>/front_depth/depth/camera_info`       | sensor_msgs/CameraInfo           |
| `<drone_ns>/gt_pose`                             | geometry_msgs/Pose               |

Check the actual topic names with `ros2 topic list` — the launch
arguments override each of these.

### Produces

| Topic                           | Type                                | QoS             |
|---------------------------------|-------------------------------------|-----------------|
| `/perception/detections`        | vision_msgs/Detection2DArray        | reliable, vol   |
| `/perception/target_seen`       | std_msgs/Bool                       | transient-local |
| `/perception/debug_image`       | sensor_msgs/Image                   | reliable, vol   |
| `/scene_graph`                  | std_msgs/String (JSON)              | transient-local |
| `/scene_graph/labeled`          | std_msgs/String (JSON) w/ labels    | transient-local |
| `/scene_graph/markers`          | visualization_msgs/MarkerArray      | reliable, vol   |
| `/scene_graph/bev`              | nav_msgs/OccupancyGrid              | transient-local |

The transient-local QoS means late-joining subscribers (RViz, future
oracle / RPT* nodes) immediately get the current state.

## LLM labeling

Rules-based labels are on by default. To use an LLM:

```bash
export OPENAI_API_KEY=sk-...
./run_perception.sh ros2 launch semantic_mapper semantic_pipeline.launch.py \
    use_llm:=true  llm_model:=gpt-4o-mini
```

Any OpenAI-compatible endpoint works:
```bash
export OPENAI_BASE_URL=http://host.docker.internal:11434/v1
./run_perception.sh ros2 launch semantic_mapper semantic_pipeline.launch.py \
    use_llm:=true  llm_model:=llama3.1:8b
```

## Bridging from FALCON (ROS1 → ROS2)

The FALCON-side topics that downstream planning needs (connectivity
graph, future `set_bbox` command) will cross the ROS1↔ROS2 bridge
automatically — `ros_bridge_docker/run_bridge.sh` already passes
`--bridge-all-1to2-topics --bridge-all-2to1-topics`. No changes needed
there.

When the LLM oracle and RPT\* nodes join, they can run in this same
perception container and talk to:
- `/scene_graph/labeled` (ours, native ROS2)
- `/falcon/connectivity_graph` (FALCON → bridge → ROS2)
- `/falcon/set_bbox` (ROS2 → bridge → FALCON)

## Tuning

Same knobs as the ROS1 version:

- `bev_resolution` — 0.15 m for apartments, 0.10 m for tight hallways.
- `door_radius` — footprint stamped into BEV to cut rooms. If rooms
  stay merged, increase it; if rooms fracture, decrease it.
- `z_slab_min/max` — which depth hits land in the BEV. Centre on cruise
  altitude.
- `door_classes` — defaults to `["door"]`. Stock YOLOv8 COCO doesn't
  have doors, so either fine-tune on DoorDetect-Dataset or pass
  `door_classes: ["refrigerator"]` to smoke-test the door-cutting.
