# perception_docker

ROS 2 Humble container. Consumes FALCON's 2D BEV (bridged from ROS 1),
runs YOLO-World + Voronoi room segmentation, and produces a scene
graph + per-room LLM labels + target-conditioned room probabilities.

```
perception_docker/
├── Dockerfile
├── cyclonedds.xml
├── entrypoint.sh
├── run_perception.sh
└── semantic_mapper/        ← ament_python pkg (bind-mounted)
```

## Build & run

```bash
# one-time on the host: start a local LLM
ollama pull qwen2.5:3b-instruct
ollama serve &
```

```bash
# rebuild + launch the pipeline
./run_perception.sh ros2 launch semantic_mapper semantic_pipeline.launch.py \
    target_object:='toilet'  start_rviz:=true
```

Retarget at runtime: `ros2 param set /llm_oracle target_object "apple"`.

Before debugging ROS, verify the LLM first:
```bash
./run_perception.sh ros2 run semantic_mapper llm_check
```

## LLM backend (env vars, all optional)

| Var              | Default                        |
|------------------|--------------------------------|
| `LLM_BACKEND`    | `ollama`  (or `openai`)        |
| `LLM_BASE_URL`   | `http://localhost:11434`       |
| `LLM_MODEL`      | `qwen2.5:3b-instruct`          |
| `LLM_API_KEY`    | empty (only for openai-compat) |
| `LLM_TIMEOUT_S`  | `30`                           |

`--net=host` + `ollama serve` on the host = container reaches it as
`localhost`, no extra config.

Dockerfile needs `RUN pip3 install --no-cache-dir requests`.

## Topics

**Consumed**

| Topic                       | Type                         |
|-----------------------------|------------------------------|
| `/falcon/bev_2d`            | nav_msgs/OccupancyGrid       |
| `/map_ros/pose`             | geometry_msgs/PoseStamped    |
| `/simple_drone/front/image_raw` | sensor_msgs/Image        |
| `/map_ros/depth`            | sensor_msgs/Image            |

**Produced**

| Topic                                     | Type                              |
|-------------------------------------------|-----------------------------------|
| `/scene_graph`                            | std_msgs/String (JSON)            |
| `/scene_graph/markers`                    | visualization_msgs/MarkerArray    |
| `/perception/detections`                  | vision_msgs/Detection2DArray      |
| `/perception/objects`                     | std_msgs/String (JSON)            |
| `/perception/object_markers`              | visualization_msgs/MarkerArray    |
| `/semantic_mapper/room_labels`            | std_msgs/String (JSON)            |
| `/semantic_mapper/room_labels/markers`    | visualization_msgs/MarkerArray    |
| `/llm_oracle/probabilities`               | std_msgs/String (JSON)            |
| `/llm_oracle/markers`                     | visualization_msgs/MarkerArray    |

Every room in `/scene_graph` carries `time_in_room_s` (τ_r),
`frontier_clusters` (F_r), and the list of `objects` observed inside it.

## Launch args (common)

| Arg                          | Default              |
|------------------------------|----------------------|
| `target_object`              | `car keys`           |
| `oracle_period_s`            | `10.0`               |
| `classifier_rate_hz`         | `1.0`                |
| `frontier_min_cluster_cells` | `4`                  |
| `start_yolo` / `start_llm` / `start_rviz` | `true` / `true` / `false` |

## Troubleshooting

| Symptom                                  | First thing to try                             |
|------------------------------------------|-------------------------------------------------|
| `LLM server did not respond to ping`     | `curl http://localhost:11434/api/tags` on host; confirm `--net=host` |
| `source: uniform_fallback` every tick    | Check the `WARN` above it. Usually bad JSON from too-small a model → try `qwen2.5:3b-instruct` or bigger |
| Classifier says `unknown` for every room | `ros2 topic echo /perception/objects --once` — YOLO may not have any confirmed objects yet |
| τ_r stays at 0                           | `ros2 topic hz /map_ros/pose`; check `.current_room` in `/scene_graph` — `null` means drone is in a door cut disk |
| F_r stays at 0                           | Lower `frontier_min_cluster_cells` to 2 or 3    |
| Room IDs churn mid-run                   | BEV geometry changed — keep `bev_xmin/xmax/...` fixed in `gazebo_exploration.launch` |
| RViz missing new markers                 | Add `MarkerArray /llm_oracle/markers` and `MarkerArray /semantic_mapper/room_labels/markers` |

## Notes

- `/scene_graph`, `/semantic_mapper/room_labels`, `/perception/objects`,
  and `/llm_oracle/probabilities` are all `TRANSIENT_LOCAL` — late
  subscribers get the current state on connect.
- Room classifier caches by frozenset of observed object classes; the
  LLM is only called when a room's class set actually changes.
- Oracle publishes a uniform distribution if the LLM fails, so RPT\*
  always has a valid input.