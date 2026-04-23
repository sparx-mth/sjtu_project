"""
semantic_pipeline.launch.py — FALCON BEV -> scene graph (+ object mapper +
                              YOLO + room classifier + LLM oracle).

Launches:
  * yolo_detector         RGB -> Detection2DArray @ 1 Hz        (toggle: start_yolo)
  * semantic_mapper_node  Voronoi + rooms + scene graph (with τ_r, F_r, objects)
  * object_mapper_node    YOLO detections -> 2D world XY
  * room_classifier_node  objects/room -> LLM -> room label     (toggle: start_llm)
  * llm_oracle_node       target + rooms -> prob distribution   (toggle: start_llm)
  * rviz2                                                       (toggle: start_rviz)

The LLM nodes talk to an LLM backend configured via environment
variables passed into the container (see run_perception.sh):
    LLM_BACKEND   ollama | openai
    LLM_BASE_URL  default http://localhost:11434  (Ollama host-net)
    LLM_MODEL     default qwen2.5:3b-instruct
    LLM_API_KEY   (only required for openai-compat servers that want one)

To launch with a different target:
    ros2 launch semantic_mapper semantic_pipeline.launch.py \
        target_object:="coffee mug"

Or change it at runtime:
    ros2 param set /llm_oracle target_object "apple"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('bev_topic',
            default_value='/falcon/bev_2d'),
        DeclareLaunchArgument('world_frame', default_value='world'),

        # Door cutting / linking / discovery radii (all metres).
        DeclareLaunchArgument('door_cut_m',          default_value='0.60'),
        DeclareLaunchArgument('door_match_radius_m', default_value='0.90'),
        DeclareLaunchArgument('door_discover_m',     default_value='0.30'),
        DeclareLaunchArgument('min_room_cells',      default_value='40'),
        DeclareLaunchArgument('room_iou_threshold',  default_value='0.15'),
        DeclareLaunchArgument('tick_rate',           default_value='2.0'),
        DeclareLaunchArgument('frontier_min_cluster_cells',
                              default_value='4'),

        # Object mapper knobs.
        DeclareLaunchArgument('min_observations',    default_value='2'),
        DeclareLaunchArgument('dedup_radius_m',      default_value='0.70'),
        DeclareLaunchArgument('min_conf',            default_value='0.25'),

        # YOLO knobs.
        DeclareLaunchArgument('yolo_model',  default_value='yolov8s-world.pt'),
        DeclareLaunchArgument('yolo_device', default_value='cuda:0'),
        DeclareLaunchArgument('yolo_min_dt', default_value='1.0'),  # 1 Hz

        # LLM knobs.
        DeclareLaunchArgument('target_object',      default_value='car keys'),
        DeclareLaunchArgument('oracle_period_s',    default_value='10.0'),
        DeclareLaunchArgument('classifier_rate_hz', default_value='1.0'),

        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('start_yolo', default_value='true'),
        DeclareLaunchArgument('start_llm',  default_value='true'),
    ]

    yolo = Node(
        package='semantic_mapper',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_yolo')),
        parameters=[{
            'model_path': LaunchConfiguration('yolo_model'),
            'device':     LaunchConfiguration('yolo_device'),
            'min_dt':     LaunchConfiguration('yolo_min_dt'),
        }],
    )

    mapper = Node(
        package='semantic_mapper',
        executable='semantic_mapper_node',
        name='semantic_mapper',
        output='screen',
        parameters=[{
            k: LaunchConfiguration(k) for k in (
                'bev_topic', 'world_frame',
                'door_cut_m', 'door_match_radius_m', 'door_discover_m',
                'min_room_cells', 'room_iou_threshold', 'tick_rate',
                'frontier_min_cluster_cells',
            )
        }],
    )

    object_mapper = Node(
        package='semantic_mapper',
        executable='object_mapper_node',
        name='object_mapper',
        output='screen',
        parameters=[{
            'world_frame':      LaunchConfiguration('world_frame'),
            'min_observations': LaunchConfiguration('min_observations'),
            'dedup_radius_m':   LaunchConfiguration('dedup_radius_m'),
            'min_conf':         LaunchConfiguration('min_conf'),
        }],
    )

    room_classifier = Node(
        package='semantic_mapper',
        executable='room_classifier_node',
        name='room_classifier',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_llm')),
        parameters=[{
            'tick_rate_hz': LaunchConfiguration('classifier_rate_hz'),
        }],
    )

    llm_oracle = Node(
        package='semantic_mapper',
        executable='llm_oracle_node',
        name='llm_oracle',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_llm')),
        parameters=[{
            'target_object':  LaunchConfiguration('target_object'),
            'tick_period_s':  LaunchConfiguration('oracle_period_s'),
        }],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('semantic_mapper'),
            'rviz', 'semantic_mapper.rviz'])],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        output='screen',
    )

    return LaunchDescription(args + [
        yolo, mapper, object_mapper, room_classifier, llm_oracle, rviz,
    ])