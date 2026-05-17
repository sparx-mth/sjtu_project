"""
room_search.launch.py — nav-to-room + spin-to-find + close-in-and-land.

Brings up:
  * (optional) yolo_detector         from semantic_mapper       start_yolo
  * (optional) object_mapper_node    from semantic_mapper       start_object_mapper
  * (optional) target_watcher_node   from semantic_mapper       start_target_watcher
                                     (with halt_duration_s=0 so its halt
                                      burst doesn't fight the orchestrator's
                                      cmd_vel during APPROACH/LAND)
  * room_search_orchestrator_node    from room_search          (always)

The detector chain is in this launch so you can run the whole task from
one place. If you already have perception_docker running, set
  start_yolo:=false  start_object_mapper:=false  start_target_watcher:=false
and we only launch the orchestrator.

NOTE: the YOLO-World vocabulary defaults to a hospital-flavoured list in
semantic_mapper. The default here adds 'keyboard' and a handful of common
desk items so the small open-vocab prompts are accurate. Override with
  yolo_vocabulary:="['keyboard','mouse','monitor']"
on the command line if you want a tighter set.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Smaller, focused vocabulary that includes the typical demo targets
# ("keyboard", "mouse", "laptop", "monitor") plus the room-furniture hits
# YOLO-World is reliable on.
ROOM_VOCAB = [
    "keyboard",
    "mouse",
    "laptop",
    "monitor",
    "tv",
    "cell phone",
    "book",
    "cup",
    "bottle",
    "person",
    "chair",
    "office chair",
    "desk",
    "table",
    "couch",
    "sofa",
]


def generate_launch_description():
    args = [
        # Mission knobs (the most important ones).
        DeclareLaunchArgument('target_object',     default_value='keyboard'),
        DeclareLaunchArgument('room_center_x',     default_value='4.0'),
        DeclareLaunchArgument('room_center_y',     default_value='5.0'),
        DeclareLaunchArgument('drone_ns',          default_value='/simple_drone'),

        # Pose source for the orchestrator (must be bridged ROS1->ROS2).
        DeclareLaunchArgument('pose_topic',        default_value='/odom_world'),
        DeclareLaunchArgument('pose_type',         default_value='odometry'),

        # Phase radii / timings (sane defaults; override if your map is tight).
        DeclareLaunchArgument('nav_arrival_radius_m',  default_value='0.50'),
        DeclareLaunchArgument('rotation_rate_rad_s',   default_value='0.5'),
        DeclareLaunchArgument('max_rotation_revs',     default_value='2.0'),
        DeclareLaunchArgument('approach_radius_m',     default_value='0.35'),
        DeclareLaunchArgument('approach_timeout_s',    default_value='90.0'),

        # YOLO knobs.
        DeclareLaunchArgument('yolo_model',     default_value='yolov8s-world.pt'),
        DeclareLaunchArgument('yolo_device',    default_value='cuda:0'),
        DeclareLaunchArgument('yolo_min_dt',    default_value='1.0'),
        DeclareLaunchArgument('yolo_conf',      default_value='0.30'),
        DeclareLaunchArgument(
            'yolo_vocabulary',
            default_value=str(ROOM_VOCAB)),

        # object_mapper knobs.
        DeclareLaunchArgument('min_observations', default_value='2'),
        DeclareLaunchArgument('dedup_radius_m',   default_value='0.40'),
        DeclareLaunchArgument('min_conf',         default_value='0.30'),

        # target_watcher: disable LLM by default (fuzzy substring works for
        # 'keyboard' vs 'keyboard'). halt_duration_s is forced to 0 so the
        # orchestrator owns /cmd_vel after target acquisition.
        DeclareLaunchArgument('use_llm',          default_value='false'),

        # Component toggles.
        DeclareLaunchArgument('start_yolo',           default_value='true'),
        DeclareLaunchArgument('start_object_mapper',  default_value='true'),
        DeclareLaunchArgument('start_target_watcher', default_value='true'),
    ]

    yolo = Node(
        package='semantic_mapper', executable='yolo_detector',
        name='yolo_detector', output='screen',
        condition=IfCondition(LaunchConfiguration('start_yolo')),
        parameters=[{
            'model_path':  LaunchConfiguration('yolo_model'),
            'device':      LaunchConfiguration('yolo_device'),
            'min_dt':      LaunchConfiguration('yolo_min_dt'),
            'conf_thresh': LaunchConfiguration('yolo_conf'),
            'vocabulary':  LaunchConfiguration('yolo_vocabulary'),
        }],
    )

    object_mapper = Node(
        package='semantic_mapper', executable='object_mapper_node',
        name='object_mapper', output='screen',
        condition=IfCondition(LaunchConfiguration('start_object_mapper')),
        parameters=[{
            'min_observations': LaunchConfiguration('min_observations'),
            'dedup_radius_m':   LaunchConfiguration('dedup_radius_m'),
            'min_conf':         LaunchConfiguration('min_conf'),
        }],
    )

    target_watcher = Node(
        package='semantic_mapper', executable='target_watcher_node',
        name='target_watcher', output='screen',
        condition=IfCondition(LaunchConfiguration('start_target_watcher')),
        parameters=[{
            'target_object':   LaunchConfiguration('target_object'),
            'use_llm':         LaunchConfiguration('use_llm'),
            # CRITICAL: the watcher's halt-burst would stamp on the
            # orchestrator's /cmd_vel during APPROACH/LAND. Disable it;
            # the orchestrator handles halt + land itself.
            'halt_duration_s': 0.0,
        }],
    )

    orchestrator = Node(
        package='room_search', executable='room_search_orchestrator_node',
        name='room_search_orchestrator', output='screen',
        parameters=[{
            'target_object':         LaunchConfiguration('target_object'),
            'room_center_x':         LaunchConfiguration('room_center_x'),
            'room_center_y':         LaunchConfiguration('room_center_y'),
            'drone_ns':              LaunchConfiguration('drone_ns'),
            'pose_topic':            LaunchConfiguration('pose_topic'),
            'pose_type':             LaunchConfiguration('pose_type'),
            'nav_arrival_radius_m':  LaunchConfiguration('nav_arrival_radius_m'),
            'rotation_rate_rad_s':   LaunchConfiguration('rotation_rate_rad_s'),
            'max_rotation_revs':     LaunchConfiguration('max_rotation_revs'),
            'approach_radius_m':     LaunchConfiguration('approach_radius_m'),
            'approach_timeout_s':    LaunchConfiguration('approach_timeout_s'),
        }],
    )

    return LaunchDescription(args + [
        yolo, object_mapper, target_watcher, orchestrator,
    ])
