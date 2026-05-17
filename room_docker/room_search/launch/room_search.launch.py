"""
room_search.launch.py — nav-to-room + spin-to-find + visual close-in + land.

Brings up:
  * (optional) yolo_detector         from semantic_mapper       start_yolo
                                     (default min_dt lowered to 0.25 s so
                                      detections arrive fast enough for the
                                      visual servo loop)
  * (optional) object_mapper_node    from semantic_mapper       start_object_mapper
                                     (kept for diagnostics / target_watcher
                                      input; the orchestrator does NOT use
                                      its world XY in the visual close-in)
  * (optional) target_watcher_node   from semantic_mapper       start_target_watcher
                                     (with halt_duration_s=0 so its halt
                                      burst doesn't fight the orchestrator's
                                      cmd_vel during VISUAL_APPROACH/LAND)
  * room_search_orchestrator_node    from room_search          (always)

Set start_yolo/object_mapper/target_watcher to false if you already run
perception_docker separately and just want this launch to add the
orchestrator.
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
        # Mission knobs.
        DeclareLaunchArgument('target_object',     default_value='keyboard'),
        DeclareLaunchArgument('room_center_x',     default_value='4.0'),
        DeclareLaunchArgument('room_center_y',     default_value='5.0'),
        DeclareLaunchArgument('drone_ns',          default_value='/simple_drone'),

        # Pose source for the orchestrator (must be bridged ROS1->ROS2).
        DeclareLaunchArgument('pose_topic',        default_value='/odom_world'),
        DeclareLaunchArgument('pose_type',         default_value='odometry'),

        # Visual inputs (depth is already bridged for perception_docker).
        DeclareLaunchArgument('detections_topic',  default_value='/perception/detections'),
        DeclareLaunchArgument('depth_topic',       default_value='/map_ros/depth'),

        # Phase radii / timings.
        DeclareLaunchArgument('nav_arrival_radius_m',  default_value='0.50'),
        DeclareLaunchArgument('rotation_rate_rad_s',   default_value='0.5'),
        DeclareLaunchArgument('max_rotation_revs',     default_value='2.0'),

        # Visual close-in (the new bits — read by room_search_orchestrator).
        DeclareLaunchArgument('rgb_image_width',         default_value='640'),
        DeclareLaunchArgument('rgb_image_height',        default_value='360'),
        DeclareLaunchArgument('visual_kp_yaw',           default_value='0.9'),
        DeclareLaunchArgument('visual_max_yaw_rate',     default_value='0.6'),
        DeclareLaunchArgument('visual_yaw_deadband',     default_value='0.20'),
        DeclareLaunchArgument('visual_vx_max',           default_value='0.20'),
        DeclareLaunchArgument('visual_slowdown_start_m', default_value='1.50'),
        DeclareLaunchArgument('visual_land_depth_m',     default_value='0.45'),
        DeclareLaunchArgument('visual_lost_hover_s',     default_value='0.6'),
        DeclareLaunchArgument('visual_giveup_s',         default_value='15.0'),

        # YOLO knobs.
        DeclareLaunchArgument('yolo_model',     default_value='yolov8s-world.pt'),
        DeclareLaunchArgument('yolo_device',    default_value='cuda:0'),
        # IMPORTANT for the visual servo loop: at 1 Hz YOLO the bbox is
        # ~half a metre stale per tick when advancing at 0.2 m/s, which
        # is on the order of the bbox size for a keyboard at close
        # range. 4 Hz keeps the closed loop crisp without overrunning
        # an RTX-class GPU.
        DeclareLaunchArgument('yolo_min_dt',    default_value='0.25'),
        DeclareLaunchArgument('yolo_conf',      default_value='0.30'),
        DeclareLaunchArgument(
            'yolo_vocabulary',
            default_value=str(ROOM_VOCAB)),

        # object_mapper knobs (kept for diagnostics / target_watcher).
        DeclareLaunchArgument('min_observations', default_value='2'),
        DeclareLaunchArgument('dedup_radius_m',   default_value='0.40'),
        DeclareLaunchArgument('min_conf',         default_value='0.30'),

        # target_watcher: disable LLM by default; halt_duration_s forced
        # to 0 so the orchestrator owns /cmd_vel after target acquisition.
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
            'detections_topic':      LaunchConfiguration('detections_topic'),
            'depth_topic':           LaunchConfiguration('depth_topic'),
            'nav_arrival_radius_m':  LaunchConfiguration('nav_arrival_radius_m'),
            'rotation_rate_rad_s':   LaunchConfiguration('rotation_rate_rad_s'),
            'max_rotation_revs':     LaunchConfiguration('max_rotation_revs'),
            'rgb_image_width':       LaunchConfiguration('rgb_image_width'),
            'rgb_image_height':      LaunchConfiguration('rgb_image_height'),
            'visual_kp_yaw':         LaunchConfiguration('visual_kp_yaw'),
            'visual_max_yaw_rate':   LaunchConfiguration('visual_max_yaw_rate'),
            'visual_yaw_deadband':   LaunchConfiguration('visual_yaw_deadband'),
            'visual_vx_max':         LaunchConfiguration('visual_vx_max'),
            'visual_slowdown_start_m': LaunchConfiguration('visual_slowdown_start_m'),
            'visual_land_depth_m':   LaunchConfiguration('visual_land_depth_m'),
            'visual_lost_hover_s':   LaunchConfiguration('visual_lost_hover_s'),
            'visual_giveup_s':       LaunchConfiguration('visual_giveup_s'),
        }],
    )

    return LaunchDescription(args + [
        yolo, object_mapper, target_watcher, orchestrator,
    ])
