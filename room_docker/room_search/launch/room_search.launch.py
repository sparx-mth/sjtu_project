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

        # Visual input (RGB-only — detector runs on the RGB topic; the
        # orchestrator subscribes to BOTH the detection stream and the
        # raw RGB stream so it can seed + propagate the LK tracker).
        DeclareLaunchArgument('detections_topic',  default_value='/perception/detections'),
        DeclareLaunchArgument('rgb_topic',         default_value='/simple_drone/front/image_raw'),

        # Phase radii / timings.
        DeclareLaunchArgument('nav_arrival_radius_m',  default_value='0.50'),
        DeclareLaunchArgument('rotation_rate_rad_s',   default_value='0.5'),
        DeclareLaunchArgument('max_rotation_revs',     default_value='2.0'),

        # Visual close-in (RGB bbox only — no depth).
        DeclareLaunchArgument('rgb_image_width',           default_value='640'),
        DeclareLaunchArgument('rgb_image_height',          default_value='360'),
        DeclareLaunchArgument('visual_kp_yaw',                default_value='0.9'),
        DeclareLaunchArgument('visual_max_yaw_rate',          default_value='0.6'),
        # Hysteresis on the YAW ↔ ADVANCE Schmitt trigger.
        # Real drone can't do yaw + forward at once, so we alternate;
        # exit < enter prevents flapping near the deadband.
        DeclareLaunchArgument('visual_yaw_deadband_enter',    default_value='0.20'),
        DeclareLaunchArgument('visual_yaw_deadband_exit',     default_value='0.08'),
        DeclareLaunchArgument('visual_vx_max',                default_value='0.20'),
        # bbox_area / image_area at which the linear vx ramp starts.
        DeclareLaunchArgument('visual_slowdown_area_frac', default_value='0.03'),
        # bbox_area / image_area that triggers LAND.
        DeclareLaunchArgument('visual_land_area_frac',     default_value='0.12'),
        DeclareLaunchArgument('visual_lost_hover_s',       default_value='0.6'),
        DeclareLaunchArgument('visual_giveup_s',           default_value='15.0'),
        DeclareLaunchArgument('visual_approach_timeout_s', default_value='90.0'),

        # Sparse Lucas-Kanade tracker (the inner loop's only work).
        DeclareLaunchArgument('track_max_corners',          default_value='80'),
        DeclareLaunchArgument('track_corner_quality',       default_value='0.05'),
        DeclareLaunchArgument('track_corner_min_dist',      default_value='5.0'),
        DeclareLaunchArgument('track_lk_win',               default_value='21'),
        DeclareLaunchArgument('track_lk_levels',            default_value='3'),
        DeclareLaunchArgument('track_min_matches',          default_value='8'),
        # If True, every fresh YOLO match while in VISUAL_APPROACH
        # re-seeds the tracker — bounds drift to the YOLO inter-arrival
        # time. Turn off only if YOLO is completely off after the first
        # hit (saves a YOLO re-anchor every yolo_min_dt seconds).
        DeclareLaunchArgument('track_re_seed_on_detection', default_value='true'),
        DeclareLaunchArgument('track_frame_buffer_len',     default_value='30'),
        DeclareLaunchArgument('track_seed_roi_margin',      default_value='0.10'),

        # YOLO knobs.
        DeclareLaunchArgument('yolo_model',     default_value='yolov8s-world.pt'),
        DeclareLaunchArgument('yolo_device',    default_value='cuda:0'),
        # YOLO is no longer in the inner control loop — sparse LK
        # optical flow propagates the bbox at camera rate. YOLO only
        # needs to fire often enough to (a) provide the initial bbox
        # during ROTATE_AND_SEARCH and (b) re-anchor the tracker if
        # it drifts. 1 Hz default is comfortable on a Jetson AGX while
        # everything else runs.
        DeclareLaunchArgument('yolo_min_dt',    default_value='1.0'),
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
            'detections_topic':          LaunchConfiguration('detections_topic'),
            'rgb_topic':                 LaunchConfiguration('rgb_topic'),
            'nav_arrival_radius_m':      LaunchConfiguration('nav_arrival_radius_m'),
            'rotation_rate_rad_s':       LaunchConfiguration('rotation_rate_rad_s'),
            'max_rotation_revs':         LaunchConfiguration('max_rotation_revs'),
            'rgb_image_width':           LaunchConfiguration('rgb_image_width'),
            'rgb_image_height':          LaunchConfiguration('rgb_image_height'),
            'visual_kp_yaw':             LaunchConfiguration('visual_kp_yaw'),
            'visual_max_yaw_rate':       LaunchConfiguration('visual_max_yaw_rate'),
            'visual_yaw_deadband_enter': LaunchConfiguration('visual_yaw_deadband_enter'),
            'visual_yaw_deadband_exit':  LaunchConfiguration('visual_yaw_deadband_exit'),
            'visual_vx_max':             LaunchConfiguration('visual_vx_max'),
            'visual_slowdown_area_frac': LaunchConfiguration('visual_slowdown_area_frac'),
            'visual_land_area_frac':     LaunchConfiguration('visual_land_area_frac'),
            'visual_lost_hover_s':       LaunchConfiguration('visual_lost_hover_s'),
            'visual_giveup_s':           LaunchConfiguration('visual_giveup_s'),
            'visual_approach_timeout_s': LaunchConfiguration('visual_approach_timeout_s'),

            'track_max_corners':          LaunchConfiguration('track_max_corners'),
            'track_corner_quality':       LaunchConfiguration('track_corner_quality'),
            'track_corner_min_dist':      LaunchConfiguration('track_corner_min_dist'),
            'track_lk_win':               LaunchConfiguration('track_lk_win'),
            'track_lk_levels':            LaunchConfiguration('track_lk_levels'),
            'track_min_matches':          LaunchConfiguration('track_min_matches'),
            'track_re_seed_on_detection': LaunchConfiguration('track_re_seed_on_detection'),
            'track_frame_buffer_len':     LaunchConfiguration('track_frame_buffer_len'),
            'track_seed_roi_margin':      LaunchConfiguration('track_seed_roi_margin'),
        }],
    )

    return LaunchDescription(args + [
        yolo, object_mapper, target_watcher, orchestrator,
    ])
