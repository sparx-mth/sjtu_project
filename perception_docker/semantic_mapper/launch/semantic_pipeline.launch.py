"""
semantic_pipeline.launch.py — FALCON BEV -> scene graph (+ object mapper +
                              YOLO + room classifier + LLM oracle + planner
                              + tracker + goal sampler).

Launches:
  * yolo_detector         RGB -> Detection2DArray @ 1 Hz        (toggle: start_yolo)
  * semantic_mapper_node  Voronoi + rooms + scene graph (with τ_r, F_r, objects)
  * object_mapper_node    YOLO detections -> 2D world XY
  * room_classifier_node  objects/room -> LLM -> room label     (toggle: start_llm)
  * llm_oracle_node       target + rooms -> prob distribution   (toggle: start_llm)
  * path_planner_node     A* on FALCON BEV -> /planned_path     (toggle: start_planner)
  * path_tracker_node     /planned_path -> /simple_drone/cmd_vel
                                                                (toggle: start_tracker)
  * goal_sampler_node     samples a room, drives planner; dwells after arrival
                          so FALCON can explore                  (toggle: start_sampler)
  * rviz2                                                        (toggle: start_rviz)

Search loop:  sample → drive to room → arrive → handoff to FALCON for
`dwell_after_arrival_s` seconds → re-sample. The tracker goes silent on
/cmd_vel during dwell so it doesn't fight FALCON's exploration.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('bev_topic',   default_value='/falcon/bev_2d'),
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
        DeclareLaunchArgument('min_observations', default_value='2'),
        DeclareLaunchArgument('dedup_radius_m',   default_value='0.70'),
        DeclareLaunchArgument('min_conf',         default_value='0.25'),

        # YOLO knobs.
        DeclareLaunchArgument('yolo_model',  default_value='yolov8s-world.pt'),
        DeclareLaunchArgument('yolo_device', default_value='cuda:0'),
        DeclareLaunchArgument('yolo_min_dt', default_value='1.0'),

        # LLM knobs.
        DeclareLaunchArgument('target_object',      default_value='car keys'),
        DeclareLaunchArgument('oracle_period_s',    default_value='10.0'),
        DeclareLaunchArgument('classifier_rate_hz', default_value='1.0'),

        # Path planner knobs.
        DeclareLaunchArgument('odom_topic',         default_value='/odom_world'),
        DeclareLaunchArgument('goal_topic',         default_value='/move_base_simple/goal'),
        DeclareLaunchArgument('inflation_radius_m', default_value='0.35'),
        DeclareLaunchArgument('plan_timeout_s',     default_value='3.0'),
        DeclareLaunchArgument('unknown_is_free',    default_value='false'),

        # Path tracker knobs (FALCON dynamics cap = 0.2 m/s, default matches).
        DeclareLaunchArgument('cmd_vel_topic', default_value='/simple_drone/cmd_vel'),
        DeclareLaunchArgument('lookahead_m',   default_value='0.8'),
        DeclareLaunchArgument('max_lin_vel',   default_value='0.2'),
        DeclareLaunchArgument('max_ang_vel',   default_value='0.6'),
        DeclareLaunchArgument('goal_tol_m',    default_value='0.3'),
        DeclareLaunchArgument('control_hz',    default_value='20.0'),

        # Goal sampler knobs.
        DeclareLaunchArgument('arrival_tol_m',         default_value='0.6'),
        DeclareLaunchArgument('plan_grace_s',          default_value='5.0'),
        DeclareLaunchArgument('max_pursue_s',          default_value='60.0'),
        DeclareLaunchArgument('dwell_after_arrival_s', default_value='15.0'),
        DeclareLaunchArgument('min_prob',              default_value='0.01'),
        DeclareLaunchArgument('sampler_seed',          default_value='-1'),

        DeclareLaunchArgument('start_rviz',           default_value='false'),
        DeclareLaunchArgument('start_yolo',           default_value='true'),
        DeclareLaunchArgument('start_llm',            default_value='true'),
        DeclareLaunchArgument('start_target_watcher', default_value='true'),
        DeclareLaunchArgument('start_planner',        default_value='true'),
        DeclareLaunchArgument('start_tracker',        default_value='true'),
        DeclareLaunchArgument('start_sampler',        default_value='true'),
    ]

    yolo = Node(
        package='semantic_mapper', executable='yolo_detector',
        name='yolo_detector', output='screen',
        condition=IfCondition(LaunchConfiguration('start_yolo')),
        parameters=[{
            'model_path': LaunchConfiguration('yolo_model'),
            'device':     LaunchConfiguration('yolo_device'),
            'min_dt':     LaunchConfiguration('yolo_min_dt'),
        }],
    )

    mapper = Node(
        package='semantic_mapper', executable='semantic_mapper_node',
        name='semantic_mapper', output='screen',
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
        package='semantic_mapper', executable='object_mapper_node',
        name='object_mapper', output='screen',
        parameters=[{
            'world_frame':      LaunchConfiguration('world_frame'),
            'min_observations': LaunchConfiguration('min_observations'),
            'dedup_radius_m':   LaunchConfiguration('dedup_radius_m'),
            'min_conf':         LaunchConfiguration('min_conf'),
        }],
    )

    room_classifier = Node(
        package='semantic_mapper', executable='room_classifier_node',
        name='room_classifier', output='screen',
        condition=IfCondition(LaunchConfiguration('start_llm')),
        parameters=[{'tick_rate_hz': LaunchConfiguration('classifier_rate_hz')}],
    )

    llm_oracle = Node(
        package='semantic_mapper', executable='llm_oracle_node',
        name='llm_oracle', output='screen',
        condition=IfCondition(LaunchConfiguration('start_llm')),
        parameters=[{
            'target_object': LaunchConfiguration('target_object'),
            'tick_period_s': LaunchConfiguration('oracle_period_s'),
        }],
    )

    target_watcher = Node(
        package='semantic_mapper', executable='target_watcher_node',
        name='target_watcher', output='screen',
        condition=IfCondition(LaunchConfiguration('start_target_watcher')),
        parameters=[{'target_object': LaunchConfiguration('target_object')}],
    )

    path_planner = Node(
        package='semantic_mapper', executable='path_planner_node',
        name='path_planner', output='screen',
        condition=IfCondition(LaunchConfiguration('start_planner')),
        parameters=[{
            'bev_topic':          LaunchConfiguration('bev_topic'),
            'odom_topic':         LaunchConfiguration('odom_topic'),
            'goal_topic':         LaunchConfiguration('goal_topic'),
            'frame_id':           LaunchConfiguration('world_frame'),
            'inflation_radius_m': LaunchConfiguration('inflation_radius_m'),
            'plan_timeout_s':     LaunchConfiguration('plan_timeout_s'),
            'unknown_is_free':    LaunchConfiguration('unknown_is_free'),
        }],
    )

    path_tracker = Node(
        package='semantic_mapper', executable='path_tracker_node',
        name='path_tracker', output='screen',
        condition=IfCondition(LaunchConfiguration('start_tracker')),
        parameters=[{
            'odom_topic':    LaunchConfiguration('odom_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'frame_id':      LaunchConfiguration('world_frame'),
            'lookahead_m':   LaunchConfiguration('lookahead_m'),
            'max_lin_vel':   LaunchConfiguration('max_lin_vel'),
            'max_ang_vel':   LaunchConfiguration('max_ang_vel'),
            'goal_tol_m':    LaunchConfiguration('goal_tol_m'),
            'control_hz':    LaunchConfiguration('control_hz'),
        }],
    )

    goal_sampler = Node(
        package='semantic_mapper', executable='goal_sampler_node',
        name='goal_sampler', output='screen',
        condition=IfCondition(LaunchConfiguration('start_sampler')),
        parameters=[{
            'goal_topic':            LaunchConfiguration('goal_topic'),
            'frame_id':              LaunchConfiguration('world_frame'),
            'arrival_tol_m':         LaunchConfiguration('arrival_tol_m'),
            'plan_grace_s':          LaunchConfiguration('plan_grace_s'),
            'max_pursue_s':          LaunchConfiguration('max_pursue_s'),
            'dwell_after_arrival_s': LaunchConfiguration('dwell_after_arrival_s'),
            'min_prob':              LaunchConfiguration('min_prob'),
            'seed':                  LaunchConfiguration('sampler_seed'),
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
        yolo, mapper, object_mapper, room_classifier, llm_oracle,
        target_watcher, path_planner, path_tracker, goal_sampler, rviz,
    ])