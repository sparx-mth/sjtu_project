"""
semantic_pipeline.launch.py

Starts YOLO-World detector + MORE-style semantic mapper + room labeler,
optionally with RViz preconfigured for top-down hospital-scale viewing.

Usage:
    ros2 launch semantic_mapper semantic_pipeline.launch.py \\
        target_text:=apple  rgb_topic:=/simple_drone/front/image_raw \\
        bbox_xmax:=25.0 bbox_ymax:=25.0  start_rviz:=true
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("drone_ns",        default_value="/simple_drone"),
        DeclareLaunchArgument("rgb_topic",       default_value="/simple_drone/front/image_raw"),
        DeclareLaunchArgument("depth_topic",     default_value="/simple_drone/front_depth/depth/image_raw"),
        DeclareLaunchArgument("cam_info_topic",  default_value="/simple_drone/front_depth/depth/camera_info"),
        DeclareLaunchArgument("pose_topic",      default_value="/simple_drone/gt_pose"),

        DeclareLaunchArgument("target_text",     default_value="apple"),
        DeclareLaunchArgument("yolo_model",      default_value="yolov8s-world.pt"),
        DeclareLaunchArgument("yolo_device",     default_value="cuda:0"),
        DeclareLaunchArgument("yolo_conf",       default_value="0.10"),
        DeclareLaunchArgument("yolo_min_dt",     default_value="1.0"),

        # ── BEV bounds & resolution ──
        DeclareLaunchArgument("bbox_xmin",       default_value="-20.0"),
        DeclareLaunchArgument("bbox_ymin",       default_value="-20.0"),
        DeclareLaunchArgument("bbox_xmax",       default_value="20.0"),
        DeclareLaunchArgument("bbox_ymax",       default_value="20.0"),
        DeclareLaunchArgument("bev_resolution",  default_value="0.15"),

        # ── MORE-style room segmentation ──
        # Door Gaussian σ in metres. Set ≈ door half-width.
        DeclareLaunchArgument("door_sigma_m",    default_value="0.45"),
        # Threshold on max-along-edge Gaussian (0..1). Higher = fewer cuts
        # (rooms more likely to merge); lower = more cuts (rooms split more).
        DeclareLaunchArgument("door_cut_thresh", default_value="0.6"),
        # MORE Alg. 1 sparsification length in cells.
        DeclareLaunchArgument("voronoi_sparsify_cells", default_value="8"),
        # Reject regions smaller than this (cells).
        DeclareLaunchArgument("min_room_cells",  default_value="60"),
        # open_space heuristics
        DeclareLaunchArgument("open_space_area_ratio",     default_value="4.0"),
        DeclareLaunchArgument("open_space_frontier_frac",  default_value="0.5"),
        # MORE object-assignment exponent (paper §S.4 uses 1.3)
        DeclareLaunchArgument("object_lambda",   default_value="1.3"),
        # IoU threshold for inheriting old room IDs across ticks
        DeclareLaunchArgument("room_iou_match",  default_value="0.30"),

        # ── LLM labeling ──
        DeclareLaunchArgument("use_llm",         default_value="false"),
        DeclareLaunchArgument("llm_model",       default_value="gpt-4o-mini"),

        # ── Visualisation ──
        DeclareLaunchArgument("publish_voronoi_mesh", default_value="false"),
        # Marker scales (metres). Defaults assume a ~40–50 m map.
        DeclareLaunchArgument("viz_room_sphere_radius_m", default_value="0.8"),
        DeclareLaunchArgument("viz_room_text_size_m",     default_value="1.2"),
        DeclareLaunchArgument("viz_room_outline_width_m", default_value="0.15"),
        DeclareLaunchArgument("viz_room_edge_width_m",    default_value="0.25"),
        DeclareLaunchArgument("viz_object_size_m",        default_value="0.4"),
        DeclareLaunchArgument("viz_object_text_size_m",   default_value="0.5"),
        DeclareLaunchArgument("viz_door_radius_m",        default_value="0.4"),
        DeclareLaunchArgument("viz_door_text_size_m",     default_value="0.6"),
        DeclareLaunchArgument("viz_door_height_m",        default_value="2.5"),
        DeclareLaunchArgument("viz_fill_rooms",           default_value="true"),
        DeclareLaunchArgument("viz_fill_open_space",      default_value="true"),
        DeclareLaunchArgument("viz_fill_alpha",           default_value="0.45"),
        DeclareLaunchArgument("viz_open_space_alpha",     default_value="0.18"),
        DeclareLaunchArgument("viz_max_fill_cells",       default_value="8000"),

        # Set start_rviz:=true to also launch RViz with the bundled config.
        DeclareLaunchArgument("start_rviz",               default_value="false"),
    ]

    yolo = Node(
        package="semantic_mapper",
        executable="yolo_detector",
        name="yolo_detector",
        output="screen",
        parameters=[{
            "rgb_topic":   LaunchConfiguration("rgb_topic"),
            "model_path":  LaunchConfiguration("yolo_model"),
            "device":      LaunchConfiguration("yolo_device"),
            "target_text": LaunchConfiguration("target_text"),
            "conf_thresh": LaunchConfiguration("yolo_conf"),
            "min_dt":      LaunchConfiguration("yolo_min_dt"),
        }],
    )

    mapper = Node(
        package="semantic_mapper",
        executable="semantic_mapper_node",
        name="semantic_mapper",
        output="screen",
        parameters=[{
            "drone_ns":       LaunchConfiguration("drone_ns"),
            "depth_topic":    LaunchConfiguration("depth_topic"),
            "cam_info_topic": LaunchConfiguration("cam_info_topic"),
            "pose_topic":     LaunchConfiguration("pose_topic"),
            "bbox_xmin":      LaunchConfiguration("bbox_xmin"),
            "bbox_ymin":      LaunchConfiguration("bbox_ymin"),
            "bbox_xmax":      LaunchConfiguration("bbox_xmax"),
            "bbox_ymax":      LaunchConfiguration("bbox_ymax"),
            "bev_resolution": LaunchConfiguration("bev_resolution"),
            "z_slab_min":     0.3,
            "z_slab_max":     1.8,
            "tick_rate":      2.0,
            "cam_offset_x":   0.2,

            "door_sigma_m":             LaunchConfiguration("door_sigma_m"),
            "door_cut_thresh":          LaunchConfiguration("door_cut_thresh"),
            "voronoi_sparsify_cells":   LaunchConfiguration("voronoi_sparsify_cells"),
            "min_room_cells":           LaunchConfiguration("min_room_cells"),
            "open_space_area_ratio":    LaunchConfiguration("open_space_area_ratio"),
            "open_space_frontier_frac": LaunchConfiguration("open_space_frontier_frac"),
            "object_lambda":            LaunchConfiguration("object_lambda"),
            "room_iou_match":           LaunchConfiguration("room_iou_match"),
            "publish_voronoi_mesh":     LaunchConfiguration("publish_voronoi_mesh"),

            "viz_room_sphere_radius_m": LaunchConfiguration("viz_room_sphere_radius_m"),
            "viz_room_text_size_m":     LaunchConfiguration("viz_room_text_size_m"),
            "viz_room_outline_width_m": LaunchConfiguration("viz_room_outline_width_m"),
            "viz_room_edge_width_m":    LaunchConfiguration("viz_room_edge_width_m"),
            "viz_object_size_m":        LaunchConfiguration("viz_object_size_m"),
            "viz_object_text_size_m":   LaunchConfiguration("viz_object_text_size_m"),
            "viz_door_radius_m":        LaunchConfiguration("viz_door_radius_m"),
            "viz_door_text_size_m":     LaunchConfiguration("viz_door_text_size_m"),
            "viz_door_height_m":        LaunchConfiguration("viz_door_height_m"),
            "viz_fill_rooms":           LaunchConfiguration("viz_fill_rooms"),
            "viz_fill_open_space":      LaunchConfiguration("viz_fill_open_space"),
            "viz_fill_alpha":           LaunchConfiguration("viz_fill_alpha"),
            "viz_open_space_alpha":     LaunchConfiguration("viz_open_space_alpha"),
            "viz_max_fill_cells":       LaunchConfiguration("viz_max_fill_cells"),
        }],
    )

    labeler = Node(
        package="semantic_mapper",
        executable="room_labeler",
        name="room_labeler",
        output="screen",
        parameters=[{
            "use_llm":  LaunchConfiguration("use_llm"),
            "model":    LaunchConfiguration("llm_model"),
        }],
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("semantic_mapper"),
            "rviz", "semantic_mapper.rviz"])],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        output="screen",
    )

    return LaunchDescription(args + [yolo, mapper, labeler, rviz])