"""
semantic_pipeline.launch.py

Launches the Voronoi-based semantic mapper + optional RViz.
No prior map, no hardcoded door list — doors are auto-detected on the
medial axis of free space.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("depth_topic",
            default_value="/simple_drone/front_depth/depth/image_raw"),
        DeclareLaunchArgument("cam_info_topic",
            default_value="/simple_drone/front_depth/depth/camera_info"),
        DeclareLaunchArgument("pose_topic",
            default_value="/simple_drone/gt_pose"),

        # BEV bounds.
        DeclareLaunchArgument("bbox_xmin", default_value="-12.0"),
        DeclareLaunchArgument("bbox_ymin", default_value="-34.0"),
        DeclareLaunchArgument("bbox_xmax", default_value= "12.0"),
        DeclareLaunchArgument("bbox_ymax", default_value= "17.0"),
        DeclareLaunchArgument("bev_resolution", default_value="0.15"),

        # Voronoi / segmentation knobs.
        # door_max_clearance_m ≈ half of a typical door width.
        # Bigger → more things get called doors. ~0.6–0.8 is reasonable.
        DeclareLaunchArgument("door_max_clearance_m", default_value="0.70"),
        DeclareLaunchArgument("door_min_sep_cells",   default_value="6"),
        # Regions narrower than this are labelled 'corridor' instead of 'room'.
        DeclareLaunchArgument("corridor_thresh_m",    default_value="1.20"),
        DeclareLaunchArgument("min_room_cells",       default_value="80"),
        # IoU floor for two ticks' rooms to be considered the same room.
        DeclareLaunchArgument("room_iou_threshold",   default_value="0.25"),
        DeclareLaunchArgument("door_match_radius_m",  default_value="0.80"),

        DeclareLaunchArgument("start_rviz", default_value="false"),
    ]

    mapper = Node(
        package="semantic_mapper",
        executable="semantic_mapper_node",
        name="semantic_mapper",
        output="screen",
        parameters=[{
            "depth_topic":            LaunchConfiguration("depth_topic"),
            "cam_info_topic":         LaunchConfiguration("cam_info_topic"),
            "pose_topic":             LaunchConfiguration("pose_topic"),
            "bbox_xmin":              LaunchConfiguration("bbox_xmin"),
            "bbox_ymin":              LaunchConfiguration("bbox_ymin"),
            "bbox_xmax":              LaunchConfiguration("bbox_xmax"),
            "bbox_ymax":              LaunchConfiguration("bbox_ymax"),
            "bev_resolution":         LaunchConfiguration("bev_resolution"),
            "door_max_clearance_m":   LaunchConfiguration("door_max_clearance_m"),
            "door_min_sep_cells":     LaunchConfiguration("door_min_sep_cells"),
            "corridor_thresh_m":      LaunchConfiguration("corridor_thresh_m"),
            "min_room_cells":         LaunchConfiguration("min_room_cells"),
            "room_iou_threshold":     LaunchConfiguration("room_iou_threshold"),
            "door_match_radius_m":    LaunchConfiguration("door_match_radius_m"),
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

    return LaunchDescription(args + [mapper, rviz])