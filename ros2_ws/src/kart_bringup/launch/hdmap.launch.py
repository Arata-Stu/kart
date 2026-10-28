"""Standalone read-only HDMap display. Does not start vehicle nodes or TF."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    config = (
        Path(get_package_share_directory("kart_bringup")) / "config/hdmap/hdmap.yaml"
    )
    overrides = {}
    for key in ("map_file", "lane_id", "line_type", "use_sim_time", "tuning_parking_enabled", "tuning_enabled", "tuning_map_dir", "tuning_tracking_frame"):
        value = LaunchConfiguration(key).perform(context)
        if value:
            if key in ("use_sim_time", "tuning_parking_enabled", "tuning_enabled"):
                if value.lower() not in ("true", "false"):
                    raise ValueError("use_sim_time must be true or false")
                value = value.lower() == "true"
            overrides[key] = value
    print(f"HDMap config: {config}; explicit overrides: {overrides}")
    return [
        Node(
            package="kart_hdmap",
            executable="hdmap_server",
            name="hdmap_server",
            parameters=[str(config), overrides],
            output="screen",
            remappings=[("planning/reference_line", LaunchConfiguration("reference_topic").perform(context))],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("tuning_parking_enabled", default_value=""),
            DeclareLaunchArgument("tuning_enabled", default_value=""),
            DeclareLaunchArgument("tuning_map_dir", default_value=""),
            DeclareLaunchArgument("tuning_tracking_frame", default_value=""),
            DeclareLaunchArgument("reference_topic", default_value="planning/reference_line"),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument("lane_id", default_value=""),
            DeclareLaunchArgument(
                "line_type",
                default_value="",
                choices=["", "centerline", "raceline", "customline"],
            ),
            DeclareLaunchArgument("use_sim_time", default_value=""),
            OpaqueFunction(function=setup),
        ]
    )
