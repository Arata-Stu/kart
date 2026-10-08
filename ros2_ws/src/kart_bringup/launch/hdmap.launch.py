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
    for key in ("map_file", "lane_id", "line_type", "use_sim_time"):
        value = LaunchConfiguration(key).perform(context)
        if value:
            if key == "use_sim_time":
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
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
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
