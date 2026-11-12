"""Observe an already-running perception pipeline with bounded-rate bbox JPEGs."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from kart_bringup.racing_models import get_profile


def debug_node(context):
    config = Path(get_package_share_directory("kart_bringup")) / "config/racing/debug_image.yaml"
    model = get_profile(LaunchConfiguration("model_profile").perform(context))
    image = LaunchConfiguration("image_topic").perform(context)
    return [
        LogInfo(msg=f"Racing debug image: {config.read_text()}, model={model['name']}"),
        Node(package="kart_opponent_tracker", executable="opponent_debug_image", namespace="/racing",
             name="opponent_debug_image", output="screen",
             parameters=[str(config), {"class_names": model["class_names"]}], remappings=[("rgb", image)]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_profile", default_value="coco", choices=["coco", "opponent"]),
        DeclareLaunchArgument("image_topic", default_value="/realsense/color/image_raw"),
        OpaqueFunction(function=debug_node),
    ])
