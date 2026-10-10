"""Vehicle mount TF owner. RealSense alone owns its internal optical TFs."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from kart_bringup.vehicle_tf import load_transforms, publisher_arguments


def start(context):
    path = LaunchConfiguration("transforms_config").perform(context)
    ready, missing = load_transforms(path)
    actions = [
        LogInfo(msg=f"Vehicle TF: {path}; active={ready}; unconfigured={missing}")
    ]
    # Static publishers are separate small processes; sensor/VSLAM container
    # ownership and intra-process image transport remain unchanged.
    actions += [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="kart_tf_" + name,
            output="screen",
            arguments=publisher_arguments(entry),
        )
        for name, entry in ready.items()
    ]
    return actions


def generate_launch_description():
    path = (
        Path(get_package_share_directory("kart_bringup"))
        / "config/vehicle/transforms.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("transforms_config", default_value=str(path)),
            OpaqueFunction(function=start),
        ]
    )
