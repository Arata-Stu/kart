"""Optional flat-road projection of confirmed RGB tracks."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory("kart_bringup")) / "config/racing/ground.yaml"
    z = LaunchConfiguration("ground_z_m")
    info = LaunchConfiguration("camera_info_topic")
    return LaunchDescription([
        DeclareLaunchArgument("ground_z_m", description="Measured road height in base_link [m]"),
        DeclareLaunchArgument("camera_info_topic", default_value="/realsense/color/camera_info"),
        LogInfo(msg=[f"Ground projection: {config.read_text()} ground_z_m=", z, ", camera_info=", info]),
        Node(package="kart_opponent_tracker", executable="opponent_ground", namespace="/racing",
             name="opponent_ground", output="screen", parameters=[str(config),
             {"ground_z_m": ParameterValue(z, value_type=float)}], remappings=[("camera_info", info)]),
    ])
