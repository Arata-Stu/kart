"""Owned direct EVS pipeline; explicit startup overrides follow operational YAML."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from kart_bringup.evs import pipeline_configuration


def setup(context):
    root = Path(get_package_share_directory("kart_bringup")) / "config/sensors"
    paths, overrides = pipeline_configuration(
        root, *(LaunchConfiguration(key).perform(context)
                for key in ("evs_backend", "evs_serial", "evs_bias_file")))
    return [
        LogInfo(msg=f"OpenEB configs={paths}; overrides={overrides}; packet default OFF"),
        Node(package="openeb_ros2", executable="openeb_tensor_pipeline",
             name="tensor_pipeline", namespace="event_camera",
             parameters=[*(str(p) for p in paths), overrides], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value="")
          for key in ("evs_backend", "evs_serial", "evs_bias_file")),
        OpaqueFunction(function=setup),
    ])
