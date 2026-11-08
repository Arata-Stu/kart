"""Ground projection, Frenet observation and shadow planning for a live closed course."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("kart_bringup"))
    config = share / "config/racing/observer.yaml"
    def include(name, args=()):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / "launch" / name)),
                                        launch_arguments=args)
    return LaunchDescription([
        DeclareLaunchArgument("ground_z_m", description="Road height relative to base_link in metres"),
        include("racing_ground.launch.py", [("ground_z_m", LaunchConfiguration("ground_z_m"))]),
        LogInfo(msg=f"Racing observer: {config}: {config.read_text()}"),
        Node(package="kart_racing", executable="racing_observer", name="racing_observer",
             parameters=[str(config)], output="screen"),
        include("racing_shadow.launch.py"),
    ])
