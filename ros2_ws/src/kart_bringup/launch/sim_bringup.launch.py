"""Terminal 2: monitor-controlled sim pipeline supervisor; initially STOP/unconfigured."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    root=Path(get_package_share_directory('kart_bringup'))
    overrides={k:LaunchConfiguration(k).perform(context) for k in ('map_root','model_root') if LaunchConfiguration(k).perform(context)}
    return [Node(package='kart_bringup',executable='sim_operator',name='sim_operator',output='screen',
                 parameters=[str(root/'config/sim/operator.yaml'),overrides])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map_root',default_value=''),
        DeclareLaunchArgument('model_root',default_value=''),OpaqueFunction(function=build)])
