"""Mapless, Joy-free ESC trial. Run exclusively; UI explicitly requests AUTO."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    root = Path(get_package_share_directory('kart_bringup')) / 'config'
    device = LaunchConfiguration('device').perform(context)
    bridge_params = [str(root/'tuning/esc_bridge.yaml')]
    if device:
        bridge_params.append({'device': device})
    return [
        Node(package='kart_system', executable='kart_operation_mode_manager',
             name='operation_mode_manager', output='screen', on_exit=Shutdown()),
        Node(package='kart_system', executable='kart_command_mux', name='command_mux',
             parameters=[str(root/'system/command_mux.yaml')], output='screen', on_exit=Shutdown()),
        Node(package='kart_vehicle', executable='kart_bridge_node', name='kart_bridge',
             parameters=bridge_params, output='screen', on_exit=Shutdown()),
        Node(package='kart_control', executable='esc_trial', name='esc_trial',
             parameters=[str(root/'tuning/esc_trial.yaml')], output='screen', on_exit=Shutdown()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='', description='Explicit serial device override; empty keeps trial YAML'),
        OpaqueFunction(function=build),
    ])
