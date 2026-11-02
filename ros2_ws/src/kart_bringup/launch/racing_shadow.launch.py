"""Single-opponent planner inspection; no camera, vehicle, TF or control nodes."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    path = Path(get_package_share_directory('kart_bringup')) / 'config/racing/predictive_spliner.yaml'
    config = yaml.safe_load(path.read_text())
    return LaunchDescription([
        LogInfo(msg=f'Racing SHADOW only: {path}: {config}'),
        Node(package='kart_racing', executable='predictive_spliner',
             name='predictive_spliner', parameters=[str(path)], output='screen'),
    ])
