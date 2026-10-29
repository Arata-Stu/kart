"""Launch the copied NVIDIA node with the private fixed-reference core."""
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    config = Path(get_package_share_directory('kart_bringup')) / 'config/localization'
    parameters = yaml.safe_load((config / 'fixed_localization.yaml').read_text())['/**/visual_slam']['ros__parameters']
    workflow = json.loads((config / 'workflow.json').read_text())
    map_dir = LaunchConfiguration('map_dir').perform(context)
    if map_dir:
        path = Path(map_dir).expanduser().resolve(strict=True)
        if (path / 'cuvslam_map').is_dir():
            path = path / 'cuvslam_map'
        parameters['load_map_folder_path'] = str(path)
    for key in ('use_sim_time', 'localize_on_startup'):
        value = LaunchConfiguration(key).perform(context)
        if value:
            if value.lower() not in ('true', 'false'):
                raise ValueError(f'{key}: expected true or false')
            parameters[key] = value.lower() == 'true'
    remappings = [(f'visual_slam/{topic}_{i}', workflow[key]) for topic, i, key in (
        ('image', 0, 'left_image'), ('image', 1, 'right_image'),
        ('camera_info', 0, 'left_info'), ('camera_info', 1, 'right_info'))]
    remappings += [('visual_slam/imu', workflow['imu']),
                   ('visual_slam/initial_pose', workflow['pose_hint']),
                   ('visual_slam/trigger_hint', workflow['request_hint']),
                   ('/diagnostics', workflow['vslam_diagnostics'])]
    return [Node(package='kart_visual_localization', executable='kart_visual_localization',
                 name='visual_slam', output='screen', parameters=[parameters], remappings=remappings)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(key, default_value='')
        for key in ('map_dir', 'use_sim_time', 'localize_on_startup')
    ] + [OpaqueFunction(function=start)])
