"""Live RC behavior and confirmed-track projection; no vehicle/AUTO startup."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from kart_bringup.mission import boolean, validate_overtake


def behavior(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode, engine = value('mode'), value('perception_engine')
    validate_overtake(mode, 'drive', engine, 'track')
    if mode == 'off':
        return []
    map_file = Path(value('map_file')).expanduser().resolve()
    if not map_file.is_file():
        raise ValueError(f'Missing behavior HDMap: {map_file}')
    root = Path(get_package_share_directory('kart_bringup')) / 'config/racing'
    config, ground = root / 'behavior.yaml', root / 'ground.yaml'
    overrides = {'mode': mode, 'map_file': str(map_file)}
    ground_overrides = {}
    if value('use_sim_time') != '':
        overrides['use_sim_time'] = ground_overrides['use_sim_time'] = boolean(value('use_sim_time'))
    baseline = '/planning/course_reference' if mode == 'active' else '/planning/reference_line'
    return [
        LogInfo(msg=f'Racing behavior: {config.read_text()}; overrides={overrides}; '
                    f'baseline={baseline}; confirmed-ground={ground.read_text()}'),
        Node(package='kart_opponent_tracker', executable='opponent_ground', namespace='/racing',
             name='opponent_ground', output='screen', parameters=[str(ground), ground_overrides]),
        Node(package='kart_racing', executable='racing_behavior', namespace='/', name='racing_behavior', output='screen',
             parameters=[str(config), {key: ParameterValue(val, value_type=str) if key in ('mode', 'map_file') else val
                                      for key, val in overrides.items()}],
             remappings=[('baseline_reference', baseline)]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='shadow', choices=['off', 'shadow', 'active']),
        DeclareLaunchArgument('map_file', description='Selected HDMap map.json or hd_map.yaml'),
        DeclareLaunchArgument('perception_engine', default_value='', description='Selected perception engine; active requires racing profile'),
        DeclareLaunchArgument('use_sim_time', default_value=''),
        OpaqueFunction(function=behavior),
    ])
