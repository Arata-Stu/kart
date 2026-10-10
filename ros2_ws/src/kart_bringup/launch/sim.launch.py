"""Simulation only: owns clock and TF; never launches USB bridge / real sensors."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def build(context):
    bringup=Path(get_package_share_directory('kart_bringup'))
    sim=Path(get_package_share_directory('kart_sim'))
    directory=LaunchConfiguration('map_dir').perform(context)
    directory=Path(directory) if directory else sim/'maps'
    name=LaunchConfiguration('map').perform(context)
    if Path(name).name != name or not name: raise ValueError('map must be a filename stem')
    file=directory/(name+'.json')
    if not file.is_file(): raise ValueError(f'Map does not exist: {file}')
    overrides={'map_file':str(file)}
    for key in ('camera_enabled','viewer_enabled','require_mode','imu_enabled','publish_truth_tf','monitor_enabled'):
        value=LaunchConfiguration(key).perform(context)
        if value:
            if value.lower() not in ('true','false'): raise ValueError(f'Invalid {key}: {value}')
            overrides[key]=value.lower()=='true'
    for key in ('stereo_hz','rgb_hz','imu_hz'):
        value=LaunchConfiguration(key).perform(context)
        if value: overrides[key]=float(value)
    rig=LaunchConfiguration('camera_rig_file').perform(context)
    if rig: overrides['camera_rig_file']=rig
    record=LaunchConfiguration('record_dir').perform(context)
    if record: overrides['record_dir']=record
    return [LogInfo(msg=f'kart_sim config={bringup / "config/sim/sim.yaml"}, overrides={overrides}'),
        Node(package='kart_sim',executable='kart_sim_node',name='kart_sim',output='screen',
             parameters=[str(bringup/'config/sim/sim.yaml'),overrides])]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map',default_value='minicar_2026'),
        DeclareLaunchArgument('map_dir',default_value=''),
        DeclareLaunchArgument('camera_enabled',default_value=''),
        DeclareLaunchArgument('viewer_enabled',default_value=''),
        DeclareLaunchArgument('require_mode',default_value=''),
        *[DeclareLaunchArgument(name,default_value='') for name in ('imu_enabled','publish_truth_tf','stereo_hz','rgb_hz','imu_hz','camera_rig_file','monitor_enabled','record_dir')],
        OpaqueFunction(function=build)])
