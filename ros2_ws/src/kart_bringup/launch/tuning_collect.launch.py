"""Mapless recording with optional fixed AUTO trials; recording/mode changes stay explicit."""
import json
import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from launch.actions import Shutdown
from launch_ros.actions import Node
from kart_bringup.configuration import load
from kart_bringup.tuning import collect_configuration, fixed_configuration

KEYS=('device','record_dir','run_name','rgb_fps','infra_fps','tracking_mode','multicam_mode')
FIXED_KEYS=('fixed_axes','steering','throttle','brake','duration_s')


def build(args):
    root=lu.get_path('kart_bringup','config')
    explicit={key:getattr(args,key) for key in KEYS}
    mission,params,remaps=collect_configuration(root,explicit)
    fixed,fixed_params=fixed_configuration(root,getattr(args,'fixed_command',''),
        {key:getattr(args,key,'') for key in FIXED_KEYS})
    load(root/'bringup.yaml',{key:explicit[key] for key in ('device','record_dir','run_name')})
    actions = [
        lu.log_info(f'Tuning collection: mapless manual recording; config={root / "tuning/cuvslam.yaml"}; effective VSLAM={params}; remaps={remaps}. Recording START and MANUAL operation are separate.'),
        lu.include('kart_bringup','launch/mission.launch.py',launch_arguments={
            'mode':'collect',**{key:explicit[key] for key in KEYS if key not in ('tracking_mode','multicam_mode')}},
            scoped=True,forwarding=False),
        lu.include('kart_bringup','launch/modules/localization/vslam.launch.py',launch_arguments={
            'container_name':'/'+mission['sensor_container'].lstrip('/'),
            'parameters_json':json.dumps(params),'remappings_json':json.dumps(remaps)},scoped=True,forwarding=False)]
    if fixed:
        actions.extend([
            lu.log_info(f'Fixed command enabled: config={root / "tuning/fixed_command.yaml"}; parameters={fixed_params}. STOP -> AUTO starts the trial; MANUAL remains joystick controlled.'),
            Node(package='kart_control',executable='fixed_command',name='fixed_command',
                output='screen',parameters=[fixed_params],on_exit=Shutdown())])
    return actions


def generate_launch_description():
    args=lu.ArgumentContainer()
    for key in (*KEYS,'fixed_command',*FIXED_KEYS):
        args.add_arg(key,'',description='Explicit override; empty preserves module YAML',cli=True)
    args.add_opaque_function(build)
    return lut.LaunchDescription(args.get_launch_actions())
