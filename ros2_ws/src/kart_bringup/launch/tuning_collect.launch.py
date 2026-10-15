"""Manual recording plus mapless odometry; recording and mode changes stay explicit."""
import json
import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from kart_bringup.configuration import load
from kart_bringup.tuning import collect_configuration

KEYS=('device','record_dir','run_name','rgb_fps','infra_fps','tracking_mode','multicam_mode')


def build(args):
    root=lu.get_path('kart_bringup','config')
    explicit={key:getattr(args,key) for key in KEYS}
    mission,params,remaps=collect_configuration(root,explicit)
    load(root/'bringup.yaml',{key:explicit[key] for key in ('device','record_dir','run_name')})
    return [
        lu.log_info(f'Tuning collection: mapless manual recording; config={root / "tuning/cuvslam.yaml"}; effective VSLAM={params}; remaps={remaps}. Recording START and MANUAL operation are separate.'),
        lu.include('kart_bringup','launch/mission.launch.py',launch_arguments={
            'mode':'collect',**{key:explicit[key] for key in KEYS if key not in ('tracking_mode','multicam_mode')}},
            scoped=True,forwarding=False),
        lu.include('kart_bringup','launch/modules/localization/vslam.launch.py',launch_arguments={
            'container_name':'/'+mission['sensor_container'].lstrip('/'),
            'parameters_json':json.dumps(params),'remappings_json':json.dumps(remaps)},scoped=True,forwarding=False)]


def generate_launch_description():
    args=lu.ArgumentContainer()
    for key in KEYS:
        args.add_arg(key,'',description='Explicit override; empty preserves module YAML',cli=True)
    args.add_opaque_function(build)
    return lut.LaunchDescription(args.get_launch_actions())
