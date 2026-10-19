"""Load EVS TensorRT + control decoder into the existing direct OpenEB process."""

import json
import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from kart_bringup.e2e import validate_evs_tensor
from kart_e2e.contract import runtime_settings
from kart_e2e.engine_cache import require_engine


def add_pipeline(args):
    config = json.loads(lu.get_path('kart_bringup', 'config/e2e/evs_launch.json').read_text())
    if set(config) != {'container_name', 'input_topic', 'model_dir', 'evs_tensor_config'}:
        raise ValueError('Unexpected EVS launch configuration keys')
    config.update({k: getattr(args, k) for k in config if getattr(args, k) != ''})
    if config['container_name'] != '/event_camera/tensor_container' or config['input_topic'] != '/event_camera/tensor':
        raise ValueError('EVS policy must use the direct OpenEB process and tensor topic')
    validate_evs_tensor(config['evs_tensor_config'], config['model_dir'])
    runtime_settings(config['model_dir'])
    require_engine(config['model_dir'])
    return [lu.log_info(f'EVS E2E: {config}; no RGB encoder; recording/AUTO remain manual'),
            lu.include('kart_bringup', 'launch/modules/e2e/inference.launch.py',
                       launch_arguments={
                           **{k: config[k] for k in ('container_name', 'input_topic', 'model_dir')},
                           **{k: getattr(args, k) for k in ('use_fixed_throttle', 'fixed_throttle', 'drive_enabled', 'use_sim_time')},
                       }, scoped=True, forwarding=False)]


def generate_launch_description():
    args = lu.ArgumentContainer()
    for key in ('container_name', 'input_topic', 'model_dir', 'evs_tensor_config',
                'use_fixed_throttle', 'fixed_throttle', 'drive_enabled', 'use_sim_time'):
        args.add_arg(key, '', cli=True)
    args.add_opaque_function(add_pipeline)
    return lut.LaunchDescription(args.get_launch_actions())
