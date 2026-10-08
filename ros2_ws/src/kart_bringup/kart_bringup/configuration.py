"""Resolve static operational config and explicitly supplied launch overrides."""
from pathlib import Path
import math
import yaml

NODE_NAMES = {
    'joy': 'kart_joy_node', 'joy_manager': 'kart_joy_manager',
    'mode_manager': 'operation_mode_manager', 'command_mux': 'command_mux',
    'bridge': 'kart_bridge', 'bag_manager': 'kart_bag_manager',
}
BOOL_KEYS = ('composed', 'enable_joy', 'enable_control', 'enable_bridge', 'enable_bag_manager', 'enable_jetson_stats')
RUNTIME_KEYS = (*BOOL_KEYS, 'container_name', 'namespace', 'bag_shutdown_timeout',
                'profile', 'device', 'record_dir', 'bag_config')


def load(path, overrides=None):
    path = Path(path).expanduser().resolve()
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or set(document) != {'launch', 'parameters'}:
        raise ValueError('bringup config must contain launch and parameters mappings')
    settings = dict(document['launch'])
    required = {*BOOL_KEYS, 'container_name', 'namespace', 'bag_shutdown_timeout', 'bag_sigkill_timeout'}
    if set(settings) != required or set(document['parameters']) != set(NODE_NAMES):
        raise ValueError('unknown or missing launch/module configuration keys')
    overrides = {k: v for k, v in (overrides or {}).items() if v != ''}
    if set(overrides) - set(RUNTIME_KEYS):
        raise ValueError('unknown runtime override')
    for key in settings:
        if key in overrides:
            value = overrides[key]
            if key in BOOL_KEYS:
                if value not in ('true', 'false'):
                    raise ValueError(f'{key} must be true or false')
                value = value == 'true'
            elif key.endswith('_timeout'):
                value = float(value)
            settings[key] = value
    for key in BOOL_KEYS:
        if type(settings[key]) is not bool:
            raise ValueError(f'{key} must be YAML boolean')
    for key in ('bag_shutdown_timeout', 'bag_sigkill_timeout'):
        value = settings[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be a positive finite number')
    if not isinstance(settings['container_name'], str) or not settings['container_name']:
        raise ValueError('container_name must not be empty')
    if not isinstance(settings['namespace'], str):
        raise ValueError('namespace must be a string')
    if settings['enable_bridge'] and not settings['enable_control']:
        raise ValueError('bridge requires enable_control=true (mode manager and mux)')

    modules = {}
    for key, node_name in NODE_NAMES.items():
        filename = document['parameters'][key]
        if key == 'bag_manager' and 'bag_config' in overrides:
            # CLI paths are relative to the caller; manifest paths to the manifest directory.
            filename = str(Path(overrides['bag_config']).expanduser().resolve())
        config_path = (path.parent / filename).resolve()
        content = yaml.safe_load(config_path.read_text())
        selector = f'/**/{node_name}'
        if not isinstance(content, dict) or set(content) != {selector}:
            raise ValueError(f'{config_path}: expected only {selector}')
        params = content[selector]['ros__parameters']
        if not isinstance(params, dict):
            raise ValueError(f'{config_path}: expected ros__parameters mapping')
        modules[key] = {'file': str(config_path), 'overrides': {}, 'effective': dict(params)}
    for argument, module, parameter in (('profile', 'joy', 'profile_path'),
                                        ('device', 'bridge', 'device'),
                                        ('record_dir', 'bag_manager', 'output_dir')):
        if argument in overrides:
            modules[module]['overrides'][parameter] = overrides[argument]
            modules[module]['effective'][parameter] = overrides[argument]
    if settings['enable_bridge']:
        if not modules['bridge']['effective'].get('device'):
            raise ValueError('bridge requires device in vehicle/bridge.yaml or device:=...')
        if any(module['effective'].get('use_sim_time', False) for module in modules.values()):
            raise ValueError('physical bridge requires system-wide use_sim_time=false')
    return settings, modules
