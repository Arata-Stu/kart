"""One route decision shared by rule and both E2E pipelines."""
from pathlib import Path

CHOICES = ('off', 'infra_imu', 'rgb_imu', 'evs_assisted')
LABELS = {'off': '無効', 'infra_imu': 'IR＋IMU', 'rgb_imu': 'RGB＋IMU',
          'evs_assisted': 'IR＋IMU＋EVS補助'}


def configuration(root, selection='', node_config=None):
    import yaml
    root = Path(root)
    switch = yaml.safe_load((root / 'launch.yaml').read_text())
    if (set(switch) != {'enabled', 'profiles'} or type(switch['enabled']) is not bool
            or set(switch['profiles']) != set(CHOICES[1:])
            or any(not isinstance(v, str) or not v for v in switch['profiles'].values())):
        raise ValueError('Invalid recovery/launch.yaml enabled/profiles')
    if selection and selection not in CHOICES:
        raise ValueError('Unknown recovery selection: ' + selection)
    params = dict(yaml.safe_load(Path(node_config or root / 'recovery.yaml').read_text())['/**/recovery']['ros__parameters'])
    enabled = switch['enabled'] if not selection else selection != 'off'
    if selection and selection != 'off':
        params.update(sensor_profile=selection, image_topic=switch['profiles'][selection])
    return enabled, params


def validate_inputs(params, sensors, evs_enabled, fixed_throttle=-1):
    if fixed_throttle >= 0:
        raise ValueError('自動復帰は固定AUTOスロットル（手動ステア）と併用できません')
    for topic, flag in (('/realsense/color/image_raw', 'enable_color'),
                        ('/realsense/infra1/image_rect_raw', 'enable_infra1'),
                        ('/realsense/infra2/image_rect_raw', 'enable_infra2')):
        if params['image_topic'] == topic and not sensors[flag]:
            raise ValueError('自動復帰に必要な画像が無効です: ' + topic)
    if not sensors['enable_gyro']:
        raise ValueError('自動復帰にはIMU角速度が必要です')
    if params['sensor_profile'] == 'evs_assisted' and not evs_enabled:
        raise ValueError('EVS補助の自動復帰にはEVSを有効にしてください（--evs）')


def route(root, use_sim_time='', node_config=None, selection=''):
    enabled, params = configuration(root, selection, node_config)
    if not enabled:
        return [], []
    config = str(node_config or (Path(root) / 'recovery.yaml'))
    from launch.actions import LogInfo
    from launch_ros.actions import Node
    overrides = {}
    if selection and selection != 'off':
        overrides.update({key: params[key] for key in ('sensor_profile', 'image_topic')})
    if use_sim_time != '':
        if str(use_sim_time).lower() not in ('true', 'false'):
            raise ValueError('use_sim_time must be true/false')
        overrides['use_sim_time'] = str(use_sim_time).lower() == 'true'
    return [('auto/control_cmd', '/recovery/input_cmd')], [
        LogInfo(msg=f'Recovery enabled: {config}; params={params}; overrides={overrides}'),
        Node(package='kart_recovery', executable='recovery', name='recovery', output='screen',
             parameters=[config, overrides]),
    ]
