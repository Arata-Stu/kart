"""Validate mapless manual collection before any sensor/vehicle process starts."""
import json
from pathlib import Path
import yaml
from .mission import composition, sensor_parameters
from .multicam import multicam_value
from .vehicle_tf import load_transforms


def collect_configuration(root, overrides=None):
    root=Path(root); overrides=overrides or {}
    allowed={'device','record_dir','run_name','rgb_fps','infra_fps','tracking_mode','multicam_mode'}
    if set(overrides)-allowed: raise ValueError('Unknown tuning collection override')
    mission=composition(root/'mission.yaml','collect')
    if not mission['create_sensor_container']:
        raise ValueError('Tuning collection must own its sensor container; stop other mission launches first')
    camera=sensor_parameters(root/'sensors/realsense.yaml',overrides.get('rgb_fps',''),overrides.get('infra_fps',''))
    if not camera['enable_infra1'] or not camera['enable_infra2']:
        raise ValueError('Tuning VSLAM requires stereo infra images')
    load_transforms(root/'vehicle/transforms.yaml',require_camera=True)
    params=yaml.safe_load((root/'tuning/cuvslam.yaml').read_text())['/**/visual_slam']['ros__parameters']
    if overrides.get('tracking_mode','')!='':
        if str(overrides['tracking_mode']) not in ('0','1'): raise ValueError('tracking_mode must be 0 or 1')
        params['tracking_mode']=int(overrides['tracking_mode'])
    if overrides.get('multicam_mode','')!='': params['multicam_mode']=multicam_value(overrides['multicam_mode'])
    if params['use_sim_time'] or params['load_map_folder_path'] or params['save_map_folder_path'] or params['enable_localization_n_mapping']:
        raise ValueError('Tuning requires real-time mapless odometry, without map load/save/mapping')
    if params['publish_map_to_odom_tf'] or params['publish_odom_to_base_tf'] or params['enable_request_hint']:
        raise ValueError('Tuning odometry must not publish dynamic TF or request localization hints')
    if params['base_frame']!='base_link': raise ValueError('Tuning collection base_frame must match bringup base_link')
    if params['tracking_mode']==1 and (not camera.get('enable_gyro') or not camera.get('enable_accel') or not camera.get('unite_imu_method')):
        raise ValueError('VIO requires enabled gyro/accel and a united IMU topic')
    flow=json.loads((root/'localization/workflow.json').read_text())
    remaps=[(f'visual_slam/{name}_{i}',flow[key]) for name,i,key in (
        ('image',0,'left_image'),('image',1,'right_image'),
        ('camera_info',0,'left_info'),('camera_info',1,'right_info'))]
    remaps.extend([('visual_slam/imu',flow['imu']),('/diagnostics',flow['vslam_diagnostics'])])
    return mission,params,remaps
