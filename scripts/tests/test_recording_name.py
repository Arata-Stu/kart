from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parents[1] / 'lib'))
sys.path.insert(0, str(Path(__file__).parents[2] / 'ros2_ws/src/kart_bringup'))
from recording_name import sensor_recording_name
from kart_bringup.mission import sensor_parameters, camera_config
from kart_bringup.evs import pipeline_configuration

ROOT = Path(__file__).parents[2] / 'ros2_ws/src/kart_bringup/config'


def test_profiles_and_disabled_sensors():
    sensors = sensor_parameters(camera_config(ROOT, 'collect'), rgb_profile='1280x720x30',
                                infra_profile='640x480x90', depth_profile='640x480x90', imu_profile='low')
    assert sensor_recording_name('collect', sensors) == 'collect_rgb1280x720x30fps_infra640x480x90fps_evsOFF_depth90fps_imuA100-G200Hz'
    sensors = sensor_parameters(camera_config(ROOT, 'collect'), rgb_profile='0',
                                infra_profile='0', depth_profile='0', imu_profile='off')
    assert sensor_recording_name('collect', sensors) == 'collect_rgbOFF_infraOFF_evsOFF_depthOFF_imuOFF'


def test_evs_rates_and_image_override():
    sensors = sensor_parameters(camera_config(ROOT, 'collect'))
    for backend in ('cpu', 'cuda_async'):
        paths, overrides = pipeline_configuration(ROOT / 'sensors', backend=backend, image='false')
        name = sensor_recording_name('collect', sensors, paths, overrides)
        assert '_evsT250Hz-IOFF_' in name
        assert len(name.encode()) <= 120
