"""Select a calibrated pose reference without depending on stereo-pair key spelling."""
from collections import Counter


def pose_reference(meta):
    cameras = meta.get('camera_params_id_to_camera_params')
    frames = meta.get('keyframes_metadata')
    if not isinstance(cameras, dict) or not isinstance(frames, list):
        raise ValueError('frames_meta requires camera_params_id_to_camera_params and keyframes_metadata')
    counts = Counter(str(f['camera_params_id']) for f in frames
                     if isinstance(f, dict) and 'camera_params_id' in f and 'camera_to_world' in f)
    valid = []
    for camera_id, count in counts.items():
        calibration = cameras.get(camera_id)
        if not isinstance(calibration, dict):
            continue
        sensor = calibration.get('sensor_meta_data')
        if not isinstance(sensor, dict):
            continue
        extrinsic = sensor.get('sensor_to_vehicle_transform')
        if isinstance(extrinsic, dict) and 'axis_angle' in extrinsic and 'translation' in extrinsic:
            valid.append((camera_id, count))
    if not valid:
        raise ValueError('No camera with both poses and sensor_to_vehicle_transform; '
                         f'pose camera IDs={sorted(counts)}, calibration IDs={sorted(cameras)}')
    # Any calibrated camera yields world<-base. Keep one camera to avoid mixing
    # simultaneous stereo poses; most observations wins, stable ID breaks ties.
    camera_id = sorted(valid, key=lambda item: (-item[1], item[0]))[0][0]
    selected = [f for f in frames if str(f.get('camera_params_id')) == camera_id]
    if any('camera_to_world' not in f or 'timestamp_microseconds' not in f for f in selected):
        raise ValueError(f'Camera {camera_id}: incomplete pose/timestamp metadata')
    return camera_id, cameras[camera_id], selected
