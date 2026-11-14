"""Human-readable planned camera profiles and sensor rates; not recorded delivery."""
import yaml


def sensor_recording_name(mode, sensors, evs_paths=None, evs_overrides=None):
    def camera(label, enabled, profile, resolution=False):
        value = sensors[profile] if resolution else sensors[profile].split('x')[-1]
        return label + (value + 'fps' if enabled else 'OFF')

    parts = [mode, camera('rgb', sensors['enable_color'], 'rgb_camera.color_profile', resolution=True),
             camera('infra', sensors['enable_infra1'] or sensors['enable_infra2'], 'depth_module.infra_profile', resolution=True)]
    if evs_paths:
        tensor = yaml.safe_load(evs_paths[1].read_text())['/**/event_tensor']['ros__parameters']
        image = yaml.safe_load(evs_paths[2].read_text())['/**/event_preprocessor']['ros__parameters']
        rate = tensor['output_rate_hz'] or 1000 / tensor['stride_ms']
        enabled = (evs_overrides or {}).get('event_image_enabled', image['event_image_enabled'])
        parts.append(f'evsT{rate:g}Hz-I' + (f"{image['event_image_fps']:g}fps" if enabled else 'OFF'))
    else:
        parts.append('evsOFF')
    parts.append(camera('depth', sensors['enable_depth'], 'depth_module.depth_profile'))
    accel = f"{sensors['accel_fps']:g}" if sensors['enable_accel'] else 'OFF'
    gyro = f"{sensors['gyro_fps']:g}" if sensors['enable_gyro'] else 'OFF'
    parts.append(f'imuA{accel}-G{gyro}Hz' if sensors['enable_accel'] or sensors['enable_gyro'] else 'imuOFF')
    return '_'.join(parts)
