"""Optional uniform test speed; does not change map geometry or stop points."""
import math


def validate_speed_override(value, enabled):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('speed_override_mps must be finite')
    if value == -1:
        return
    if not enabled or not .05 <= value <= 1.5:
        raise ValueError('speed override requires allow_speed_override=true and 0.05..1.5 m/s')


def override_speeds(speeds, value, nominal_limit=0.):
    if value != -1:
        return [float(value) if v > 0 else v for v in speeds]
    return [min(v, nominal_limit) for v in speeds] if nominal_limit > 0 else list(speeds)


def centerline_speed(lane, override=-1.0):
    """-1 follows the selected map lane; zero is an explicit stopped target."""
    if type(override) not in (int, float) or not math.isfinite(override) or (override != -1 and override < 0):
        raise ValueError('Centerline speed must be -1 (map) or finite and nonnegative')
    if override != -1:
        return float(override)
    return float(lane.get('centerline_speed_mps', 1.5) if lane else 1.5)
