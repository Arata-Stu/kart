"""Shared validation for local preview lookahead options (not ROS settings)."""
import math


def lookahead_values(options):
    mode = options.get('lookahead_mode', 'speed_interpolated')
    if mode not in ('fixed', 'speed_interpolated'):
        raise ValueError('先読み方式が不正です')
    values = [options.get(key, default) for key, default in (
        ('lookahead_m', .8), ('lookahead_low_speed_m', .8), ('lookahead_high_speed_m', 3.))]
    if any(type(v) not in (int, float) or not math.isfinite(v) or not .05 <= v <= 20 for v in values):
        raise ValueError('先読み距離は0.05〜20 mで指定してください')
    if values[2] < values[1]:
        raise ValueError('高速時の先読み距離は低速時以上にしてください')
    return [int(mode == 'speed_interpolated'), *values]
