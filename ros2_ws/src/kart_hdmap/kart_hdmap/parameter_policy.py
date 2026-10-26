"""Bounded, ROS-free policy for the Studio parameter editor."""
import math

from .tuning import digest

TARGETS = {
    'vehicle': ('kart_bridge', ('steering_offset', 'steering_scale', 'initial_steering_trim', 'steering_trim_limit')),
    'tracking': ('pure_pursuit', ('lookahead_m', 'max_speed_mps', 'max_cross_track_m',
                                'goal_tolerance_m', 'deceleration_mps2', 'wheelbase_m', 'max_steering_rad')),
    'speed': ('speed_controller', ('kp', 'ki', 'kd', 'kff', 'integral_limit', 'derivative_tau_s',
                                 'throttle_limit', 'brake_limit', 'max_target_speed_mps')),
    'map': ('hdmap_server', ('line_type', 'centerline_speed_mps')),
    'recovery': ('recovery', (
        'history_span_s', 'history_offset_s', 'history_min_samples', 'history_max_gap_s',
        'min_forward_throttle', 'startup_grace_s', 'stuck_duration_s', 'stationary_flow_px_s',
        'moving_flow_px_s', 'stationary_gyro_rad_s', 'brake_command', 'brake_min_s',
        'stop_confirm_s', 'stop_timeout_s', 'reverse_neutral_s', 'reverse_throttle',
        'reverse_duration_s', 'post_reverse_pulse_throttle', 'post_reverse_pulse_s',
        'recovery_steering_scale', 'recovery_steering_limit',
        'resume_neutral_s', 'max_recovery_attempts', 'progress_reset_s',
        'max_recovery_rotation_rad', 'max_tick_gap_s', 'sensor_profile', 'image_topic',
        'input_timeout_s', 'sensor_timeout_s', 'processing_width', 'image_rate_hz')),
}

LABELS = {
    'post_reverse_pulse_throttle': '後退後の制動に入る正パルス出力 (0.002..1)',
    'post_reverse_pulse_s': '後退後の正パルス保持時間 (s)',
    'steering_offset': 'スケール適用後のステア補正 (-0.5..0.5)',
    'steering_scale': 'ステア出力の符号・倍率',
    'initial_steering_trim': '起動時のステアtrim (offsetとは別)',
    'steering_trim_limit': 'D-pad trimの上限',
    'wheelbase_m': 'ホイールベース (m)', 'max_steering_rad': '校正した最大舵角 (rad)',
    'max_target_speed_mps': '速度制御の独立上限 (m/s)', 'integral_limit': '積分出力上限',
    'derivative_tau_s': '微分filter時定数 (s)', 'history_min_samples': '舵角履歴の最低sample数',
    'history_max_gap_s': '舵角履歴の最大欠落時間 (s)', 'reverse_neutral_s': '後退前の中立時間 (s)',
    'resume_neutral_s': '復帰後の中立時間 (s)', 'progress_reset_s': '復帰回数resetの進展時間 (s)',
    'max_recovery_rotation_rad': '復帰中の回転量上限 (rad)', 'max_tick_gap_s': '制御周期の最大間隔 (s)',
    'sensor_profile': 'センサ構成', 'image_topic': '画像topic', 'input_timeout_s': '指令の期限 (s)',
    'sensor_timeout_s': 'センサの期限 (s)', 'processing_width': '画像処理幅 (px)',
    'image_rate_hz': '画像処理上限 (Hz)',
    'lookahead_m': '前方参照の経路長 (m)', 'max_speed_mps': '目標速度上限 (m/s)',
    'max_cross_track_m': '経路からの許容距離 (m)', 'goal_tolerance_m': '終点の許容距離 (m)',
    'deceleration_mps2': '目標減速度 (m/s²)', 'centerline_speed_mps': 'Center速度 (m/s、-1=地図の保存値)',
    'line_type': '実車の参照ライン', 'stuck_duration_s': 'スタック判定継続時間 (s)',
    'stationary_flow_px_s': '停車判定の画像flow上限 (px/s)',
    'moving_flow_px_s': '移動判定の画像flow下限 (px/s)',
    'stationary_gyro_rad_s': '停車判定の角速度上限 (rad/s)',
    'reverse_throttle': '後退の正規化出力 (0..1)', 'reverse_duration_s': '後退時間 (s)',
    'brake_command': 'ブレーキの正規化出力 (0..1)', 'brake_min_s': '最低ブレーキ時間 (s)',
    'stop_confirm_s': '停車確認の継続時間 (s)', 'stop_timeout_s': '停車確認の期限 (s)',
    'recovery_steering_scale': '後退舵角の倍率', 'recovery_steering_limit': '後退舵角上限 (0..1)',
    'max_recovery_attempts': '復帰試行回数上限', 'history_span_s': '舵角履歴の対象時間 (s)',
    'history_offset_s': '舵角履歴の時間オフセット (s)', 'startup_grace_s': '走行開始後の猶予 (s)',
    'kp': '比例ゲイン', 'ki': '積分ゲイン', 'kd': '微分ゲイン', 'kff': '速度比例FFゲイン',
    'throttle_limit': '前進出力上限 (0..1)', 'brake_limit': 'ブレーキ出力上限 (0..1)',
}


def target(key):
    if key not in TARGETS:
        raise ValueError('Unknown tuning parameter target')
    return TARGETS[key]


def catalog(key, fields):
    node, names = target(key)
    if [f['name'] for f in fields] != list(names):
        raise ValueError('Incomplete parameter response')
    result = dict(target=key, node=node, available=True, fields=fields)
    result['revision'] = digest(result)
    return result


def changes_for(snapshot, changes, revision):
    if revision != snapshot['revision']:
        raise ValueError('Jetsonの設定が変わりました。再取得してください')
    if not isinstance(changes, dict) or not changes or len(changes) > 40:
        raise ValueError('変更項目を指定してください')
    fields = {row['name']: row for row in snapshot['fields']}
    result = {}
    for name, value in changes.items():
        row = fields.get(name)
        if row is None or row['read_only']:
            raise ValueError(name + ': 起動時固定または未対応の項目です')
        kind = row['type']
        if kind == 3 and type(value) in (int, float) and math.isfinite(value):
            value = float(value)
        elif kind == 2 and type(value) is int:
            pass
        elif kind == 4 and type(value) is str:
            if name != 'line_type' or value not in ('centerline', 'raceline', 'customline'):
                raise ValueError('参照ラインが不正です')
        else:
            raise ValueError(name + ': 値の型が不正です')
        result[name] = value
    return result
