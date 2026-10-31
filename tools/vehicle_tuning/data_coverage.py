"""Distribution diagnostics, not a model accuracy or vehicle safety certificate."""
from collections import Counter, defaultdict
import math


def diagnose(rows, segments, cfg):
    width = 0.05
    min_seconds, min_visits = 1.0, 3
    visits = defaultdict(list)
    cells = defaultdict(float)
    transitions = Counter()
    usable = [r for r in rows if not r['reason'] and r['brake'] == 0]
    observed = max((r['throttle'] for r in usable), default=None)
    target = cfg.get('coverage_max_throttle')
    upper = target if target is not None else observed
    recommendations = []

    def add(code, evidence, action):
        recommendations.append(dict(code=code, evidence=evidence, recommendation=action))

    previous = None
    anchor = None
    current_key = None
    for row in rows:
        if row['reason'] or row['brake'] != 0:
            previous = anchor = current_key = None
            continue
        key = min(19, int(math.floor((row['throttle'] + 1e-10) / width)))
        contiguous = (previous is not None and row['mode'] == previous['mode']
                      and 0 < row['t']-previous['t'] <= cfg['odom_max_gap_s'])
        if not contiguous:
            anchor = row['throttle']
            current_key = None
        if key != current_key:
            visits[key].append(0.0)
            current_key = key
        if contiguous:
            dt = row['t']-previous['t']
            # Only count intervals whose endpoints belong to the same bin.
            prev_key = min(19, int(math.floor((previous['throttle'] + 1e-10) / width)))
            if key == prev_key:
                visits[key][-1] += dt
                speed_key = math.floor(previous['speed_mps']/cfg['speed_bin_mps'])
                cells[(key, speed_key)] += dt
            delta = row['throttle']-anchor
            if abs(delta) > cfg['command_change_threshold']:
                transitions['increase' if delta > 0 else 'decrease'] += 1
                anchor = row['throttle']
        previous = row

    bins = []
    if upper is not None:
        for key in range(min(19, int(math.floor((upper + 1e-10)/width)))+1):
            durations = visits[key]
            seconds = sum(durations)
            repeats = sum(d >= .3 for d in durations)
            status = 'unobserved' if not durations else 'sparse' if seconds < min_seconds or repeats < min_visits else 'observed'
            bins.append(dict(throttle_min=round(key*width, 3), throttle_max=round((key+1)*width, 3),
                             seconds=seconds, visits_over_0_3s=repeats, status=status))
            if status != 'observed':
                add('throttle_'+status, f'{key*width:.2f}〜{(key+1)*width:.2f}: {seconds:.2f}秒 / 0.3秒以上の滞在{repeats}回',
                    '使用予定の入力域なら、異なる速度からこの入力域へ変更する区間を繰り返し記録する。')
    if not usable:
        add('no_valid_data', '品質・モード・直進条件を満たす非制動データが0点',
            '除外理由と記録topicを確認する。データ不足と走行条件の未実施は区別できない。')
    for direction, label in [('increase', '上げる'), ('decrease', '下げる')]:
        count = transitions[direction]
        if count < 3:
            add('input_'+direction, f'閾値を超える入力変化（{label}）: {count}回',
                f'有効な連続走行中にスロットルを{label}操作を追加する。')
    coasts = [s for s in segments if s['kind'] == 'coast']
    if len(coasts) < 3:
        add('coast', f'解析可能な惰行区間: {len(coasts)}区間', '前進中にスロットルを0へ戻す惰行区間を追加する。')
    steady = [s for s in segments if s['steady_speed_mps'] is not None]
    if len(steady) < 3 or len({round(s['throttle']/cfg['command_bin']) for s in steady}) < 2:
        add('steady', f'定常末尾のある区間: {len(steady)}区間',
            '使用する複数の入力値で速度が落ち着くまで保持する。定常速度と応答時間を分離するための補助データになる。')
    if len(segments) < cfg['min_fit_segments']:
        add('constant_segments', f'既存モデル用の一定入力区間: {len(segments)} / {cfg["min_fit_segments"]}',
            '既存の加減速モデルも求める場合は一定入力の保持区間を追加する。変動入力データそのものの無効を意味しない。')
    return dict(schema=1, basis='quality_valid_straight_nonbraking_before_constant_segment_filter',
                valid_samples=len(usable), observed_max_throttle=observed, requested_max_throttle=target,
                thresholds=dict(throttle_bin=width, min_seconds=min_seconds, min_visits=min_visits,
                                min_visit_seconds=.3, transition_delta=cfg['command_change_threshold']),
                throttle_bins=bins,
                speed_throttle_bins=[dict(throttle_min=round(k*width,3), speed_min_mps=v*cfg['speed_bin_mps'], seconds=t)
                                     for (k,v),t in sorted(cells.items())],
                transitions=dict(increase=transitions['increase'], decrease=transitions['decrease']),
                recommendations=recommendations,
                limitations=['分布に基づく目安であり、モデル精度・同定可能性・安全な出力範囲を保証しない。',
                             '未指定の使用予定上限は推測しない。未観測の速度×入力の組合せすべてを不足とは扱わない。',
                             '旋回・低速・異常・欠落・制動区間は分布対象外。欠落を跨ぐ入力変化は数えない。',
                             '滞在回数は独立試行数ではない。入力変化回数は閾値累積方式で、応答が完了した試行数ではない。',
                             '速度はVSLAM推定値。変動入力の動的モデル推定や不足領域の応答補完は行わない。'])
