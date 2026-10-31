"""Compare measured conditions; coverage never establishes collision causality."""
from collections import defaultdict
import math

DEFAULTS = dict(steering_bin=.1, speed_bin=.5, curvature_bin=.1,
                max_gap_s=.12, min_seconds=1., min_recordings=2, min_visits=3,
                min_visit_seconds=.3)


def settings(overrides):
    if set(overrides)-set(DEFAULTS):
        raise ValueError('Unknown coverage settings')
    cfg = dict(DEFAULTS, **overrides)
    for k,v in cfg.items():
        if type(v) not in (int,float) or not math.isfinite(v) or v <= 0:
            raise ValueError(f'{k} must be positive and finite')
        if k in ('min_recordings','min_visits') and int(v) != v:
            raise ValueError(f'{k} must be an integer')
    return cfg


def key(row, dimension, cfg):
    fields = {'steering': [('steering','steering_bin')],
              'steering_speed': [('steering','steering_bin'),('speed','speed_bin')],
              'curvature_speed': [('curvature','curvature_bin'),('speed','speed_bin')]}[dimension]
    if any(row.get(f) is None or not math.isfinite(row[f]) for f,_ in fields):
        return None
    return tuple(math.floor(row[f]/cfg[w]+1e-9) for f,w in fields)


def distribution(records, dimension, cfg):
    cells = {}
    for recording in records:
        previous, active, duration = None, None, 0.
        def flush():
            if active is not None and duration >= cfg['min_visit_seconds']:
                cells[active]['visits'] += 1
        for row in recording['rows']:
            k = key(row, dimension, cfg) if row['valid'] else None
            contiguous = (previous is not None and row['mode']==previous['mode']
                          and 0 < (row['stamp_ns']-previous['stamp_ns'])/1e9 <= cfg['max_gap_s'])
            if k != active or not contiguous:
                flush(); active, duration = k, 0.
            if k is not None:
                cell = cells.setdefault(k, dict(seconds=0., samples=0, visits=0, recordings=set(), examples=[]))
                cell['samples'] += 1
                if len(cell['examples']) < 3:
                    cell['examples'].append(dict(recording=recording['id'], stamp_ns=row['stamp_ns'], artifact=row.get('artifact')))
                if contiguous and previous['valid'] and key(previous,dimension,cfg)==k:
                    dt = (row['stamp_ns']-previous['stamp_ns'])/1e9
                    duration += dt; cell['seconds'] += dt
                    cell['recordings'].add(recording['id'])
            previous = row
        flush()
    return {k:dict(v, recordings=sorted(v['recordings'])) for k,v in cells.items()}


def status(cell, cfg):
    if cell is None: return 'unobserved'
    return 'observed_support' if (cell['seconds'] >= cfg['min_seconds'] and
             len(cell['recordings']) >= cfg['min_recordings'] and cell['visits'] >= cfg['min_visits']) else 'sparse'


def compare(records, cfg):
    result = dict(settings=cfg, distributions={}, comparisons=[], events=[], evaluation_samples=[], recommendations=[])
    dimensions = ('steering','steering_speed','curvature_speed')
    baselines = {}
    available = {role:any(r['role']==role and any(row['valid'] for row in r['rows']) for r in records) for role in ('training','reference')}
    result['baseline_available'] = available
    for role in ('training','reference'):
        baselines[role] = {}
        for dim in dimensions:
            dist = distribution([r for r in records if r['role']==role], dim, cfg)
            baselines[role][dim] = dist
            widths = {'steering':[cfg['steering_bin']], 'steering_speed':[cfg['steering_bin'],cfg['speed_bin']],
                      'curvature_speed':[cfg['curvature_bin'],cfg['speed_bin']]}[dim]
            result['distributions'][role+'_'+dim] = [dict(bin=list(k),
                ranges=[[round(i*w,6),round((i+1)*w,6)] for i,w in zip(k,widths)],
                **v, status=status(v,cfg)) for k,v in sorted(dist.items())]
    for recording in records:
        if recording['role'] != 'evaluation': continue
        for row in recording['rows']:
            annotated = dict(recording=recording['id'], **row)
            for baseline in ('training','reference'):
                for dim in dimensions:
                    k = key(row,dim,cfg) if row['valid'] else None
                    annotated[baseline+'_'+dim] = 'unknown' if k is None or not available[baseline] else status(baselines[baseline][dim].get(k),cfg)
            result['evaluation_samples'].append(annotated)
        windows = [('whole_recording', recording['rows'])]
        for event in recording.get('events',[]):
            stamp, seconds = event['stamp_ns'], event.get('before_s',3.)
            if type(stamp) is not int or not math.isfinite(seconds) or seconds <= 0:
                raise ValueError('Event requires integer stamp_ns and positive before_s')
            window = [r for r in recording['rows'] if stamp-int(seconds*1e9) <= r['stamp_ns'] < stamp]
            covered = sum((b['stamp_ns']-a['stamp_ns'])/1e9 for a,b in zip(window,window[1:])
                          if a['valid'] and b['valid'] and a['mode']==b['mode']
                          and 0 < (b['stamp_ns']-a['stamp_ns'])/1e9 <= cfg['max_gap_s'])
            result['events'].append(dict(recording=recording['id'], name=event['name'], stamp_ns=stamp,
                                        before_s=seconds, samples=len(window), valid_samples=sum(r['valid'] for r in window),
                                        valid_contiguous_seconds=covered, requested_seconds=seconds,
                                        status='available' if covered >= .8*seconds else 'partial' if covered>0 else 'unknown',
                                        interval='[event-before, event); post-impact excluded'))
            windows.append((event['name'],window))
        for window_name, rows in windows:
            for baseline in ('training','reference'):
                for dim in dimensions:
                    counts = defaultdict(int)
                    for row in rows:
                        k = key(row,dim,cfg) if row['valid'] else None
                        counts['unknown' if k is None or not available[baseline] else status(baselines[baseline][dim].get(k),cfg)] += 1
                    result['comparisons'].append(dict(recording=recording['id'], window=window_name,
                        baseline=baseline, dimension=dim, samples=len(rows), **{s:counts[s] for s in ('unknown','unobserved','sparse','observed_support')}))
    for item in result['comparisons']:
        if item['baseline'] != 'training': continue
        if item['unobserved'] or item['sparse']:
            result['recommendations'].append(dict(recording=item['recording'],window=item['window'],dimension=item['dimension'],
                evidence=f"未観測{item['unobserved']}点 / 少ない{item['sparse']}点",
                action='evaluation_coverage.csvの該当時刻を確認し、良好な手動走行で類似条件を追加収集する候補。AUTO操舵をそのまま教師にしない。'))
        if item['unknown']:
            result['recommendations'].append(dict(recording=item['recording'],window=item['window'],dimension=item['dimension'],
                evidence=f"判定不能{item['unknown']}点",action='追加収集の前にpose・同期・除外理由を確認する。'))
    result['limitations'] = [
        'trainingは指定されたsamples.jsonlの分布。学習時のsplit・sampling・augmentation・重み付けは自動再現しない。',
        'observed_supportは指定した分布閾値を満たしたという意味。学習十分性・安全性・衝突原因の証明ではない。',
        '画像・EVSの見た目、照明、障害物、横位置、モデル予測誤差は未評価。同じbinでも同じシーンとは限らない。',
        'AUTO操舵はモデル/制御器の要求値であり正解教師ではない。曲率は実際に推定された軌跡で、望ましい軌跡ではない。',
        '別bagの座標を一致と仮定せず、コース上の同一地点・周回を自動対応しない。recordingsは独立したbag数で周回数ではない。',
        '速度・曲率はVSLAM由来。欠落や衝突直前の推定不良はunknownとし、支持ありへ補完しない。',
        '衝突時刻はユーザー指定。衝突検知や原因の自動判定は行わず、指定時刻より前だけを比較する。']
    return result
