"""Portable straight-line longitudinal identification using VSLAM as a proxy."""
from bisect import bisect_right
from collections import Counter, defaultdict, deque
import math
import numpy as np


def validate_config(cfg):
    if cfg['clock'] not in ('bag', 'header'):
        raise ValueError('clock must be bag or header')
    if not cfg['vehicle_frame'] or not cfg['allowed_modes'] or any(m not in (1, 2) for m in cfg['allowed_modes']):
        raise ValueError('vehicle_frame required; allowed_modes must be AUTO=1 or MANUAL=2')
    for key, value in cfg.items():
        if key in ('clock', 'vehicle_frame', 'allowed_modes', 'topics'):
            continue
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key}: must be finite and positive')
    if cfg.get('coverage_max_throttle', 1) > 1:
        raise ValueError('coverage_max_throttle must be in (0, 1]')
    if cfg['min_speed_mps'] >= cfg['max_speed_mps']:
        raise ValueError('min_speed_mps must be less than max_speed_mps')
    if len(set(cfg['topics'].values())) != len(cfg['topics']):
        raise ValueError('Topic mappings must be distinct')


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > .01:
        raise ValueError('Invalid static TF quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def static_transform(transforms, target, source):
    """Return target<-source; no dynamic TF, inferred mounting or identity fallback."""
    graph, parents = defaultdict(list), {}
    for tf in transforms:
        parent, child = tf['parent'], tf['child']
        matrix = np.eye(4)
        matrix[:3, :3], matrix[:3, 3] = rotation(tf['q']), tf['xyz']
        if not parent or not child or parent == child or not np.isfinite(matrix).all():
            raise ValueError('Invalid static TF')
        if child in parents:
            old_parent, old_matrix = parents[child]
            if parent != old_parent or not np.allclose(matrix, old_matrix, atol=1e-8, rtol=0):
                raise ValueError(f'Static transform changed for {child}; split recording')
            continue
        parents[child] = (parent, matrix)
        graph[parent].append((child, matrix))
        graph[child].append((parent, np.linalg.inv(matrix)))
    # Refuse malformed frame trees even if the requested path happens to be short.
    for start in parents:
        seen, frame = set(), start
        while frame in parents:
            if frame in seen:
                raise ValueError('Cycle in static TF')
            seen.add(frame)
            frame = parents[frame][0]
    queue, visited = deque([(target, np.eye(4))]), {target}
    while queue:
        frame, transform = queue.popleft()
        if frame == source:
            return transform
        for child, edge in graph[frame]:
            if child not in visited:
                visited.add(child)
                queue.append((child, transform @ edge))
    raise ValueError(f'Missing static TF: {target} <- {source}; specify the real vehicle frame/record /tf_static')


def prepare(data, cfg):
    if data.get('missing_required'):
        raise ValueError('Missing required topics: ' + ', '.join(data['missing_required']))
    prepared, quality = {}, {}
    required = ('odom', 'command', 'mode', 'bridge', 'path') + (() if data.get('offline_pose') else ('vslam',))
    origin = min(r['bag_ns'] for r in data.get('odom', [])) if data.get('odom') else 0
    for key in dict.fromkeys((*required, 'vslam', 'imu', 'speed_diagnostics')):
        rows = data.get(key, [])
        if key in required and not rows:
            raise ValueError(f'No usable {key} messages (check diagnostic hardware_id)')
        out, offsets, last_bag, last_time = [], [], None, None
        repeated_headers = 0
        clamped_headers, max_clamp_ns = 0, 0
        last_header = None
        for row in rows:
            bag, header = row['bag_ns'], row.get('header_ns')
            if type(bag) is not int or (last_bag is not None and bag < last_bag):
                raise ValueError(f'{key}: bag clock goes backwards; split recording')
            last_bag = bag
            if header is not None:
                if type(header) is not int or header <= 0:
                    raise ValueError(f'{key}: invalid/zero header timestamp')
                offsets.append((bag-header)/1e9)
                repeated_headers += header == last_header
                last_header = header
            # Headerless board path has reception time only; explicit in the report.
            selected = header if cfg['clock'] == 'header' and header is not None else bag
            if last_time is not None and selected < last_time:
                correction_ns = last_time - selected
                tolerance_ns = round(cfg.get('command_header_backstep_tolerance_s', .005) * 1e9)
                if key != 'command' or cfg['clock'] != 'header' or correction_ns > tolerance_ns:
                    raise ValueError(f'{key}: selected clock goes backwards; split recording')
                # Keep reception order and the original header for freshness checks.
                # Compare against the high-water mark so repeated backsteps cannot drift.
                selected = last_time
                clamped_headers += 1
                max_clamp_ns = max(max_clamp_ns, correction_ns)
            if key == 'odom' and last_time == selected:
                raise ValueError('odom: duplicate timestamp')
            last_time = selected
            out.append({**row, 't': (selected-origin)/1e9})
        times = [r['t'] for r in out]
        duration = times[-1]-times[0] if len(times) > 1 else 0
        quality[key] = {'count': len(out), 'rate_hz': (len(out)-1)/duration if duration > 0 else None,
                        'max_gap_s': max(np.diff(times), default=0), 'repeated_headers': repeated_headers,
                        'clamped_header_count': clamped_headers,
                        'max_header_clamp_s': max_clamp_ns / 1e9,
                        'bag_minus_header_median_s': float(np.median(offsets)) if offsets else None,
                        'bag_minus_header_p95_s': float(np.percentile(offsets, 95)) if offsets else None}
        if offsets and max(abs(x) for x in offsets) > cfg['max_header_bag_offset_s']:
            raise ValueError(f'{key}: header/bag clock mismatch or excessive age; do not mix replay wall time with sensor time')
        prepared[key] = (times, out)
    return prepared, quality, origin


def previous(series, t, age):
    times, rows = series
    idx = bisect_right(times, t) - 1
    return rows[idx] if idx >= 0 and 0 <= t-times[idx] <= age else None


def analyze(data, cfg, turning=False):
    validate_config(cfg)
    streams, quality, origin = prepare(data, cfg)
    trims = [(r['steering'],r['throttle']) for r in data.get('trim',[])]
    if trims and (not np.isfinite(trims).all() or not np.allclose(trims,trims[0],rtol=0,atol=1e-6)):
        raise ValueError('Vehicle trim changed or invalid; split the recording into fixed-trim trials')
    odom = streams['odom'][1]
    frames = {(r['frame'], r['child']) for r in odom}
    if len(frames) != 1 or any(not f for f in next(iter(frames))):
        raise ValueError('Odometry frame IDs empty or changing; split recording')
    frame, child = next(iter(frames))
    tf = static_transform(data.get('tf_static', []), cfg['vehicle_frame'], child)
    rows, dropped = [], Counter()
    last_t = None
    for od in odom:
        t = od['t']
        cmd = previous(streams['command'], t, cfg['command_max_age_s'])
        mode = previous(streams['mode'], t, cfg['mode_max_age_s'])
        diag = previous(streams['vslam'], t, cfg['diagnostic_max_age_s'])
        bridge = previous(streams['bridge'], t, cfg['diagnostic_max_age_s'])
        path = previous(streams['path'], t, cfg['path_max_age_s'])
        omega = tf[:3, :3] @ np.array(od['angular'], float)
        velocity = tf[:3, :3] @ np.array(od['linear'], float) - np.cross(omega, tf[:3, 3])
        reasons = [od['pose_reason']] if od.get('pose_reason') else []
        if not np.isfinite(velocity).all() or not np.isfinite(omega).all():
            reasons.append('nonfinite_odometry')
        elif velocity[0] < cfg['min_speed_mps'] or velocity[0] > cfg['max_speed_mps']:
            reasons.append('speed_outside_range')
        if abs(omega[2]) > cfg['max_yaw_rate_rps'] or abs(velocity[1]) > cfg['max_lateral_speed_mps']:
            reasons.append('turn_or_lateral_motion')
        if last_t is not None and t-last_t > cfg['odom_max_gap_s']:
            reasons.append('odometry_gap')
        last_t = t
        if cmd is None:
            reasons.append('missing_command')
        else:
            values = [cmd[k] for k in ('steering', 'throttle', 'brake', 'reverse')]
            if (not all(math.isfinite(v) for v in values) or abs(values[0]) > 1 or
                    any(v < 0 or v > 1 for v in values[1:]) or sum(v > 0 for v in values[1:]) > 1):
                reasons.append('invalid_command')
            if not turning and abs(cmd['steering']) > cfg['max_steering']:
                reasons.append('steering')
            if cmd['reverse'] > 0:
                reasons.append('reverse')
            # Mux preserves source stamps. A freshly recorded stale command is still stale.
            if (cmd['bag_ns']-cmd['header_ns'])/1e9 > cfg['command_max_age_s'] or cmd['header_ns'] > cmd['bag_ns']:
                reasons.append('stale_source_command')
        if (mode is None or mode['mode'] not in cfg['allowed_modes'] or
                not 0 <= (mode['bag_ns']-mode['header_ns'])/1e9 <= cfg['mode_max_age_s']):
            reasons.append('mode')
        if not data.get('offline_pose') and (diag is None or diag['level'] != 0 or diag['values'].get('vo_status') != 'OK'):
            reasons.append('vslam_not_ok')
        if (bridge is None or bridge['level'] != 0 or
                bridge['values'].get('host_active') != 'true' or
                bridge['values'].get('esc_brake_rejected') != 'false' or
                bridge['values'].get('esc_command_state') not in ('neutral', 'neutral_initial', 'neutral_after_forward',
                                                                 'neutral_after_brake', 'forward', 'brake')):
            reasons.append('bridge_not_ready')
        if path is None or path['value'] != 2:
            reasons.append('not_host_path')
        imu = previous(streams['imu'], t, cfg['odom_max_gap_s'])
        speed_diag = previous(streams['speed_diagnostics'], t, .3)
        target = None
        if speed_diag and speed_diag.get('message') == 'active':
            try:
                target = float(speed_diag['values']['target_mps'])
                if not math.isfinite(target): target = None
            except (KeyError, ValueError): pass
        row = {'t': t, 'speed_mps': float(velocity[0]), 'yaw_rate_rps': float(omega[2]),
               'throttle': cmd['throttle'] if cmd else None, 'brake': cmd['brake'] if cmd else None,
               'steering': cmd['steering'] if cmd else None, 'mode': mode['mode'] if mode else None,
               'target_mps': target,
               'imu_accel_norm_mps2': float(np.linalg.norm(imu['linear'])) if imu else None,
               'acceleration_mps2': None, 'smooth_speed_mps': None, 'segment': None,
               'reason': '|'.join(reasons), 'quality_reason': '|'.join(reasons)}
        dropped.update(reasons)
        rows.append(row)

    coverage_rows = [dict(row) for row in rows] if not turning else []

    # Contiguous, approximately constant commands. Compare to the segment anchor,
    # not the preceding sample, so slow command ramps cannot masquerade as plateaus.
    groups, group = [], []
    for row in rows:
        changed = group and (row['mode'] != group[0]['mode'] or
            any(abs(row[k]-group[0][k]) > cfg['command_change_threshold'] for k in (('throttle','brake','steering') if turning else ('throttle','brake'))))
        if row['reason'] or changed:
            if group: groups.append(group)
            group = []
        if not row['reason']: group.append(row)
    if group: groups.append(group)
    segments, selected = [], []
    half = cfg['smoothing_window_s']/2
    for group in groups:
        if group[-1]['t']-group[0]['t'] < cfg['min_segment_s']:
            for row in group: row['reason']='short_command_segment'
            dropped['short_command_segment']+=len(group)
            continue
        times = np.array([r['t'] for r in group])
        speeds = np.array([r['speed_mps'] for r in group])
        usable = []
        for row in group:
            if row['t'] < times[0]+cfg['settle_after_command_s']+half or row['t'] > times[-1]-half:
                row['reason']='settling_or_smoothing_edge'
                dropped[row['reason']]+=1
                continue
            mask = abs(times-row['t']) <= half + 1e-9
            if mask.sum() < 3:
                row['reason']='insufficient_smoothing_samples'
                dropped[row['reason']]+=1
                continue
            x = times[mask]-row['t']
            slope, intercept = np.linalg.lstsq(np.column_stack((x, np.ones(len(x)))), speeds[mask], rcond=None)[0]
            if abs(slope) > cfg['max_abs_acceleration_mps2']:
                dropped['acceleration_outlier'] += 1
                row['reason'] = 'acceleration_outlier'
                continue
            row.update(acceleration_mps2=float(slope), smooth_speed_mps=float(intercept), segment=len(segments))
            usable.append(row)
        if not usable: continue
        throttle, brake = (float(np.median([r[k] for r in usable])) for k in ('throttle','brake'))
        steady = []
        # Only a contiguous trailing plateau qualifies for equilibrium kff fitting.
        for r in reversed(usable):
            if abs(r['acceleration_mps2']) > cfg['steady_acceleration_mps2']: break
            if steady and steady[-1]['t']-r['t'] > cfg['odom_max_gap_s']: break
            steady.append(r)
        steady_ok = len(steady)>1 and steady[0]['t']-steady[-1]['t'] >= cfg['steady_min_duration_s']
        segments.append({'id': len(segments), 'start_s': group[0]['t'], 'end_s': group[-1]['t'],
            'samples': len(usable), 'throttle': throttle, 'brake': brake,
            'kind': 'brake' if brake>0 else 'accel' if throttle>0 else 'coast',
            'speed_min_mps': min(r['speed_mps'] for r in usable),
            'speed_max_mps': max(r['speed_mps'] for r in usable),
            'median_acceleration_mps2': float(np.median([r['acceleration_mps2'] for r in usable])),
            'steady_speed_mps': float(np.mean([r['speed_mps'] for r in steady])) if steady_ok and throttle>0 and brake==0 else None})
        selected.extend(usable)
    bins = defaultdict(list)
    for row in selected:
        kind = 'brake' if row['brake']>0 else 'accel' if row['throttle']>0 else 'coast'
        cmd = row['brake'] if kind == 'brake' else row['throttle']
        key = (kind, math.floor(row['smooth_speed_mps']/cfg['speed_bin_mps']), round(cmd/cfg['command_bin']))
        bins[key].append(row)
    table = [{'kind': k[0], 'speed_bin_start_mps': k[1]*cfg['speed_bin_mps'],
              'command_bin_center': k[2]*cfg['command_bin'], 'samples': len(v),
              'segments': len({r['segment'] for r in v}),
              'acceleration_median_mps2': float(np.median([r['acceleration_mps2'] for r in v])),
              'acceleration_p10_mps2': float(np.percentile([r['acceleration_mps2'] for r in v],10)),
              'acceleration_p90_mps2': float(np.percentile([r['acceleration_mps2'] for r in v],90))}
             for k,v in sorted(bins.items())]
    steady = [s for s in segments if s['steady_speed_mps'] is not None]
    kff = None
    if len(steady) >= 3 and len({round(s['throttle']/cfg['command_bin']) for s in steady}) >= 2:
        v = np.array([s['steady_speed_mps'] for s in steady]); u = np.array([s['throttle'] for s in steady])
        gain = float(v@u/(v@v))
        kff = {'value': gain, 'steady_segments': len(steady), 'command_rmse': float(np.sqrt(np.mean((gain*v-u)**2))),
               'speed_range_mps': [float(v.min()), float(v.max())],
               'note': 'Origin-constrained u=kff*v; equilibrium approximation, not acceleration feedforward.'}
    model = {'usable': False, 'reason': 'Turning analysis; no longitudinal fit'} if turning else fit_model(selected, segments, cfg)
    if turning: kff = None
    errors = [r['target_mps']-r['speed_mps'] for r in selected if r['target_mps'] is not None and r['mode']==1]
    warnings = ['VSLAM is an uncalibrated speed proxy, not ground truth.',
                'Clock/estimator/actuator delays are not separately identified.',
                'Bridge diagnostics are sparse command-history observations, not proof of physical braking.',
                'IMU acceleration norm includes gravity; it is a visual cross-check only.']
    for key in ('imu', 'image', 'trim'):
        if not data.get('inventory', {}).get(cfg['topics'][key], {}).get('count') and not data.get(key):
            warnings.append(f'{key} missing: recommended recording requirement not met.')
    if data.get('offline_pose'): warnings.append('Offline pose derivatives: geometric checks only, no online tracking-status guarantee.')
    if not selected: warnings.append('No usable constant-command segments. Review filters and recording.')
    summary = {'schema': 1, 'offline_pose': data.get('offline_pose'), 'source': data.get('source'), 'clock': cfg['clock'], 'origin_bag_ns': origin,
               'headerless_path_clock': 'bag', 'odom_frame': frame, 'odom_child': child,
               'vehicle_frame': cfg['vehicle_frame'], 'static_transform': tf.tolist(),
               'samples': len(rows), 'selected_samples': len(selected), 'segments': len(segments),
               'excluded_reasons_overlapping': dict(dropped), 'quality': quality,
               'inventory': data.get('inventory', {}), 'kff_candidate': kff, 'model': model,
               'observed_trim': list(trims[0]) if trims else None,
               'auto_speed_rmse_mps': float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
               'warnings': warnings, 'config': cfg}
    if not turning:
        from data_coverage import diagnose
        summary['coverage'] = diagnose(coverage_rows, segments, cfg)
    return summary, rows, segments, table


def fit_model(rows, segments, cfg):
    """Fit a=A*throttle-B*brake-C*v-D, with whole segments held out."""
    unavailable = lambda reason: {'usable': False, 'reason': reason}
    if len(segments) < cfg['min_fit_segments']:
        return unavailable('Need at least min_fit_segments independent constant-command segments')
    has_brake = sum(s['kind']=='brake' for s in segments) >= 3
    # With insufficient braking excitation, omit braking rows instead of treating them as coasting.
    usable = [s for s in segments if has_brake or s['kind']!='brake']
    train_ids, val_ids = [], []
    for kind in ('accel','coast','brake'):
        ids = [s['id'] for s in usable if s['kind']==kind]
        if len(ids)>=3:
            val_ids.extend(ids[-max(1,len(ids)//4):]); train_ids.extend(ids[:-max(1,len(ids)//4)])
        else: train_ids.extend(ids)
    if len(val_ids)<2 or len(train_ids)<3:
        return unavailable('Need repeated acceleration/coast segments for independent segment validation')
    def design(ids):
        part = [r for r in rows if r['segment'] in ids]
        x = np.array([[r['throttle'], -r['smooth_speed_mps'], -1.] + ([-r['brake']] if has_brake else []) for r in part])
        y = np.array([r['acceleration_mps2'] for r in part])
        counts = Counter(r['segment'] for r in part)
        weights = np.array([1/math.sqrt(counts[r['segment']]) for r in part])
        return x,y,weights
    x,y,w = design(train_ids); xv,yv,_ = design(val_ids)
    scale = np.linalg.norm(x*w[:,None],axis=0)
    if np.any(scale<1e-9): return unavailable('Insufficient command/speed excitation')
    normalized = x*w[:,None]/scale
    condition = float(np.linalg.cond(normalized))
    if np.linalg.matrix_rank(normalized)<x.shape[1] or condition>1000:
        return unavailable('Ill-conditioned fit: vary speed and command independently, include coasting')
    coefficients = np.linalg.lstsq(normalized,y*w,rcond=None)[0]/scale
    a,c,d = coefficients[:3]; b = coefficients[3] if has_brake else 0.
    if a<=0 or c<=0 or d<0 or (has_brake and b<=0):
        return unavailable('Nonphysical fitted signs; insufficient data/model mismatch (no silent clipping)')
    rmse = float(np.sqrt(np.mean((xv@coefficients-yv)**2)))
    baseline = float(np.sqrt(np.mean((yv-np.mean(y))**2)))
    # Compare to constant acceleration as well as the absolute threshold.
    passed = rmse <= cfg['max_validation_rmse_mps2'] and rmse < baseline
    fit_rows = [r for r in rows if r['segment'] in train_ids]
    rollout_errors = []
    for sid in val_ids:
        part = [r for r in rows if r['segment']==sid]
        speed = part[0]['smooth_speed_mps']
        for before,after in zip(part,part[1:]):
            # Do not integrate across rejected/outlier samples.
            dt=after['t']-before['t']
            if dt>cfg['odom_max_gap_s']:
                speed=after['smooth_speed_mps']; continue
            speed=max(0.,speed+dt*(a*before['throttle']-b*before['brake']-c*speed-d))
            rollout_errors.append(speed-after['smooth_speed_mps'])
    speed_rmse=float(np.sqrt(np.mean(np.square(rollout_errors)))) if rollout_errors else None
    passed = passed and speed_rmse is not None and speed_rmse<=cfg['max_validation_speed_rmse_mps']
    return {'usable': bool(passed), 'reason': 'Held-out segment check passed' if passed else 'Held-out acceleration error too large',
            'equation': 'a = A*throttle - B*brake - C*v - D; v>=0',
            'A': float(a), 'B': float(b), 'C': float(c), 'D': float(d), 'braking_identified': has_brake,
            'train_segment_ids': train_ids, 'validation_segment_ids': val_ids,
            'validation_acceleration_rmse_mps2': rmse, 'baseline_acceleration_rmse_mps2': baseline,
            'validation_speed_rollout_rmse_mps': speed_rmse,
            'condition_number': condition, 'assumed_delay_s': cfg['assumed_delay_s'],
            'speed_range_mps': [min(r['smooth_speed_mps'] for r in fit_rows), max(r['smooth_speed_mps'] for r in fit_rows)],
            'max_throttle_observed': max(r['throttle'] for r in fit_rows),
            'max_brake_observed': max(r['brake'] for r in fit_rows)}


def validate_feedforward(settings):
    speeds, throttles = settings.get('ff_speed_mps', []), settings.get('ff_throttle', [])
    if len(speeds) != len(throttles) or (speeds and len(speeds) < 2):
        raise ValueError('Feedforward arrays must have equal length >= 2, or both be empty')
    if (any(not math.isfinite(v) or v <= 0 for v in speeds) or
        any(not math.isfinite(u) or not 0 <= u <= 1 for u in throttles) or
        any(b <= a for a, b in zip(speeds, speeds[1:])) or
        any(b < a for a, b in zip(throttles, throttles[1:]))):
        raise ValueError('Invalid feedforward table')
    for key in ('throttle_rise_rate', 'throttle_fall_rate'):
        if not math.isfinite(settings.get(key, 0)) or settings.get(key, 0) < 0:
            raise ValueError(f'Invalid {key}')


def feedforward(target, settings):
    speeds, throttles = settings.get('ff_speed_mps', []), settings.get('ff_throttle', [])
    if not speeds:
        return settings['kff'] * target
    return float(np.interp(target, [0., *speeds], [0., *throttles]))


def simulate(model, settings, targets, stage_s, delay_s):
    """Illustrative scalar model only; mirrors SpeedPid equations, not ROS gates."""
    if not model.get('usable'): raise ValueError('Model has not passed held-out checks')
    for key in ('kp','ki','kd','kff','integral_limit','derivative_tau_s','throttle_limit','brake_limit'):
        if not math.isfinite(settings[key]) or settings[key]<0: raise ValueError(f'Invalid {key}')
    if any(settings[k]>1 for k in ('integral_limit','throttle_limit','brake_limit')):
        raise ValueError('Normalized limits must be <=1')
    validate_feedforward(settings)
    if settings['brake_limit']>0 and not model['braking_identified']:
        raise ValueError('Braking model unavailable; use brake_limit=0 or record braking trials')
    if not math.isfinite(stage_s) or stage_s<=0 or not math.isfinite(delay_s) or delay_s<0:
        raise ValueError('Invalid duration/delay')
    if not targets or any(not math.isfinite(v) or v<0 for v in targets): raise ValueError('Invalid targets')
    dt=.02; speed=targets[0]; integral=derivative=0.; previous_speed=speed
    history, result = [], []
    previous_throttle = 0.
    # Begin at the first requested speed to avoid claiming a launch-from-rest model.
    for index in range(math.ceil(stage_s*len(targets)/dt)):
        t=index*dt; target=targets[min(int(t/stage_s),len(targets)-1)]
        error=target-speed
        if target==0: integral=derivative=0.; previous_speed=speed
        derivative += dt/(settings['derivative_tau_s']+dt)*(-(speed-previous_speed)/dt-derivative)
        previous_speed=speed
        p=settings['kp']*error; d=settings['kd']*derivative; ff=feedforward(target, settings)
        candidate=float(np.clip(integral+settings['ki']*error*dt,-settings['integral_limit'],settings['integral_limit']))
        trial=p+candidate+d+ff
        rise, fall = settings.get('throttle_rise_rate', 0), settings.get('throttle_fall_rate', 0)
        ceiling = min(settings['throttle_limit'], previous_throttle + rise * dt) if rise else settings['throttle_limit']
        floor = min(ceiling, max(0., previous_throttle - fall * dt)) if fall else 0.
        if not ((trial>ceiling and error>0) or (trial < (floor if floor > 0 else -settings['brake_limit']) and error<0)):
            integral=candidate
        u=float(np.clip(p+integral+d+ff,-settings['brake_limit'],settings['throttle_limit']))
        throttle=float(np.clip(max(0.,u),floor,ceiling)) if target>0 and u>=0 else 0.
        brake=max(0.,-u)
        previous_throttle=throttle
        if target==0: integral=derivative=0.
        history.append((t,throttle,brake))
        delayed = math.floor((t-delay_s)/dt+1e-9)
        # Before the first candidate command arrives, use equilibrium input at initial speed.
        applied = history[delayed][1:] if delayed>=0 else ((model['C']*speed+model['D'])/model['A'],0.)
        acceleration=model['A']*applied[0]-model['B']*applied[1]-model['C']*speed-model['D']
        extrapolated = (not model['speed_range_mps'][0]<=speed<=model['speed_range_mps'][1] or
                        applied[0]>model['max_throttle_observed'] or applied[1]>model['max_brake_observed'])
        result.append({'t':t,'target_mps':target,'speed_mps':speed,'throttle':throttle,'brake':brake,
                       'acceleration_mps2':acceleration,'extrapolated':extrapolated})
        speed=max(0.,speed+acceleration*dt)
        if not math.isfinite(speed) or speed>100: raise ValueError('Simulation diverged')
    return result
