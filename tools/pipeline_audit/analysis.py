"""Timestamp evidence, exact-frame matching and spike extraction, without clock correction."""
from bisect import bisect_left, bisect_right
from collections import defaultdict
import math
from statistics import median

from .bag import COMMAND, SENSORS


def validate(config):
    if config.get('schema') != 1:
        raise ValueError('Unsupported config schema')
    required = {'source', 'tensor_input', 'tensor_output', 'decoder', 'auto', 'final', 'mode',
                'e2e_status', 'recovery_active', 'recovery_status', 'active_path', 'output_channels',
                'evs_timing', 'logs'}
    if set(config['topics']) != required or any(not isinstance(v, str) or not v.startswith('/') for v in config['topics'].values()):
        raise ValueError('topics must contain the documented roles with absolute topic names')
    other_topics = {k: v for k, v in config['topics'].items() if k != 'source'}
    if len(set(other_topics.values())) != len(other_topics) or (config['topics']['source'] in other_topics.values() and config['topics']['source'] != config['topics']['evs_timing']):
        raise ValueError('Topic roles must be distinct (source may equal evs_timing)')
    for key in ('state_timeout_s', 'mode_timeout_s', 'match_window_s', 'age_spike_ms', 'gap_factor', 'gap_floor_ms'):
        if type(config[key]) not in (float, int) or not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if not 0 <= config['future_tolerance_ms'] <= 100:
        raise ValueError('future_tolerance_ms must be 0..100')
    if type(config['plot_max_points']) is not int or not 100 <= config['plot_max_points'] <= 100000:
        raise ValueError('plot_max_points must be an integer in 100..100000')
    if not isinstance(config['sync_topics'], list) or any(not isinstance(v, str) or not v.startswith('/') for v in config['sync_topics']):
        raise ValueError('sync_topics must be absolute topic names')


def stats(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return dict(count=0, mean=None, p50=None, p95=None, p99=None, max=None, min=None)
    def percentile(p):
        x = (len(values) - 1) * p
        a = int(x)
        return values[a] + (values[min(a + 1, len(values) - 1)] - values[a]) * (x - a)
    return dict(count=len(values), mean=sum(values) / len(values), p50=percentile(.5),
                p95=percentile(.95), p99=percentile(.99), max=values[-1], min=values[0])


def analyze(data, config):
    validate(config)
    records = data['records']
    if not records:
        raise ValueError('Empty recording')
    if any(type(r.get('bag_ns')) is not int or r['bag_ns'] < 0 for r in records):
        raise ValueError('bag_ns must be nonnegative integer nanoseconds')
    # AnyReader merges by recorded receipt time. Never infer original receipt order across clock resets.
    records = sorted(records, key=lambda r: r['bag_ns'])
    origin = records[0]['bag_ns']
    topics = config['topics']
    by_topic = defaultdict(list)
    for row in records:
        by_topic[row['topic']].append(row)
    series = defaultdict(list)
    events, streams, matches, coverage = [], [], [], []

    def point(panel, name, row, value, unit='ms', step=False):
        if type(value) not in (float, int) or not math.isfinite(value):
            return
        series[(panel, name, unit, step)].append(((row['bag_ns'] - origin) / 1e9, float(value)))

    def event(row, kind, detail, value=None, threshold=None):
        events.append(dict(t_s=(row['bag_ns'] - origin) / 1e9, bag_ns=row['bag_ns'],
                           topic=row['topic'], kind=kind, value=value, threshold=threshold, detail=detail))

    ages = defaultdict(list)
    for topic, rows in by_topic.items():
        positive_intervals = [(b['bag_ns'] - a['bag_ns']) / 1e6 for a, b in zip(rows, rows[1:]) if b['bag_ns'] > a['bag_ns']]
        gap_limit = max(config['gap_floor_ms'], config['gap_factor'] * median(positive_intervals)) if positive_intervals else config['gap_floor_ms']
        stamped = [r for r in rows if type(r.get('stamp_ns')) is int and r['stamp_ns'] > 0 and not r.get('error')]
        duplicate = regression = zero = errors = receipt_duplicate = 0
        previous = None
        previous_stamp = None
        for row in rows:
            if row.get('error'):
                errors += 1
                event(row, 'decode_error', row['error'])
            for warning in row.get('warnings', []):
                event(row, 'invalid_payload', warning)
            stamp = row.get('stamp_ns')
            if stamp is not None and stamp <= 0:
                zero += 1
                event(row, 'invalid_stamp', 'Nonpositive source timestamp; not used for frame matching')
            if previous is not None:
                delta = (row['bag_ns'] - previous['bag_ns']) / 1e6
                receipt_duplicate += delta == 0
                if row['type'] in SENSORS or topic in topics.values():
                    point('cadence', topic + ' bag interval', row, delta)
                if delta > gap_limit and (row['type'] in SENSORS or row['type'] == COMMAND):
                    event(row, 'receipt_gap', 'Recorded receive gap; may be producer, transport or recorder', delta, gap_limit)
            if stamp is not None and stamp > 0 and not row.get('error'):
                age = (row['bag_ns'] - stamp) / 1e6
                ages[topic].append(age)
                if row['type'] in SENSORS or row['type'] == COMMAND or topic == topics['source']:
                    point('age', topic + ' header age', row, age)
                if age < -config['future_tolerance_ms']:
                    event(row, 'future_stamp', 'Header is ahead of recorder clock; not negative processing time', age, -config['future_tolerance_ms'])
                if age > config['age_spike_ms'] and (row['type'] in SENSORS or row['type'] == COMMAND):
                    event(row, 'header_age_spike', 'Large header-to-recorder age; clock offset and queueing are not separated', age, config['age_spike_ms'])
                if previous_stamp is not None:
                    delta = (stamp - previous_stamp) / 1e6
                    duplicate += delta == 0
                    if delta < 0:
                        regression += 1
                        event(row, 'stamp_regression', 'Timestamp regression or out-of-order arrival; frame identity can be ambiguous', delta)
                    if row['type'] in SENSORS:
                        point('cadence', topic + ' header interval', row, delta)
                previous_stamp = stamp
            previous = row
        streams.append(dict(topic=topic, type=rows[0]['type'], count=len(rows), stamped=len(stamped),
                            invalid_stamps=zero, decode_errors=errors, duplicate_stamps=duplicate,
                            regressed_stamps=regression, duplicate_bag_times=receipt_duplicate,
                            median_interval_ms=median(positive_intervals) if positive_intervals else None,
                            gap_threshold_ms=gap_limit, **{'age_' + k: v for k, v in stats(ages[topic]).items()}))

    state_times = {name: [r['bag_ns'] for r in rows] for name, rows in by_topic.items()}

    def state(role, when, timeout):
        rows = by_topic.get(topics[role], [])
        idx = bisect_right(state_times.get(topics[role], []), when) - 1
        if idx < 0 or (when - rows[idx]['bag_ns']) / 1e9 > timeout or rows[idx].get('error'):
            return None
        row = rows[idx]
        stamp = row.get('stamp_ns')
        if stamp is not None and (stamp <= 0 or not -config['future_tolerance_ms'] <= (when - stamp) / 1e6 <= timeout * 1000):
            return None
        return row['payload']

    recovery_recorded = bool(by_topic.get(topics['recovery_active']) or by_topic.get(topics['recovery_status']))

    def classification(row):
        mode = state('mode', row['bag_ns'], config['mode_timeout_s'])
        if mode is None:
            return 'mode_unknown_or_stale'
        if mode.get('mode') != 1:
            return 'not_auto'
        e2e = state('e2e_status', row['bag_ns'], config['state_timeout_s'])
        if e2e is None or e2e.get('data') != 'active':
            return 'e2e_not_confirmed_active'
        recovery = state('recovery_active', row['bag_ns'], config['state_timeout_s'])
        recovery_status = state('recovery_status', row['bag_ns'], config['state_timeout_s'])
        if (recovery and recovery.get('data')) or (recovery_status and recovery_status.get('json', {}).get('active')):
            return 'recovery_active'
        if recovery_recorded and recovery is None:
            return 'recovery_unknown_or_stale'
        if not recovery_recorded:
            return 'recovery_unobserved'
        return 'auto_e2e_passthrough'

    sources = defaultdict(list)
    for row in by_topic.get(topics['source'], []):
        if type(row.get('stamp_ns')) is int and row['stamp_ns'] > 0 and not row.get('error'):
            sources[row['stamp_ns']].append(row)
    used = set()
    qualified = []
    stage_first = {}
    for role in ('tensor_input', 'tensor_output', 'decoder', 'auto', 'final'):
        counts = defaultdict(int)
        for row in by_topic.get(topics[role], []):
            stamp = row.get('stamp_ns')
            status = classification(row) if row['type'] == COMMAND else 'tensor_stamp_match'
            candidates = [s for s in sources.get(stamp, []) if abs(row['bag_ns'] - s['bag_ns']) <= config['match_window_s'] * 1e9]
            reason = 'matched' if len(candidates) == 1 else 'ambiguous_source_stamp' if candidates else 'no_exact_source_stamp'
            source = candidates[0] if len(candidates) == 1 else None
            key = (role, source['bag_ns'], stamp) if source else None
            repeated = key in used if key else False
            if key:
                used.add(key)
            delta = (row['bag_ns'] - source['bag_ns']) / 1e6 if source else None
            source_age = (source['bag_ns'] - stamp) / 1e6 if source else None
            if delta is not None and delta < 0:
                reason = 'recorder_order_inversion'
                event(row, reason, 'Matching frame recorded after downstream command; not a causal negative latency', delta)
            if source_age is not None and (source_age < -config['future_tolerance_ms'] or source_age > config['match_window_s'] * 1000):
                reason = 'source_clock_or_age_suspect'
            valid = (reason == 'matched' and not repeated and status in ('auto_e2e_passthrough', 'tensor_stamp_match'))
            counts['messages'] += 1
            counts[reason] += 1
            counts['repeated'] += repeated
            counts['qualified_first_frames'] += valid
            counts['source_state_' + status] += 1
            item = dict(t_s=(row['bag_ns'] - origin) / 1e9, topic=row['topic'], stage=role,
                        bag_ns=row['bag_ns'], source_stamp_ns=stamp, source_bag_ns=source['bag_ns'] if source else None,
                        source_to_stage_recorded_ms=delta, header_age_ms=(row['bag_ns'] - stamp) / 1e6 if stamp and stamp > 0 else None,
                        state=status, repeated=repeated, qualified=valid, reason=reason)
            matches.append(item)
            if valid:
                qualified.append(item)
                stage_first[(stamp, source['bag_ns'], role)] = item
                point('matched', role + ' first frame recorded delta', row, delta)
                if delta > config['age_spike_ms']:
                    event(row, 'matched_frame_spike', 'Same-frame first observation gap; recorder delay included', delta, config['age_spike_ms'])
            elif reason == 'matched' and not repeated:
                point('matched', role + ' first frame (source state unverified)', row, delta)
        coverage.append(dict(stage=role, topic=topics[role], **dict(counts)))

    # Differences only between qualified first observations of the identical frame.
    for item in qualified:
        stages = ('tensor_input', 'tensor_output', 'decoder', 'auto', 'final')
        index = stages.index(item['stage'])
        for earlier in reversed(stages[:index]):
            other = stage_first.get((item['source_stamp_ns'], item['source_bag_ns'], earlier))
            if other:
                delta = (item['bag_ns'] - other['bag_ns']) / 1e6
                row = dict(bag_ns=item['bag_ns'], topic=item['topic'])
                if delta < 0:
                    event(row, 'stage_order_inversion', earlier + ' recorded after ' + item['stage'], delta)
                else:
                    point('matched', earlier + ' -> ' + item['stage'] + ' recorded delta', row, delta)
                break

    # Nearest header distance is a synchronization diagnostic, not a clock-offset estimate.
    sync = []
    for topic in config['sync_topics']:
        rows = [r for r in by_topic.get(topic, []) if type(r.get('stamp_ns')) is int and r['stamp_ns'] > 0 and not r.get('error')]
        rows.sort(key=lambda r: r['stamp_ns'])
        stamps = [r['stamp_ns'] for r in rows]
        distances = []
        for stamp, source_rows in sources.items():
            for source in source_rows:
                idx = bisect_left(stamps, stamp)
                candidates = [rows[j] for j in (idx - 1, idx) if 0 <= j < len(rows)
                              and abs(rows[j]['bag_ns'] - source['bag_ns']) <= config['match_window_s'] * 1e9]
                if candidates:
                    nearest = min(candidates, key=lambda r: abs(r['stamp_ns'] - stamp))
                    delta = (nearest['stamp_ns'] - stamp) / 1e6
                    distances.append(delta)
                    point('sync', topic + ' nearest header - source', source, delta)
        sync.append(dict(topic=topic, **stats(distances)))

    last_states = {}
    last_evs = None
    for row in records:
        p, topic = row['payload'], row['topic']
        if topic == topics['evs_timing'] and not row.get('error'):
            timing = p.get('json', {})
            identity = (timing.get('stream_id'), timing.get('sensor_epoch'))
            sequence = timing.get('sequence')
            if last_evs is not None:
                if identity != last_evs[0]:
                    event(row, 'evs_epoch_change', 'EVS stream/epoch changed; no timestamp correction applied')
                elif type(sequence) is int and type(last_evs[1]) is int and sequence != last_evs[1] + 1:
                    event(row, 'evs_sequence_gap', 'EVS metadata sequence is not consecutive', sequence - last_evs[1])
            last_evs = (identity, sequence)
            generated = timing.get('generated_ros_ns')
            if type(generated) is int and generated > 0:
                point('age', 'EVS metadata generation -> recorder', row, (row['bag_ns'] - generated) / 1e6)
            point('cadence', 'EVS accumulation window', row, p.get('evs_window_ms'))
        if row['type'] == COMMAND:
            if topic in (topics['final'], topics['auto'], topics['decoder']):
                for key in ('steering', 'throttle', 'brake', 'reverse'):
                    point('command', topic + ' ' + key, row, p.get(key), 'normalized', True)
        if topic in (topics['mode'], topics['e2e_status'], topics['recovery_active'], topics['recovery_status'], topics['active_path']):
            value = p.get('mode', p.get('data'))
            if topic == topics['recovery_status']:
                value = p.get('json', {}).get('state')
            if value != last_states.get(topic):
                event(row, 'state_change', str(value))
                last_states[topic] = value
            if topic == topics['e2e_status']:
                value = int(value == 'active')
            elif topic == topics['recovery_status']:
                value = int(p.get('json', {}).get('active', False))
            if isinstance(value, bool):
                value = int(value)
            point('state', topic, row, value, 'state', True)
        if topic == topics['output_channels']:
            for index, value in enumerate(p.get('data', [])[:2]):
                point('board', ('servo' if index == 0 else 'ESC') + ' reported output', row, value, 'us', True)
        if row['type'] == 'sensor_msgs/msg/Imu':
            if row.get('error'):
                continue
            frame = ' [frame=' + p['frame_id'] + ']' if p.get('frame_id') else ' [frame=unknown]'
            for prefix, unit in (('gyro', 'rad/s'), ('accel', 'm/s²')):
                for axis in 'xyz':
                    key = prefix + '_' + axis
                    point('motion', topic + frame + ' ' + key, row, p.get(key), unit)
        if row['type'] == 'nav_msgs/msg/Odometry':
            point('motion', topic + ' speed_x proxy', row, p.get('speed_x'), 'm/s')
        for entry in p.get('status', []):
            if entry['level'] >= 1:
                event(row, 'diagnostic_warning', entry['name'] + ': ' + entry['message'])
            point('state', entry['name'] + ' diagnostic level', row, entry['level'], 'state', True)
            for key, raw in entry.get('values', {}).items():
                try:
                    number = float(raw)
                except (ValueError, TypeError):
                    continue
                if math.isfinite(number):
                    point('diagnostic', entry['name'] + '/' + key, row, number, 'reported value')
        if topic == topics['logs'] and p.get('level', 0) >= 30:
            event(row, 'ros_warning', p.get('name', '') + ': ' + p.get('message', ''))

    limitations = [
        'bag_ns is the recorder timestamp, not the producer callback, publish time or physical sensor exposure time.',
        'Header age includes sensor clock offset, transport, processing and recorder queues; these are not individually identified.',
        'Same-frame intervals use exact timestamps and first observations. Repeated control publications are retained separately.',
        'AUTO, active E2E and fresh inactive recovery evidence are required for qualified command timing; unclassified samples stay visible.',
        'Timestamp equality alone does not prove model lineage or physical synchronization. Ambiguous/reused source stamps are excluded.',
        'Nearest IMU/image stamp distance is not physical clock-offset calibration. No timestamps are shifted or corrected.',
        'AnyReader merges by bag timestamp; original arrival order across a recorder-clock reset cannot be recovered.',
        'Board output_channels are reported PWM settings, not measured ESC-pin waveforms, ESC feedback or measured vehicle response.',
        'VSLAM speed is a proxy, not calibrated ground truth. Correlated spikes do not establish causation.',
        'Control/IMU response comparison uses recorder times and raw sensor axes. No vehicle-frame rotation, gravity removal, clock correction or actuator-delay calibration is applied.',
        'Manual A/B time difference includes recording skew, drivetrain, vehicle dynamics, speed and surface effects; a constant input does not identify a response delay.',
        'Tensor topics are optional. Missing tensor captures cannot reveal GPU-only inference time or queue depth.',
        'EVS timing uses packet-receipt clock mapping, not hardware clock synchronization; accumulation windows and stream/epoch changes remain separate.',
        'Image callback-to-command, bridge serial transmit, board receive and physical actuator delay require additional instrumentation.',
    ]
    missing = [dict(role=k, topic=v) for k, v in topics.items() if not by_topic.get(v)]
    # Use only recorded context at or before the event, never future evidence.
    for entry in events:
        when = entry['bag_ns']
        mode = state('mode', when, config['mode_timeout_s']) or {}
        e2e = state('e2e_status', when, config['state_timeout_s']) or {}
        recovery = state('recovery_active', when, config['state_timeout_s']) or {}
        path = state('active_path', when, config['state_timeout_s']) or {}
        outputs = (state('output_channels', when, config['state_timeout_s']) or {}).get('data', [])
        entry.update(mode=mode.get('mode'), e2e_status=e2e.get('data'),
                     recovery_active=recovery.get('data'), host_path=path.get('data'),
                     esc_reported_us=outputs[1] if len(outputs) >= 2 else None)
    summary = dict(schema='kart.pipeline_audit.v1', bag=data.get('bag'), origin_bag_ns=origin,
                   duration_s=(records[-1]['bag_ns'] - origin) / 1e9,
                   config=config, inventory=data.get('inventory', {}), missing_topics=missing,
                   streams=streams, coverage=coverage, synchronization=sync,
                   observed_latency_ms={role: stats([m['source_to_stage_recorded_ms'] for m in matches
                                                    if m['stage'] == role and m['reason'] == 'matched' and not m['repeated']])
                                        for role in ('tensor_input', 'tensor_output', 'decoder', 'auto', 'final')},
                   qualified_latency_ms={role: stats([m['source_to_stage_recorded_ms'] for m in qualified if m['stage'] == role])
                                         for role in ('tensor_input', 'tensor_output', 'decoder', 'auto', 'final')},
                   event_counts={kind: sum(e['kind'] == kind for e in events) for kind in sorted({e['kind'] for e in events})},
                   limitations=limitations)
    return summary, matches, sorted(events, key=lambda e: e['bag_ns']), [
        dict(panel=key[0], name=key[1], unit=key[2], step=key[3], points=points)
        for key, points in series.items()]
