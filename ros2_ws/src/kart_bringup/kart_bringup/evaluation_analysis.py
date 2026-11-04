"""Read-only result-bag analysis. Header time and receipt wall time stay separate."""
import argparse
from bisect import bisect_left
from collections import Counter
import json
import math
from pathlib import Path
import statistics


SLAM = '/visual_slam/vis/slam_odometry'
VO = '/visual_slam/tracking/odometry'


def stats(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return dict(count=0, median=None, p95=None, max=None, mean=None)
    return dict(count=len(values), median=statistics.median(values),
                p95=values[math.ceil(.95 * len(values)) - 1], max=values[-1],
                mean=statistics.mean(values))


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(q):
    norm = math.sqrt(sum(v*v for v in q))
    if not math.isfinite(norm) or norm == 0:
        raise ValueError('Invalid quaternion')
    x, y, z, w = (v / norm for v in q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def angle(v):
    return math.atan2(math.sin(v), math.cos(v))


def reference_rows(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        r = list(map(float, line.split()))
        if len(r) != 8 or not all(map(math.isfinite, r)):
            raise ValueError('Reference must contain finite TUM rows: t x y z qx qy qz qw')
        if rows and r[0] <= rows[-1]['t']:
            raise ValueError('Reference timestamps must strictly increase')
        rows.append(dict(t=r[0], x=r[1], y=r[2], z=r[3], yaw=yaw(r[4:])))
    if not rows:
        raise ValueError('Reference TUM is empty')
    return rows


def compare_reference(poses, reference, parent, child, max_gap):
    times = [r['t'] for r in reference]
    errors = []
    previous = None
    for p in poses:
        if p['parent'] != parent or p['child'] != child:
            raise ValueError('Reference frames differ from SLAM pose; transform explicitly before comparison')
        if previous is not None and p['t'] <= previous:
            raise ValueError('SLAM timestamps repeat/regress; analyze separate replay runs')
        previous = p['t']
        i = bisect_left(times, p['t'])
        if i < len(times) and abs(times[i] - p['t']) < 1e-7:
            r = reference[i]
        elif i == 0 or i == len(times) or times[i] - times[i-1] > max_gap:
            continue
        else:
            a, b = reference[i-1], reference[i]
            f = (p['t'] - a['t']) / (b['t'] - a['t'])
            r = {k: a[k] + f*(b[k]-a[k]) for k in ('x', 'y', 'z')}
            r['yaw'] = a['yaw'] + f*angle(b['yaw']-a['yaw'])
        errors.append(dict(t=p['t'], xy_m=math.hypot(p['x']-r['x'], p['y']-r['y']),
                           yaw_deg=abs(math.degrees(angle(p['yaw']-r['yaw'])))))
    return dict(matched=len(errors), unmatched=len(poses)-len(errors),
                xy_m=stats([e['xy_m'] for e in errors]),
                yaw_deg=stats([e['yaw_deg'] for e in errors])), errors


def summarize(poses, diagnostics, max_gap):
    result = {'poses': {}, 'tracking': {}}
    for topic, rows in poses.items():
        intervals = [b['t']-a['t'] for a, b in zip(rows, rows[1:])]
        receipt = [b['receipt']-a['receipt'] for a, b in zip(rows, rows[1:])]
        steps = [math.hypot(b['x']-a['x'], b['y']-a['y']) for a,b in zip(rows, rows[1:])
                 if 0 < b['t']-a['t'] <= max_gap]
        result['poses'][topic] = dict(count=len(rows), frames=sorted(set((p['parent'],p['child']) for p in rows)),
            header_interval_s=stats([v for v in intervals if v > 0]),
            receipt_interval_s=stats([v for v in receipt if v > 0]),
            header_gap_count=sum(v > max_gap for v in intervals),
            nonincreasing_header_count=sum(v <= 0 for v in intervals),
            consecutive_xy_step_m=stats(steps))
    lost_runs = 0
    lost_seconds = known_seconds = unknown_seconds = 0.
    previous = None
    for d in diagnostics:
        delta = d['t'] - previous['t'] if previous else None
        continuous = delta is not None and 0 < delta <= max_gap
        if d['vo'] == 'Lost' and (not continuous or previous['vo'] != 'Lost'):
            lost_runs += 1
        if continuous and previous['vo'] in ('Lost', 'OK'):
            known_seconds += delta
            if previous['vo'] == 'Lost':
                lost_seconds += delta
        elif delta is not None and delta > 0:
            unknown_seconds += delta
        previous = d
    result['tracking'] = dict(diagnostic_count=len(diagnostics), observed_lost_runs=lost_runs,
        observed_lost_s=lost_seconds, classified_interval_s=known_seconds,
        unclassified_interval_s=unknown_seconds,
        nonincreasing_header_count=sum(b['t']<=a['t'] for a,b in zip(diagnostics,diagnostics[1:])),
        trailing_state=diagnostics[-1]['vo'] if diagnostics else None,
        note='Last state duration is censored; missing diagnostics are unknown, never OK.')
    return result


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions('', ''))
    topics = {r.name: r.type for r in reader.get_all_topics_and_types()}
    wanted = {t: k for t,k in topics.items() if t in (SLAM, VO, '/localization/pose_hint', '/rosout')
              or k == 'diagnostic_msgs/msg/DiagnosticArray'}
    expected = {SLAM:'nav_msgs/msg/Odometry', VO:'nav_msgs/msg/Odometry',
                '/localization/pose_hint':'geometry_msgs/msg/PoseWithCovarianceStamped', '/rosout':'rcl_interfaces/msg/Log'}
    for t,k in expected.items():
        if t in wanted and wanted[t] != k:
            raise ValueError(f'Unexpected topic type: {t}: {wanted[t]}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
    classes = {t:get_message(k) for t,k in wanted.items()}
    poses = {SLAM:[], VO:[]}
    diagnostics, hints, events = [], [], []
    counts = Counter()
    while reader.has_next():
        topic, raw, receipt_ns = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(raw, classes[topic])
        counts[topic] += 1
        receipt = receipt_ns * 1e-9
        if topic in poses:
            p,q = msg.pose.pose.position, msg.pose.pose.orientation
            row = dict(t=seconds(msg.header.stamp), receipt=receipt, parent=msg.header.frame_id,
                       child=msg.child_frame_id, x=p.x,y=p.y,z=p.z,yaw=yaw([q.x,q.y,q.z,q.w]))
            if not all(math.isfinite(row[k]) for k in ('t','x','y','z','yaw')):
                raise ValueError(f'Non-finite pose on {topic}')
            poses[topic].append(row)
        elif wanted[topic] == 'diagnostic_msgs/msg/DiagnosticArray':
            for status in msg.status:
                if status.hardware_id == 'visual_slam':
                    values = {v.key:v.value for v in status.values}
                    diagnostics.append(dict(t=seconds(msg.header.stamp), receipt=receipt,
                        vo=values.get('vo_status','Unknown'), localized=values.get('localized_in_exist_map','Unknown')))
        elif topic == '/localization/pose_hint':
            hints.append(dict(t=seconds(msg.header.stamp), receipt=receipt))
        elif topic == '/rosout' and ('visual_slam' in msg.name or 'visual_global_localization' in msg.name):
            events.append(dict(t=seconds(msg.stamp), receipt=receipt, node=msg.name, text=msg.msg))
    return poses, diagnostics, hints, events, dict(counts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, help='Evaluation directory containing bag/ and manifest.json, or bag directory')
    parser.add_argument('--output', type=Path, help='New analysis directory (default: RUN/analysis)')
    parser.add_argument('--reference-tum', type=Path)
    parser.add_argument('--reference-parent-frame', default='')
    parser.add_argument('--reference-child-frame', default='')
    parser.add_argument('--max-gap', type=float, default=.25, help='Maximum classified diagnostic interval / pose step interval, seconds')
    parser.add_argument('--reference-max-gap', type=float, default=.1, help='Maximum TUM interpolation interval, seconds')
    args = parser.parse_args()
    if any(not math.isfinite(v) or v <= 0 for v in (args.max_gap,args.reference_max_gap)):
        parser.error('Gap thresholds must be positive and finite')
    if args.reference_tum and not (args.reference_parent_frame and args.reference_child_frame):
        parser.error('Reference comparison requires explicit parent and child frame names')
    run = args.run.expanduser().resolve(strict=True)
    bag = run/'bag' if (run/'bag').is_dir() else run
    if not (bag/'metadata.yaml').is_file():
        parser.error('Bag must be finalized (metadata.yaml missing); stop bringup with Ctrl-C first')
    output = (args.output or run/'analysis').expanduser().resolve()
    if output.exists():
        parser.error(f'Refusing to overwrite {output}')
    poses, diagnostics, hints, events, counts = read_bag(bag)
    summary = summarize(poses, diagnostics, args.max_gap)
    summary.update(source_bag=str(bag), message_counts=counts, hint_count=len(hints),
        thresholds=dict(max_gap_s=args.max_gap, reference_max_gap_s=args.reference_max_gap),
        warnings=[], reference=None,
        notes=['Receipt timestamps are wall time; header timestamps are source replay time.',
               'Pose step is not automatically a jump/error. Lost statistics do not cover silent gaps.',
               'Hint output is not service request time; success logs do not prove map merge or tracking recovery.'])
    for topic in (SLAM, VO):
        if not poses[topic]: summary['warnings'].append(f'No messages: {topic}')
    if not diagnostics: summary['warnings'].append('No VSLAM diagnostics; tracking health unknown')
    errors = []
    if args.reference_tum:
        reference = reference_rows(args.reference_tum)
        summary['reference'], errors = compare_reference(poses[SLAM], reference,
            args.reference_parent_frame, args.reference_child_frame, args.reference_max_gap)
        summary['reference'].update(path=str(args.reference_tum.resolve()),
            parent_frame=args.reference_parent_frame, child_frame=args.reference_child_frame,
            note='No fitted alignment. Measures reference consistency, not independent ground truth.')
        if not errors: summary['warnings'].append('No matched reference timestamps')
    native_log = run/'process_output.log'
    summary['native_reindex_error_count'] = native_log.read_text(errors='replace').count(
        'Failed to reindex pose graph') if native_log.exists() else None
    summary['localization_success_log_count'] = sum('Successfully localized at' in e['text'] for e in events)
    summary['localization_failure_log_count'] = sum('Failed to localize in map' in e['text'] for e in events)
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    fig = make_subplots(rows=3, cols=1, subplot_titles=('SLAM map trajectory (VO not overlaid across frames)',
        'Tracking state by source header time: OK=1 / Lost=0', 'Reference consistency: XY m / yaw degrees'))
    rows = poses[SLAM]
    fig.add_trace(go.Scatter(x=[p['x'] for p in rows],y=[p['y'] for p in rows],mode='lines',name='SLAM map'),row=1,col=1)
    fig.add_trace(go.Scatter(x=[d['t'] for d in diagnostics],y=[1 if d['vo']=='OK' else 0 if d['vo']=='Lost' else None for d in diagnostics],mode='markers',name='VO state'),row=2,col=1)
    for key in ('xy_m','yaw_deg'):
        fig.add_trace(go.Scatter(x=[e['t'] for e in errors],y=[e[key] for e in errors],mode='lines',name=key),row=3,col=1)
    fig.update_yaxes(scaleanchor='x',scaleratio=1,row=1,col=1)
    fig.update_layout(height=1100)
    output.mkdir(parents=True, exist_ok=False)
    (output/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False))
    (output/'samples.json').write_text(json.dumps(dict(poses=poses,diagnostics=diagnostics,hints=hints,events=events,reference_errors=errors),allow_nan=False))
    fig.write_html(str(output/'report.html'),include_plotlyjs=True)
    print(json.dumps(summary,indent=2,ensure_ascii=False))
    print(f'Report: {output / "report.html"}')
