#!/usr/bin/env python3
"""Compare sim truth and cuVSLAM odometry using header stamps, without ROS/GPU."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def read_bag(path, pose_topic):
    from rosbags.highlevel import AnyReader
    poses, statuses = [], []
    with AnyReader([Path(path).expanduser().resolve()]) as reader:
        connections = [c for c in reader.connections if c.topic in (pose_topic, '/visual_slam/status')]
        if not any(c.topic == pose_topic for c in connections):
            raise ValueError(f'{path}: missing {pose_topic}')
        for connection, _, raw in reader.messages(connections=connections):
            # Status is a custom message: deserialization needs its recorded definition.
            message = reader.deserialize(raw, connection.msgtype)
            stamp = message.header.stamp
            time = int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)
            if connection.topic == '/visual_slam/status':
                statuses.append((time, int(message.vo_state)))
                continue
            pose = message.pose.pose if hasattr(message.pose, 'pose') else message.pose
            p, q = pose.position, pose.orientation
            poses.append((time, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
                          message.header.frame_id, getattr(message, 'child_frame_id', '')))
    if len(poses) < 2:
        raise ValueError(f'{pose_topic}: need at least two poses')
    if len({row[3] for row in poses}) != 1 or len({row[4] for row in poses}) != 1:
        raise ValueError(f'{pose_topic}: frame IDs change during recording')
    stamps = np.array([row[0] for row in poses], dtype=np.int64)
    if np.any(np.diff(stamps) <= 0):
        raise ValueError(f'{pose_topic}: header stamps repeat/go backwards; split replay runs')
    xyz = np.array([row[1] for row in poses], float)
    quats = np.array([row[2] for row in poses], float)
    if not np.isfinite(xyz).all():
        raise ValueError(f'{pose_topic}: nonfinite position')
    for q in quats:
        rotation(q)
    return stamps, xyz, quats, poses[0][3], poses[0][4], statuses


def compare(truth, estimate, max_gap=0.05):
    tt, tp, tq = truth[:3]
    et, ep, eq = estimate[:3]
    # Relative seconds retain precision even when ROS uses Unix epoch stamps.
    origin = int(tt[0])
    ts = (tt-origin)/1e9
    es = (et-origin)/1e9
    right = np.searchsorted(tt, et, side='left')
    right = np.clip(right, 1, len(tt)-1)
    left = right-1
    exact = tt[right] == et
    valid = (et >= tt[0]) & (et <= tt[-1]) & (exact | ((tt[right]-tt[left])/1e9 <= max_gap))
    if np.count_nonzero(valid) < 3:
        raise ValueError('Insufficient shared header times; check use_sim_time / override_publishing_stamp / truth gaps')
    es, ep, eq = es[valid], ep[valid], eq[valid]
    l, r = left[valid], right[valid]
    fraction = ((et[valid]-tt[l])/(tt[r]-tt[l]))[:, None]
    positions = tp[l]*(1-fraction)+tp[r]*fraction
    # Short truth intervals: normalized shortest-path interpolation of rotations.
    qright = tq[r].copy()
    qright[np.sum(tq[l]*qright, axis=1) < 0] *= -1
    quats = tq[l]*(1-fraction)+qright*fraction
    quats /= np.linalg.norm(quats, axis=1)[:, None]
    align = rotation(quats[0]) @ rotation(eq[0]).T
    aligned = (ep-ep[0]) @ align.T + positions[0]
    error = aligned-positions
    distance_error = np.linalg.norm(error, axis=1)
    angles = []
    for gt, est in zip(quats, eq):
        relative = rotation(gt).T @ align @ rotation(est)
        angles.append(np.degrees(np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1))))
    dt = np.diff(es)
    truth_step = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    estimate_step = np.linalg.norm(np.diff(aligned, axis=0), axis=1)
    truth_length, estimate_length = float(truth_step.sum()), float(estimate_step.sum())
    # Avoid speed estimates over missing VSLAM output intervals.
    speed_valid = (dt > 0) & (dt <= .1)
    gt_speed, est_speed = truth_step[speed_valid]/dt[speed_valid], estimate_step[speed_valid]/dt[speed_valid]
    statuses = estimate[5]
    counts = {str(state): sum(s == state for _, s in statuses) for state in sorted({s for _, s in statuses})}
    summary = {
        'alignment': 'initial full SE(3), no scale fitting; identical header stamps, interpolated truth',
        'truth_frame': truth[3], 'estimate_frame': estimate[3],
        'truth_child_frame': truth[4], 'estimate_child_frame': estimate[4],
        'samples': len(es), 'unmatched_estimates': int(len(et)-len(es)),
        'start_stamp_sec': float(es[0]+origin/1e9), 'duration_sec': float(es[-1]-es[0]),
        'position_rmse_m': float(np.sqrt(np.mean(distance_error**2))),
        'position_p95_m': float(np.percentile(distance_error,95)),
        'position_max_m': float(distance_error.max()), 'final_position_error_m': float(distance_error[-1]),
        'orientation_rmse_deg': float(np.sqrt(np.mean(np.array(angles)**2))),
        'truth_path_length_m': truth_length, 'estimated_path_length_m': estimate_length,
        'path_length_ratio': estimate_length/truth_length if truth_length > 1e-6 else None,
        'truth_mean_speed_mps': float(gt_speed.mean()) if len(gt_speed) else None,
        'estimated_mean_speed_mps': float(est_speed.mean()) if len(est_speed) else None,
        'speed_rmse_mps': float(np.sqrt(np.mean((est_speed-gt_speed)**2))) if len(gt_speed) else None,
        'largest_estimate_gap_sec': float(dt.max()),
        'status_counts_whole_estimate_bag': counts,
        'limitations': 'Partial overlap only; initial estimated pose may already contain drift. Path length is sampling/noise dependent. No truth TF or recorded clock is republished.'}
    rows = np.column_stack([es-es[0], positions, aligned, distance_error, angles])
    return summary, rows


def save(output, summary, rows):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    with (output/'samples.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['elapsed_sec','truth_x','truth_y','truth_z','estimate_x','estimate_y','estimate_z','position_error_m','orientation_error_deg'])
        writer.writerows(rows)
    points = np.vstack([rows[:,1:3], rows[:,4:6]])
    lower, upper = points.min(axis=0), points.max(axis=0)
    scale = 640/max(float((upper-lower).max()), .1)
    def path(xy):
        pixels = (xy-lower)*scale
        return ' '.join(f'{x+60:.2f},{740-y:.2f}' for x,y in pixels)
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="820" height="820" viewBox="0 0 820 820">
<rect width="820" height="820" fill="white"/>
<text x="30" y="30" font-family="sans-serif">XY trajectory, initial SE(3) alignment; no scale fit</text>
<text x="30" y="55" fill="#167a37" font-family="sans-serif">Truth</text>
<text x="110" y="55" fill="#bb2929" font-family="sans-serif">VSLAM</text>
<polyline points="{path(rows[:,1:3])}" fill="none" stroke="#167a37" stroke-width="2"/>
<polyline points="{path(rows[:,4:6])}" fill="none" stroke="#bb2929" stroke-width="2"/>
<text x="30" y="800" font-family="sans-serif">Equal XY scale; overlap {summary['duration_sec']:.2f}s; position RMSE {summary['position_rmse_m']:.3f} m</text>
</svg>'''
    (output/'trajectory.svg').write_text(svg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--truth-bag', required=True)
    parser.add_argument('--estimate-bag', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    truth = read_bag(args.truth_bag, '/sim/odometry')
    estimate = read_bag(args.estimate_bag, '/visual_slam/tracking/odometry')
    if truth[4] != estimate[4]:
        raise ValueError(f'Child frames differ: {truth[4]} vs {estimate[4]}; transform to the same vehicle origin first')
    summary, rows = compare(truth, estimate)
    save(args.output, summary, rows)
    print(json.dumps(summary, indent=2, allow_nan=False))
    print(f'Report: {Path(args.output).resolve()}')


if __name__ == '__main__':
    main()
