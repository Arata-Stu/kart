"""Read official odom TUM poses; derive rear-axle motion without ROS replay."""
import copy
from decimal import Decimal, InvalidOperation
import hashlib
from pathlib import Path
import numpy as np
from analysis import rotation, static_transform


def attach_poses(data, cfg, path, pose_frame):
    path = Path(path).resolve()
    if path.name != 'odom_poses.tum':
        raise ValueError('Use official odom_poses.tum, not optimized/SLAM keyframes')
    if not pose_frame:
        raise ValueError('--pose-frame is required: confirm the official output body frame')
    # T_world_vehicle = T_world_sensor * T_sensor_vehicle.
    extrinsic = static_transform(data.get('tf_static', []), pose_frame, cfg['vehicle_frame'])
    stamps, positions, rotations = [], [], []
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError('TUM requires timestamp tx ty tz qx qy qz qw')
        try:
            ns = int(Decimal(fields[0]) * 10**9)
        except (InvalidOperation, OverflowError, ValueError) as exc:
            raise ValueError('Invalid TUM timestamp') from exc
        values = np.asarray(fields[1:], float)
        if ns <= 0 or not np.isfinite(values).all() or (stamps and ns <= stamps[-1]):
            raise ValueError('Invalid/non-monotonic TUM timestamp or pose')
        pose = np.eye(4)
        pose[:3, :3], pose[:3, 3] = rotation(values[3:]), values[:3]
        vehicle = pose @ extrinsic
        stamps.append(ns); positions.append(vehicle[:3, 3]); rotations.append(vehicle[:3, :3])
    if len(stamps) < 5:
        raise ValueError('At least five poses required')
    commands = data.get('command', [])
    times_cmd = [r['header_ns'] for r in commands if r.get('header_ns')]
    if not times_cmd or min(stamps[-1], max(times_cmd)) <= max(stamps[0], min(times_cmd)):
        raise ValueError('Pose/command timestamps do not overlap; no automatic time shifting')
    t = (np.asarray(stamps, dtype=np.int64)-stamps[0])/1e9
    pos = np.asarray(positions)
    yaw = np.unwrap([np.arctan2(r[1, 0], r[0, 0]) for r in rotations])
    result = copy.deepcopy(data)
    result['odom'] = []
    half = cfg['smoothing_window_s']/2
    for i, ns in enumerate(stamps):
        reason = '' if rotations[i][2,2] >= np.cos(.2) else 'pose_nonplanar'
        ids = np.arange(np.searchsorted(t, t[i]-half-1e-9),
                        np.searchsorted(t, t[i]+half+1e-9, side='right'))
        if len(ids) < 3 or t[i]-t[ids[0]] < half*.8 or t[ids[-1]]-t[i] < half*.8:
            reason = 'pose_window_edge'
        if (len(ids)>1 and np.max(np.diff(t[ids])) > cfg['odom_max_gap_s']) or (i>0 and t[i]-t[i-1]>cfg['odom_max_gap_s']) or (i+1<len(t) and t[i+1]-t[i]>cfg['odom_max_gap_s']):
            reason = 'pose_gap'
        # Reject windows with implausible displacement or orientation jumps.
        if len(ids) > 1:
            dt = np.diff(t[ids])
            speed = np.linalg.norm(np.diff(pos[ids], axis=0), axis=1)/dt
            turn = abs(np.diff(yaw[ids]))/dt
            if np.any(speed > cfg['max_speed_mps']) or np.any(turn > cfg['max_yaw_rate_rps']):
                reason = 'pose_jump'
        linear, wz = np.zeros(3), 0.
        if not reason:
            design = np.column_stack((t[ids]-t[i], np.ones(len(ids))))
            world_velocity = np.linalg.lstsq(design, pos[ids], rcond=None)[0][0]
            linear = rotations[i].T @ world_velocity
            wz = float(np.linalg.lstsq(design, yaw[ids], rcond=None)[0][0])
        result['odom'].append(dict(bag_ns=ns, header_ns=ns, frame='offline_world',
            child=cfg['vehicle_frame'], linear=linear.tolist(), angular=[0., 0., wz],
            pose_reason=reason))
    result['offline_pose'] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        pose_frame=pose_frame, vehicle_frame=cfg['vehicle_frame'], sensor_from_vehicle=extrinsic.tolist(),
        quality='Geometric checks only; online tracking diagnostics unavailable',
        smoothing_window_s=cfg['smoothing_window_s'])
    result['missing_required'] = [cfg['topics'][k] for k in ('command','mode','bridge','path') if not result.get(k)]
    return result
