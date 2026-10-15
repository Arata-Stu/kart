"""Read-only rosbag2 adapter. Keep bag and header time; never decode image pixels."""
from pathlib import Path

TYPES = {
    'odom': 'nav_msgs/msg/Odometry', 'command': 'kart_interfaces/msg/ControlCommand',
    'mode': 'kart_interfaces/msg/OperationModeState',
    'vslam': 'diagnostic_msgs/msg/DiagnosticArray', 'bridge': 'diagnostic_msgs/msg/DiagnosticArray',
    'path': 'std_msgs/msg/UInt8', 'imu': 'sensor_msgs/msg/Imu',
    'trim': 'kart_interfaces/msg/ControlTrim',
    'image': 'sensor_msgs/msg/Image', 'speed_diagnostics': 'diagnostic_msgs/msg/DiagnosticArray',
}
REQUIRED = ('odom', 'command', 'mode', 'vslam', 'bridge', 'path')


def stamp(msg):
    h = getattr(msg, 'header', None)
    return None if h is None else int(h.stamp.sec) * 10**9 + int(h.stamp.nanosec)


def xyz(v):
    return [float(v.x), float(v.y), float(v.z)]


def read_bag(path, cfg):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    store = get_typestore(Stores.LATEST)
    root = Path(__file__).resolve().parents[2]
    for name in ('ControlCommand', 'OperationModeState', 'ControlTrim'):
        source = root / 'ros2_ws/src/kart_interfaces/msg' / (name + '.msg')
        store.register(get_types_from_msg(source.read_text(), 'kart_interfaces/msg/' + name))
    data = {key: [] for key in TYPES}
    data['tf_static'] = []
    with AnyReader([Path(path).expanduser().resolve()], default_typestore=store) as reader:
        inventory = {c.topic: {'type': c.msgtype, 'count': c.msgcount} for c in reader.connections}
        # Multiple connections on one topic can occur after publisher restart.
        for name in inventory:
            inventory[name]['count'] = sum(c.msgcount for c in reader.connections if c.topic == name)
        problems = []
        for key, expected in TYPES.items():
            conns = [c for c in reader.connections if c.topic == cfg['topics'][key]]
            if any(c.msgtype != expected for c in conns):
                raise ValueError(f'{key}: topic has unexpected type; expected {expected}')
            if key in REQUIRED and not sum(c.msgcount for c in conns):
                problems.append(cfg['topics'][key])
        reverse = {v: k for k, v in cfg['topics'].items()}
        selected = [c for c in reader.connections if c.topic in reverse or c.topic == '/tf_static']
        # Metadata suffices for image availability; keep the large pixel payload out of memory.
        selected = [c for c in selected if c.topic != cfg['topics']['image']]
        for connection, receive, raw in reader.messages(connections=selected):
            msg = reader.deserialize(raw, connection.msgtype)
            if connection.topic == '/tf_static':
                for tf in msg.transforms:
                    q = tf.transform.rotation
                    data['tf_static'].append({'parent': tf.header.frame_id,
                        'child': tf.child_frame_id, 'xyz': xyz(tf.transform.translation),
                        'q': [float(q.x), float(q.y), float(q.z), float(q.w)]})
                continue
            key = reverse[connection.topic]
            row = {'bag_ns': int(receive), 'header_ns': stamp(msg)}
            if key == 'odom':
                row.update(frame=msg.header.frame_id, child=msg.child_frame_id,
                           linear=xyz(msg.twist.twist.linear), angular=xyz(msg.twist.twist.angular))
            elif key == 'command':
                row.update({k: float(getattr(msg, k)) for k in ('steering', 'throttle', 'brake', 'reverse')})
            elif key == 'mode':
                row['mode'] = int(msg.mode)
            elif key == 'trim':
                row.update(steering=float(msg.steering), throttle=float(msg.throttle))
            elif key == 'path':
                row['value'] = int(msg.data)
            elif key == 'imu':
                row.update(frame=msg.header.frame_id, linear=xyz(msg.linear_acceleration),
                           angular=xyz(msg.angular_velocity))
            else:
                identities = {'vslam': 'visual_slam', 'bridge': 'JPBB-01', 'speed_diagnostics': 'speed_controller'}
                matches = [s for s in msg.status if s.hardware_id == identities[key]]
                if not matches:
                    continue
                if len(matches) != 1:
                    raise ValueError(f'Multiple {identities[key]} statuses in one diagnostic message')
                status = matches[0]
                row.update(level=int(status.level), message=status.message,
                           values={v.key: v.value for v in status.values})
            data[key].append(row)
    data['inventory'] = inventory
    data['missing_required'] = problems
    data['source'] = str(Path(path).resolve())
    return data
