"""Extract small metadata; image/tensor payloads are never deserialized or retained."""
from pathlib import Path
import json
import struct
import math

COMMAND = 'kart_interfaces/msg/ControlCommand'
SENSORS = {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
           'sensor_msgs/msg/CameraInfo', 'sensor_msgs/msg/Imu', 'nav_msgs/msg/Odometry'}
HEAVY = {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
         'isaac_ros_tensor_msgs/msg/TensorList'}


def header_stamp(raw):
    """Header-first plain CDR only. Encapsulation kind is big endian, body may be either."""
    if len(raw) < 12 or bytes(raw[:2]) not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('Unsupported CDR representation for header-only extraction')
    sec, nano = struct.unpack_from('<iI' if raw[1] == 1 else '>iI', raw, 4)
    if not 0 <= nano < 1_000_000_000:
        raise ValueError('Invalid timestamp nanoseconds')
    return sec * 1_000_000_000 + nano


def stamp(msg):
    value = getattr(getattr(msg, 'header', None), 'stamp', getattr(msg, 'stamp', None))
    return None if value is None else int(value.sec) * 10**9 + int(value.nanosec)


def extract(msg, kind):
    if kind == COMMAND:
        return {key: float(getattr(msg, key)) for key in ('steering', 'throttle', 'brake', 'reverse')}
    if kind == 'kart_interfaces/msg/OperationModeState':
        return {'mode': int(msg.mode), 'source': msg.source}
    if kind == 'diagnostic_msgs/msg/DiagnosticArray':
        return {'status': [dict(name=s.name, level=int(s.level), message=s.message,
                                values={v.key: v.value for v in s.values}) for s in msg.status]}
    if kind == 'rcl_interfaces/msg/Log':
        return dict(level=int(msg.level), name=msg.name, message=msg.msg)
    if hasattr(msg, 'data'):
        value = msg.data
        if hasattr(value, 'tolist'):
            value = value.tolist()
        if isinstance(value, str) and value.startswith('{'):
            try:
                return {'data': value, 'json': json.loads(value)}
            except (ValueError, TypeError):
                pass
        return {'data': value}
    if kind == 'sensor_msgs/msg/Imu':
        return {f'gyro_{axis}': float(getattr(msg.angular_velocity, axis)) for axis in 'xyz'}
    if kind == 'nav_msgs/msg/Odometry':
        v = msg.twist.twist
        return dict(speed_x=float(v.linear.x), yaw_rate=float(v.angular.z))
    return {}


def read_bag(path, config):
    from rosbags.highlevel import AnyReader, AnyReaderError
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    path = Path(path).expanduser().resolve()
    store = get_typestore(Stores.LATEST)
    for source in (Path(__file__).resolve().parents[2] / 'ros2_ws/src/kart_interfaces/msg').glob('*.msg'):
        store.register(get_types_from_msg(source.read_text(), 'kart_interfaces/msg/' + source.stem))
    wanted = set(config['topics'].values()) | set(config['sync_topics'])
    expected_roles = {**{k: {COMMAND} for k in ('decoder', 'auto', 'final')},
                      'mode': {'kart_interfaces/msg/OperationModeState'},
                      'e2e_status': {'std_msgs/msg/String'}, 'recovery_status': {'std_msgs/msg/String'},
                      'recovery_active': {'std_msgs/msg/Bool'}, 'active_path': {'std_msgs/msg/UInt8'},
                      'output_channels': {'std_msgs/msg/Int32MultiArray'},
                      'evs_timing': {'std_msgs/msg/String'}, 'logs': {'rcl_interfaces/msg/Log'},
                      'tensor_input': {'isaac_ros_tensor_msgs/msg/TensorList'},
                      'tensor_output': {'isaac_ros_tensor_msgs/msg/TensorList'}}
    rows, inventory = [], {}
    with AnyReader([path], default_typestore=store) as reader:
        if not reader.is2:
            raise ValueError('Only rosbag2 recordings are supported')
        for c in reader.connections:
            entry = inventory.setdefault(c.topic, dict(types=[], count=0, connections=0))
            entry['count'] += c.msgcount
            entry['connections'] += 1
            if c.msgtype not in entry['types']:
                entry['types'].append(c.msgtype)
        connections = [c for c in reader.connections if c.topic in wanted or c.msgtype in SENSORS
                       or c.msgtype == 'diagnostic_msgs/msg/DiagnosticArray']
        for connection, received, raw in reader.messages(connections=connections):
            row = dict(topic=connection.topic, type=connection.msgtype, bag_ns=int(received),
                       stamp_ns=None, connection=connection.id, payload={}, error=None)
            try:
                for role, types in expected_roles.items():
                    if connection.topic == config['topics'][role] and connection.msgtype not in types:
                        raise ValueError(f'{role}: unexpected type {connection.msgtype}; expected {sorted(types)}')
                if connection.msgtype in HEAVY:
                    row['stamp_ns'] = header_stamp(raw)
                else:
                    msg = reader.deserialize(raw, connection.msgtype)
                    row['stamp_ns'], row['payload'] = stamp(msg), extract(msg, connection.msgtype)
                if connection.topic == config['topics']['evs_timing']:
                    timing = row['payload'].get('json', {})
                    if timing.get('schema') != 'openeb.tensor_timing.v1':
                        raise ValueError('Unsupported EVS timing schema')
                    row['stamp_ns'] = int(timing['tensor_stamp_ros_ns'])
                    row['payload']['evs_window_ms'] = ((int(timing['window_end_sensor_us']) - int(timing['window_start_sensor_us'])) / 1000
                                                       if 'window_end_sensor_us' in timing and 'window_start_sensor_us' in timing else None)
            except (ValueError, KeyError, TypeError, AttributeError, struct.error, AnyReaderError) as error:
                row['error'] = str(error)
            def finite(value):
                if isinstance(value, dict):
                    return {k: finite(v) for k, v in value.items()}
                if isinstance(value, list):
                    return [finite(v) for v in value]
                if isinstance(value, float) and not math.isfinite(value):
                    row.setdefault('warnings', []).append('Nonfinite numeric payload; shown as missing')
                    return None
                return value
            row['payload'] = finite(row['payload'])
            rows.append(row)
    if not rows:
        raise ValueError('No supported selected messages; inspect bag topics and --config')
    return dict(schema=1, bag=str(path), inventory=inventory, records=rows)
