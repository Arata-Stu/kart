"""Bag-wide recording intent and causal per-sample human teacher selection."""

from bisect import bisect_right
from collections import Counter
import json
import math

from .data import previous, valid_label

AUTO_SCHEMA = 'kart.auto_fixed_throttle.v1'
LEGACY_SCHEMA = 'kart.manual_fixed_throttle.v1'
CONFIG_TIMEOUT_NS = 300_000_000


def parse_config(data):
    """Reject ambiguous metadata rather than silently treating recovery as teaching."""
    row = json.loads(data)
    if not isinstance(row, dict) or row.get('schema') not in (AUTO_SCHEMA, LEGACY_SCHEMA):
        raise ValueError('Unsupported fixed_config schema; cannot determine recording intent')
    if any(type(row.get(k)) is not bool for k in ('enabled', 'armed')):
        raise ValueError('fixed_config requires boolean enabled/armed')
    for key in ('throttle', 'speed_scale', 'entry_neutral_s'):
        if type(row.get(key)) not in (int, float) or not math.isfinite(row[key]):
            raise ValueError(f'Invalid fixed_config {key}')
    if (type(row.get('mode')) is not int or row['mode'] not in (1, 2, 3, 4)
            or not 0 < row['speed_scale'] <= 1 or not 0 <= row['entry_neutral_s'] <= 60
            or (row['enabled'] and not 0 <= row['throttle'] <= row['speed_scale'])
            or (not row['enabled'] and row['throttle'] != -1)):
        raise ValueError('Invalid fixed_config range or mode')
    return row


def read_fixed_configs(bag, topic):
    """String has no header: use rosbag receipt time in the ROS clock domain."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        return []
    if types[topic] != 'std_msgs/msg/String':
        raise ValueError(f'Wrong fixed_config type: {topic}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    records = []
    while reader.has_next():
        _, raw, stamp = reader.read_next()
        if stamp <= 0:
            raise ValueError('Missing fixed_config receipt timestamp')
        msg = deserialize_message(raw, get_message(types[topic]))
        records.append((stamp, parse_config(msg.data)))
    return sorted(records, key=lambda row: row[0])


class TeacherSelection:
    def __init__(self, configs=(), modes=()):
        self.configs = sorted(configs, key=lambda row: row[0])
        self.fixed_auto = any(row['schema'] == AUTO_SCHEMA and row['enabled']
                              for _, row in self.configs)
        self.modes = sorted(modes, key=lambda row: row[0])
        self.auto_starts = []
        start, last = None, None
        for stamp, mode in self.modes:
            if mode.mode != 1:
                start = None
            elif start is None or last is None or stamp-last > CONFIG_TIMEOUT_NS:
                start = stamp
            self.auto_starts.append((stamp, start))
            last = stamp
        self.accepted = Counter()
        self.rejected = Counter()

    def select(self, command, mode, stamp, command_stamp=None):
        reason = None
        if command is None or mode is None or not valid_label(command):
            reason = 'invalid_teacher'
        elif not self.fixed_auto:
            if mode.mode != 2:
                reason = 'not_manual'
        elif mode.mode != 1:
            reason = 'not_fixed_auto'  # MANUAL recovery/adjustment is never teaching.
        else:
            config = previous(self.configs, stamp, CONFIG_TIMEOUT_NS)
            if (config is None or config['schema'] != AUTO_SCHEMA or not config['enabled']
                    or not config['armed'] or config['mode'] != 1):
                reason = 'fixed_config_unavailable_or_inactive'
            elif not math.isclose(command.throttle, config['throttle'], rel_tol=0, abs_tol=1e-6):
                reason = 'fixed_throttle_mismatch'
            else:
                index = bisect_right(self.auto_starts, stamp, key=lambda row: row[0])-1
                start = self.auto_starts[index][1] if index >= 0 else None
                if start is None or stamp-start < config['entry_neutral_s']*1e9:
                    reason = 'auto_entry_neutral'
                elif command_stamp is not None and command_stamp < start:
                    reason = 'command_before_auto_entry'
        if reason:
            self.rejected[reason] += 1
            return None
        source = 'fixed_auto' if self.fixed_auto else 'manual'
        self.accepted[source] += 1
        return source

    def metadata(self):
        return dict(policy='fixed_auto_only' if self.fixed_auto else 'manual_only',
                    fixed_config_records=len(self.configs),
                    fixed_config_timeout_ms=CONFIG_TIMEOUT_NS/1e6,
                    fixed_config_clock='bag_receipt', accepted=dict(self.accepted),
                    rejected=dict(self.rejected))
