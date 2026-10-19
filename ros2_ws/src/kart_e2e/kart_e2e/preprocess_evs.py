"""RAW + recorded TensorTiming + automatically selected human commands -> raw-count policy dataset.

RAW is decoded with time shifting DISABLED (sensor microseconds). No guessed
record-start offset, interpolation across epochs, rectification or RGB input.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np

from .contract import evs_spec
from .data import previous, previous_record
from .teacher_selection import TeacherSelection, read_fixed_configs


def histogram(events, start_us, end_us, sensor_shape, content_shape, latest_us=None):
    """OpenEB none histogram: [start,end), positive bins then negative bins."""
    sh, sw = sensor_shape
    h, w = content_shape
    if end_us - start_us != 40000 or sh % h or sw % w or sh // h != sw // w:
        raise ValueError('Expected 40ms window and integer sum pooling')
    if events.dtype.names is None or not {'x', 'y', 'p', 't'} <= set(events.dtype.names):
        raise ValueError('Expected Metavision x/y/p/t structured events')
    if len(events) and (np.any(np.diff(events['t'].astype(np.int64)) < 0)
                        or np.any(events['x'] < 0) or np.any(events['x'] >= sw)
                        or np.any(events['y'] < 0) or np.any(events['y'] >= sh)
                        or not np.isin(events['p'], [0, 1]).all()):
        raise ValueError('Invalid event coordinates, polarity or timestamp reset')
    mask = (events['t'] >= start_us) & (events['t'] < end_us)
    if latest_us is not None:
        mask &= events['t'] <= latest_us
    selected = events[mask]
    bins = (selected['t'].astype(np.int64) - start_us) // 4000
    channels = np.where(selected['p'] != 0, 0, 10) + bins
    x = selected['x'].astype(np.int64) * w // sw
    y = selected['y'].astype(np.int64) * h // sh
    indexes = (channels * h + y) * w + x
    return np.bincount(indexes, minlength=20*h*w).reshape(20, h, w).astype(np.float32)


def validate_timing(row):
    required = ('sequence', 'sensor_epoch', 'tensor_stamp_ros_ns',
                'window_start_sensor_us', 'window_end_sensor_us',
                'sensor_to_ros_offset_ns', 'latest_event_sensor_us',
                'window_start_ros_ns', 'window_end_ros_ns')
    if (row.get('schema') != 'openeb.tensor_timing.v1'
            or not isinstance(row.get('stream_id'), str) or not row['stream_id']
            or any(type(row.get(k)) is not int for k in required)
            or row['sequence'] < 0 or row['sensor_epoch'] < 0
            or row['tensor_stamp_ros_ns'] <= 0
            or row['window_start_sensor_us'] < 0
            or row['window_end_sensor_us'] - row['window_start_sensor_us'] != 40000):
        raise ValueError('Malformed TensorTiming or unsupported window')
    for which in ('start', 'end'):
        if row[f'window_{which}_ros_ns'] != (row[f'window_{which}_sensor_us'] * 1000
                                             + row['sensor_to_ros_offset_ns']):
            raise ValueError('Inconsistent sensor-to-ROS mapping')
    if row.get('shape', [0, 0])[0:2] != [1, 20]:
        raise ValueError('Expected 20-channel TensorTiming')
    return row


def bag_records(bag, command_topic, mode_topic, timing_topic, clock):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    expected = {command_topic: 'kart_interfaces/msg/ControlCommand',
                mode_topic: 'kart_interfaces/msg/OperationModeState',
                timing_topic: 'std_msgs/msg/String'}
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if any(types.get(k) != v for k, v in expected.items()):
        raise ValueError(f'Missing/wrong EVS learning topics; expected {expected}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(expected)))
    commands, modes, timings = [], [], []
    while reader.has_next():
        topic, raw, received = reader.read_next()
        msg = deserialize_message(raw, get_message(types[topic]))
        if topic == timing_topic:
            timings.append(validate_timing(json.loads(msg.data)))
        else:
            stamp = received if clock == 'bag' else (msg.header.stamp.sec * 10**9
                                                     + msg.header.stamp.nanosec)
            if stamp <= 0:
                raise ValueError('Missing command/mode timestamp')
            (commands if topic == command_topic else modes).append((stamp, msg))
    commands.sort(key=lambda r: r[0])
    modes.sort(key=lambda r: r[0])
    return commands, modes, timings


def discover_raw(bag, timings):
    """Use only an unambiguous closed RAW beside the bag metadata."""
    bag = Path(bag).resolve(strict=True)
    candidates = sorted(p for p in bag.iterdir() if p.suffix.lower() == '.raw')
    if len(candidates) != 1:
        raise ValueError(f'RAW自動検出: {bag} にRAWが{len(candidates)}件あります。'
                         'RAWが1件の場合のみ自動対応します。複数の場合はRAW manifestを指定してください')
    raw = candidates[0]
    if raw.is_symlink() or not raw.is_file() or raw.stat().st_size == 0:
        raise ValueError('RAW自動検出: 空ファイル・シンボリックリンクは使用できません')
    identities = {(r['stream_id'], r['sensor_epoch']) for r in timings}
    if len(identities) != 1:
        raise ValueError('RAW自動検出: bagのstream/epochが一意ではありません。RAW manifestを指定してください')
    stream_id, epoch = next(iter(identities))
    return [dict(path=str(raw), stream_id=stream_id, sensor_epoch=epoch)]


def load_manifest(path):
    data = json.loads(Path(path).read_text())
    if data.get('schema') != 'kart.evs_raw_manifest.v1' or not data.get('files'):
        raise ValueError('Expected kart.evs_raw_manifest.v1 with files')
    files = []
    for row in data['files']:
        if (not isinstance(row.get('stream_id'), str) or not row['stream_id']
                or type(row.get('sensor_epoch')) is not int or row['sensor_epoch'] < 0):
            raise ValueError('Each RAW requires explicit stream_id and sensor_epoch')
        raw = Path(row['path']).expanduser()
        raw = raw if raw.is_absolute() else Path(path).resolve().parent / raw
        raw = raw.resolve(strict=True)
        if raw.suffix.lower() != '.raw' or not raw.is_file():
            raise ValueError('Expected a closed RAW file')
        files.append(dict(path=str(raw), stream_id=row['stream_id'],
                          sensor_epoch=row['sensor_epoch']))
    if len({r['path'] for r in files}) != len(files):
        raise ValueError('RAW file listed more than once')
    return files


class RawWindows:
    """Sequential, bounded-window reader. Never seek backwards across resets."""
    def __init__(self, path, factory=None):
        if factory is None:
            from metavision_core.event_io import RawReader
            factory = RawReader
        self.reader = factory(str(path), do_time_shifting=False)
        self.shape = tuple(self.reader.get_size())
        self.buffer = None
        self.first = self.last = None
        self.requested = -1

    def window(self, start, end):
        if start < self.requested:
            raise ValueError('RAW windows moved backwards; split sensor epochs')
        self.requested = start
        while self.last is None or self.last < end:
            if self.reader.is_done():
                break
            chunk = self.reader.load_n_events(100000)
            if not len(chunk):
                if self.reader.is_done():
                    break
                raise ValueError('RAW reader made no progress')
            if (np.any(np.diff(chunk['t'].astype(np.int64)) < 0)
                    or (self.last is not None and int(chunk['t'][0]) < self.last)):
                raise ValueError('Timestamp reset inside RAW; separate epochs before extraction')
            if self.first is None:
                self.first = int(chunk['t'][0])
            self.last = int(chunk['t'][-1])
            if self.buffer is not None:
                self.buffer = self.buffer[self.buffer['t'] >= start]
                self.buffer = np.concatenate((self.buffer, chunk))
            else:
                self.buffer = chunk.copy()
        # Conservative coverage: never zero-fill partial windows at RAW start/end.
        if self.first is None or self.first > start or self.last < end:
            return None
        self.buffer = self.buffer[self.buffer['t'] >= start]
        return self.buffer[self.buffer['t'] < end]


def extract(files, commands, modes, timings, meta, output, *, sample_stride=10,
            max_skew_ms=100, reader_factory=None, provenance=None, source_guard=None,
            fixed_configs=()):
    selection = TeacherSelection(fixed_configs, modes)
    spec = evs_spec(meta)
    if sample_stride < 1 or not 0 < max_skew_ms <= 1000:
        raise ValueError('Invalid sample stride/skew')
    groups, last_sequence, last_window = {}, {}, {}
    missing = 0
    for raw_row in timings:
        row = validate_timing(raw_row)
        stream, epoch = row['stream_id'], row['sensor_epoch']
        seq, key = row['sequence'], (stream, epoch)
        if stream in last_sequence:
            previous_seq, previous_epoch = last_sequence[stream]
            if seq <= previous_seq or epoch < previous_epoch:
                raise ValueError('Duplicate/reversed timing sequence or epoch')
            missing += seq - previous_seq - 1
        if key in last_window and row['window_end_sensor_us'] <= last_window[key]:
            raise ValueError('Non-increasing sensor windows inside epoch')
        last_sequence[stream] = (seq, epoch)
        last_window[key] = row['window_end_sensor_us']
        groups.setdefault(key, []).append(row)
    if not groups:
        raise ValueError('No TensorTiming records')
    for file in files:
        if (file['stream_id'], file['sensor_epoch']) not in groups:
            raise ValueError('RAW stream/epoch absent from bag timing')
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    counters = Counter()
    used = set()
    signatures = {f['path']: (Path(f['path']).stat().st_size, Path(f['path']).stat().st_mtime_ns)
                  for f in files}
    raw_provenance = []
    with tempfile.TemporaryDirectory(prefix='.evs-dataset-', dir=output.parent) as tmp:
        stage = Path(tmp) / 'output'
        (stage / 'counts').mkdir(parents=True)
        with (stage / 'samples.jsonl').open('w') as stream:
            for file in files:
                raw = RawWindows(file['path'], reader_factory)
                if list(raw.shape) != spec['sensor_shape']:
                    raise ValueError('RAW sensor size differs from encoder metadata')
                rows = groups[(file['stream_id'], file['sensor_epoch'])][::sample_stride]
                for row in rows:
                    stamp = row['tensor_stamp_ros_ns']
                    record = previous_record(commands, stamp, int(max_skew_ms * 1e6))
                    command = record[1] if record is not None else None
                    mode = previous(modes, stamp, 300_000_000)
                    source = selection.select(command, mode, stamp, record[0] if record else None)
                    if source is None:
                        counters['invalid_teacher'] += 1
                        continue
                    start, end = row['window_start_sensor_us'], row['window_end_sensor_us']
                    events = raw.window(start, end)
                    if events is None:
                        counters['outside_raw_coverage'] += 1
                        continue
                    key = (row['stream_id'], row['sensor_epoch'], row['sequence'])
                    if key in used:
                        raise ValueError('Overlapping RAW files cover the same training window')
                    used.add(key)
                    counts = histogram(events, start, end, raw.shape,
                                       (spec['height'], spec['width']), row['latest_event_sensor_us'])
                    name = f'counts/{len(used)-1:08d}.npy'
                    np.save(stage / name, counts, allow_pickle=False)
                    sample = dict(counts=name, stamp_ns=stamp, teacher_source=source, steering=command.steering,
                                  throttle=command.throttle, timing=row, raw=file['path'])
                    stream.write(json.dumps(sample) + '\n')
                with Path(file['path']).open('rb') as source:
                    sha = hashlib.file_digest(source, 'sha256').hexdigest()
                raw_provenance.append(dict(file, sha256=sha))
        if not used:
            raise ValueError('No usable human teacher EVS windows; check RAW coverage/timing/teachers')
        for path, signature in signatures.items():
            stat = Path(path).stat()
            if (stat.st_size, stat.st_mtime_ns) != signature:
                raise ValueError('RAW changed during extraction; stop recording first')
        result = dict(schema=1, modality='evs', model_spec=spec, evs_metadata=meta,
                      samples=len(used), skipped=dict(counters), teacher_selection=selection.metadata(), timing_sequence_gaps=missing,
                      sample_stride=sample_stride, max_skew_ms=max_skew_ms,
                      raw_time_shifting=False, raw_files=raw_provenance,
                      label_time='tensor_stamp_ros_ns', extraction=provenance or {},
                      limitation='RAW reconstruction cannot reproduce online dropped/late packets; offset is receipt-derived')
        if source_guard is not None:
            source_guard()
        (stage / 'metadata.json').write_text(json.dumps(result, indent=2))
        stage.rename(output)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path)
    p.add_argument('output', type=Path)
    source = p.add_mutually_exclusive_group()
    source.add_argument('--raw-manifest', type=Path)
    source.add_argument('--raw', type=Path, nargs='+',
                        help='RAW files when the bag contains exactly one stream/epoch')
    p.add_argument('--model-json', type=Path, required=True)
    p.add_argument('--command-topic', default='/teleop/control_cmd')
    p.add_argument('--mode-topic', default='/operation_mode/state')
    p.add_argument('--fixed-config-topic', default='/teleop/fixed_config')
    p.add_argument('--timing-topic', default='/event_camera/tensor_timing')
    p.add_argument('--clock', choices=['header', 'bag'], default='header')
    p.add_argument('--sample-stride', type=int, default=10)
    p.add_argument('--max-skew-ms', type=float, default=100)
    a = p.parse_args()
    if a.sample_stride < 1 or not 0 < a.max_skew_ms <= 1000:
        p.error('sample-stride >= 1 and 0 < max-skew-ms <= 1000 required')
    meta = json.loads(a.model_json.read_text())
    meta = meta.get('evs_metadata', meta)
    evs_spec(meta)
    def bag_signature():
        return sorted((str(p.relative_to(a.bag)), p.stat().st_size, p.stat().st_mtime_ns)
                      for p in a.bag.rglob('*') if p.is_file())
    before = bag_signature()
    if not before:
        p.error('No bag files found')
    def guard():
        if bag_signature() != before:
            raise ValueError('Bag changed during extraction; stop recording first')
    commands, modes, timings = bag_records(a.bag, a.command_topic, a.mode_topic, a.timing_topic, a.clock)
    fixed_configs = read_fixed_configs(a.bag, a.fixed_config_topic)
    print(f"Teacher policy: {TeacherSelection(fixed_configs).metadata()['policy']}")
    if a.raw_manifest:
        files = load_manifest(a.raw_manifest)
    elif not a.raw:
        files = discover_raw(a.bag, timings)
        print(f"RAW auto-detected: {files[0]['path']}")
    else:
        identities = {(r['stream_id'], r['sensor_epoch']) for r in timings}
        if len(identities) != 1:
            p.error(f'RAW stream/epoch mapping required; use --raw-manifest. Bag identities: {sorted(identities)}')
        stream_id, epoch = next(iter(identities))
        files = [dict(path=str(path.resolve(strict=True)), stream_id=stream_id,
                      sensor_epoch=epoch) for path in a.raw]
        if any(Path(f['path']).suffix.lower() != '.raw' for f in files):
            p.error('--raw expects RAW files')
        if len({f['path'] for f in files}) != len(files):
            p.error('RAW file listed more than once')
    provenance = {k: ([str(p.resolve()) for p in v] if isinstance(v, list) else
                      str(v) if isinstance(v, Path) else v) for k, v in vars(a).items()}
    result = extract(files, commands, modes, timings, meta, a.output,
                     sample_stride=a.sample_stride, max_skew_ms=a.max_skew_ms,
                     provenance=provenance, source_guard=guard, fixed_configs=fixed_configs)
    print(f"Exported {result['samples']} EVS samples; skipped {result['skipped']}")


if __name__ == '__main__':
    main()
