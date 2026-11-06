"""Conservative EVT3 wrap-origin alignment; never fit an arbitrary time offset."""
import math
from pathlib import Path

WRAP_US = 1 << 24
TOLERANCE_US = 1_000_000


def infer_alignment(timings, raw_first, raw_last, recording_start_ns=None):
    if not timings or len({r['stream_id'] for r in timings}) != 1:
        raise ValueError('RAW自動補正: TensorTimingは単一streamが必要です')
    base = timings[0]['sensor_to_ros_offset_ns']
    offsets, ends, latests, residuals = {}, [], [], []
    previous = None
    for row in timings:
        epoch = row['sensor_epoch']
        # delta offset compensates a decoder reset without changing ROS stamps.
        delta_us = (row['sensor_to_ros_offset_ns'] - base) / 1000
        turns = round(delta_us / WRAP_US)
        residual = delta_us - turns * WRAP_US
        if abs(residual) > TOLERANCE_US:
            raise ValueError('RAW自動補正: ROS/センサーoffsetが整数周回差に一致しません')
        relative = turns * WRAP_US
        if epoch in offsets and offsets[epoch] != relative:
            raise ValueError('RAW自動補正: 同じepoch内で周回対応が変化しています')
        offsets[epoch] = relative
        end = row['window_end_sensor_us'] + relative
        latest = row['latest_event_sensor_us'] + relative
        if previous is not None:
            if (row['sequence'] <= previous['sequence'] or epoch < previous['sensor_epoch']
                    or row['tensor_stamp_ros_ns'] <= previous['tensor_stamp_ros_ns']
                    or end <= ends[-1] or latest < latests[-1] - 5000):
                raise ValueError('RAW自動補正: 補正後のtimingが逆行・重複しています')
        ends.append(end); latests.append(latest); residuals.append(residual)
        previous = row
    # Both recording edges must identify the SAME integer wrap. A mere overlap
    # is insufficient, especially for recordings longer than one wrap.
    first, last = latests[0], latests[-1]
    low = max(math.ceil((raw_first-first-TOLERANCE_US)/WRAP_US),
              math.ceil((raw_last-last-TOLERANCE_US)/WRAP_US))
    high = min(math.floor((raw_first-first+TOLERANCE_US)/WRAP_US),
               math.floor((raw_last-last+TOLERANCE_US)/WRAP_US))
    if low != high:
        raise ValueError(f'RAW自動補正: 開始・終了で共通する周回差を一意に決定できません。'
                         f'RAW=[{raw_first},{raw_last}], timing=[{first},{last}], candidates=[{low},{high}]')
    if recording_start_ns is not None:
        first_ros = timings[0]['latest_event_sensor_us']*1000 + base
        if abs(first_ros-recording_start_ns) > 2*TOLERANCE_US*1000:
            raise ValueError('RAW自動補正: RAW録画開始metadataとbagの開始時刻が一致しません')
    shift = low * WRAP_US
    return {str(e): offset+shift for e, offset in offsets.items()}, dict(
        method='evt3_integer_wrap_recording_edges_v1', wrap_us=WRAP_US,
        edge_tolerance_us=TOLERANCE_US, raw_first_us=raw_first, raw_last_us=raw_last,
        first_edge_residual_us=first+shift-raw_first,
        last_edge_residual_us=last+shift-raw_last,
        max_offset_residual_us=max(abs(x) for x in residuals),
        recording_start_ros_ns=recording_start_ns,
        limitation='Receipt-derived offset and recording edges; not event identity or hardware sync proof')


def scan_raw(path, max_reorder_us, factory):
    import numpy as np
    reader = factory(str(path), do_time_shifting=False)
    first = high = None
    count = 0
    try:
        while not reader.is_done():
            events = reader.load_n_events(1000000)
            if not len(events):
                if reader.is_done():
                    break
                raise ValueError('RAW自動補正: reader made no progress')
            t = events['t'].astype(np.int64)
            if np.any(t < 0):
                raise ValueError('RAW自動補正: negative RAW timestamp')
            running = np.maximum.accumulate(t)
            if high is not None:
                running = np.maximum(running, high)
            if np.any(running-t > max_reorder_us):
                raise ValueError('RAW自動補正: RAW自体の時刻逆行が許容範囲を超えています')
            first = int(t.min()) if first is None else min(first, int(t.min()))
            high = int(running[-1]); count += len(t)
    finally:
        if hasattr(reader, 'close'):
            reader.close()
    if first is None:
        raise ValueError('RAW自動補正: RAWにイベントがありません')
    return first, high, count


def auto_align(path, timings, max_reorder_us=5000, factory=None):
    import numpy as np
    # Only EVT3 has the wrap period assumed by this algorithm.
    with Path(path).open('rb') as source:
        header = source.read(65536).split(b'% end', 1)[0]
    if b'EVT3' not in header and b'% evt 3.0' not in header:
        raise ValueError('RAW自動補正: EVT3ヘッダーを確認できません。manifestを指定してください')
    if factory is None:
        if int(np.__version__.split('.')[0]) >= 2:
            raise ValueError('RAW自動補正にはNumPy 1.26のEVS RAW抽出用Pythonを使用してください')
        from metavision_core.event_io import RawReader
        factory = RawReader
    start = None
    sidecar = Path(str(path)+'.metadata.yaml')
    if sidecar.exists():
        import yaml
        metadata = yaml.safe_load(sidecar.read_text())
        start = metadata.get('recording_start_ros_time_nanoseconds')
        if type(start) is not int or start <= 0:
            raise ValueError('RAW自動補正: recording_start_ros_time_nanosecondsが不正です')
        if Path(metadata.get('raw_file', '')).name != Path(path).name:
            raise ValueError('RAW自動補正: sidecarのRAW名が一致しません')
    print(f"RAW自動補正: 時刻範囲を走査中 {path}", flush=True)
    first, last, count = scan_raw(path, max_reorder_us, factory)
    offsets, evidence = infer_alignment(timings, first, last, start)
    evidence['events_scanned'] = count
    return dict(path=str(Path(path).resolve()), stream_id=timings[0]['stream_id'],
                sensor_epoch=timings[0]['sensor_epoch'], sensor_epochs=sorted(int(e) for e in offsets),
                epoch_offsets_us=offsets, auto_alignment=evidence)
