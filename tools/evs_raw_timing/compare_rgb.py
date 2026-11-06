#!/usr/bin/env python3
"""Compare extracted EVS samples with RGB headers, without changing timestamps."""
import argparse
import base64
from bisect import bisect_left
import html
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image


def rgb_image(msg):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    if msg.encoding not in channels:
        raise ValueError(f'Unsupported RGB encoding: {msg.encoding}')
    c = channels[msg.encoding]
    pixels = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = pixels[:, :msg.width*c].reshape(msg.height, msg.width, c)
    if c == 1:
        pixels = np.repeat(pixels, 3, axis=2)
    elif msg.encoding.startswith('bgr'):
        pixels = pixels[:, :, [2, 1, 0]]
    else:
        pixels = pixels[:, :, :3]
    return Image.fromarray(pixels)


def evs_image(counts):
    if counts.ndim != 3 or counts.shape[0] != 20:
        raise ValueError('Expected 20-channel raw counts')
    positive, negative = counts[:10].sum(0), counts[10:].sum(0)
    scale = max(float(np.percentile(positive + negative, 99)), 1)
    return Image.fromarray(np.uint8(np.clip(np.stack(
        [positive, np.zeros_like(positive), negative], -1) / scale, 0, 1)*255))


def embed(image):
    image.thumbnail((640, 480))
    buffer = io.BytesIO()
    image.save(buffer, format='PNG')
    return '<img src="data:image/png;base64,' + base64.b64encode(buffer.getvalue()).decode() + '">'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path)
    p.add_argument('dataset', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--rgb-topic', default='/realsense/color/image_raw')
    p.add_argument('--samples', type=int, default=12)
    p.add_argument('--max-delta-ms', type=float, default=100)
    a = p.parse_args()
    if not 1 <= a.samples <= 100 or not 0 < a.max_delta_ms <= 1000:
        p.error('samples: 1..100; max-delta-ms: 0..1000')
    if a.output.exists():
        p.error('Output exists; choose a new file')
    rows = [json.loads(line) for line in (a.dataset/'samples.jsonl').read_text().splitlines() if line.strip()]
    if not rows:
        p.error('Empty dataset')
    rows.sort(key=lambda r: r['stamp_ns'])
    rows = [rows[i] for i in np.unique(np.linspace(0, len(rows)-1, min(len(rows), a.samples), dtype=int))]
    stamps = [r['stamp_ns'] for r in rows]
    best = [None]*len(rows)
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image as RosImage
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(a.bag), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics.get(a.rgb_topic) != 'sensor_msgs/msg/Image':
        p.error(f'RGB Image topic missing: {a.rgb_topic}; record RGB and EVS together')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[a.rgb_topic]))
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, RosImage)
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        if stamp <= 0:
            continue
        lo = bisect_left(stamps, stamp - int(a.max_delta_ms*1e6))
        for i in range(lo, len(stamps)):
            delta = stamp - stamps[i]
            if delta < -a.max_delta_ms*1e6:
                break
            if best[i] is None or abs(delta) < abs(best[i][0]):
                best[i] = (delta, stamp, rgb_image(msg))
    cards = []
    for row, match in zip(rows, best):
        timing = row['timing']
        info = dict(tensor_header_ros_ns=row['stamp_ns'], rgb_header_ros_ns=match[1] if match else None,
                    rgb_minus_tensor_ms=match[0]/1e6 if match else None,
                    window_start_ros_ns=timing['window_start_ros_ns'], window_end_ros_ns=timing['window_end_ros_ns'],
                    raw_time_offset_us=row.get('raw_time_offset_us', 0),
                    command_stamp_ns=row.get('command_stamp_ns'), command_age_ns=row.get('command_age_ns'),
                    steering=row['steering'], throttle=row['throttle'])
        counts_path = (a.dataset/row['counts']).resolve()
        if not counts_path.is_relative_to(a.dataset.resolve()):
            raise ValueError('Counts path escapes dataset')
        images = embed(match[2]) if match else '<p>No RGB within tolerance</p>'
        images += embed(evs_image(np.load(counts_path, allow_pickle=False)))
        cards.append('<section><div>'+images+'</div><pre>'+html.escape(json.dumps(info, indent=2))+'</pre></section>')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text('<!doctype html><meta charset="utf-8"><title>RGB / EVS timing</title>'
                       '<style>body{font-family:sans-serif}section{border-bottom:1px solid #888;padding:16px}img{max-width:48%;vertical-align:top}pre{white-space:pre-wrap}</style>'
                       '<h1>RGB (left) / EVS (right)</h1><p>Nearest ROS header; EVS integrates 40 ms. '
                       'Red=positive, blue=negative. No spatial calibration or timestamp correction is applied here. '
                       'Visual agreement is an offset sanity check, not hardware synchronization proof.</p>'+''.join(cards))
    print(f'{a.output}: RGB matched {sum(x is not None for x in best)}/{len(rows)}')


if __name__ == '__main__':
    main()
