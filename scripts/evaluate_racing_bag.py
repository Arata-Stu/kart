#!/usr/bin/env python3
"""Feed recorded RGB frames to racing_perception and render its real decoder output."""
import argparse
import html
import json
import os
import signal
import subprocess
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray


def read_frames(bag, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cls = {"sensor_msgs/msg/Image": Image,
           "sensor_msgs/msg/CompressedImage": CompressedImage}.get(types.get(topic))
    if cls is None:
        raise ValueError(f"Missing or unsupported RGB topic {topic}: {types.get(topic)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while reader.has_next():
        _, raw, _ = reader.read_next()
        yield deserialize_message(raw, cls)


def rgb_message(msg):
    buf = np.frombuffer(msg.data, np.uint8)
    if isinstance(msg, CompressedImage):
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("Invalid compressed RGB")
    else:
        if (msg.width, msg.height) != (640, 480) or msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"Expected 640x480 rgb8/bgr8, got {msg.width}x{msg.height} {msg.encoding}")
        bgr = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
    if bgr.shape != (480, 640, 3):
        raise ValueError(f"Expected 640x480 image, got {bgr.shape}")
    image = Image(height=480, width=640, encoding="rgb8", step=1920)
    image.header = msg.header
    image.data = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).tobytes()
    return image, bgr.copy()


def wait(node, predicate, timeout, detail):
    end = time.monotonic() + timeout
    while not predicate():
        if not rclpy.ok() or time.monotonic() >= end:
            raise RuntimeError(f"Timed out: {detail}")
        rclpy.spin_once(node, timeout_sec=0.05)


def key(header):
    return (header.frame_id, header.stamp.sec, header.stamp.nanosec)


def evaluate(args, tmp, node):
    pub = node.create_publisher(Image, "/racing/eval/image", qos_profile_sensor_data)
    wanted = None
    received = None

    def collect(msg):
        nonlocal received
        if key(msg.header) == wanted:
            received = msg

    sub = node.create_subscription(Detection2DArray, "/racing/detections", collect, qos_profile_sensor_data)
    wait(node, lambda: pub.get_subscription_count() and sub.get_publisher_count(), args.timeout,
         "launch racing_perception with image_topic:=/racing/eval/image in the same isolated ROS domain")
    frames = []
    for i, raw in enumerate(read_frames(args.bag, args.topic)):
        if i % args.stride:
            continue
        msg, canvas = rgb_message(raw)
        wanted, received = key(msg.header), None
        pub.publish(msg)
        wait(node, lambda: received is not None, args.timeout, f"decoder result for frame {i}, {wanted}")
        boxes = []
        for d in received.detections:
            b = d.bbox
            x, y = b.center.position.x, b.center.position.y
            boxes.append(dict(cx=x, cy=y, w=b.size_x, h=b.size_y,
                              score=d.results[0].hypothesis.score, class_id=d.results[0].hypothesis.class_id))
            lt = (round(x-b.size_x/2), round(y-b.size_y/2))
            rb = (round(x+b.size_x/2), round(y+b.size_y/2))
            cv2.rectangle(canvas, lt, rb, (0, 230, 80), 2)
            cv2.putText(canvas, f"opponent {boxes[-1]['score']:.2f}", (max(0, lt[0]), max(18, lt[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 230, 80), 2)
        name = f"frame_{i:07d}.jpg"
        if not cv2.imwrite(str(tmp / name), canvas):
            raise OSError(name)
        frames.append(dict(index=i, stamp_ns=wanted[1]*10**9+wanted[2], frame_id=wanted[0],
                           image=name, detections=boxes))
        if len(frames) >= args.frames:
            break
    if not frames:
        raise ValueError("No RGB frames in the selected topic")
    report = dict(bag=str(args.bag.resolve()), topic=args.topic, engine=str(args.engine.resolve()),
                  engine_sha256=engine_hash(args.engine), stride=args.stride, frames=frames)
    (tmp / "report.json").write_text(json.dumps(report, indent=2))
    cards = [f'<figure><img loading="lazy" width="640" src="{v["image"]}">'
             f'<figcaption>Frame {v["index"]} · {v["stamp_ns"]} ns · {len(v["detections"])} detections</figcaption></figure>'
             for v in frames]
    (tmp / "index.html").write_text('<meta charset="utf-8"><title>Racing detector</title><h1>'
                                    + html.escape(args.bag.name) + '</h1>' + ''.join(cards))


def engine_hash(path):
    import hashlib
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/realsense/color/image_raw")
    parser.add_argument("--engine", required=True, type=Path)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--timeout", type=float, default=60)
    parser.add_argument("--domain", type=int, choices=range(1, 102), default=91, metavar="1..101")
    args = parser.parse_args()
    if min(args.stride, args.frames) < 1 or not 0 < args.timeout < float("inf"):
        parser.error("stride, frames and timeout must be positive")
    if args.output.exists() or not args.engine.is_file():
        parser.error("Specify a new output directory and the same engine as the launched detector")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Keep offline messages away from a vehicle's default DDS domain.
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    rclpy.init()
    node = rclpy.create_node("racing_bag_evaluator")
    launch = subprocess.Popen(["ros2", "launch", "kart_bringup", "racing_perception.launch.py",
                               f"engine:={args.engine.resolve()}", "image_topic:=/racing/eval/image"],
                              start_new_session=True)
    try:
        with tempfile.TemporaryDirectory(dir=args.output.parent) as temp:
            result = Path(temp) / "result"
            result.mkdir()
            evaluate(args, result, node)
            result.rename(args.output)
        print(args.output / "index.html")
    finally:
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
