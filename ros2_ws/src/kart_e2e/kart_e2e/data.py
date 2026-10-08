"""Portable timestamp/label validation and raw ROS Image decoding."""

import math
from bisect import bisect_right


def previous(records, stamp, tolerance_ns):
    index = bisect_right(records, stamp, key=lambda item: item[0]) - 1
    if index < 0 or not 0 <= stamp - records[index][0] <= tolerance_ns:
        return None
    return records[index][1]


def valid_label(command):
    values = [
        float(getattr(command, key))
        for key in ("steering", "throttle", "brake", "reverse")
    ]
    return (
        all(math.isfinite(v) for v in values)
        and -1 <= values[0] <= 1
        and 0 <= values[1] <= 1
        and values[2] == 0
        and values[3] == 0
    )


def decode_image(msg):
    from PIL import Image

    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    if msg.encoding not in channels:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    count = channels[msg.encoding]
    if (
        msg.width <= 0
        or msg.height <= 0
        or msg.step < msg.width * count
        or len(msg.data) != msg.step * msg.height
    ):
        raise ValueError("Malformed image dimensions/stride")
    mode = {1: "L", 3: "RGB", 4: "RGBA"}[count]
    raw = {
        "rgb8": "RGB",
        "bgr8": "BGR",
        "rgba8": "RGBA",
        "bgra8": "BGRA",
        "mono8": "L",
    }[msg.encoding]
    return Image.frombytes(
        mode, (msg.width, msg.height), bytes(msg.data), "raw", raw, msg.step
    ).convert("RGB")
