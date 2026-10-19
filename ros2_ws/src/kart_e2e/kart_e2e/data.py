"""Portable timestamp/label validation and raw ROS Image decoding."""

import math
from bisect import bisect_right


def previous_record(records, stamp, tolerance_ns):
    index = bisect_right(records, stamp, key=lambda item: item[0]) - 1
    if index < 0 or not 0 <= stamp - records[index][0] <= tolerance_ns:
        return None
    return records[index]


def previous(records, stamp, tolerance_ns):
    record = previous_record(records, stamp, tolerance_ns)
    return record[1] if record is not None else None


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


def load_rgb(source, *, suffix=None):
    """Read lossless uint8 HWC RGB NPY or a legacy Pillow image.

    source may be a path or a byte stream (provide suffix for byte streams).
    Never unpickle NumPy inputs. Keep the original resolution and pixel values.
    """
    from pathlib import Path
    import numpy as np
    from PIL import Image

    suffix = Path(source).suffix.lower() if suffix is None else suffix.lower()
    if suffix == '.npy':
        rgb = np.load(source, allow_pickle=False)
        if (not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8
                or rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) <= 0):
            raise ValueError('RGB NPY must be uint8 [height,width,3]')
        return Image.fromarray(np.ascontiguousarray(rgb))
    with Image.open(source) as image:
        return image.convert('RGB')


def save_rgb(image, path, image_format='npy'):
    """Uncompressed RGB NPY by default; PNG remains an explicit compatibility option."""
    import numpy as np
    if image_format == 'npy':
        np.save(path, np.asarray(image.convert('RGB'), dtype=np.uint8), allow_pickle=False)
    elif image_format == 'png':
        image.save(path, format='PNG', compress_level=1)
    else:
        raise ValueError('image_format must be npy or png')
