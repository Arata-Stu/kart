"""Bounded exact-frame overlays and JPEG rendering without ROS dependencies."""
from collections import OrderedDict
from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class Settings:
    max_fps: float = 5.0
    jpeg_quality: int = 70
    match_wait_s: float = .20
    cache_size: int = 8
    cache_ttl_s: float = 1.0
    max_boxes: int = 100
    image_timeout_s: float = 1.0

    def __post_init__(self):
        for name, lower, upper in (('max_fps', .1, 30.), ('match_wait_s', 0., 2.),
                                   ('cache_ttl_s', .05, 10.), ('image_timeout_s', .05, 10.)):
            value = getattr(self, name)
            if not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f'{name} must be finite and in {lower}..{upper}')
        for name, lower, upper in (('jpeg_quality', 1, 100), ('cache_size', 1, 32), ('max_boxes', 1, 1000)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or not lower <= value <= upper:
                raise ValueError(f'{name} must be an integer in {lower}..{upper}')
        if self.cache_ttl_s < self.match_wait_s:
            raise ValueError('cache_ttl_s must be at least match_wait_s')
        if self.image_timeout_s < self.match_wait_s:
            raise ValueError('image_timeout_s must be at least match_wait_s')


@dataclass(frozen=True)
class Box:
    cx: float
    cy: float
    width: float
    height: float
    class_id: str
    score: float
    track_id: str = ''


@dataclass(frozen=True)
class Result:
    count: int
    boxes: tuple


@dataclass(frozen=True)
class Frame:
    key: tuple
    image: np.ndarray
    received: float
    detections: object = None
    tracks: object = None


def header_key(header):
    return (int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec), str(header.frame_id))


def valid_key(key):
    return isinstance(key[0], int) and key[0] > 0 and bool(key[1])


def decode_image(width, height, encoding, step, data):
    """Read raw camera pixels without resizing, cropping or cv_bridge."""
    if (width, height) != (640, 480) or encoding not in ('rgb8', 'bgr8', 'mono8'):
        raise ValueError(f'Expected 640x480 rgb8/bgr8/mono8, got {width}x{height} {encoding}')
    channels = 1 if encoding == 'mono8' else 3
    if step < width * channels or len(data) < height * step:
        raise ValueError('Invalid image step or data length')
    pixels = np.frombuffer(data, dtype=np.uint8, count=height * step).reshape(height, step)
    pixels = pixels[:, :width * channels].reshape(height, width, channels)
    if encoding == 'rgb8':
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    if encoding == 'mono8':
        return cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
    return pixels.copy()


def result_from_message(message, max_boxes):
    boxes = []
    for detection in message.detections[:max_boxes]:
        b = detection.bbox
        values = (b.center.position.x, b.center.position.y, b.size_x, b.size_y)
        if not all(math.isfinite(v) for v in values) or min(values[2:]) <= 0:
            continue
        hypotheses = [r.hypothesis for r in detection.results[:80]
                      if r.hypothesis.class_id and math.isfinite(r.hypothesis.score)
                      and 0 <= r.hypothesis.score <= 1]
        if not hypotheses:
            continue
        best = max(hypotheses, key=lambda h: h.score)
        boxes.append(Box(*map(float, values), str(best.class_id)[:64], float(best.score), str(detection.id)[:64]))
    return Result(len(message.detections), tuple(boxes))


class OverlayBuffer:
    """Only exact stamp+frame matches; late results never decorate another image."""
    def __init__(self, settings):
        self.settings = settings
        self.images = OrderedDict()
        self.results = {name: OrderedDict() for name in ('detections', 'tracks')}
        self.last_image_stamp = None

    def clear(self):
        self.images.clear()
        for cache in self.results.values():
            cache.clear()
        self.last_image_stamp = None

    def _bounded(self, cache, protected=()):
        while len(cache) > self.settings.cache_size:
            victim = next((key for key in cache if key not in protected), next(iter(cache)))
            del cache[victim]

    def _expire(self, now):
        for cache in self.results.values():
            for key, (_, received) in list(cache.items()):
                if now - received > self.settings.cache_ttl_s:
                    del cache[key]

    def add_image(self, key, image, now):
        if self.last_image_stamp is not None and key[0] < self.last_image_stamp:
            # A new camera/bag epoch may reuse old stamps. Do not reuse its old
            # boxes; results alone never reset a newer pending image.
            self.clear()
        self.last_image_stamp = key[0]
        self._expire(now)
        if key not in self.images:
            self.images[key] = Frame(key, image, now)
        self._bounded(self.images)

    def add_result(self, channel, key, result, now):
        self._expire(now)
        if not valid_key(key):
            return
        cache = self.results[channel]
        # The first result for a source frame is authoritative for this preview.
        if key not in cache:
            cache[key] = (result, now)
        # Results from an unsampled high-FPS camera must not evict a result
        # already paired with one of the admitted low-rate preview images.
        self._bounded(cache, self.images)

    def take(self, now):
        self._expire(now)
        for key, frame in list(self.images.items()):
            if now - frame.received > self.settings.image_timeout_s:
                del self.images[key]
                for cache in self.results.values():
                    cache.pop(key, None)
        ready = []
        for key, frame in self.images.items():
            matched = valid_key(key) and all(key in cache for cache in self.results.values())
            if matched or now - frame.received >= self.settings.match_wait_s:
                ready.append(key)
        if not ready:
            return None
        # Drop older preview candidates if rendering fell behind the camera.
        key = ready[-1]
        selected = self.images[key]
        while self.images:
            previous, _ = self.images.popitem(last=False)
            if previous == key:
                break
        channels = {}
        for name, cache in self.results.items():
            entry = cache.pop(key, None) if valid_key(key) else None
            channels[name] = None if entry is None else entry[0]
        return Frame(key, selected.image, selected.received, **channels)


def _caption(result, label):
    if result is None:
        return f'{label}: MISSING (no matching stamp/frame)'
    shown = len(result.boxes)
    return f'{label}: {result.count}' + (f' ({shown} boxes shown)' if result.count != shown else '')


def _box_label(box, names, tracking):
    label = f'class {box.class_id[:16]}'
    if box.class_id.isdigit() and int(box.class_id) < len(names) and names[int(box.class_id)]:
        label = names[int(box.class_id)][:32]
    if tracking and box.track_id:
        label += f' ID {box.track_id[:32]}'
    return f'{label} {box.score:.2f}'


def draw_frame(frame, class_names=()):
    image = frame.image.copy()
    for result, color, tracking in ((frame.detections, (0, 220, 0), False),
                                    (frame.tracks, (255, 100, 0), True)):
        if result is None:
            continue
        for box in result.boxes:
            x1, y1 = max(0, round(box.cx - box.width / 2)), max(0, round(box.cy - box.height / 2))
            x2, y2 = min(639, round(box.cx + box.width / 2)), min(479, round(box.cy + box.height / 2))
            if x1 >= x2 or y1 >= y2:
                continue
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            label = _box_label(box, class_names, tracking)
            size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .45, 1)
            label_x = min(x1, max(0, 638 - size[0]))
            # Separate green detection labels above from blue track IDs below.
            label_y = (min(474, y2 + 17) if y2 < 457 else max(82, y2 - 7)) if tracking else max(82, y1 - 4)
            cv2.rectangle(image, (label_x, label_y - size[1] - 2),
                          (label_x + size[0] + 2, min(479, label_y + baseline)), (15, 15, 15), -1)
            cv2.putText(image, label, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    cv2.rectangle(image, (0, 0), (639, 66), (20, 20, 20), -1)
    for row, text in enumerate((_caption(frame.detections, 'DETECTIONS'),
                                 _caption(frame.tracks, 'CAR TRACKS'),
                                 f'stamp {frame.key[0]} | {frame.key[1][:80]}' if valid_key(frame.key)
                                 else 'SOURCE HEADER INVALID - overlays disabled')):
        cv2.putText(image, text, (6, 18 + row * 21), cv2.FONT_HERSHEY_SIMPLEX, .43,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return image


def placeholder(reason):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    for row, text in enumerate(('NO SOURCE IMAGE - diagnostic placeholder', reason[:80],
                                 'Detections and tracks are not overlaid.')):
        cv2.putText(image, text, (12, 190 + row * 35), cv2.FONT_HERSHEY_SIMPLEX,
                    .6, (0, 200, 255), 1, cv2.LINE_AA)
    return image


def encode_jpeg(image, quality):
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError('JPEG encoding failed')
    return encoded.tobytes()
