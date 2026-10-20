"""Small frame-flow estimator and optional event-image activity veto (no VSLAM)."""
import cv2
import numpy as np


def image_array(msg):
    channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}
    if msg.encoding not in channels:
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')
    n = channels[msg.encoding]
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * n:
        raise ValueError('Invalid image dimensions/step')
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if data.size != msg.height * msg.step:
        raise ValueError('Invalid image payload size')
    pixels = data.reshape(msg.height, msg.step)[:, :msg.width * n]
    return pixels.reshape(msg.height, msg.width, n) if n > 1 else pixels


def grayscale(pixels, encoding, width):
    if pixels.ndim == 3:
        code = {'rgb8': cv2.COLOR_RGB2GRAY, 'bgr8': cv2.COLOR_BGR2GRAY,
                'rgba8': cv2.COLOR_RGBA2GRAY, 'bgra8': cv2.COLOR_BGRA2GRAY}[encoding]
        pixels = cv2.cvtColor(pixels, code)
    height = max(16, round(pixels.shape[0] * width / pixels.shape[1]))
    return cv2.resize(pixels, (width, height), interpolation=cv2.INTER_AREA)


class FrameMotion:
    def __init__(self, width=212, min_features=20, max_features=120,
                 min_cells=4, fb_error_px=1.0, max_interval_s=0.25):
        self.width = width
        self.min_features, self.max_features = min_features, max_features
        self.min_cells, self.fb_error, self.max_interval = min_cells, fb_error_px, max_interval_s
        self.previous = self.stamp = None

    def update(self, gray, stamp):
        previous, old_stamp = self.previous, self.stamp
        self.previous, self.stamp = gray.copy(), stamp
        if previous is None or previous.shape != gray.shape:
            return False, 0.0, 0
        dt = stamp - old_stamp
        if not 0 < dt <= self.max_interval:
            return False, 0.0, 0
        points = cv2.goodFeaturesToTrack(previous, self.max_features, 0.01, 5)
        if points is None or len(points) < self.min_features:
            return False, 0.0, 0
        following, ok, _ = cv2.calcOpticalFlowPyrLK(previous, gray, points, None)
        if following is None:
            return False, 0.0, 0
        finite = np.isfinite(following).all(axis=(1, 2))
        following[~finite] = points[~finite]
        back, back_ok, _ = cv2.calcOpticalFlowPyrLK(gray, previous, following, None)
        if back is None:
            return False, 0.0, 0
        good = (ok.ravel().astype(bool) & back_ok.ravel().astype(bool) & finite
                & (np.linalg.norm((back - points).reshape(-1, 2), axis=1) <= self.fb_error))
        source = points.reshape(-1, 2)[good]
        target = following.reshape(-1, 2)[good]
        if len(source) < self.min_features:
            return False, 0.0, len(source)
        h, w = gray.shape
        cells = set(zip(np.minimum(3, (source[:, 0] * 4 / w).astype(int)),
                        np.minimum(2, (source[:, 1] * 3 / h).astype(int))))
        if len(cells) < self.min_cells:
            return False, 0.0, len(source)
        speed = np.linalg.norm(target - source, axis=1) / dt
        return bool(np.isfinite(speed).all()), float(np.median(speed)), len(source)


def event_activity(pixels, encoding, min_neighbors=2):
    """Fraction of supported colored event pixels in red_blue_white visualization.

    Brightness is not an event count. This is an optional motion/noise veto,
    never a metric velocity, direction, or standalone standstill observation.
    """
    if encoding not in ('rgb8', 'bgr8') or pixels.ndim != 3:
        raise ValueError('EVS assist requires red_blue_white rgb8/bgr8 event_image')
    color_range = pixels.max(axis=2).astype(np.int16) - pixels.min(axis=2).astype(np.int16)
    active = (color_range > 32).astype(np.uint8)
    neighbors = cv2.boxFilter(active, cv2.CV_16U, (3, 3), normalize=False)
    supported = (active != 0) & (neighbors >= min_neighbors)
    return float(supported.mean())
