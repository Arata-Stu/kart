"""ROS-free validation, projection and freshness state for map-only detections."""
from collections import Counter
from dataclasses import dataclass, field
import math
import time

from .ground import ground_point


MAX_AGE_S = .5
MARKER_LIFETIME_S = .5
COLOURS = {'rc_car': (57 / 255, 220 / 255, 176 / 255),
           'obstacle': (209 / 255, 107 / 255, 165 / 255)}


def valid_frame(value):
    return (isinstance(value, str) and 0 < len(value) <= 256 and not value.startswith('/')
            and not any(character.isspace() for character in value))


def stamp_ns(stamp):
    if (type(stamp.sec) is not int or type(stamp.nanosec) is not int
            or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000):
        return 0
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def header_key(header):
    return stamp_ns(header.stamp), header.frame_id


@dataclass(frozen=True)
class Settings:
    projection_frame: str = 'base_link'
    target_frame: str = 'map'
    ground_z_m: float = float('nan')
    max_range_m: float = 10.
    confidence_threshold: float = .25
    rc_car_class_id: str = '0'
    obstacle_class_id: str = '1'
    max_objects: int = 100

    def __post_init__(self):
        if not valid_frame(self.projection_frame) or not valid_frame(self.target_frame):
            raise ValueError('projection_frame / target_frame must be valid TF frame names')
        if (type(self.ground_z_m) not in (int, float) or not math.isfinite(self.ground_z_m)
                or type(self.max_range_m) not in (int, float) or not 0 < self.max_range_m <= 1000
                or type(self.confidence_threshold) not in (int, float)
                or not 0 <= self.confidence_threshold <= 1):
            raise ValueError('Set measured ground_z_m, max_range_m in (0,1000], and confidence in [0,1]')
        classes = (self.rc_car_class_id, self.obstacle_class_id)
        if (not all(isinstance(value, str) and len(value) <= 64 for value in classes)
                or not any(classes) or (all(classes) and classes[0] == classes[1])):
            raise ValueError('Use distinct class IDs; an empty ID disables that class')
        if type(self.max_objects) is not int or not 1 <= self.max_objects <= 1000:
            raise ValueError('max_objects must be an integer in [1,1000]')


@dataclass(frozen=True)
class ProjectedDetection:
    detection_id: str
    class_id: str
    class_name: str
    score: float
    point: tuple


@dataclass
class Outcome:
    reason: str
    objects: list = field(default_factory=list)
    rejected: dict = field(default_factory=dict)
    detail: str = ''


def projection_issue(detections, info, now_ns=None):
    stamp, frame = header_key(detections.header)
    info_stamp, info_frame = header_key(info.header)
    if stamp <= 0 or info_stamp <= 0:
        return 'invalid_stamp'
    if stamp != info_stamp:
        return 'not_synchronized'
    if not valid_frame(frame) or not valid_frame(info_frame) or frame != info_frame:
        return 'invalid_frame'
    if now_ns is not None:
        if now_ns <= 0:
            return 'clock_unavailable'
        age = (now_ns - stamp) / 1_000_000_000
        if age > MAX_AGE_S or age < -.05:
            return 'capture_stale'
    if ((info.width, info.height) != (640, 480) or info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
            or info.roi.x_offset or info.roi.y_offset
            or (info.roi.width, info.roi.height) not in ((0, 0), (640, 480))):
        return 'invalid_image_shape'
    if (info.distortion_model not in ('plumb_bob', 'rational_polynomial') or len(info.k) != 9
            or not all(math.isfinite(value) for value in info.k) or min(info.k[0], info.k[4]) <= 0
            or any(abs(info.k[index]) > 1e-6 for index in (1, 3, 6, 7)) or abs(info.k[8] - 1) > 1e-6
            or len(info.d) not in (0, 4, 5, 8, 12, 14) or not all(math.isfinite(value) for value in info.d)):
        return 'invalid_calibration'
    return None


def transform_parts(transform):
    t, q = transform.translation, transform.rotation
    translation, quaternion = (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in translation + quaternion):
        raise ValueError('nonfinite TF')
    norm = math.hypot(*quaternion)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('invalid TF quaternion')
    return translation, tuple(value / norm for value in quaternion)


def transform_point(point, transform):
    """Apply target<-projection AFTER the intersection in the body frame."""
    translation, q = transform_parts(transform)
    x, y, z = map(float, point)
    # v' = v + w*(2*q.xyz cross v) + q.xyz cross (2*q.xyz cross v).
    cross = (2 * (q[1] * z - q[2] * y), 2 * (q[2] * x - q[0] * z), 2 * (q[0] * y - q[1] * x))
    rotated = (x + q[3] * cross[0] + q[1] * cross[2] - q[2] * cross[1],
               y + q[3] * cross[1] + q[2] * cross[0] - q[0] * cross[2],
               z + q[3] * cross[2] + q[0] * cross[1] - q[1] * cross[0])
    result = tuple(value + offset for value, offset in zip(rotated, translation))
    if not all(math.isfinite(value) for value in result):
        raise ValueError('nonfinite projected point')
    return result


def _contact(det, info):
    box = det.bbox
    cx, cy, sx, sy = box.center.position.x, box.center.position.y, box.size_x, box.size_y
    theta = getattr(box.center, 'theta', 0.)
    if not all(math.isfinite(value) for value in (cx, cy, sx, sy, theta)) or min(sx, sy) <= 0 or abs(theta) > 1e-6:
        return None, 'invalid_bbox'
    x1, x2, y1, bottom = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    # A box reaching the last image row may have lost the tire/ground contact.
    if bottom >= info.height - 1:
        return None, 'bottom_clipped'
    if x1 < 0 or x2 > info.width or y1 < 0 or not 0 < bottom < info.height - 1:
        return None, 'bbox_outside_image'
    return (cx, bottom), None


def selected_hypothesis(det, settings):
    classes = {value for value in (settings.rc_car_class_id, settings.obstacle_class_id) if value}
    candidates = [(float(result.hypothesis.score), result.hypothesis.class_id) for result in det.results[:16]
                  if result.hypothesis.class_id in classes and math.isfinite(result.hypothesis.score)
                  and settings.confidence_threshold <= result.hypothesis.score <= 1]
    return max(candidates) if candidates else None


def project_points(detections, info, camera_to_projection, projection_to_target, settings, now_ns=None):
    issue = projection_issue(detections, info, now_ns)
    if issue:
        return Outcome(issue)
    try:
        translation, quaternion = transform_parts(camera_to_projection)
        transform_parts(projection_to_target)
    except ValueError as error:
        return Outcome('invalid_tf', detail=str(error))
    rejected, objects = Counter(), []
    classes = {value: name for value, name in ((settings.rc_car_class_id, 'rc_car'),
                                               (settings.obstacle_class_id, 'obstacle')) if value}
    rejected['object_limit'] = max(0, len(detections.detections) - settings.max_objects)
    for index, det in enumerate(detections.detections[:settings.max_objects]):
        selected = selected_hypothesis(det, settings)
        if selected is None:
            rejected['class_or_score'] += 1
            continue
        score, class_id = selected
        # Detection2DArray is authoritative. A populated per-detection header
        # must agree; an entirely default per-detection header is permitted.
        header = getattr(det, 'header', None)
        if (header is not None and (header.stamp.sec or header.stamp.nanosec or header.frame_id)
                and header_key(header) != header_key(detections.header)):
            rejected['detection_header'] += 1
            continue
        uv, issue = _contact(det, info)
        if issue:
            rejected[issue] += 1
            continue
        point = ground_point(uv, info.k, info.d, info.distortion_model, translation, quaternion,
                             settings.ground_z_m, math.inf)
        if point is None:
            rejected['no_ground_intersection'] += 1
            continue
        if math.hypot(*point[:2]) > settings.max_range_m:
            rejected['out_of_range'] += 1
            continue
        try:
            target = transform_point(point, projection_to_target)
        except ValueError:
            rejected['nonfinite_point'] += 1
            continue
        identity = det.id if isinstance(det.id, str) and det.id else f'frame:{index}'
        objects.append(ProjectedDetection(identity[:128], class_id, classes[class_id], score, target))
    if objects:
        reason = 'projected'
    elif not detections.detections:
        reason = 'zero_detections'
    elif sum(rejected.values()) == rejected['class_or_score']:
        reason = 'zero_selected'
    else:
        reason = 'all_rejected'
    return Outcome(reason, objects, {key: value for key, value in rejected.items() if value})


class ProjectionStatus:
    """Monotonic input freshness and exact-key admission; duplicates never refresh."""
    REASONS = {
        'waiting_detections': (1, '検出未受信'), 'detections_stale': (1, '検出入力途絶'),
        'waiting_camera_info': (1, 'CameraInfo未受信'), 'camera_info_stale': (1, 'CameraInfo入力途絶'),
        'not_synchronized': (1, '検出とCameraInfoの撮影stampが完全一致していません'),
        'invalid_stamp': (1, '撮影stampが無効です'), 'invalid_frame': (1, '撮影frameが空・不一致・不正です'),
        'clock_unavailable': (1, 'ROS時刻が未取得です'), 'capture_stale': (1, '撮影stampが古いか未来です'),
        'invalid_image_shape': (1, '640x480 raw RGBの校正・ROI条件と一致しません'),
        'invalid_calibration': (1, 'CameraInfoの内部校正・歪み設定が無効です'),
        'camera_tf_unavailable': (1, '撮影時刻のcamera→車体TFがありません'),
        'target_tf_unavailable': (1, '撮影時刻の車体→map TFがありません'), 'invalid_tf': (1, 'TFが不正です'),
        'zero_detections': (0, '検出0件（地図表示なし）'), 'zero_selected': (0, '対象class・score条件を満たす検出0件'),
        'all_rejected': (1, '対象検出を路面投影で全て除外しました'), 'projected': (0, '撮影時刻のmap路面表示を更新しました'),
    }

    def __init__(self):
        self.received = {'detections': 0, 'camera_info': 0}
        self.keys = {'detections': None, 'camera_info': None}
        self.at = {'detections': None, 'camera_info': None}
        self.synchronized_key = None
        self.synchronized_at = None
        self.input_count = self.sync_count = self.total_projected = 0
        self.outcome = Outcome('waiting_detections')

    def receive(self, channel, message, now=None):
        now = time.monotonic() if now is None else now
        key = header_key(message.header)
        self.received[channel] += 1
        if key == self.keys[channel]:
            return False
        previous = self.keys[channel]
        if channel == 'detections' and previous is not None and key[0] < previous[0]:
            self.synchronized_key = None
            self.synchronized_at = None
        self.keys[channel], self.at[channel] = key, now
        if channel == 'detections':
            self.input_count = len(message.detections)
        return True

    def admit(self, message):
        key = header_key(message.header)
        return key == self.keys['detections'] and key != self.synchronized_key

    def synchronized(self, message, outcome, now=None):
        if outcome.reason not in self.REASONS:
            raise ValueError('Unknown map projection reason')
        self.synchronized_key = header_key(message.header)
        self.synchronized_at = time.monotonic() if now is None else now
        self.sync_count += 1
        self.total_projected += len(outcome.objects)
        self.outcome = outcome

    def input_issue(self, now=None):
        now = time.monotonic() if now is None else now
        for channel, waiting, stale in (('detections', 'waiting_detections', 'detections_stale'),
                                        ('camera_info', 'waiting_camera_info', 'camera_info_stale')):
            key, at = self.keys[channel], self.at[channel]
            if key is None:
                return waiting
            elif not 0 <= now - at <= MAX_AGE_S:
                return stale
            elif key[0] <= 0:
                return 'invalid_stamp'
            elif not valid_frame(key[1]):
                return 'invalid_frame'
        if self.keys['detections'][1] != self.keys['camera_info'][1]:
            return 'invalid_frame'
        return None

    def view(self, now=None):
        now = time.monotonic() if now is None else now
        reason = self.input_issue(now)
        if reason is None:
            if (self.synchronized_key != self.keys['detections'] or self.synchronized_at is None
                    or not 0 <= now - self.synchronized_at <= MAX_AGE_S):
                reason = 'not_synchronized'
            else:
                reason = self.outcome.reason
        level, message = self.REASONS[reason]
        values = dict(reason=reason, detections_received=self.received['detections'],
                      camera_info_received=self.received['camera_info'], synchronized_received=self.sync_count,
                      projected_points=self.total_projected, last_detections_count=self.input_count,
                      last_projection_count=len(self.outcome.objects),
                      rc_car_count=sum(obj.class_name == 'rc_car' for obj in self.outcome.objects),
                      obstacle_count=sum(obj.class_name == 'obstacle' for obj in self.outcome.objects),
                      rejected=str(self.outcome.rejected), detail=self.outcome.detail[:256])
        for channel in self.keys:
            values[channel + '_stamp_ns'] = None if self.keys[channel] is None else self.keys[channel][0]
            values[channel + '_frame'] = None if self.keys[channel] is None else str(self.keys[channel][1])[:256]
            values[channel + '_age_s'] = None if self.at[channel] is None else round(now - self.at[channel], 3)
        return dict(level=level, message=message, values={key: 'unavailable' if value is None else str(value)
                                                         for key, value in values.items()})
