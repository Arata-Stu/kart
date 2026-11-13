"""Bounded display-only map contact points and projection diagnostics.

These data never participate in tuning revisions, STOP gates or planning.
Source stamps and local monotonic receipt times must both remain fresh.
"""
import math
import time


def _stamp(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _fresh(received, stamp, maximum, available, now, source_now_ns):
    age = None if received is None else now - received
    source_age = 0. if source_now_ns is None or stamp is None else (source_now_ns - stamp) / 1e9
    fresh = bool(available and stamp is not None and stamp > 0 and age is not None
                 and 0 <= age <= maximum and -.05 <= source_age <= maximum)
    remaining = max(0., min(maximum - age, maximum - source_age)) if fresh else 0.
    return fresh, None if age is None else max(0., age), remaining


class GroundPerception:
    MAX_OBJECTS = 100
    MAX_AGE_S = 1.0
    MAX_DIAGNOSTIC_AGE_S = 2.5

    def __init__(self):
        self.received = self.stamp = None
        self.frame = None
        self.valid = False
        self.projection_valid, self.reason = False, ''
        self.count = 0
        self.objects = []
        self.diagnostic_received = self.diagnostic_stamp = None
        self.diagnostic = dict(reason='', message='', level=1)
        self.diagnostic_valid = False

    def capture(self, message, valid=True, now=None):
        self.received = time.monotonic() if now is None else now
        self.stamp, self.frame = _stamp(message.header), str(message.header.frame_id)[:256]
        self.projection_valid = getattr(message, 'valid', False) is True
        self.reason = str(getattr(message, 'reason', 'invalid_result'))[:128]
        self.valid = bool(valid and self.frame == 'map' and self.projection_valid)
        self.count, self.objects = len(message.objects), []
        for item in message.objects[:self.MAX_OBJECTS]:
            point = item.point
            coordinates = (point.x, point.y, point.z)
            # class_name is explicit model-profile metadata. Never guess from
            # raw numeric IDs: COCO class 0 is a person, not an RC car.
            if (item.class_name not in ('rc_car', 'obstacle') or not item.class_id
                    or not all(math.isfinite(v) for v in coordinates)
                    or not math.isfinite(item.score) or not 0 <= item.score <= 1):
                continue
            self.objects.append(dict(detection_id=str(item.detection_id)[:128],
                                     class_id=str(item.class_id)[:128], kind=item.class_name,
                                     score=float(item.score), **dict(zip(('x', 'y', 'z'), map(float, coordinates)))))
        if self.count and not self.objects:
            self.valid = False

    def capture_diagnostic(self, message, valid=True, now=None):
        for status in message.status[:32]:
            if status.name != 'perception_ground/projection':
                continue
            self.diagnostic_received = time.monotonic() if now is None else now
            self.diagnostic_stamp = _stamp(message.header)
            self.diagnostic_valid = bool(valid and status.level in (0, 1, 2, 3))
            reason = next((str(value.value)[:128] for value in status.values[:64] if value.key == 'reason'), '')
            self.diagnostic = dict(reason=reason, message=str(status.message)[:512], level=int(status.level))
            return

    def view(self, available, diagnostic_available, now=None, source_now_ns=None):
        now = time.monotonic() if now is None else now
        fresh, age, remaining = _fresh(self.received, self.stamp, self.MAX_AGE_S,
                                       available, now, source_now_ns)
        event_fresh = fresh
        fresh = fresh and self.valid
        diag_fresh, diag_age, diag_remaining = _fresh(
            self.diagnostic_received, self.diagnostic_stamp, self.MAX_DIAGNOSTIC_AGE_S,
            diagnostic_available, now, source_now_ns)
        diag_fresh = diag_fresh and self.diagnostic_valid
        diagnostic = dict(available=bool(diagnostic_available), fresh=diag_fresh, age_s=diag_age,
                          fresh_for_s=diag_remaining if diag_fresh else 0.,
                          **self.diagnostic)
        if event_fresh and not self.projection_valid:
            # Invalid clear events carry their own state atomically. Do not
            # report a fresh empty/zero while the 1 Hz diagnostic still says OK.
            diagnostic = dict(available=True, fresh=True, age_s=age, fresh_for_s=remaining,
                              reason=self.reason, message='接地点の更新が無効です', level=1,
                              source='map_points')
        # A fresh failure diagnosis invalidates earlier successful coordinates,
        # even if a network status request races ahead of the empty point event.
        if diag_fresh and diagnostic['level'] > 0:
            fresh = False
        return dict(available=bool(available), fresh=bool(fresh), age_s=age,
                    projection_valid=self.projection_valid, reason=self.reason,
                    fresh_for_s=remaining if fresh else 0., frame_id=self.frame,
                    stamp_ns=None if self.stamp is None else str(self.stamp),
                    count=self.count if fresh else None, objects=list(self.objects) if fresh else [],
                    truncated=bool(fresh and self.count > len(self.objects)), diagnostic=diagnostic)
