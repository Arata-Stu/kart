"""Bounded, display-only snapshots of image-space detections and tracks."""
import math
import time


class PerceptionStatus:
    MAX_OBJECTS = 10
    MAX_AGE_S = 2.0

    def __init__(self):
        self.received = None
        self.stamp_ns = None
        self.frame_id = None
        self.valid = False
        self.count = None
        self.objects = []

    def capture(self, message, valid=True, now=None):
        self.received = time.monotonic() if now is None else now
        self.valid = bool(valid)
        stamp = message.header.stamp
        self.stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        self.frame_id = str(message.header.frame_id)[:256]
        self.count = len(message.detections)
        self.objects = []
        # Never scan/copy an unbounded detection list in the control executor.
        for detection in message.detections[:self.MAX_OBJECTS]:
            box = detection.bbox
            bbox = [box.center.position.x, box.center.position.y, box.size_x, box.size_y]
            if not all(math.isfinite(v) for v in bbox) or min(bbox[2:]) <= 0:
                continue
            hypotheses = [result.hypothesis for result in detection.results[:80]
                          if result.hypothesis.class_id and math.isfinite(result.hypothesis.score)
                          and 0 <= result.hypothesis.score <= 1]
            if not hypotheses:
                continue
            hypothesis = max(hypotheses, key=lambda value: value.score)
            self.objects.append(dict(id=str(detection.id)[:128],
                                     class_id=str(hypothesis.class_id)[:128],
                                     score=float(hypothesis.score), bbox=[float(v) for v in bbox]))

    def view(self, available, now=None, source_now_ns=None):
        now = time.monotonic() if now is None else now
        age = None if self.received is None else now - self.received
        source_fresh = self.stamp_ns is not None and self.stamp_ns > 0
        remaining_source = self.MAX_AGE_S
        if source_fresh and source_now_ns is not None:
            source_age = (source_now_ns - self.stamp_ns) / 1e9
            source_fresh = -.05 <= source_age <= self.MAX_AGE_S
            remaining_source = self.MAX_AGE_S - source_age
        fresh = bool(available and self.valid and source_fresh and age is not None
                     and 0 <= age <= self.MAX_AGE_S)
        return dict(available=bool(available), fresh=fresh,
                    age_s=None if age is None else max(0, age),
                    fresh_for_s=max(0, min(self.MAX_AGE_S - age, remaining_source)) if fresh else 0.,
                    stamp_ns=None if self.stamp_ns is None else str(self.stamp_ns),
                    frame_id=self.frame_id, count=self.count if fresh else None,
                    objects=list(self.objects) if fresh else [],
                    truncated=bool(fresh and self.count > len(self.objects)))
