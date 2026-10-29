"""Bounded, timestamped display samples; independent of ROS and vehicle control."""
from collections import deque
import math
import uuid


def validate_rate(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 1000:
        raise ValueError('tuning_pose_rate_hz must be finite in 0..1000 (0 follows TF)')


class PoseHistory:
    def __init__(self, capacity=12000):
        self.samples = deque(maxlen=capacity)
        self.stream = uuid.uuid4().hex
        self.sequence = 0
        self.last_seen = None
        self.last_sample = None
        self.broken = True

    def add(self, stamp_ns, pose, valid, rate=0.):
        validate_rate(rate)
        if not valid:
            self.broken = True
            return
        if self.last_seen is not None and stamp_ns < self.last_seen:
            self.samples.clear()
            self.stream = uuid.uuid4().hex
            self.last_sample = None
            self.broken = True
        if stamp_ns == self.last_seen:
            return
        gap = self.last_seen is not None and stamp_ns - self.last_seen > 500_000_000
        self.broken = self.broken or gap
        self.last_seen = stamp_ns
        if self.last_sample is not None and rate and stamp_ns - self.last_sample < 1e9 / rate:
            return
        self.sequence += 1
        self.samples.append(dict(seq=self.sequence, stamp_ns=str(stamp_ns), **pose,
                                 break_before=self.broken))
        self.last_sample = stamp_ns
        self.broken = False

    def batch(self, stream=None, after=0):
        if isinstance(after, bool) or not isinstance(after, int) or after < 0:
            raise ValueError('pose_after must be a nonnegative integer')
        reset = stream != self.stream or after > self.sequence
        gap = bool(self.samples and after < self.samples[0]['seq'] - 1)
        return dict(pose_stream=self.stream, pose_reset=reset or gap,
                    pose_samples=[p for p in self.samples if reset or p['seq'] > after],
                    pose_cursor=self.sequence)
