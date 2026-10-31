"""Bounded display history of the speed controller's own measurements."""
from collections import deque
import math
import time
import uuid


class SpeedHistory:
    def __init__(self):
        self.stream = uuid.uuid4().hex
        self.samples = deque(maxlen=650)
        self.last_stamp = None

    def capture(self, stamp, statuses, fresh, now=None):
        now = time.monotonic() if now is None else now
        matches = [s for s in statuses if s.name == 'speed_controller' and s.hardware_id == 'speed_controller']
        if len(matches) != 1:
            return
        status = matches[0]
        valid = fresh and status.message == 'active'
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.samples.clear()
            self.stream = uuid.uuid4().hex
        self.last_stamp = stamp
        values = {v.key: v.value for v in status.values}
        try:
            target, measured = (float(values[k]) for k in ('target_mps', 'measured_mps'))
            valid = valid and all(math.isfinite(v) for v in (target, measured))
        except (KeyError, TypeError, ValueError):
            valid = False
            target = measured = None
        self.samples.append(dict(t=now, target=target if valid else None,
                                 measured=measured if valid else None))

    def view(self, now=None):
        now = time.monotonic() if now is None else now
        while self.samples and self.samples[0]['t'] < now - 60:
            self.samples.popleft()
        return dict(stream=self.stream, now=now, samples=list(self.samples))
