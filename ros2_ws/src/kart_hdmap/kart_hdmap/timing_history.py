"""Display-only sampled controller timing; never used for control decisions."""
from collections import deque
import math
import time


class TimingHistory:
    def __init__(self, name, hardware, ages):
        self.name, self.hardware, self.ages = name, hardware, ages
        self.samples = deque(maxlen=650)
        self.last_stamp = None

    def capture(self, stamp, statuses, fresh, now=None):
        now = time.monotonic() if now is None else now
        matches = [s for s in statuses if s.name == self.name and s.hardware_id == self.hardware]
        if len(matches) != 1:
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.samples.clear()
        self.last_stamp = stamp
        status = matches[0]
        values = {v.key: v.value for v in status.values}
        row = dict(t=now)
        active = status.message == 'active' if self.name == 'speed_controller' else status.level == 0
        for key in ('processing_ms', *self.ages):
            try:
                value = float(values[key])
                valid = fresh and math.isfinite(value)
                valid = valid and (value >= 0 if key == 'processing_ms' else active)
                row[key] = value if valid else None
            except (KeyError, TypeError, ValueError):
                row[key] = None
        # Approximately 10 Hz for display. Keep validity transitions as gaps.
        if self.samples and now-self.samples[-1]['t'] < .1:
            if all((row[k] is None) == (self.samples[-1][k] is None) for k in row if k != 't'):
                return
        self.samples.append(row)

    def view(self, now=None):
        now = time.monotonic() if now is None else now
        while self.samples and self.samples[0]['t'] < now-60:
            self.samples.popleft()
        return dict(now=now, samples=list(self.samples))
