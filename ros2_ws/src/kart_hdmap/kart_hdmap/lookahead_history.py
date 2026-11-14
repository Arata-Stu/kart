"""Bounded display samples from the lateral controller's own calculation."""
from collections import deque
import math
import time
import uuid


class LookaheadHistory:
    MAX_AGE_S = .5
    FIELDS = ('lookahead_m', 'target_distance_m', 'measured_speed_mps', 'target_speed_mps')

    def __init__(self):
        self.stream = uuid.uuid4().hex
        self.samples = deque(maxlen=650)
        self.last_stamp = None
        self.last_received = None
        self.expired = False

    @classmethod
    def empty(cls, now):
        return dict(t=now, controller=None, lookahead_mode=None,
                    **dict.fromkeys(cls.FIELDS))

    def expire(self, now):
        if (self.last_received is not None and not self.expired and
                now - self.last_received > self.MAX_AGE_S):
            self.expired = True
            if self.samples and any(self.samples[-1][key] is not None for key in self.FIELDS):
                self.samples.append(self.empty(self.last_received + self.MAX_AGE_S))

    @staticmethod
    def number(values, key):
        try:
            value = float(values[key])
        except (KeyError, TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(value):
            return None
        # Display sanity limits, independent of controller settings and gates.
        if key == 'measured_speed_mps':
            return value if abs(value) <= 100 else None
        lower = 0 < value if key == 'lookahead_m' else 0 <= value
        upper = 100 if key == 'target_speed_mps' else 1000
        return value if lower and value <= upper else None

    def capture(self, stamp, statuses, fresh, now=None):
        now = time.monotonic() if now is None else now
        self.expire(now)
        matches = [s for s in statuses
                   if s.name == 'lateral_controller' and s.hardware_id == 'kart_control']
        if len(matches) != 1 or not math.isfinite(stamp) or stamp <= 0:
            return
        # Repeated source samples must not acquire a newer receipt time.
        if stamp == self.last_stamp:
            return
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.samples.clear()
            self.stream = uuid.uuid4().hex
        self.last_stamp, self.last_received, self.expired = stamp, now, False
        row = self.empty(now)
        status = matches[0]
        values = {v.key: v.value for v in status.values}
        if fresh and status.level == 0:
            controller = values.get('controller')
            if controller in ('pure_pursuit', 'mpc'):
                row['controller'] = controller
                mode = values.get('lookahead_mode')
                if mode in ('fixed', 'speed_interpolated'):
                    row['lookahead_mode'] = mode
                for key in self.FIELDS:
                    row[key] = self.number(values, key)
        # Approximately 10 Hz; retain every validity, controller and mode change.
        if self.samples and now - self.samples[-1]['t'] < .1:
            previous = self.samples[-1]
            if (all(row[key] == previous[key] for key in ('controller', 'lookahead_mode')) and
                    all((row[key] is None) == (previous[key] is None) for key in self.FIELDS)):
                return
        self.samples.append(row)

    def view(self, now=None):
        now = time.monotonic() if now is None else now
        self.expire(now)
        while self.samples and self.samples[0]['t'] < now - 60:
            self.samples.popleft()
        return dict(stream=self.stream, now=now, samples=list(self.samples))
