"""ROS-independent, bounded-memory lap and request state machines."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Settings:
    min_lap_time_s: float = 5.0
    min_lap_fraction: float = 0.9
    max_lateral_distance_m: float = 1.0
    max_pose_speed_mps: float = 12.0
    max_progress_speed_mps: float = 12.0
    jump_tolerance_m: float = 0.15
    reverse_tolerance_m: float = 0.2
    max_sample_gap_s: float = 0.5
    finish_zone_fraction: float = 0.15

    def validate(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('Lap settings must be positive finite values')
        if not .5 <= self.min_lap_fraction <= 1.0:
            raise ValueError('min_lap_fraction must be in [0.5, 1]')
        if not 0 < self.finish_zone_fraction < .25:
            raise ValueError('finish_zone_fraction must be in (0, 0.25)')


class LapDetector:
    """Count near-complete forward traversals ending at the finish crossing.

    Any discontinuity discards the partial lap. The count survives invalidation.
    A lap requires net forward progress, not accumulated positive jitter.
    """

    def __init__(self, settings=Settings()):
        settings.validate()
        self.settings = settings
        self.points = ()
        self.segments = ()
        self.length = 0.0
        self.count = 0
        self.reason = 'reference_unavailable'
        self.invalidate(self.reason)

    def invalidate(self, reason):
        self.last = None
        self.armed_at = None
        self.progress = 0.0
        self.reverse = 0.0
        self.reason = reason

    def set_reference(self, points, closed=True):
        try:
            points = tuple((float(x), float(y)) for x, y in points)
            if not closed or not 3 <= len(points) <= 10000:
                raise ValueError('Require a closed reference with 3..10000 points')
            if any(not math.isfinite(v) for p in points for v in p):
                raise ValueError('Reference coordinates must be finite')
            segments, length = [], 0.0
            for a, b in zip(points, points[1:] + points[:1]):
                dx, dy = b[0] - a[0], b[1] - a[1]
                distance = math.hypot(dx, dy)
                if distance <= 1e-6:
                    raise ValueError('Reference has a repeated/degenerate endpoint')
                segments.append((a, dx, dy, distance, length))
                length += distance
            if length < 1.0:
                raise ValueError('Reference length must be at least 1 m')
        except (TypeError, ValueError):
            self.points, self.segments, self.length = (), (), 0.0
            self.invalidate('invalid_reference')
            raise
        if points == self.points:
            return False
        self.points, self.segments, self.length = points, tuple(segments), length
        self.invalidate('reference_changed')
        return True

    def project(self, x, y):
        best = None
        for a, dx, dy, length, start in self.segments:
            t = max(0., min(1., ((x - a[0]) * dx + (y - a[1]) * dy) / length**2))
            distance = math.hypot(x - a[0] - t * dx, y - a[1] - t * dy)
            item = (distance, (start + t * length) % self.length)
            if best is None or item < best:
                best = item
        return best

    def sample(self, x, y, stamp):
        if not self.segments:
            self.invalidate('reference_unavailable')
            return None
        if any(not math.isfinite(v) for v in (x, y, stamp)) or stamp <= 0:
            self.invalidate('invalid_pose')
            return None
        distance, s = self.project(x, y)
        if distance > self.settings.max_lateral_distance_m:
            self.invalidate('off_reference')
            return None
        previous = self.last
        self.last = (x, y, stamp, s)
        if previous is None:
            # Starting at the finish should emit after the first full circuit.
            # An arbitrary start still needs min_lap_fraction before the first
            # finish; a shorter initial partial circuit merely re-arms there.
            self.armed_at, self.progress, self.reverse = stamp, 0., 0.
            self.reason = 'tracking_lap'
            return None
        px, py, pt, ps = previous
        dt = stamp - pt
        if dt == 0:
            self.last = previous
            return None
        if dt < 0 or dt > self.settings.max_sample_gap_s:
            self.invalidate('pose_time_discontinuity')
            return None
        cfg = self.settings
        delta = (s - ps + self.length / 2) % self.length - self.length / 2
        if (math.hypot(x - px, y - py) > cfg.jump_tolerance_m + cfg.max_pose_speed_mps * dt
                or abs(delta) > cfg.jump_tolerance_m + cfg.max_progress_speed_mps * dt):
            self.invalidate('pose_or_projection_jump')
            return None
        self.progress += delta
        # Small reverse jitter must be earned back before making forward progress.
        self.reverse = max(0., self.reverse - delta)
        if self.reverse > cfg.reverse_tolerance_m:
            self.invalidate('reverse_motion')
            return None
        crossing = (delta > 0 and ps > self.length * (1 - cfg.finish_zone_fraction)
                    and s < self.length * cfg.finish_zone_fraction)
        if not crossing:
            self.reason = 'tracking_lap' if self.armed_at is not None else 'waiting_first_finish_crossing'
            return None
        event = None
        if (self.armed_at is not None and self.progress + 1e-6 >= cfg.min_lap_fraction * self.length
                and stamp - self.armed_at + 1e-9 >= cfg.min_lap_time_s):
            self.count += 1
            event = {'lap': self.count, 'stamp_s': stamp, 'lap_time_s': stamp - self.armed_at,
                     'course_length_m': self.length, 'progress_m': self.progress}
        self.armed_at, self.progress, self.reverse = stamp, 0., 0.
        self.reason = 'lap_complete' if event else 'lap_armed'
        return event


class RequestState:
    """One request at a time, with timeout latching and no automatic retry.

    VGL success only acknowledges acceptance. Completion requires a later hint
    followed by a fresh healthy VSLAM diagnostic, provided by the adapter.
    """

    def __init__(self, needs_hint=False):
        self.needs_hint = needs_hint
        self.state = 'idle'
        self.started = None
        self.hint_at = None
        self.accepted = False
        self.requests = self.completed = self.failures = self.skipped = 0
        self.message = ''

    @property
    def busy(self):
        return self.state != 'idle'

    def start(self, wall):
        if self.busy:
            self.skipped += 1
            return False
        self.requests += 1
        self.state, self.started, self.hint_at = 'service_pending', wall, None
        self.accepted, self.message = False, ''
        return True

    def reply(self, success, message=''):
        self.message = str(message)[:512]
        if not success:
            self.failures += 1
            self.state = 'idle'
        elif self.needs_hint:
            self.accepted = True
            self.state = 'searching'
        else:
            self.completed += 1
            self.state = 'idle'

    def fail_uncertain(self, message):
        if not self.state.startswith('suppressed'):
            self.failures += 1
        self.state, self.message = 'suppressed_unknown_result', str(message)[:512]

    def hint(self, wall):
        if self.needs_hint and self.busy and wall >= self.started:
            self.hint_at = wall

    def healthy_diagnostic(self, wall):
        if (self.needs_hint and self.busy and self.accepted and self.hint_at is not None
                and wall >= self.hint_at):
            self.completed += 1
            self.state, self.message = 'idle', 'hint followed by healthy VSLAM diagnostic; accuracy not certified'

    def check_timeout(self, wall, service_timeout, result_timeout):
        if not self.busy or self.state.startswith('suppressed'):
            return
        limit = service_timeout if self.state == 'service_pending' else result_timeout
        if wall - self.started > limit:
            self.fail_uncertain('request timeout; no retry or upstream cancellation')

    def summary(self):
        return {key: getattr(self, key) for key in
                ('state', 'requests', 'completed', 'failures', 'skipped', 'message')}
