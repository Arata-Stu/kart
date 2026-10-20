"""ROS-independent, bounded steering-history recovery. Times are control seconds."""
from collections import deque
from dataclasses import dataclass
import math
from statistics import median


@dataclass(frozen=True)
class Settings:
    history_span_s: float = 0.5
    history_offset_s: float = 0.15
    history_min_samples: int = 5
    history_max_gap_s: float = 0.12
    min_forward_throttle: float = 0.03
    startup_grace_s: float = 1.0
    stuck_duration_s: float = 0.8
    stationary_flow_px_s: float = 2.0
    moving_flow_px_s: float = 6.0
    stationary_gyro_rad_s: float = 0.12
    brake_command: float = 0.1
    brake_min_s: float = 0.2
    stop_confirm_s: float = 0.25
    stop_timeout_s: float = 2.0
    reverse_neutral_s: float = 0.2
    reverse_throttle: float = 0.08
    reverse_duration_s: float = 0.5
    recovery_steering_scale: float = 1.0
    recovery_steering_limit: float = 0.5
    resume_neutral_s: float = 0.3
    max_recovery_attempts: int = 2
    progress_reset_s: float = 3.0
    max_recovery_rotation_rad: float = 0.35
    max_tick_gap_s: float = 0.25

    def __post_init__(self):
        for key, value in vars(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        for key in ('history_span_s', 'history_max_gap_s', 'stuck_duration_s',
                    'stop_confirm_s', 'stop_timeout_s', 'reverse_duration_s',
                    'progress_reset_s', 'max_recovery_rotation_rad', 'max_tick_gap_s'):
            if getattr(self, key) <= 0:
                raise ValueError(f'{key} must be positive')
        for key in ('history_min_samples', 'max_recovery_attempts'):
            if type(getattr(self, key)) is not int or getattr(self, key) < 1:
                raise ValueError(f'{key} must be a positive integer')
        for key in ('min_forward_throttle', 'brake_command', 'reverse_throttle',
                    'recovery_steering_limit'):
            if getattr(self, key) > 1:
                raise ValueError(f'{key} must be <= 1')
        if self.reverse_throttle <= 0 or self.brake_command <= 0 or self.min_forward_throttle <= 0:
            raise ValueError('forward threshold, brake and reverse commands must be positive')
        if self.moving_flow_px_s <= self.stationary_flow_px_s:
            raise ValueError('moving threshold must exceed stationary threshold')
        if self.stop_timeout_s <= self.brake_min_s + self.stop_confirm_s:
            raise ValueError('stop timeout too short')


@dataclass(frozen=True)
class Command:
    steering: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    reverse: float = 0.0

    def valid(self):
        values = (self.steering, self.throttle, self.brake, self.reverse)
        return (all(math.isfinite(x) for x in values)
                and abs(self.steering) <= 1
                and all(0 <= x <= 1 for x in values[1:])
                and sum(x > 0 for x in values[1:]) <= 1)


@dataclass(frozen=True)
class Observation:
    valid: bool = False
    flow: float = 0.0  # Median bidirectionally checked image flow, px/s at processing resolution.
    gyro: float = 0.0  # Angular speed norm; camera/base axes need not coincide.
    evs_quiet: bool = True  # Auxiliary veto only, never standalone evidence of standstill.


class Recovery:
    ACTIVE = {'braking', 'reverse_neutral', 'reversing', 'resume_neutral'}

    def __init__(self, settings=Settings()):
        self.p = settings
        self.history = deque()
        self.state = 'inactive'
        self.reason = ''
        self.armed = False
        self.attempts = 0
        self.fixed_steering = 0.0
        self.selected_interval = None
        self.last_time = None
        self.last_history_stamp = None
        self.suspect_at = self.stopped_at = self.progress_at = None
        self.phase_at = self.auto_at = 0.0
        self.rotation = 0.0
        self.reverse_progress = False

    @property
    def active(self):
        return self.state in self.ACTIVE

    def fault(self, reason):
        self.state, self.reason, self.armed = 'fault', reason, False
        self.history.clear()
        self.suspect_at = self.stopped_at = self.progress_at = None
        return Command()

    def _phase(self, state, now):
        self.state, self.phase_at = state, now
        self.stopped_at = None

    def _stationary(self, obs):
        return (obs.flow <= self.p.stationary_flow_px_s
                and obs.gyro <= self.p.stationary_gyro_rad_s and obs.evs_quiet)

    def _select_steering(self):
        end = self.suspect_at - self.p.history_offset_s
        start = end - self.p.history_span_s
        samples = [(t, s) for t, s in self.history if start <= t <= end]
        if len(samples) < self.p.history_min_samples:
            return False
        times = [start] + [t for t, _ in samples] + [end]
        if max(b - a for a, b in zip(times, times[1:])) > self.p.history_max_gap_s:
            return False
        value = median(s for _, s in samples) * self.p.recovery_steering_scale
        self.fixed_steering = max(-self.p.recovery_steering_limit,
                                  min(self.p.recovery_steering_limit, value))
        self.selected_interval = (start, end)
        return True

    def step(self, now, mode, command, obs, *, healthy=True, history_stamp=None):
        if not math.isfinite(now):
            return self.fault('invalid_clock')
        dt = 0 if self.last_time is None else now - self.last_time
        self.last_time = now
        if dt < 0 or dt > self.p.max_tick_gap_s:
            return self.fault('clock_or_control_gap')
        if mode not in (0, 1, 2, 3, 4):
            return self.fault('invalid_mode')
        if mode == 3:  # Only STOP rearms a fault/startup, never MANUAL/PROPO.
            self.state, self.reason, self.armed = 'inactive', '', True
            self.history.clear()
            self.last_history_stamp = None
            self.suspect_at = self.stopped_at = self.progress_at = None
            self.attempts, self.rotation = 0, 0.0
            return Command()
        if mode != 1:
            if self.state != 'fault':
                self.state = 'inactive'
            self.history.clear()
            self.last_history_stamp = None
            self.suspect_at = self.stopped_at = self.progress_at = None
            return Command()
        if not self.armed or self.state == 'fault':
            return self.fault(self.reason or 'stop_required')
        if not healthy or not command.valid():
            return self.fault('input_stale_invalid_or_conflict')
        if not obs.valid or not all(math.isfinite(v) and v >= 0 for v in (obs.flow, obs.gyro)):
            return self.fault('observation_unavailable')
        if self.state == 'inactive':
            self._phase('driving', now)
            self.auto_at = now
        if self.active:
            self.rotation += obs.gyro * dt
            if self.rotation > self.p.max_recovery_rotation_rad:
                return self.fault('rotation_limit')
            return self._recover(now, command, obs)
        # Only distinct upstream samples actually selected in normal AUTO are stored.
        stamp = now if history_stamp is None else history_stamp
        if command.throttle >= self.p.min_forward_throttle and command.brake == command.reverse == 0:
            if self.last_history_stamp is None or stamp > self.last_history_stamp:
                self.history.append((now, command.steering))
                self.last_history_stamp = stamp
        retain = self.p.history_span_s + self.p.history_offset_s + self.p.stuck_duration_s + 1
        while self.history and now - self.history[0][0] > retain:
            self.history.popleft()
        moving = obs.flow >= self.p.moving_flow_px_s and obs.gyro <= self.p.stationary_gyro_rad_s
        if moving and command.throttle >= self.p.min_forward_throttle:
            self.progress_at = now if self.progress_at is None else self.progress_at
            if now - self.progress_at >= self.p.progress_reset_s:
                self.attempts = 0
        else:
            self.progress_at = None
        suspect = (now - self.auto_at >= self.p.startup_grace_s
                   and command.throttle >= self.p.min_forward_throttle
                   and command.brake == command.reverse == 0 and self._stationary(obs))
        if not suspect:
            self.suspect_at = None
            self.state = 'driving'
            return command
        if self.suspect_at is None:
            self.suspect_at = now
        self.state = 'suspect'
        if now - self.suspect_at < self.p.stuck_duration_s:
            return command
        if self.attempts >= self.p.max_recovery_attempts:
            return self.fault('attempt_limit')
        if not self._select_steering():
            return self.fault('insufficient_history')
        self.attempts += 1
        self.rotation, self.reverse_progress = 0.0, False
        self._phase('braking', now)
        return Command(brake=self.p.brake_command)

    def _recover(self, now, command, obs):
        elapsed = now - self.phase_at
        if self.state == 'braking':
            # Dwell is visual evidence only, not calibrated physical stop measurement.
            if self._stationary(obs):
                self.stopped_at = now if self.stopped_at is None else self.stopped_at
            else:
                self.stopped_at = None
            if elapsed > self.p.stop_timeout_s:
                return self.fault('stop_not_observed')
            if (elapsed >= self.p.brake_min_s and self.stopped_at is not None
                    and now - self.stopped_at >= self.p.stop_confirm_s):
                self._phase('reverse_neutral', now)
                return Command()
            return Command(brake=self.p.brake_command)
        if self.state == 'reverse_neutral':
            if not self._stationary(obs):
                return self.fault('motion_before_reverse')
            if elapsed >= self.p.reverse_neutral_s:
                self._phase('reversing', now)
                return Command(steering=self.fixed_steering, reverse=self.p.reverse_throttle)
            return Command()
        if self.state == 'reversing':
            self.reverse_progress |= (obs.flow >= self.p.moving_flow_px_s
                                      and obs.gyro <= self.p.stationary_gyro_rad_s)
            if elapsed >= self.p.reverse_duration_s:
                self._phase('resume_neutral', now)
                return Command()
            return Command(steering=self.fixed_steering, reverse=self.p.reverse_throttle)
        if self._stationary(obs):
            self.stopped_at = now if self.stopped_at is None else self.stopped_at
        else:
            self.stopped_at = None
        if elapsed > self.p.resume_neutral_s + self.p.stop_timeout_s:
            return self.fault('stop_after_reverse_not_observed')
        if (elapsed >= self.p.resume_neutral_s and self.stopped_at is not None
                and now - self.stopped_at >= self.p.stop_confirm_s):
            if not self.reverse_progress:
                return self.fault('no_reverse_progress')
            self._phase('driving', now)
            self.auto_at = now
            self.suspect_at = self.progress_at = None
            self.history.clear()
            self.last_history_stamp = None
            return Command()  # Fresh upstream sample must arrive after release.
        return Command()
