import math
import pytest
from kart_recovery.core import Command, Observation, Recovery, Settings

STILL = Observation(True, 0, 0)
MOVING = Observation(True, 15, 0)
FORWARD = Command(steering=.3, throttle=.1)


class Trial:
    def __init__(self, **kwargs):
        self.r = Recovery(Settings(**kwargs))
        self.t = 10.0
        self.step(mode=3)

    def step(self, command=FORWARD, observation=STILL, mode=1, **kwargs):
        self.t += .02
        return self.r.step(self.t, mode, command, observation, **kwargs)

    def until(self, state, observation=STILL, command=FORWARD, limit=400):
        for _ in range(limit):
            out = self.step(command=command, observation=observation)
            if self.r.state == state:
                return out
            assert self.r.state != 'fault', self.r.reason
        pytest.fail(f'Never reached {state}, got {self.r.state}')


def test_full_recovery_same_sign_fixed_steering_and_duration():
    t = Trial()
    out = t.until('braking')
    assert out.brake == t.r.p.brake_command and out.reverse == 0
    assert t.r.fixed_steering == .3
    assert t.r.selected_interval[1] == pytest.approx(t.r.suspect_at - .15)
    t.until('reverse_neutral')
    out = t.until('reversing')
    started = t.t
    assert out == Command(steering=.3, reverse=.08)
    # Changing upstream steering/throttle cannot change the frozen maneuver.
    for _ in range(24):
        out = t.step(Command(-.9, .2), MOVING)
        assert out == Command(steering=.3, reverse=.08)
    t.until('resume_neutral', MOVING, Command(-.9, .2))
    assert t.t - started == pytest.approx(.5, abs=.021)
    assert t.r.reverse_progress
    t.until('driving', STILL)
    assert t.r.attempts == 1


def test_history_excludes_suspect_and_offset_and_clamps():
    t = Trial(recovery_steering_scale=2)
    # Clean pre-contact history followed by a wildly different stuck command.
    while t.t < 11.05:
        t.step(Command(-.4, .1), MOVING)
    t.until('suspect', STILL, Command(.9, .1))
    t.until('braking', STILL, Command(.9, .1))
    assert t.r.fixed_steering == -.5


def test_unknown_or_textureless_is_not_stuck():
    t = Trial()
    out = t.step(observation=Observation(False))
    assert out == Command() and t.r.state == 'fault'
    assert t.r.reason == 'observation_unavailable'


def test_evs_activity_veto_and_rotation_do_not_trigger():
    for observation in (Observation(True, 0, 0, False), Observation(True, 0, .2)):
        t = Trial()
        for _ in range(200):
            t.step(observation=observation)
        assert t.r.state == 'driving' and t.r.attempts == 0


def test_stale_inputs_cancel_reverse_and_stop_is_required():
    t = Trial()
    t.until('reversing')
    assert t.step(healthy=False) == Command()
    assert t.r.state == 'fault'
    t.step(mode=2)
    t.step()
    assert t.r.state == 'fault'
    t.step(mode=3)
    t.step(observation=MOVING)
    assert t.r.state == 'driving'


def test_stop_or_manual_cancels_immediately():
    for mode in (2, 3, 4):
        t = Trial()
        t.until('reversing')
        assert t.step(mode=mode) == Command()
        assert not t.r.active
        assert not t.r.history


def test_startup_auto_and_clock_jumps_require_stop():
    r = Recovery()
    assert r.step(10, 1, FORWARD, STILL) == Command()
    assert r.state == 'fault'
    for jump in (-1, .5):
        t = Trial()
        t.until('reversing')
        assert t.r.step(t.t + jump, 1, FORWARD, STILL) == Command()
        assert t.r.state == 'fault'


def test_no_reverse_movement_and_rotation_limit_fail_closed():
    t = Trial()
    t.until('resume_neutral')
    for _ in range(100):
        t.step()
        if t.r.state == 'fault':
            break
    assert t.r.reason == 'no_reverse_progress'
    t = Trial(max_recovery_rotation_rad=.05)
    t.until('reversing')
    for _ in range(10):
        t.step(observation=Observation(True, 15, 1))
    assert t.r.reason == 'rotation_limit'


def test_repeated_attempts_are_bounded():
    t = Trial(max_recovery_attempts=1)
    t.until('reversing')
    t.until('resume_neutral', MOVING)
    t.until('driving', STILL)
    for _ in range(300):
        t.step()
        if t.r.state == 'fault':
            break
    assert t.r.reason == 'attempt_limit'


def test_repeated_source_sample_cannot_fabricate_history():
    t = Trial()
    for _ in range(200):
        t.step(history_stamp=10)
        if t.r.state == 'fault':
            break
    assert t.r.reason == 'insufficient_history'


@pytest.mark.parametrize('command', [Command(math.nan), Command(reverse=2),
                                      Command(throttle=.1, reverse=.1)])
def test_invalid_commands(command):
    t = Trial()
    assert t.step(command) == Command()
    assert t.r.state == 'fault'


@pytest.mark.parametrize('settings', [{'reverse_throttle': 0}, {'history_span_s': -1},
                                     {'max_recovery_attempts': 0}, {'reverse_duration_s': math.nan},
                                     {'moving_flow_px_s': 1}, {'stop_timeout_s': .1}])
def test_settings_validation(settings):
    with pytest.raises(ValueError):
        Settings(**settings)
