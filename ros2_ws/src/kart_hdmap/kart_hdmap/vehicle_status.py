"""Bounded telemetry cache; ROS stamps and local receipt must both be fresh."""
import json
import math
import time

STATES = {'inactive', 'driving', 'suspect', 'braking', 'reverse_neutral',
          'reversing', 'post_reverse_pulse', 'post_reverse_braking', 'resume_neutral', 'fault'}


class VehicleStatus:
    def __init__(self):
        self.samples = {}

    def recovery(self, raw):
        self.samples.pop('recovery', None)
        try:
            if len(raw) > 65536:
                return
            data = json.loads(raw)
            stamp = data['stamp']
            if (data['schema'] != 'kart.recovery.v1' or data['state'] not in STATES or
                    type(stamp) not in (float, int) or not math.isfinite(stamp) or
                    type(data['attempts']) is not int or data['attempts'] < 0 or
                    not isinstance(data['reason'], str)):
                return
            self.samples['recovery'] = (stamp, time.monotonic(), dict(
                state=data['state'], reason=data['reason'][:200], attempts=data['attempts']))
        except (ValueError, KeyError, TypeError):
            pass

    def bridge(self, stamp, statuses):
        matches = [s for s in statuses if s.name == 'kart_bridge' and s.hardware_id == 'JPBB-01']
        if not matches:
            return
        self.samples.pop('bridge', None)
        if len(matches) != 1:
            return
        status = matches[0]
        # ROS byte fields can arrive as a single raw byte rather than an int.
        level = status.level
        if isinstance(level, bytes) and len(level) == 1:
            level = level[0]
        if type(level) is not int or not 0 <= level <= 3:
            return
        values = {v.key: v.value for v in status.values}
        fields = {k: str(values[k])[:200] for k in (
            'host_active', 'host_phase', 'command_fresh', 'fault_bits', 'selector',
            'esc_command_state', 'esc_negative_action', 'esc_brake_rejected', 'steering_trim', 'steering_offset', 'steering_scale') if k in values}
        self.samples['bridge'] = (stamp, time.monotonic(), dict(
            level=level, message=status.message[:200], values=fields))

    def racing(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples['racing'] = (stamp, time.monotonic(), dict(
            state=msg.state[:64], reason=msg.reason[:160], valid=msg.valid,
            valid_for_sec=msg.valid_for_sec))

    def view(self, now, publishers):
        result = {}
        for key, limit in [('bridge', 2.5), ('recovery', .8), ('racing', .5)]:
            count = publishers.get(key, 0)
            sample = self.samples.get(key)
            state = 'absent' if count == 0 else 'conflict' if count != 1 else 'waiting'
            if count == 1 and sample:
                stamp, received, data = sample
                if key == 'racing' and not data['valid']:
                    # Faults often carry the stale input stamp that caused rejection.
                    if 0 <= time.monotonic()-received <= limit:
                        result[key] = dict(availability='fresh', **data)
                        continue
                    state = 'stale'
                if key == 'racing':
                    limit = min(data['valid_for_sec'], limit)
                if stamp > 0 and (0 if key == 'racing' else -.05) <= now - stamp <= limit and 0 <= time.monotonic()-received <= limit:
                    result[key] = dict(availability='fresh', **data)
                    continue
                state = 'stale'
            result[key] = dict(availability=state)
        return result
