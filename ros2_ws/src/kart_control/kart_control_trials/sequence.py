"""Bounded, one-shot signed ESC sequence. All times are monotonic seconds."""
import math


def validate_steps(rows, limit=0.3):
    if not isinstance(rows, list) or not 1 <= len(rows) <= 32:
        raise ValueError('1〜32段階を指定してください')
    result = []
    for row in rows:
        if not isinstance(row, dict) or set(row) != {'output', 'seconds'}:
            raise ValueError('各段階はoutputとsecondsで指定してください')
        u, duration = row['output'], row['seconds']
        if any(type(x) not in (int, float) or not math.isfinite(x) for x in (u, duration)):
            raise ValueError('有限の数値を指定してください')
        if abs(u) > limit or not 0.02 <= duration <= 20:
            raise ValueError(f'出力は±{limit}以内、時間は0.02〜20秒です')
        result.append(dict(output=float(u), seconds=float(duration)))
    if sum(x['seconds'] for x in result) > 60:
        raise ValueError('試験時間は合計60秒以内です')
    return result


class Sequence:
    def __init__(self):
        self.phase = 'idle'
        self.reason = ''
        self.run_id = None
        self.rows = []
        self.index = -1
        self.output = 0.0
        self.started = self.stage_at = self.last_tick = self.lease_at = 0.0

    @property
    def active(self):
        return self.phase in ('arming', 'running')

    def start(self, run_id, rows, now):
        if self.active:
            raise ValueError('試験は既に実行中です')
        self.run_id, self.rows = run_id, rows
        self.phase, self.reason, self.index, self.output = 'arming', '', -1, 0.0
        self.started = self.stage_at = self.last_tick = self.lease_at = now

    def stop(self, reason='ui_stop'):
        self.output = 0.0
        self.phase = 'complete' if reason == 'complete' else 'stopped'
        self.reason = reason

    def heartbeat(self, run_id, now):
        if self.active and run_id == self.run_id:
            self.lease_at = now

    def tick(self, now, ready, healthy, lease_s=3.0):
        if not self.active:
            return
        if not math.isfinite(now) or now < self.last_tick or now-self.last_tick > 0.1:
            self.stop('control_deadline')
        elif now-self.lease_at > lease_s:
            self.stop('ui_timeout')
        elif not healthy:
            self.stop('mode_board_or_publisher_fault')
        elif self.phase == 'arming':
            if now-self.started > 2.0:
                self.stop('arming_timeout')
            elif ready and now-self.started >= 0.3:
                self.phase, self.index, self.stage_at = 'running', 0, now
                self.output = self.rows[0]['output']
        elif now-self.stage_at >= self.rows[self.index]['seconds']:
            # Never skip a short neutral stage after a late callback.
            self.index += 1
            if self.index == len(self.rows):
                self.stop('complete')
            else:
                self.stage_at = now
                self.output = self.rows[self.index]['output']
        self.last_tick = now
