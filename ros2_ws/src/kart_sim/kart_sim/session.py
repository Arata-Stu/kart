"""Monitor requests are drained on the simulation thread, never in HTTP handlers."""
from queue import Queue, Empty
import time
from .autonomy import WaypointController
from .recording import SimRecorder
from .operator_link import OperatorLink


class SimSession:
    def __init__(self, sim, record_dir):
        self.controller = WaypointController(sim)
        self.recorder = SimRecorder(record_dir)
        self.requests = Queue(maxsize=64)
        self.manual = (0., 0., 1., 0.)
        self.error = ''
        self.bev_enabled = False
        self.owns_control = False
        self.operator = OperatorLink()
        self.kart_active = False
        self.kart_requested_at = 0.
        self.invalidate = lambda: None

    def request(self, action, value=None):
        if action not in ('auto_start', 'stop', 'record_start', 'record_stop', 'reset', 'manual',
                          'truth_speed','kart_speed','bev','kart_configure','kart_auto','kart_line','kart_refresh','kart_unload'):
            raise ValueError('Unknown action')
        if action in ('truth_speed', 'kart_speed') and value is not None:
            if type(value) not in (int, float) or not .05 <= value <= 1.5:
                raise ValueError('Trial speed must be 0.05..1.5 m/s, or null to restore')
        if action == 'bev' and type(value) is not bool:
            raise ValueError('bev requires a boolean')
        if action == 'manual':
            if not isinstance(value, list) or len(value) != 2:
                raise ValueError('manual requires [steering, throttle]')
            if any(not isinstance(v, (float, int)) or not -1 <= v <= 1 for v in value):
                raise ValueError('Manual values must be -1..1')
        self.requests.put_nowait((action, value))

    def drain(self, sim):
        while True:
            try:
                action, value = self.requests.get_nowait()
            except Empty:
                break
            try:
                if action == 'truth_speed':
                    self.controller.set_speed(value)
                elif action == 'bev':
                    self.bev_enabled=value
                elif action.startswith('kart_'):
                    if action not in ('kart_refresh', 'kart_speed'):
                        self.controller.stop();self.manual=(0.,0.,1.,0.);self.owns_control=True;self.kart_active=False
                        self.invalidate()
                    self.operator.request(action,value)
                    if action == 'kart_auto':
                        self.kart_active=True;self.kart_requested_at=time.monotonic();self.owns_control=False
                elif action == 'auto_start':
                    self.kart_active=False;self.invalidate()
                    if self.operator.send:self.operator.request('kart_stop')
                    self.controller.start(sim)
                    self.manual = (0., 0., 1., 0.)
                    self.owns_control = True
                elif action == 'stop':
                    self.kart_active=False;self.invalidate()
                    if self.operator.send:self.operator.request('kart_stop')
                    self.owns_control = True
                    self.controller.stop()
                    self.manual = (0., 0., 1., 0.)
                elif action == 'manual':
                    self.kart_active=False;self.invalidate()
                    if self.operator.send:self.operator.request('kart_stop')
                    self.owns_control = True
                    self.controller.stop()
                    steering, throttle = value
                    self.manual = (steering, max(throttle, 0), 0., max(-throttle, 0))
                elif action == 'record_start':
                    self.recorder.start(sim)
                elif action == 'record_stop':
                    self.recorder.stop()
                elif action == 'reset':
                    if self.operator.state.get('running'):
                        raise ValueError('Unload kart bringup before resetting the simulation clock')
                    if self.recorder.writer:
                        raise ValueError('Stop recording before resetting the simulation clock')
                    self.controller.stop()
                    self.owns_control = True
                    self.manual = (0., 0., 1., 0.)
                    sim.reset()
                self.error = ''
            except Exception as exc:
                self.error = str(exc)
        mode_failed=self.operator.state.get('mode',3)!=1 and time.monotonic()-self.kart_requested_at>1.
        if self.kart_active and (not self.operator.ready() or mode_failed):
            self.kart_active=False;self.owns_control=True;self.manual=(0.,0.,1.,0.);self.invalidate()
            self.error='kart readiness/heartbeat lost; stopped'
            if self.operator.send:self.operator.request('kart_stop')

    def output(self, sim):
        return self.controller.output(sim) if self.controller.enabled else self.manual

    def append(self, sim, samples):
        try:
            self.recorder.append(sim, samples)
        except Exception as exc:
            self.recorder.error = str(exc)
            self.error = 'Recording failed: '+str(exc)
            try:
                self.recorder.stop()
            except Exception as close_error:
                self.error += '; finalization failed: '+str(close_error)

    def status(self):
        return dict(autodrive=self.controller.status(), bag=self.recorder.status(), error=self.error,
                    manual_command=list(self.manual), bev_enabled=self.bev_enabled,
                    control_owner='kart' if self.kart_active else 'truth_auto' if self.controller.enabled else 'monitor' if self.owns_control else 'external',
                    operator=self.operator.status())

    def close(self):
        self.controller.stop()
        self.recorder.stop()
