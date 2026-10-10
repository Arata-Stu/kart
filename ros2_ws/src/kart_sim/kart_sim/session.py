"""Monitor requests are drained on the simulation thread, never in HTTP handlers."""
from queue import Queue, Empty
from .autonomy import WaypointController
from .recording import SimRecorder


class SimSession:
    def __init__(self, sim, record_dir):
        self.controller = WaypointController(sim)
        self.recorder = SimRecorder(record_dir)
        self.requests = Queue(maxsize=64)
        self.manual = (0., 0., 1., 0.)
        self.error = ''
        self.owns_control = False

    def request(self, action, value=None):
        if action not in ('auto_start', 'stop', 'record_start', 'record_stop', 'reset', 'manual'):
            raise ValueError('Unknown action')
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
                if action == 'auto_start':
                    self.controller.start(sim)
                    self.manual = (0., 0., 1., 0.)
                    self.owns_control = True
                elif action == 'stop':
                    self.owns_control = True
                    self.controller.stop()
                    self.manual = (0., 0., 1., 0.)
                elif action == 'manual':
                    self.owns_control = True
                    self.controller.stop()
                    steering, throttle = value
                    self.manual = (steering, max(throttle, 0), 0., max(-throttle, 0))
                elif action == 'record_start':
                    self.recorder.start(sim)
                elif action == 'record_stop':
                    self.recorder.stop()
                elif action == 'reset':
                    if self.recorder.writer:
                        raise ValueError('Stop recording before resetting the simulation clock')
                    self.controller.stop()
                    self.owns_control = True
                    self.manual = (0., 0., 1., 0.)
                    sim.reset()
                self.error = ''
            except Exception as exc:
                self.error = str(exc)

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
        return dict(autodrive=self.controller.status(), bag=self.recorder.status(), error=self.error)

    def close(self):
        self.controller.stop()
        self.recorder.stop()
