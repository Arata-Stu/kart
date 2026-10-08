"""ROS-independent, ordered native RAW service lifecycle for a bag session."""
from collections import deque
from pathlib import Path
import time


class RawRelay:
    def __init__(self, submit, *, timeout=5.0, clock=time.monotonic):
        self.submit, self.timeout, self.clock = submit, timeout, clock
        self.queue = deque()
        self.steps = deque()
        self.future = None
        self.deadline = 0.0
        self.owns_raw = False
        self.state = 'idle'
        self.last_event = 'RAW relay idle'
        self.last_error = ''

    def request(self, command, label=''):
        if command == 'MARK':
            self.last_event = f'RAW mark: {label}'
            return
        if command == 'START' and not Path(label).is_absolute():
            self.last_error = 'RAW START requires an absolute session directory'
            self.last_event, self.state = self.last_error, 'error'
            return
        if len(self.queue) >= 16:
            # Keep a stop request even when repeated start/split requests flood the queue.
            if command == 'STOP':
                self.queue.clear()
            else:
                self.last_error = 'RAW request queue full'
                self.last_event, self.state = self.last_error, 'error'
                return
        self.queue.append((command, label))

    @property
    def pending(self):
        return bool(self.queue or self.steps or self.future)

    def _fail(self, reason, *, uncertain=False):
        self.last_error = reason
        self.last_event, self.state = reason, 'error'
        operation = self.steps[0][0] if self.steps else ''
        if operation == 'start' and not uncertain:
            self.owns_raw = False
        self.steps.clear()
        self.future = None
        # Do not stop a manually started RAW writer after a rejected directory change.
        if self.owns_raw and operation != 'stop':
            self.queue.appendleft(('STOP', 'cleanup after failure'))

    def tick(self):
        now = self.clock()
        if self.future is not None:
            if self.future.done():
                operation = self.steps[0][0]
                try:
                    success, message = self.future.result()
                except Exception as error:
                    self._fail(f'RAW {operation}: {error}', uncertain=True)
                    return
                if not success:
                    self._fail(f'RAW {operation} rejected: {message}')
                    return
                self.steps.popleft()
                self.future = None
                if operation == 'start':
                    self.owns_raw, self.state = True, 'recording'
                elif operation == 'stop':
                    self.owns_raw, self.state = False, 'idle'
                self.last_event = f'RAW {operation} acknowledged: {message}'
                self.deadline = now + self.timeout
            elif now >= self.deadline:
                self._fail('RAW service response timeout; writer state unverified', uncertain=True)
                return
            else:
                return
        if not self.steps and self.queue:
            command, label = self.queue.popleft()
            if command == 'START':
                if self.owns_raw:
                    self.last_event = 'RAW START ignored: session already active'
                    return
                self.last_error = ''
                self.state = 'starting'
                self.steps.extend([('set_directory', label), ('start', '')])
            elif command == 'STOP':
                if not self.owns_raw:
                    self.last_event = 'RAW STOP ignored: relay does not own a writer'
                    return
                self.state = 'stopping'
                self.steps.append(('stop', ''))
            elif command == 'SPLIT' and self.owns_raw:
                self.steps.append(('split', ''))
            else:
                self.last_event = f'RAW {command} ignored: no owned session'
                return
            self.deadline = now + self.timeout
        if self.steps:
            operation, argument = self.steps[0]
            try:
                future = self.submit(operation, argument)
            except Exception as error:
                self._fail(f'RAW {operation} submit failed: {error}')
                return
            if future is None:
                if now >= self.deadline:
                    self._fail(f'RAW {operation} service unavailable')
                return
            if operation == 'start':
                self.owns_raw = True  # A sent START is uncertain until its response arrives.
            self.future = future
            self.deadline = now + self.timeout
