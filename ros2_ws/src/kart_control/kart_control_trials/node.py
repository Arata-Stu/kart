"""Explicit opt-in ESC bench/trial runner. HTTP is loopback-only; ROS owns all state."""
import json
import math
import queue
import threading
import time
import uuid
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Int32MultiArray, UInt8, String
from kart_interfaces.msg import ControlCommand, OperationModeRequest, OperationModeState
from .sequence import Sequence, validate_steps


class TrialNode(Node):
    def __init__(self):
        super().__init__('esc_trial')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('ESC trials require use_sim_time=false')
        def param(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=not any(p.name == "use_sim_time" and p.value for p in params),
            reason="ESC trials require the real clock"))
        self.limit = param('max_output', 0.3)
        self.lease_s = param('ui_timeout_s', 3.0)
        port = param('http_port', 8783)
        if not math.isfinite(self.limit) or not 0 < self.limit <= 1:
            raise ValueError('max_output must be 0..1 (exclusive zero)')
        if not math.isfinite(self.lease_s) or not 1 <= self.lease_s <= 5:
            raise ValueError('ui_timeout_s must be 1..5')
        if not 1024 <= port <= 65535:
            raise ValueError('invalid http_port')
        self.model = Sequence()
        self.last_sequence = ('idle', -1, 0.0, '')
        self.requests = queue.Queue(maxsize=16)
        self.stop_event = threading.Event()
        self.stream = str(uuid.uuid4())
        self.events = deque(maxlen=6000)
        self.seq = 0
        self.generation = 0
        self.used_ids = set()
        self.mode = None
        self.mode_source = ''
        self.mode_at = self.diag_at = self.pwm_at = self.path_at = -math.inf
        self.mode_stamp = 0.0
        self.diag = {}
        self.diag_ok = False
        self.pwm = None
        self.path = None
        self.auto_sent = False
        self.last_sample = self.last_stop = -math.inf
        self.command = self.create_publisher(ControlCommand, 'auto/control_cmd', 1)
        self.mode_request = self.create_publisher(OperationModeRequest, 'operation_mode/request', 10)
        self.record = self.create_publisher(String, 'control/esc_trial_events', 100)
        self.create_subscription(OperationModeState, 'operation_mode/state', self.on_mode, 10)
        self.create_subscription(DiagnosticArray, 'diagnostics', self.on_diag, 100)
        self.create_subscription(Int32MultiArray, 'vehicle/output_channels', self.on_pwm, 100)
        self.create_subscription(UInt8, 'vehicle/active_path', self.on_path, 100)
        self.timer = self.create_timer(0.01, self.tick)
        node = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_POST(self):
                self.connection.settimeout(2)
                try:
                    if self.path not in ('/status', '/start', '/stop'):
                        raise ValueError('Unknown action')
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 16384:
                        raise ValueError('Invalid request size')
                    body = json.loads(self.rfile.read(length))
                    if not isinstance(body, dict):
                        raise ValueError('Expected object')
                    if self.path == '/stop':
                        # Independent of the normal queue; cancels queued starts too.
                        node.stop_event.set()
                        response = {'stop_requested': True, 'confirmed': False}
                    else:
                        reply = queue.Queue(maxsize=1)
                        node.requests.put_nowait((self.path, body, reply, time.monotonic()+0.5))
                        response = reply.get(timeout=1)
                    raw = json.dumps(response, allow_nan=False).encode()
                    self.send_response(200)
                except (ValueError, queue.Empty, queue.Full) as exc:
                    raw = json.dumps({'error': str(exc) or 'Trial request expired'}).encode()
                    self.send_response(400)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers()
                try:
                    self.wfile.write(raw)
                except OSError:
                    pass
        self.server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def event(self, kind, data):
        self.seq += 1
        row = dict(seq=self.seq, mono_s=time.monotonic(), ros_stamp_ns=str(self.get_clock().now().nanoseconds),
                   kind=kind, data=data, run_id=self.model.run_id)
        self.events.append(row)
        msg = String(); msg.data = json.dumps(row, allow_nan=False)
        self.record.publish(msg)

    def on_mode(self, msg):
        if (self.mode, self.mode_source) != (msg.mode, msg.source):
            self.event('mode', dict(mode=msg.mode, source=msg.source))
        self.mode, self.mode_source = msg.mode, msg.source
        self.mode_at = time.monotonic()
        self.mode_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.model.active and (msg.mode not in (1, 3) or
                (self.auto_sent and msg.mode == 3) or
                (msg.mode == 1 and msg.source != 'esc_trial:'+str(self.model.run_id))):
            self.model.stop('external_mode_change')

    def on_diag(self, msg):
        for row in msg.status:
            if row.name != 'kart_bridge':
                continue
            values = {v.key: v.value for v in row.values}
            keys = ('host_active', 'host_phase', 'esc_command_state', 'esc_negative_action',
                    'esc_brake_rejected', 'fault_bits', 'selector', 'command_fresh')
            current = {k: values.get(k) for k in keys}
            current.update(level=row.level, message=row.message)
            if current != self.diag:
                self.event('bridge', current)
            self.diag, self.diag_ok, self.diag_at = current, row.level == 0, time.monotonic()

    def on_pwm(self, msg):
        values = list(msg.data)
        if len(values) != 2:
            return
        if values != self.pwm:
            self.event('pwm', dict(servo_us=values[0], esc_us=values[1]))
        self.pwm, self.pwm_at = values, time.monotonic()

    def on_path(self, msg):
        if msg.data != self.path:
            self.event('board_path', dict(path=msg.data))
        self.path, self.path_at = msg.data, time.monotonic()

    def healthy(self, now):
        ros = self.get_clock().now().nanoseconds*1e-9
        return (now-self.mode_at <= 0.3 and 0 <= ros-self.mode_stamp <= 0.3 and
                now-self.diag_at <= 0.3 and now-self.pwm_at <= 0.2 and now-self.path_at <= 0.2 and
                self.diag_ok and self.diag.get('selector') == '1' and
                self.diag.get('fault_bits') == '0' and self.path in (0, 2) and
                self.count_publishers('auto/control_cmd') == 1 and
                self.count_publishers('vehicle/control_cmd') == 1 and
                self.count_publishers('operation_mode/state') == 1)

    def request_mode(self, mode):
        msg = OperationModeRequest(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = mode; msg.source = 'esc_trial:'+str(self.model.run_id)
        self.mode_request.publish(msg)

    def snapshot(self, body, now):
        after = body.get('after', 0)
        if type(after) is not int or after < 0:
            raise ValueError('Invalid event cursor')
        reset = body.get('stream') != self.stream or after > self.seq or (self.events and after < self.events[0]['seq']-1)
        return dict(stream=self.stream, generation=self.generation, reset=bool(reset), cursor=self.seq,
                    events=[e for e in self.events if reset or e['seq'] > after],
                    phase=self.model.phase, reason=self.model.reason, run_id=self.model.run_id,
                    index=self.model.index, output=self.model.output, steps=self.model.rows,
                    elapsed_s=max(0, now-self.model.started) if self.model.run_id else 0,
                    mode=self.mode, bridge=self.diag, pwm=self.pwm, board_path=self.path,
                    healthy=self.healthy(now), max_output=self.limit, ui_timeout_s=self.lease_s,
                    ages_s=dict(mode=now-self.mode_at if math.isfinite(self.mode_at) else None,
                                bridge=now-self.diag_at if math.isfinite(self.diag_at) else None,
                                pwm=now-self.pwm_at if math.isfinite(self.pwm_at) else None))

    def tick(self):
        now = time.monotonic()
        cancelled = self.stop_event.is_set()
        if cancelled:
            self.stop_event.clear()
            self.model.stop('ui_stop')
            self.generation += 1
            self.request_mode(3)
            self.event('stop', {'reason': 'ui_stop'})
        # Bounded per tick; STOP wins over all requests already waiting.
        for _ in range(4):
            try:
                action, body, reply, deadline = self.requests.get_nowait()
            except queue.Empty:
                break
            try:
                if now > deadline:
                    raise ValueError('Request expired; not executed')
                if action == '/start':
                    if cancelled or self.stop_event.is_set():
                        raise ValueError('STOP cancelled start')
                    if body.get('generation') != self.generation or body.get('stream') != self.stream:
                        raise ValueError('Trial session changed; refresh before starting')
                    run_id = body.get('run_id')
                    if not isinstance(run_id, str) or str(uuid.UUID(run_id)) != run_id:
                        raise ValueError('A UUID run_id is required')
                    if run_id in self.used_ids:
                        raise ValueError('This run_id has already been used; do not retry')
                    if len(self.used_ids) >= 10000:
                        raise ValueError('Restart trial node before further trials')
                    rows = validate_steps(body.get('steps'), self.limit)
                    if self.mode != 3 or not self.healthy(now) or self.path != 0:
                        raise ValueError('Fresh STOP, healthy board in HOST selection, and exclusive trial launch required')
                    self.model.start(run_id, rows, now)
                    self.used_ids.add(run_id); self.auto_sent = False
                    self.event('start', {'steps': rows})
                else:
                    self.model.heartbeat(body.get('run_id'), now)
                reply.put_nowait(self.snapshot(body, now))
            except (ValueError, TypeError, AttributeError) as exc:
                reply.put_nowait({'error': str(exc)})
        if cancelled:
            # Drain beyond the per-tick quota: no pending start can survive STOP.
            while True:
                try:
                    _, _, reply, _ = self.requests.get_nowait()
                    reply.put_nowait({'error': 'STOP cancelled pending request'})
                except queue.Empty:
                    break
        healthy = self.healthy(now) and self.mode in (1, 3)
        ready = self.mode == 1 and self.path == 2 and self.diag.get('host_phase') == '3'
        self.model.tick(now, ready, healthy, self.lease_s)
        if self.model.phase == 'running' and not ready:
            self.model.stop('host_authority_lost')
        # No Joy pulse insertion: positive -> throttle, negative -> reverse.
        # Existing bridge models whether the negative input is braking/reversing.
        msg = ControlCommand(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.throttle = max(0., self.model.output); msg.reverse = max(0., -self.model.output)
        self.command.publish(msg)
        if self.model.phase == 'arming' and not self.auto_sent and now-self.model.started >= 0.05:
            self.request_mode(1); self.auto_sent = True
        if not self.model.active and self.model.run_id and now-self.last_stop >= 0.1:
            self.request_mode(3); self.last_stop = now
        current = (self.model.phase, self.model.index, self.model.output, self.model.reason)
        if current != self.last_sequence:
            self.last_sequence = current
            self.event('sequence', dict(phase=current[0], index=current[1], output=current[2], reason=self.model.reason))
        if now-self.last_sample >= 0.05:
            self.event('sample', dict(phase=self.model.phase, index=self.model.index,
                       output=self.model.output, pwm=self.pwm, esc=self.diag.get('esc_command_state'),
                       board_path=self.path, healthy=healthy))
            self.last_sample = now


def main():
    rclpy.init()
    node = TrialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.model.stop('shutdown')
        node.request_mode(3)
        node.server.shutdown()
        node.destroy_node()
        rclpy.shutdown()
