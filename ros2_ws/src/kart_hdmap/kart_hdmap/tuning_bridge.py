"""Optional loopback HTTP adapter. All ROS mutations run in the executor thread."""
import copy
import json
import math
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .tuning import MAX_BYTES, StopGate, digest, encoded, map_identity, validate


class TuningBridge:
    def __init__(self, node):
        from kart_interfaces.msg import OperationModeState, TrackingReference
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from .vehicle_status import VehicleStatus
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer, TransformListener
        from pathlib import Path
        root = Path(node.settings['tuning_map_dir'])
        if not node.settings['tuning_map_dir'] or not root.is_absolute():
            raise ValueError('tuning_map_dir must be the absolute localization bundle root')
        if not Path(node.settings['map_file']).resolve().is_relative_to(root.resolve()):
            raise ValueError('map_file must belong to tuning_map_dir')
        frame = node.settings['tuning_tracking_frame']
        if not frame or frame.startswith('/') or frame == node.settings['frame_id']:
            raise ValueError('Invalid tuning_tracking_frame')
        if node.settings['frame_id'] != 'map':
            raise ValueError('Tuning requires map-frame geometry')
        self.node = node
        self.identity = map_identity(node.settings['tuning_map_dir'])
        self.gate = StopGate()
        self.vehicle = VehicleStatus()
        self.previous = None
        self.pose = None
        self.tracking = None
        self.localized = False
        self.localized_at = float('-inf')
        self.requests = queue.Queue(maxsize=4)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.subscriptions = [
            node.create_subscription(String, 'recovery/status', lambda m: self.vehicle.recovery(m.data), 1),
            node.create_subscription(DiagnosticArray, 'diagnostics', self.vehicle_diagnostics, 10),
            node.create_subscription(TrackingReference, 'control/tracking_reference', self.reference, 1),
            node.create_subscription(OperationModeState, 'operation_mode/state', self.mode, 1),
            node.create_subscription(Odometry, 'visual_slam/tracking/odometry', self.speed, qos_profile_sensor_data),
            node.create_subscription(DiagnosticArray, 'localization/vslam/diagnostics', self.localization, 10),
        ]
        from .parameter_bridge import ParameterBridge
        self.parameters = ParameterBridge(self)
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_POST(self):
                try:
                    if self.headers.get('Host') != '127.0.0.1:8782' or self.headers.get('Origin'):
                        raise ValueError('Local SSH transport required')
                    if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                        raise ValueError('JSON required')
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= MAX_BYTES + 4096:
                        raise ValueError('Invalid request size')
                    self.connection.settimeout(3)
                    body = json.loads(self.rfile.read(size))
                    if not isinstance(body, dict):
                        raise ValueError('Object required')
                    reply = queue.Queue(maxsize=1)
                    bridge.requests.put_nowait((self.path, body, reply, time.monotonic() + 4))
                    result = reply.get(timeout=5)
                except Exception as exc:
                    result = {'error': str(exc)}
                raw = encoded(result)
                self.send_response(400 if 'error' in result else 200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

        self.server = ThreadingHTTPServer(('127.0.0.1', 8782), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.timer = node.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def fresh(self, stamp, limit):
        from rclpy.time import Time
        age = (self.node.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        return Time.from_msg(stamp).nanoseconds > 0 and -.05 <= age <= limit

    def mode(self, msg):
        if self.fresh(msg.header.stamp, .3):
            self.gate.update_mode(msg.mode)
        else:
            self.gate.mode = None

    def speed(self, msg):
        v = msg.twist.twist.linear
        speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) if self.fresh(msg.header.stamp, .5) else None
        self.gate.update_speed(speed)

    def reference(self, msg):
        self.tracking = None
        points = (msg.vehicle_position, msg.target)
        values = [v for p in points for v in (p.x, p.y, p.z)] + [msg.lookahead_m, msg.target_distance_m]
        if (msg.valid and msg.header.frame_id == self.node.settings['frame_id'] and
                self.fresh(msg.header.stamp, .5) and all(math.isfinite(v) for v in values) and
                msg.lookahead_m > 0 and msg.target_distance_m >= 0):
            self.tracking = (msg, time.monotonic())

    def tracking_status(self):
        if self.tracking is None:
            return None
        msg, received = self.tracking
        if time.monotonic() - received > .5 or not self.fresh(msg.header.stamp, .5):
            return None
        return dict(origin=[msg.vehicle_position.x, msg.vehicle_position.y],
                    target=[msg.target.x, msg.target.y], lookahead_m=msg.lookahead_m,
                    target_distance_m=msg.target_distance_m)

    def vehicle_diagnostics(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.vehicle.bridge(stamp, msg.status)

    def localization(self, msg):
        for status in msg.status:
            if status.hardware_id == 'visual_slam':
                values = {v.key: v.value for v in status.values}
                self.localized = (self.fresh(msg.header.stamp, 1.5) and
                                  values.get('localized_in_exist_map') == 'Yes' and values.get('vo_status') == 'OK')
                self.localized_at = time.monotonic()

    def active(self):
        n = self.node
        return dict(document=n.document, lane_id=n.selected, line_type=n.line_type,
                    centerline_speed_mps=n.runtime_centerline_speed,
                    speed_override_mps=n.settings['speed_override_mps'])

    def revision(self):
        return digest(self.active())

    def issue(self):
        if self.node.count_publishers('planning/reference_line') != 1:
            return '参照ラインの配信元が競合しています'
        return self.gate.issue(allow_running=True)

    def status(self, known):
        now = time.monotonic()
        revision = self.revision()
        result = dict(map_id=self.identity, revision=revision,
                      mode=self.gate.mode if now - self.gate.mode_at <= .3 else None,
                      speed_mps=self.gate.speed if now - self.gate.speed_at <= .5 else None,
                      pose=self.pose, tracking_reference=self.tracking_status(), localized=self.localized and now - self.localized_at <= 1.5,
                      apply_issue=self.issue(), stop_issue=self.gate.issue(),
                      can_rollback=self.previous is not None)
        if hasattr(self, 'vehicle'):
            result['vehicle'] = self.vehicle.view(self.node.get_clock().now().nanoseconds / 1e9,
                dict(bridge=int(self.node.count_publishers('diagnostics') > 0),
                     recovery=self.node.count_publishers('recovery/status')))
        if revision != known:
            result['active'] = self.active()
        return result

    def replace(self, candidate):
        n = self.node
        # Suppress intermediate publication until the whole state is committed.
        from rclpy.parameter import Parameter
        n.tuning_transaction = True
        try:
            result = n.set_parameters_atomically([
                Parameter('line_type', value=candidate['line_type']),
                Parameter('speed_override_mps', value=float(candidate['speed_override_mps'])),
                Parameter('centerline_speed_mps', value=float(candidate['centerline_speed_mps'])),
            ])
            if not result.successful:
                raise ValueError(result.reason)
            n.document = candidate['document']
            n.selected = candidate['lane_id']
        finally:
            n.tuning_transaction = False
        self.tracking = None
        n.runtime_centerline_speed = candidate['centerline_speed_mps']
        n.publish()

    def tick(self):
        from rclpy.time import Time
        self.parameters.tick()
        n = self.node
        self.pose = None
        try:
            tf = self.buffer.lookup_transform(n.settings['frame_id'], n.settings['tuning_tracking_frame'], Time())
            t, q = tf.transform.translation, tf.transform.rotation
            norm = sum(v*v for v in (q.x, q.y, q.z, q.w))
            if self.fresh(tf.header.stamp, .5) and all(math.isfinite(v) for v in (t.x, t.y, norm)) and abs(norm-1) < .01:
                self.pose = dict(x=t.x, y=t.y, yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        except Exception:
            pass
        try:
            action, body, reply, deadline = self.requests.get_nowait()
        except queue.Empty:
            return
        if action in ('/parameters_get', '/parameters_set'):
            self.parameters.begin(action, body, reply, deadline)
        else:
            reply.put(self.handle(action, body, deadline))

    def handle(self, action, body, deadline):
        """Validate and commit one request; called only by the ROS executor."""
        try:
            if time.monotonic() > deadline:
                raise ValueError('Request expired; retry status')
            if body.get('map_id') != self.identity:
                raise ValueError('NotebookとJetsonの地図が一致しません')
            if action not in ('/status', '/apply', '/rollback'):
                raise ValueError('Unknown tuning action')
            if action != '/status':
                parameters = getattr(self, 'parameters', None)
                if parameters is not None and parameters.pending is not None:
                    raise ValueError('パラメータ処理中です。結果確認後に地図を適用してください')
                issue = self.issue()
                if issue:
                    raise ValueError(issue)
                if body.get('expected_revision') != self.revision():
                    raise ValueError('適用版が変わりました。状態を再取得してください')
                old = copy.deepcopy(self.active())
                if action == '/apply':
                    snapshot = body['snapshot']
                    document = validate(snapshot, self.identity)
                    candidate = dict(document=document, lane_id=snapshot['lane_id'],
                                     line_type=snapshot['line_type'], centerline_speed_mps=snapshot['centerline_speed_mps'],
                                     speed_override_mps=-1.0)
                else:
                    if self.previous is None:
                        raise ValueError('戻せる版がありません')
                    candidate = self.previous
                # Recheck after validation; mode/speed evidence must remain fresh.
                issue = self.issue()
                if issue:
                    raise ValueError(issue)
                try:
                    self.replace(candidate)
                except Exception:
                    self.replace(old)
                    raise
                self.previous = old
            return self.status(body.get('known_revision'))
        except Exception as exc:
            return {'error': str(exc)}

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
