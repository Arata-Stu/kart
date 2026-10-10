"""ROS adapter for the JetPilot-derived recorder lifecycle."""
from dataclasses import fields
import time
import signal
from threading import Event

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.task import Future
from .raw_relay import RawRelay
from kart_interfaces.msg import BagRequest, BagStatus
from .recorder import Recorder, Settings


class BagManagerNode(Node):
    def __init__(self):
        super().__init__('kart_bag_manager')
        defaults = Settings()
        values = {}
        for field in fields(defaults):
            value = getattr(defaults, field.name)
            initial = Parameter.Type.STRING_ARRAY if isinstance(value, list) and not value else value
            parameter = self.declare_parameter(field.name, initial, ParameterDescriptor(read_only=True))
            values[field.name] = ([] if parameter.value is None else parameter.value)
        self.settings = Settings(**values)
        # Resolve topic selections exactly like this manager's namespace; absolute names remain global.
        self.settings.topics = [self.resolve_topic_name(t) for t in self.settings.topics]
        self.settings.exclude_topics = [self.resolve_topic_name(t) for t in self.settings.exclude_topics]
        raw_topic = self.settings.raw_recording_request_topic
        if raw_topic and self.resolve_topic_name(raw_topic) == self.resolve_topic_name('bag/request'):
            raise ValueError('raw_recording_request_topic must not loop back into bag/request')
        self.recorder = Recorder(self.settings)
        self.request_sub = self.create_subscription(BagRequest, 'bag/request', self.handle_request, 10)
        self.status_pub = self.create_publisher(BagStatus, 'bag/status', 10)
        self.raw_pub = self.create_publisher(BagRequest, raw_topic, 10) if raw_topic else None
        self.raw_relay = None
        self.raw_clients = {}
        self.raw_diagnostics_pub = None
        driver = self.settings.raw_recording_driver_node.rstrip('/')
        if driver:
            self.raw_clients['set_directory'] = self.create_client(
                SetParametersAtomically, driver + '/set_parameters_atomically')
            # Services are relative to the driver's namespace, not its node name.
            prefix = driver.rsplit('/', 1)[0]
            for operation in ('start', 'stop', 'split'):
                self.raw_clients[operation] = self.create_client(
                    Trigger, prefix + '/' + operation + '_raw_recording')
            self.raw_relay = RawRelay(self.submit_raw_service,
                timeout=self.settings.raw_recording_service_timeout_s)
            self.raw_diagnostics_pub = self.create_publisher(DiagnosticArray, 'bag/raw_diagnostics', 10)
        self._announced = False
        self._next_split = 0.0
        self._last_status = 0.0
        self._last_event = ''
        self._last_raw_event = ''
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.update, clock=self._steady)
        self.get_logger().info(f'Recorder idle; output_dir={self.recorder.output_dir}; L1 START / R1 STOP')

    def handle_request(self, request):
        if request.command == BagRequest.START:
            self.recorder.start(request.label)
        elif request.command == BagRequest.STOP:
            self.recorder.stop()
        elif request.command == BagRequest.SPLIT:
            self.recorder.last_event = 'manual split ignored: use recording_split_duration_s'
        elif request.command == BagRequest.MARK:
            self.recorder.last_event = f'mark: {request.label}'
            self.send_raw(BagRequest.MARK, request.label)
        else:
            self.recorder.last_event = f'unknown request: {request.command}'
        self.update(force_status=True)

    def submit_raw_service(self, operation, argument):
        client = self.raw_clients[operation]
        if not client.service_is_ready():
            return None
        request = SetParametersAtomically.Request() if operation == 'set_directory' else Trigger.Request()
        if operation == 'set_directory':
            request.parameters = [Parameter('raw_recording_dir', value=argument).to_parameter_msg()]
        normalized = Future()
        source = client.call_async(request)
        def completed(future):
            try:
                response = future.result()
                if operation == 'set_directory':
                    normalized.set_result((response.result.successful, response.result.reason))
                else:
                    normalized.set_result((response.success, response.message))
            except Exception as error:
                normalized.set_exception(error)
        source.add_done_callback(completed)
        return normalized

    def send_raw(self, command, label):
        if self.raw_relay is not None:
            names = {BagRequest.START: 'START', BagRequest.STOP: 'STOP',
                     BagRequest.SPLIT: 'SPLIT', BagRequest.MARK: 'MARK'}
            self.raw_relay.request(names[command], label)
            self.raw_relay.tick()
        if self.raw_pub is not None:
            msg = BagRequest()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'kart_bag_manager'
            msg.command, msg.label = command, label
            self.raw_pub.publish(msg)

    def update(self, force_status=False):
        self.recorder.tick()
        if self.raw_relay is not None:
            self.raw_relay.tick()
        if self.raw_relay is not None and self.raw_relay.last_event != self._last_raw_event:
            self._last_raw_event = self.raw_relay.last_event
            log = self.get_logger().error if self.raw_relay.last_error else self.get_logger().info
            log(self._last_raw_event)
        now = time.monotonic()
        if self.recorder.recording and not self._announced:
            self.send_raw(BagRequest.START, self.recorder.current_uri)
            self._announced = True
            self._next_split = now + self.settings.recording_split_duration_s
        elif not self.recorder.recording and self._announced:
            self.send_raw(BagRequest.STOP, self.recorder.last_event)
            self._announced = False
        if self._announced and self.settings.recording_split_duration_s > 0 and now >= self._next_split:
            self.send_raw(BagRequest.SPLIT, 'scheduled_split')
            self._next_split = now + self.settings.recording_split_duration_s
        if self.recorder.last_event != self._last_event:
            self._last_event = self.recorder.last_event
            message = f'{self.recorder.phase}: {self._last_event}; uri={self.recorder.current_uri}'
            # rclpy caches severity per call site; keep separate sites for each level.
            if self.recorder.phase == 'error':
                self.get_logger().error(message)
            else:
                self.get_logger().info(message)
            force_status = True
        if force_status or now - self._last_status >= self.settings.status_period_s:
            msg = BagStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'kart_bag_manager'
            msg.recording = self.recorder.recording
            msg.current_uri, msg.last_event = self.recorder.current_uri, self.recorder.last_event
            msg.message = self.recorder.phase
            if msg.recording and (self.settings.start_paused or self.settings.snapshot_mode):
                msg.message += ': start_paused/snapshot configured; writing not verified'
            self.status_pub.publish(msg)
            if self.raw_diagnostics_pub is not None:
                diagnostic = DiagnosticArray()
                diagnostic.header = msg.header
                status = DiagnosticStatus()
                status.name = self.get_fully_qualified_name() + '/native_raw'
                status.level = DiagnosticStatus.ERROR if self.raw_relay.last_error else DiagnosticStatus.OK
                status.message = self.raw_relay.last_event
                status.values = [KeyValue(key='state', value=self.raw_relay.state),
                    KeyValue(key='owned_writer', value=str(self.raw_relay.owns_raw).lower()),
                    KeyValue(key='last_error', value=self.raw_relay.last_error)]
                diagnostic.status = [status]
                self.raw_diagnostics_pub.publish(diagnostic)
            self._last_status = now

    def destroy_node(self):
        self.timer.cancel()
        try:
            if self._announced and rclpy.ok():
                self.send_raw(BagRequest.STOP, 'node shutdown')
                self._announced = False
        finally:
            self.recorder.close()
        if rclpy.ok():
            self.update(force_status=True)
        # Let the queued RAW STOP complete while the ROS context is still alive.
        if self.raw_relay is not None and rclpy.ok():
            deadline = time.monotonic() + self.settings.raw_recording_service_timeout_s
            while self.raw_relay.pending and rclpy.ok() and time.monotonic() < deadline:
                self.raw_relay.tick()
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.raw_relay.owns_raw or self.raw_relay.pending:
                self.get_logger().error('RAW shutdown acknowledgement unverified')
        return super().destroy_node()


def main(args=None):
    # Keep the ROS context alive until RAW STOP and recorder finalization have completed.
    shutdown_requested = Event()
    previous_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for sig in previous_handlers:
        signal.signal(sig, lambda _sig, _frame: shutdown_requested.set())
    node = None
    try:
        node = BagManagerNode()
        while rclpy.ok() and not shutdown_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)
