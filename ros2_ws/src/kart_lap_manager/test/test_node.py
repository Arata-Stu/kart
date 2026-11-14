"""Exercise ROS callbacks with exact-time TF/service fakes; no ROS installation needed."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from kart_lap_manager.core import LapDetector, RequestState


def module(name, **values):
    result = ModuleType(name)
    result.__dict__.update(values)
    return result


class TransformError(Exception):
    pass


class Message:
    def __init__(self, **values):
        self.__dict__.update(values)


class NodeDouble:
    """Keep rclpy's read-only client registry while exercising the constructor.

    Lyrical Node.clients is a read-only property. A plain object stub and
    __new__ callback tests cannot catch accidental replacement of that API.
    """
    parameter_overrides = {}

    def __init__(self, name, **_node_options):
        self.node_name = name
        self._created_clients = []
        self._created_publishers = []
        self._created_subscriptions = []
        self._created_timers = []
        self.log_messages = []

    @property
    def clients(self):
        yield from self._created_clients

    @property
    def publishers(self):
        yield from self._created_publishers

    @property
    def subscriptions(self):
        yield from self._created_subscriptions

    @property
    def timers(self):
        yield from self._created_timers

    def declare_parameter(self, name, default, _descriptor):
        return NS(value=self.parameter_overrides.get(name, default))

    def create_subscription(self, msg_type, topic, callback, qos):
        subscription = NS(msg_type=msg_type, topic=topic, callback=callback, qos=qos)
        self._created_subscriptions.append(subscription)
        return subscription

    def create_publisher(self, msg_type, topic, qos):
        publisher = NS(msg_type=msg_type, topic=topic, qos=qos, publish=lambda _message: None)
        self._created_publishers.append(publisher)
        return publisher

    def create_client(self, srv_type, service):
        client = NS(srv_type=srv_type, srv_name=service, service_is_ready=lambda: False)
        self._created_clients.append(client)
        return client

    def create_timer(self, period, callback, *, clock):
        timer = NS(period=period, callback=callback, clock=clock)
        self._created_timers.append(timer)
        return timer

    def get_logger(self):
        return NS(info=self.log_messages.append)


class BufferDouble:
    def __init__(self, *, node=None):
        self.node = node


stubs = {
    'rclpy': module('rclpy'),
    'rclpy.node': module('rclpy.node', Node=NodeDouble),
    'rclpy.clock': module('rclpy.clock', Clock=Message, ClockType=NS(STEADY_TIME='steady')),
    'rclpy.time': module('rclpy.time', Time=NS(from_msg=lambda value: value)),
    'rclpy.qos': module('rclpy.qos', DurabilityPolicy=NS(TRANSIENT_LOCAL='transient_local'),
                        QoSProfile=Message, ReliabilityPolicy=NS(RELIABLE='reliable'),
                        qos_profile_sensor_data=object()),
    'tf2_ros': module('tf2_ros', Buffer=BufferDouble, TransformListener=lambda *_args: NS(),
                     TransformException=TransformError),
}
for package, symbols in {
    'diagnostic_msgs.msg': ['DiagnosticArray'], 'geometry_msgs.msg': ['PoseWithCovarianceStamped'],
    'kart_interfaces.msg': ['ReferenceLine'], 'nav_msgs.msg': ['Odometry'],
    'rcl_interfaces.msg': ['ParameterDescriptor'], 'std_msgs.msg': ['String'],
}.items():
    stubs[package] = module(package, **{symbol: Message for symbol in symbols})
stubs['std_srvs.srv'] = module('std_srvs.srv', Trigger=NS(Request=Message))
for name in list(stubs):
    parent = name.split('.')[0]
    if parent not in stubs:
        stubs[parent] = module(parent)
spec = importlib.util.spec_from_file_location('kart_lap_manager._node_test',
                                             Path(__file__).parents[1] / 'kart_lap_manager/node.py')
adapter = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, stubs):
    spec.loader.exec_module(adapter)


def header(stamp, frame='odom'):
    whole = int(stamp)
    return NS(frame_id=frame, stamp=NS(sec=whole, nanosec=round((stamp-whole)*1e9)))


def position(s):
    s %= 40.
    if s < 10:
        return s, 0.
    if s < 20:
        return 10., s-10
    if s < 30:
        return 30-s, 10.
    return 0., 40-s


def tf(x, y, yaw=0.):
    import math
    return NS(transform=NS(translation=NS(x=x,y=y,z=0.),
                            rotation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))))


class Client:
    def __init__(self):
        self.calls = 0
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, _):
        self.calls += 1
        return NS(done=lambda: True, result=lambda: NS(success=True,message='accepted'))


class AdapterTests(unittest.TestCase):
    def test_constructor_preserves_ros_client_registry_for_each_feature_selection(self):
        for vgl, cache in ((False, False), (True, False), (False, True), (True, True)):
            with self.subTest(vgl=vgl, cache=cache), patch.object(NodeDouble, 'parameter_overrides',
                    {'vgl_enabled': vgl, 'cache_clear_enabled': cache}):
                node = adapter.LapManager()
            self.assertEqual(len(list(node.clients)), int(vgl) + int(cache))
            self.assertEqual(len(list(node.subscriptions)), 4)
            self.assertEqual(len(list(node.publishers)), 2)
            self.assertEqual(len(list(node.timers)), 1)
            self.assertIs(node.buffer.node, node, 'TF buffer must use the node ROS clock')
            for key, enabled, service in (
                ('vgl', vgl, '/visual_localization/trigger_localization'),
                ('cache', cache, '/kart/cache/clear'),
            ):
                client = node.maintenance_clients[key]
                if enabled:
                    self.assertIn(client, list(node.clients))
                    self.assertEqual(client.srv_name, service)
                else:
                    self.assertIsNone(client)

    def node(self):
        n = adapter.LapManager.__new__(adapter.LapManager)
        n.settings = dict(adapter.DEFAULTS, vgl_enabled=True, cache_clear_enabled=True)
        n.detector = LapDetector()
        n.detector.set_reference([(0.,0.),(10.,0.),(10.,10.),(0.,10.)])
        n.diag_stamp = n.diag_at = n.pose_stamp = n.pose_at = None
        n.diag_good = False
        n.pending = n.last_odom_stamp = n.last_clock = n.last_map_to_odom = None
        n.hint_stamp = n.request_ros_stamp = None
        n.request_lap_anchor = None
        n.last_status = -1e9
        n.vgl, n.cache = RequestState(True), RequestState()
        n.futures = dict(vgl=None,cache=None)
        n.maintenance_clients = dict(vgl=Client(),cache=Client())
        n.events, n.statuses, n.logs = [], [], []
        n.event_pub = NS(publish=n.events.append)
        n.status_pub = NS(publish=n.statuses.append)
        n.get_logger = lambda: NS(info=n.logs.append, warning=lambda *args, **kw: n.logs.append(args[0]))
        n.now = 10.
        n.now_s = lambda: n.now
        n.correction = (0.,0.)
        n.position = (0.,0.)
        n.tf_ready = True

        def lookup(target, source, stamp):
            if not n.tf_ready:
                raise TransformError()
            return tf(*(n.position if source == 'base_link' else n.correction))

        n.buffer = NS(lookup_transform=lookup, clear=lambda: None)
        return n

    def diagnostic(self, n, good=True):
        n.on_diagnostic(NS(header=header(n.now), status=[NS(hardware_id='visual_slam',level=0,
            values=[NS(key='localized_in_exist_map',value='Yes' if good else 'No'),
                    NS(key='vo_status',value='OK')])]))

    def hint(self, n, stamp):
        n.on_hint(NS(header=header(stamp,'map'), pose=NS(pose=NS(
            position=NS(x=0.,y=0.,z=0.), orientation=NS(x=0.,y=0.,z=0.,w=1.)))))

    def test_every_lap_after_own_delayed_image_stamped_vgl_hint(self):
        n = self.node()
        hint_at = None
        observed = 0
        with patch.object(adapter.time, 'monotonic', lambda: n.now):
            for i in range(526):
                n.now, n.position = 10.+i*.1, position(17.+i*.2)
                if hint_at is not None and i == hint_at:
                    # Input image is older than pose_timeout_s=.5 after inference.
                    self.hint(n, n.request_ros_stamp)
                    hint_at = None
                self.diagnostic(n)
                n.on_odometry(NS(header=header(n.now)))
                n.poll_requests(n.now)
                if len(n.events) != observed:
                    observed = len(n.events)
                    hint_at = i+8
        self.assertEqual(len(n.events), 2)
        self.assertEqual(n.maintenance_clients['vgl'].calls, 2)
        self.assertEqual(n.maintenance_clients['cache'].calls, 2)
        self.assertEqual(n.vgl.completed, 2)

    def test_external_or_late_hint_discards_partial_lap(self):
        n = self.node()
        n.detector.armed_at, n.detector.progress = 8., 4.
        self.hint(n, 9.5)
        self.assertIsNone(n.detector.armed_at)
        n.vgl.start(1.)
        n.request_ros_stamp = n.request_lap_anchor = 1.
        n.detector.armed_at, n.detector.progress = 1., 4.
        self.hint(n, 9.5)
        self.assertIsNone(n.detector.armed_at)

    def test_short_eight_second_lap_keeps_each_lap_with_one_point_five_second_hint_delay(self):
        n = self.node()
        hint_at = None
        observed = 0
        with patch.object(adapter.time, 'monotonic', lambda: n.now):
            for i in range(311):
                n.now, n.position = 10.+i*.1, position(17.+i*.5)
                if hint_at is not None and i == hint_at:
                    self.assertGreater(n.detector.progress / n.detector.length, .15)
                    self.hint(n, n.request_ros_stamp)
                    hint_at = None
                self.diagnostic(n)
                n.on_odometry(NS(header=header(n.now)))
                n.poll_requests(n.now)
                if len(n.events) != observed:
                    observed = len(n.events)
                    hint_at = i+15
        self.assertEqual(len(n.events), 3)
        self.assertEqual(n.maintenance_clients['vgl'].calls, 3)
        self.assertEqual(n.maintenance_clients['cache'].calls, 3)
        self.assertEqual(n.vgl.completed, 3)

    def test_own_hint_cannot_preserve_an_anchor_replaced_by_a_quality_reset(self):
        n = self.node()
        n.detector.armed_at, n.detector.progress = 9., 2.
        n.request('vgl', 9.)
        n.poll_requests(9.)
        self.assertEqual(n.request_lap_anchor, 9.)
        n.detector.invalidate('quality_loss')
        n.detector.sample(0.,0.,9.5)
        self.assertEqual(n.detector.armed_at, 9.5)
        self.hint(n, n.request_ros_stamp)
        self.assertIsNone(n.detector.armed_at)

    def test_map_correction_jump_and_unhealthy_diagnostic_abort_partial(self):
        n = self.node()
        with patch.object(adapter.time, 'monotonic', lambda: n.now):
            self.diagnostic(n)
            n.on_odometry(NS(header=header(n.now)))
            n.detector.armed_at, n.detector.progress = 8., 2.
            n.now += .1
            n.correction = (.3,0.)
            n.on_odometry(NS(header=header(n.now)))
            self.assertEqual(n.detector.armed_at, n.now)
            self.assertEqual(n.detector.progress, 0.)
            self.assertEqual(n.events, [])
            n.detector.armed_at, n.detector.progress = 8., 2.
            n.now += .1
            self.diagnostic(n, good=False)
            self.assertIsNone(n.detector.armed_at)

    def test_tf_wait_keeps_original_stamp_and_expires_without_event(self):
        n = self.node()
        with patch.object(adapter.time, 'monotonic', lambda: n.now):
            self.diagnostic(n)
            n.tf_ready = False
            n.on_odometry(NS(header=header(n.now)))
            original = n.pending
            n.now += .1
            n.on_odometry(NS(header=header(n.now)))
            self.assertIs(n.pending, original)
            n.now += .5
            n.retry_pose()
            self.assertIsNone(n.pending)
            self.assertEqual(n.events, [])

    def test_cache_timeout_uses_result_budget_and_requests_do_not_overlap(self):
        n = self.node()
        n.maintenance_clients['cache'] = NS(service_is_ready=lambda: True,
                               call_async=lambda _: NS(done=lambda: False))
        n.request('cache', 1.)
        n.poll_requests(5.)
        self.assertEqual(n.cache.state, 'service_pending')
        n.request('cache', 5.)
        self.assertEqual(n.cache.requests,1)
        n.poll_requests(17.)
        self.assertEqual(n.cache.state, 'suppressed_unknown_result')

    def test_duplicate_pose_does_not_refresh_freshness_and_clock_rewind_discards_lap(self):
        n = self.node()
        with patch.object(adapter.time, 'monotonic', lambda: n.now):
            self.diagnostic(n)
            n.on_odometry(NS(header=header(n.now)))
            received = n.pose_at
            n.now += .1
            n.on_odometry(NS(header=header(10.)))
            self.assertEqual(n.pose_at, received)
            n.detector.armed_at = 9.
            n.last_clock = n.now
            n.vgl.start(n.now)
            n.now = 2.
            n.tick()
            self.assertIsNone(n.detector.armed_at)
            self.assertIsNone(n.last_odom_stamp)
            self.assertEqual(n.vgl.state, 'suppressed_unknown_result')
            self.assertEqual(n.events, [])

    def test_rewind_clears_old_epoch_tf_before_accepting_new_poses(self):
        n = self.node()
        # A sub-second rewind must clear even if tf2's one-second jump callback
        # would not fire. An old run has transforms for the repeated stamp.
        cached = {'base_link': tf(0., 0.), 'odom': tf(0., 0.)}
        clears = []

        def lookup(_target, source, _stamp):
            if source not in cached:
                raise TransformError()
            return cached[source]

        def clear():
            clears.append(True)
            cached.clear()

        n.buffer = NS(lookup_transform=lookup, clear=clear)
        n.last_clock = 10.
        n.detector.sample(0., .2, 9.9)
        n.detector.count = 2
        n.now = 9.8
        with patch.object(adapter.time, 'monotonic', lambda: 100.):
            n.tick()
            self.assertEqual(len(clears), 1)
            self.diagnostic(n)
            n.on_odometry(NS(header=header(n.now)))
            self.assertIsNotNone(n.pending, 'wait for TF from the new playback epoch')
            self.assertIsNone(n.detector.last, 'old same-stamp TF must not seed a lap')
            self.assertEqual(n.detector.count, 2)
            self.assertEqual(n.events, [])
            cached.update(base_link=tf(1., 0.), odom=tf(0., 0.))
            n.retry_pose()
            self.assertIsNone(n.pending)
            self.assertEqual(n.detector.last[:2], (1., 0.))
            self.assertEqual(n.detector.count, 2)

    def test_lap_time_uses_bag_stamps_at_half_normal_and_double_playback_rate(self):
        for rate in (.5, 1., 2.):
            with self.subTest(rate=rate):
                n = self.node()
                n.settings['base_frame'] = 'camera_link'
                n.maintenance_clients = dict(vgl=None, cache=None)
                n.buffer = NS(lookup_transform=lambda _target, source, _stamp:
                              tf(*(n.position if source == 'camera_link' else n.correction)))
                wall = [100.]
                with patch.object(adapter.time, 'monotonic', lambda: wall[0]):
                    for i in range(401):
                        n.now, n.position = 10.+i*.1, position(i*.2)
                        wall[0] = 100.+i*.1/rate
                        self.diagnostic(n)
                        n.on_odometry(NS(header=header(n.now)))
                        n.tick()
                import json
                events = [json.loads(message.data) for message in n.events]
                self.assertEqual(len(events), 2)
                self.assertTrue(all(abs(event['lap_time_s']-20.) < 1e-9 for event in events))

    def test_paused_ros_clock_discards_partial_lap_and_keeps_steady_request_deadline(self):
        n = self.node()
        wall = [100.]
        with patch.object(adapter.time, 'monotonic', lambda: wall[0]):
            self.diagnostic(n)
            n.on_odometry(NS(header=header(n.now)))
            self.assertIsNotNone(n.detector.armed_at)
            n.request('vgl', wall[0])
            n.poll_requests(wall[0])
            self.assertEqual(n.vgl.state, 'searching')
            wall[0] += .6  # ROS time and all measurement stamps remain paused.
            n.tick()
            self.assertIsNone(n.detector.armed_at)
            self.assertEqual(n.detector.reason, 'pose_stale')
            self.assertEqual(n.vgl.state, 'searching')
            wall[0] = 131.
            n.tick()
            self.assertEqual(n.vgl.state, 'suppressed_unknown_result')
            self.assertEqual(n.events, [])


if __name__ == '__main__':
    unittest.main()
