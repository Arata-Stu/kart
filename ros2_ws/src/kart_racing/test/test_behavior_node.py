"""Real callback boundary tests with ROS imports, messages and constructor faked.

This runs no ROS executor, TF listener, DDS graph, parameter service or hardware.
The real Planner/Settings are used; fake publishers only inspect callback output.
"""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from kart_racing.behavior import Planner, Settings


def header(stamp=10_000_000_000, frame='map'):
    sec, ns = divmod(stamp, 1_000_000_000)
    return NS(stamp=NS(sec=sec, nanosec=ns), frame_id=frame)


class Message:
    def __init__(self, **values):
        self.__dict__.update(values)


class Reference:
    def __init__(self):
        self.header, self.points, self.speeds = header(0, ''), [], []
        self.lane_id, self.line_type, self.closed = '', '', False


class Pose:
    def __init__(self, **values):
        self.header = values.get('header', header(0, ''))
        self.pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=0.))


class PathMessage:
    def __init__(self, **values):
        self.header, self.poses = values.get('header', header(0, '')), []


class Marker(Message):
    LINE_STRIP, ADD = 4, 0

    def __init__(self, **values):
        super().__init__(**values)
        self.pose = NS(orientation=NS(w=0.))
        self.scale, self.color, self.lifetime = NS(), NS(), NS()


class MarkerArray:
    def __init__(self):
        self.markers = []


class Time:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        return header(self.nanoseconds).stamp

    @classmethod
    def from_msg(cls, stamp):
        return cls(stamp.sec * 1_000_000_000 + stamp.nanosec)


class Publisher:
    def __init__(self, topic):
        self.topic, self.messages = topic, []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class FakeNode:
    mode = 'shadow'

    def __init__(self, name):
        self.values, self.publishers, self.current_ns = {}, [], 10_000_000_000
        self.publisher_counts = {}

    def declare_parameter(self, name, value, *args):
        value = self.mode if name == 'mode' else '/tmp/fake-map.json' if name == 'map_file' else value
        self.values[name] = value
        return NS(value=value)

    def get_parameter(self, name):
        return NS(value=self.values[name])

    def create_publisher(self, kind, topic, *args):
        result = Publisher(topic)
        self.publishers.append(result)
        return result

    def count_publishers(self, topic):
        return self.publisher_counts.get(topic, sum(p.topic == topic for p in self.publishers))

    def create_subscription(self, *args):
        return object()

    create_service = create_subscription
    add_on_set_parameters_callback = create_subscription
    add_post_set_parameters_callback = create_subscription
    create_timer = lambda self, *args, **kwargs: object()

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=self.current_ns))

    def get_logger(self):
        return NS(warning=Mock())


class BehaviorNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        names = ('rclpy', 'rclpy.clock', 'rclpy.node', 'rclpy.qos', 'rclpy.time',
                 'geometry_msgs', 'geometry_msgs.msg', 'kart_interfaces', 'kart_interfaces.msg',
                 'nav_msgs', 'nav_msgs.msg', 'rcl_interfaces', 'rcl_interfaces.msg',
                 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv', 'tf2_ros',
                 'visualization_msgs', 'visualization_msgs.msg')
        modules = {name: ModuleType(name) for name in names}
        modules['rclpy.node'].Node = FakeNode
        modules['rclpy.clock'].Clock = lambda **kwargs: object()
        modules['rclpy.clock'].ClockType = NS(STEADY_TIME=1)
        modules['rclpy.qos'].QoSProfile = lambda **kwargs: NS(**kwargs)
        modules['rclpy.qos'].DurabilityPolicy = NS(TRANSIENT_LOCAL=1)
        modules['rclpy.qos'].ReliabilityPolicy = NS(RELIABLE=1)
        modules['rclpy.qos'].qos_profile_sensor_data = object()
        modules['rclpy.time'].Time = Time
        modules['geometry_msgs.msg'].Point = Message
        modules['geometry_msgs.msg'].PoseStamped = Pose
        for name in ('GroundDetections', 'OpponentGroundPoints', 'OperationModeState'):
            setattr(modules['kart_interfaces.msg'], name, Message)
        modules['kart_interfaces.msg'].ReferenceLine = Reference
        modules['nav_msgs.msg'].Odometry, modules['nav_msgs.msg'].Path = Message, PathMessage
        modules['rcl_interfaces.msg'].ParameterDescriptor = Message
        modules['rcl_interfaces.msg'].SetParametersResult = Message
        modules['std_msgs.msg'].String = Message
        modules['std_srvs.srv'].Trigger = object
        modules['tf2_ros'].Buffer = lambda: NS()
        modules['tf2_ros'].TransformListener = lambda *args: object()
        modules['tf2_ros'].TransformException = type('TransformException', (Exception,), {})
        modules['visualization_msgs.msg'].Marker = Marker
        modules['visualization_msgs.msg'].MarkerArray = MarkerArray
        path = Path(__file__).resolve().parents[1] / 'kart_racing/behavior_node.py'
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('kart_racing.behavior_node_boundary_test', path)
            cls.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.module)

    def node(self, mode='active'):
        document = dict(schema='kart.hdmap.visualization.v1', frame_id='map', obstacles=[], lanes=[
            dict(id='main', closed=False, left=[[0, 2], [15, 2]], right=[[0, -2], [15, -2]])])
        with patch.object(FakeNode, 'mode', mode), patch.object(self.module, 'load', return_value=document):
            return self.module.BehaviorNode()

    @staticmethod
    def result(**changes):
        return dict(state='CLEAR', reason='clear', valid=True, path=[[1., 0.], [2., 0.]],
                    speeds=[.8, .8], target_speed=.8, candidates=[], **changes)

    def test_constructor_creates_control_authority_only_in_active(self):
        shadow = self.node('shadow')
        self.assertIsNone(shadow.control)
        self.assertNotIn('planning/reference_line', [p.topic for p in shadow.publishers])
        shadow.publish(self.result(), 9_900_000_000, 10_000_000_000)
        self.assertEqual(len(shadow.candidate.messages), 1)
        self.assertIn('planning/reference_line', [p.topic for p in self.node('active').publishers])

    def test_source_fault_before_start_is_empty_without_arming_fault_latch(self):
        node = self.node()
        node.issue = lambda *args: 'perception_timeout'
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            node.tick()
        self.assertFalse(node.started)
        self.assertIsNone(node.planner.fault)
        self.assertEqual(node.control.messages[-1].points, [])
        self.assertEqual(json.loads(node.status_pub.messages[-1].data)['state'], 'FAULT')

    def test_source_fault_after_start_holds_zero_route_and_remains_latched(self):
        node = self.node()
        node.started = node.planner.started = True
        node.planner.last_path, node.planner.last_speeds = [[1., 0.], [2., 0.]], [.8, .8]
        node.issue = lambda *args: 'perception_timeout'
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            node.tick()
        self.assertIsNotNone(node.planner.fault)
        self.assertTrue(all(speed == 0. for speed in node.control.messages[-1].speeds))
        self.assertEqual(json.loads(node.status_pub.messages[-1].data)['state'], 'FAULT')
        node.issue = lambda *args: ''
        node.ego, node.signature = (1., 0., 0., 0.), 'baseline'
        with patch.object(self.module.time, 'monotonic', return_value=20.1):
            node.tick()
        self.assertTrue(all(speed == 0. for speed in node.control.messages[-1].speeds))
        self.assertEqual(json.loads(node.status_pub.messages[-1].data)['state'], 'FAULT')

    def test_output_is_bounded_and_clamped_and_source_stamp_is_not_publication_time(self):
        node = self.node()
        result = self.result()
        result['speeds'] = [2., .3]
        node.publish(result, 9_000_000_000, 10_000_000_000)
        message = node.control.messages[-1]
        self.assertEqual(message.speeds, [.8, .3])
        self.assertTrue(all(math.isfinite(v) for p in message.points for v in (p.x, p.y, p.z)))
        self.assertEqual(message.header.stamp.sec, 9)
        status = json.loads(node.status_pub.messages[-1].data)
        self.assertEqual(status['stamp_ns'], '9000000000')
        self.assertEqual(status['published_at_ns'], '10000000000')
        for points in (300, 301):
            result['path'], result['speeds'] = [[float(i), 0.] for i in range(points)], [.5]*points
            node.publish(result, 9_000_000_000, 10_000_000_000)
            self.assertEqual(len(node.control.messages[-1].points), points if points == 300 else 0)
        result['path'], result['speeds'] = [[1., 0.], [2., 0.]], [-.1, .2]
        node.publish(result, 9_000_000_000, 10_000_000_000)
        self.assertEqual(node.control.messages[-1].points, [])

    def test_tick_uses_oldest_required_source_stamp(self):
        node = self.node()
        node.issue = lambda *args: 'waiting'
        node.pose_stamp, node.inputs.map_stamp = 9_900_000_000, 9_800_000_000
        node.inputs.raw_rc, node.inputs.rc_stamp = [(2., 0.)], 9_700_000_000
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            node.tick()
        status = json.loads(node.status_pub.messages[-1].data)
        self.assertEqual(status['stamp_ns'], '9700000000')
        self.assertEqual(status['published_at_ns'], '10000000000')

    def test_parameter_callback_rejects_running_and_out_of_bounds_changes(self):
        node = self.node()
        node.gate = NS(issue=lambda: 'STOP and standstill required')
        before = node.settings
        result = node.validate_parameters([NS(name='speed_cap', value=.5)])
        self.assertFalse(result.successful)
        self.assertIn('STOP', result.reason)
        self.assertEqual(node.settings, before)
        node.gate.issue = lambda: ''
        self.assertFalse(node.validate_parameters([NS(name='speed_cap', value=99.)]).successful)
        self.assertTrue(node.validate_parameters([NS(name='speed_cap', value=.5)]).successful)
        self.assertEqual(node.settings, before, 'validation has no settings side effect')

    def test_stopped_parameter_apply_before_baseline_does_not_crash_or_arm_driving(self):
        node = self.node()
        node.apply_parameters([NS(name='speed_cap', value=.5)])
        self.assertEqual(node.settings.speed_cap, .5)
        self.assertEqual(node.planner.settings.speed_cap, .5)
        self.assertIsNone(node.reference)
        self.assertFalse(node.started)
        self.assertEqual(node.signature, '')

    def test_selected_line_permissions_survive_configuration_parameter_apply_and_reset(self):
        from kart_hdmap.passing_zones import geometry_revision
        node = self.node()
        reference = self.reference(node)
        reference.line_type = 'raceline'
        points = [[point.x, point.y] for point in reference.points]
        zone = dict(id='race-left', start_index=0, end_index=2, side='left',
                    geometry_revision=geometry_revision(points, False))
        node.document['lanes'][0]['lines'] = {'raceline': dict(points=points, passing_zones=[zone])}
        node.configure_reference()
        self.assertEqual([row['id'] for row in node.planner.zones], ['race-left'])
        with patch.object(node.planner, 'set_reference', wraps=node.planner.set_reference) as configure:
            node.apply_parameters([NS(name='speed_cap', value=.5)])
        self.assertEqual(configure.call_args.kwargs['line_type'], 'raceline')
        self.assertIsNotNone(node.planner.fault)
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            response = node.reset(None, NS())
        self.assertTrue(response.success, response.message)
        self.assertEqual([row['id'] for row in node.planner.zones], ['race-left'])
        self.assertIsNone(node.planner.fault)

    def test_duplicate_odometry_stamp_does_not_refresh_pose_receipt(self):
        node = self.node()
        node.buffer = NS(lookup_transform=lambda *args: NS(transform=NS(
            translation=NS(x=1., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.))))
        node.gate = NS(update_speed=Mock())
        for stamp in (9_800_000_000, 9_900_000_000, 10_000_000_000):
            with patch.object(self.module.time, 'monotonic', return_value=20.):
                node.on_pose(NS(header=header(stamp, 'odom')))
        self.assertEqual(node.pose_at, 20.)
        node.gate.update_speed.reset_mock()
        with patch.object(self.module.time, 'monotonic', return_value=20.1):
            node.on_pose(NS(header=header(10_000_000_000, 'odom')))
        self.assertEqual(node.pose_at, 20.)
        node.gate.update_speed.assert_not_called()

    def delayed_tf(self, node, x=.1):
        ready = {'value': False}
        def lookup(*_args):
            if not ready['value']:
                raise self.module.TransformException('TF arrives after its source message')
            return NS(transform=NS(translation=NS(x=x, y=0., z=0.),
                                   rotation=NS(x=0., y=0., z=0., w=1.)))
        node.buffer = NS(lookup_transform=Mock(side_effect=lookup))
        return ready

    def test_delayed_pose_tf_is_retried_by_tick_without_overwriting_pending_or_refreshing_ages(self):
        node = self.node()
        node.issue = lambda *_args: 'waiting_for_other_inputs'
        node.gate = NS(update_speed=Mock())
        node.ego_motion.add(9_800_000_000, 0., 0.)
        node.ego_motion.add(9_900_000_000, .05, 0.)
        ready = self.delayed_tf(node)
        original = NS(header=header(10_000_000_000, 'odom'))
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            node.on_pose(original)
        self.assertIs(node.pending_pose[0], original)
        self.assertIsNone(node.ego)
        node.current_ns = 10_050_000_000
        with patch.object(self.module.time, 'monotonic', return_value=20.05):
            node.on_pose(NS(header=header(10_050_000_000, 'odom')))
        self.assertIs(node.pending_pose[0], original, 'new odometry must not starve a pending TF lookup')
        self.assertEqual(node.pending_pose[1], 20.)
        ready['value'], node.current_ns = True, 10_100_000_000
        with patch.object(self.module.time, 'monotonic', return_value=20.1):
            node.tick()
        self.assertIsNone(node.pending_pose)
        self.assertEqual(node.pose_stamp, 10_000_000_000)
        self.assertEqual(node.pose_at, 20., 'delayed processing must retain original receipt time')
        self.assertAlmostEqual(node.ego[0], .1)
        node.gate.update_speed.assert_called_once()
        self.assertTrue(all(call.args[:2] == ('map', 'rear_axle') and call.args[2].nanoseconds == 10_000_000_000
                            for call in node.buffer.lookup_transform.call_args_list))

    def test_pending_pose_expires_by_capture_or_receipt_time_without_querying_late_tf(self):
        for current_ns, wall in ((10_210_000_000, 20.01), (10_000_000_000, 20.21)):
            with self.subTest(current_ns=current_ns, wall=wall):
                node = self.node()
                self.delayed_tf(node)
                with patch.object(self.module.time, 'monotonic', return_value=20.):
                    node.on_pose(NS(header=header(10_000_000_000, 'odom')))
                self.assertIsNotNone(node.pending_pose)
                node.buffer.lookup_transform.reset_mock()
                node.current_ns = current_ns
                with patch.object(self.module.time, 'monotonic', return_value=wall):
                    node.retry_pose()
                self.assertIsNone(node.pending_pose)
                self.assertIsNone(node.ego)
                node.buffer.lookup_transform.assert_not_called()

    def test_delayed_confirmed_rc_tf_preserves_capture_stamp_and_receipt_and_keeps_one_pending(self):
        node = self.node()
        node.issue = lambda *_args: 'waiting_for_other_inputs'
        ready = self.delayed_tf(node, x=1.)
        original = NS(header=header(10_000_000_000, 'base_link'),
                      tracks=[NS(track_id='17', point=NS(x=2., y=0., z=0.))])
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            node.on_rc(original)
        node.current_ns = 10_050_000_000
        with patch.object(self.module.time, 'monotonic', return_value=20.05):
            node.on_rc(NS(header=header(10_050_000_000, 'base_link'), tracks=[]))
        self.assertIs(node.pending_rc[0], original)
        self.assertEqual(node.pending_rc[1], 20.)
        ready['value'], node.current_ns = True, 10_100_000_000
        with patch.object(self.module.time, 'monotonic', return_value=20.1):
            node.tick()
        self.assertIsNone(node.pending_rc)
        self.assertTrue(node.inputs.rc_valid)
        self.assertEqual(node.inputs.rc_stamp, 10_000_000_000)
        self.assertEqual(node.inputs.rc_at, 20.)
        self.assertEqual(node.inputs.records['rc:17']['observed_at'], 20.)
        self.assertEqual(node.inputs.records['rc:17']['x'], 3.)
        self.assertTrue(all(call.args[:2] == ('map', 'base_link') and call.args[2].nanoseconds == 10_000_000_000
                            for call in node.buffer.lookup_transform.call_args_list))

    def test_pending_confirmed_rc_expires_with_source_or_paused_clock_receipt_age(self):
        for current_ns, wall in ((10_410_000_000, 20.01), (10_000_000_000, 20.41)):
            with self.subTest(current_ns=current_ns, wall=wall):
                node = self.node()
                self.delayed_tf(node)
                node.inputs.rc_valid = True
                with patch.object(self.module.time, 'monotonic', return_value=20.):
                    node.on_rc(NS(header=header(10_000_000_000, 'base_link'),
                                  tracks=[NS(track_id='17', point=NS(x=2., y=0., z=0.))]))
                self.assertIsNotNone(node.pending_rc)
                node.buffer.lookup_transform.reset_mock()
                node.current_ns = current_ns
                with patch.object(self.module.time, 'monotonic', return_value=wall):
                    node.retry_rc()
                self.assertIsNone(node.pending_rc)
                self.assertFalse(node.inputs.rc_valid)
                self.assertNotIn('rc:17', node.inputs.records)
                node.buffer.lookup_transform.assert_not_called()

    def reference(self, node):
        reference = Reference()
        reference.header, reference.lane_id = header(), 'main'
        reference.line_type, reference.speeds = 'centerline', [.8, .8, .8]
        reference.points = [Message(x=x, y=0., z=0.) for x in (0., 5., 12.)]
        node.reference, node.reference_at, node.document_at = reference, 20., 20.
        node.pose_stamp, node.pose_at, node.ego = 10_000_000_000, 20., (1., 0., 0., 0.)
        node.gate = NS(issue=lambda: '')
        return reference

    def test_active_requires_the_single_reference_publisher_but_shadow_has_no_control_authority(self):
        node = self.node()
        self.reference(node)
        node.signature = 'baseline'
        node.inputs.map_stamp, node.inputs.map_at, node.inputs.map_valid = 10_000_000_000, 20., True
        self.assertEqual(node.issue(10_000_000_000, 20.), '')
        node.publisher_counts['planning/reference_line'] = 2
        self.assertEqual(node.issue(10_000_000_000, 20.), 'reference_publisher_conflict')
        node.mode = 'shadow'
        self.assertEqual(node.issue(10_000_000_000, 20.), '')

    def test_reset_requires_stop_and_baseline_proximity_then_clears_the_fault_latch(self):
        node = self.node()
        self.reference(node)
        node.planner.fault = ('FAULT', 'input_lost', [[1., 0.], [2., 0.]])
        original = node.planner
        node.gate.issue = lambda: 'STOP required'
        self.assertFalse(node.reset(None, NS()).success)
        node.gate.issue = lambda: ''
        node.ego = (1., .5, 0., 0.)
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            response = node.reset(None, NS())
        self.assertFalse(response.success)
        self.assertIn('基準ライン', response.message)
        self.assertIs(node.planner, original)
        node.ego = (1., .05, 0., 0.)
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            response = node.reset(None, NS())
        self.assertTrue(response.success)
        self.assertIsNone(node.planner.fault)
        self.assertFalse(node.started)
        self.assertIsNot(node.planner, original)

    def test_reset_rejects_missing_or_degenerate_reference_without_callback_exception(self):
        node = self.node()
        reference = self.reference(node)
        reference.points[1] = reference.points[0]
        node.reference_at = None
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            self.assertFalse(node.reset(None, NS()).success)
        node.reference = None
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            self.assertFalse(node.reset(None, NS()).success)

    def test_reset_rejects_stale_pose_even_if_stop_gate_is_ready(self):
        node = self.node()
        self.reference(node)
        original = node.planner
        node.pose_stamp, node.pose_at = 9_700_000_000, 19.7
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            response = node.reset(None, NS())
        self.assertFalse(response.success)
        self.assertEqual(response.message, 'fresh_pose_required')
        self.assertIs(node.planner, original)

    def test_input_fault_after_planning_is_rechecked_before_publishing(self):
        node = self.node()
        self.reference(node)
        node.configure_reference()
        node.issue = Mock(side_effect=['', 'perception_timeout'])
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            node.tick()
        self.assertEqual(node.issue.call_count, 2)
        self.assertIsNotNone(node.planner.fault)
        self.assertTrue(all(speed == 0. for speed in node.control.messages[-1].speeds))
        self.assertEqual(json.loads(node.status_pub.messages[-1].data)['state'], 'FAULT')

    def test_two_raw_rc_detections_cannot_both_disappear_into_one_confirmed_track(self):
        node = self.node()
        node.buffer = NS(lookup_transform=lambda *args: NS(transform=NS(
            translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.))))
        for index in range(9):
            message = NS(header=header(9_600_000_000+index*50_000_000, 'base_link'),
                         tracks=[NS(track_id='17', point=NS(x=2., y=0., z=0.))])
            with patch.object(self.module.time, 'monotonic', return_value=20.+index*.05):
                node.on_rc(message)
        objects = [NS(class_name='rc_car', score=.9, point=NS(x=2., y=y, z=0.)) for y in (0., .3)]
        with patch.object(self.module.time, 'monotonic', return_value=20.4):
            node.on_map(NS(header=header(), valid=True, reason='projected', objects=objects))
        rows = node.inputs.objects(20.4, .4, 5.)
        self.assertTrue(any(row['id'].startswith('untracked_rc:') and row['y'] == .3 for row in rows),
                        'a second visible RC beside the tracked car must remain an unknown object')

    def test_map_object_capacity_overflow_is_not_a_successful_partial_clear_scene(self):
        node = self.node()
        objects = [NS(class_name='obstacle', score=.9, point=NS(x=float(i), y=0., z=0.))
                   for i in range(node.inputs.MAX_OBJECTS+1)]
        with patch.object(self.module.time, 'monotonic', return_value=20.):
            node.on_map(NS(header=header(), valid=True, reason='projected', objects=objects))
        self.assertFalse(node.inputs.map_valid, 'visible objects beyond the behavior budget must fail closed')


if __name__ == '__main__':
    unittest.main()
