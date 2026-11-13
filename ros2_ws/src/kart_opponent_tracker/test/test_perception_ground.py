"""Map-only geometry, exact synchronization, freshness, and ROS adapter contracts."""
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from kart_opponent_tracker.perception_ground import (
    COLOURS, MAX_AGE_S, Outcome, ProjectedDetection, ProjectionStatus, Settings,
    project_points, projection_issue, stamp_ns, transform_point,
)


def stamp(value=1_000_000_000):
    seconds, nanos = divmod(value, 1_000_000_000)
    return NS(sec=seconds, nanosec=nanos)


def header(value=1_000_000_000, frame='camera_color_optical_frame'):
    return NS(stamp=stamp(value), frame_id=frame)


def info(value=1_000_000_000):
    return NS(header=header(value), width=640, height=480, binning_x=0, binning_y=0,
              roi=NS(x_offset=0, y_offset=0, width=0, height=0),
              k=[400., 0., 320., 0., 400., 240., 0., 0., 1.], d=[0.] * 5, distortion_model='plumb_bob')


def detection(class_id='0', score=.9, *, x=320., y=400., width=80., height=80., identity=''):
    return NS(id=identity, header=header(0, ''), bbox=NS(center=NS(position=NS(x=x, y=y), theta=0.),
                                                       size_x=width, size_y=height),
              results=[NS(hypothesis=NS(class_id=class_id, score=score))])


def message(objects=(), value=1_000_000_000):
    return NS(header=header(value), detections=list(objects))


def transform(t=(0., 0., 0.), q=(0., 0., 0., 1.)):
    return NS(translation=NS(x=t[0], y=t[1], z=t[2]), rotation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))


CAMERA = transform((0., 0., 1.), (-.5, .5, -.5, .5))
IDENTITY = transform()


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.settings = Settings(ground_z_m=0.)

    def project(self, objects, *, camera=CAMERA, target=IDENTITY, settings=None):
        return project_points(message(objects), info(), camera, target, settings or self.settings,
                              now_ns=1_050_000_000)

    def test_body_ground_intersection_precedes_tilted_map_transform(self):
        angle = math.pi / 6
        target = transform((10., 20., 3.), (0., math.sin(angle / 2), 0., math.cos(angle / 2)))
        result = self.project([detection()], target=target)
        self.assertEqual(result.reason, 'projected')
        point = result.objects[0].point
        self.assertAlmostEqual(point[0], 10 + math.sqrt(3))
        self.assertAlmostEqual(point[1], 20.)
        self.assertAlmostEqual(point[2], 2.)  # Not an intersection with map z=ground_z.

    def test_both_classes_keep_scores_ids_and_target_coordinates(self):
        target = transform((10., 20., 3.), (0., 0., math.sqrt(.5), math.sqrt(.5)))
        result = self.project([detection(identity='decoder-7'), detection('1', .8, x=400.)], target=target)
        self.assertEqual([obj.class_name for obj in result.objects], ['rc_car', 'obstacle'])
        self.assertEqual([obj.class_id for obj in result.objects], ['0', '1'])
        self.assertEqual([obj.detection_id for obj in result.objects], ['decoder-7', 'frame:1'])
        self.assertEqual([obj.score for obj in result.objects], [.9, .8])
        for actual, expected in zip(result.objects[0].point, (10., 22., 3.)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(result.objects[1].point[0], 10.4)
        self.assertAlmostEqual(result.objects[1].point[1], 22.)

    def test_only_source_class_overrides_are_allowed_and_disabled_class_is_absent(self):
        settings = Settings(ground_z_m=0., rc_car_class_id='2', obstacle_class_id='')
        result = self.project([detection('0'), detection('1'), detection('2')], settings=settings)
        self.assertEqual([(obj.class_id, obj.class_name) for obj in result.objects], [('2', 'rc_car')])
        self.assertEqual(result.rejected, {'class_or_score': 2})

    def test_invalid_bottom_bbox_and_range_are_distinguished(self):
        cases = [([detection(y=450., height=60.)], 'bottom_clipped'),
                 ([detection(x=5., width=20.)], 'bbox_outside_image'),
                 ([detection(width=float('nan'))], 'invalid_bbox'),
                 ([detection(y=230., height=40.)], 'out_of_range'),
                 ([detection(y=210., height=40.)], 'no_ground_intersection')]
        for objects, reason in cases:
            with self.subTest(reason=reason):
                result = self.project(objects)
                self.assertEqual(result.reason, 'all_rejected')
                self.assertEqual(result.rejected, {reason: 1})
                self.assertEqual(result.objects, [])

    def test_zero_detections_and_zero_selected_are_distinct(self):
        self.assertEqual(self.project([]).reason, 'zero_detections')
        result = self.project([detection('9'), detection(score=.249), detection(score=float('inf'))])
        self.assertEqual(result.reason, 'zero_selected')
        self.assertEqual(result.rejected, {'class_or_score': 3})
        self.assertEqual(self.project([detection(score=.25)]).reason, 'projected')

    def test_projection_count_and_hypothesis_scan_are_bounded(self):
        result = self.project([detection(), detection('1'), detection()], settings=Settings(ground_z_m=0., max_objects=2))
        self.assertEqual(len(result.objects), 2)
        self.assertEqual(result.rejected, {'object_limit': 1})
        obj = detection('9')
        obj.results *= 16
        obj.results += detection().results
        self.assertEqual(self.project([obj]).reason, 'zero_selected')

    def test_populated_detection_header_must_match_array_header(self):
        obj = detection()
        obj.header = header(2_000_000_000)
        result = self.project([obj])
        self.assertEqual(result.rejected, {'detection_header': 1})

    def test_nonfinite_invalid_tf_is_rejected_and_nonunit_tf_is_normalized(self):
        for invalid in (transform(q=(0., 0., 0., 0.)), transform(t=(float('nan'), 0, 0)),
                        transform(q=(0., float('inf'), 0., 1.))):
            with self.subTest(invalid=invalid):
                self.assertEqual(self.project([detection()], target=invalid).reason, 'invalid_tf')
        self.assertEqual(transform_point((1., 2., 3.), transform(q=(0., 0., 0., 2.))), (1., 2., 3.))

    def test_bad_startup_settings_fail_instead_of_assuming_ground_height(self):
        for values in ({}, dict(ground_z_m=float('inf')), dict(ground_z_m=0., max_range_m=0.),
                       dict(ground_z_m=0., projection_frame='/base_link'), dict(ground_z_m=0., target_frame=''),
                       dict(ground_z_m=0., confidence_threshold=float('nan')), dict(ground_z_m=0., max_objects=True),
                       dict(ground_z_m=0., rc_car_class_id='0', obstacle_class_id='0'),
                       dict(ground_z_m=0., rc_car_class_id='', obstacle_class_id='')):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Settings(**values)


class CalibrationTests(unittest.TestCase):
    def test_exact_stamp_and_frame_are_required(self):
        self.assertEqual(projection_issue(message(), info(1_000_000_001)), 'not_synchronized')
        camera = info()
        camera.header.frame_id = 'other_camera'
        self.assertEqual(projection_issue(message(), camera), 'invalid_frame')
        self.assertEqual(projection_issue(message(value=0), info(0)), 'invalid_stamp')
        bad = stamp()
        bad.nanosec = 1_000_000_000
        self.assertEqual(stamp_ns(bad), 0)

    def test_capture_time_age_future_and_ros_clock_unavailable(self):
        self.assertIsNone(projection_issue(message(), info(), 1_500_000_000))
        self.assertEqual(projection_issue(message(), info(), 1_500_000_001), 'capture_stale')
        self.assertEqual(projection_issue(message(), info(), 900_000_000), 'capture_stale')
        self.assertEqual(projection_issue(message(), info(), 0), 'clock_unavailable')

    def test_nonraw_image_geometry_and_invalid_intrinsics_are_rejected(self):
        for field, value, reason in (('width', 320, 'invalid_image_shape'), ('binning_x', 2, 'invalid_image_shape'),
                                     ('distortion_model', 'equidistant', 'invalid_calibration'),
                                     ('d', [float('nan')] * 5, 'invalid_calibration'),
                                     ('k', [0.] * 9, 'invalid_calibration')):
            camera = info()
            setattr(camera, field, value)
            with self.subTest(field=field):
                self.assertEqual(projection_issue(message(), camera), reason)
        camera = info()
        camera.roi.y_offset = 10
        self.assertEqual(projection_issue(message(), camera), 'invalid_image_shape')


class FreshnessTests(unittest.TestCase):
    def test_waiting_unmatched_failure_and_stale_success_are_diagnostic(self):
        state = ProjectionStatus()
        self.assertEqual(state.view(0.)['values']['reason'], 'waiting_detections')
        detections, camera = message([detection()]), info()
        state.receive('detections', detections, 0.)
        self.assertEqual(state.view(.01)['values']['reason'], 'waiting_camera_info')
        state.receive('camera_info', camera, .02)
        self.assertEqual(state.view(.03)['values']['reason'], 'not_synchronized')
        state.synchronized(detections, Outcome('projected', [ProjectedDetection('0', '0', 'rc_car', .9, (2., 0., 0.))]), .04)
        self.assertEqual(state.view(.05)['level'], 0)
        self.assertEqual(state.view(.05)['values']['rc_car_count'], '1')
        self.assertEqual(state.view(.51)['values']['reason'], 'detections_stale')

    def test_duplicate_headers_do_not_refresh_or_reproject(self):
        state, detections, camera = ProjectionStatus(), message([detection()]), info()
        state.receive('detections', detections, 0.)
        state.receive('camera_info', camera, 0.)
        self.assertTrue(state.admit(detections))
        state.synchronized(detections, Outcome('projected'), .01)
        self.assertFalse(state.admit(detections))
        self.assertFalse(state.receive('detections', detections, .49))
        self.assertFalse(state.receive('camera_info', camera, .49))
        self.assertEqual(state.view(.51)['values']['reason'], 'detections_stale')

    def test_delayed_sync_does_not_replace_latest_input_and_rewind_reuses_stamp_safely(self):
        state, older = ProjectionStatus(), message([detection()])
        state.receive('detections', older, 0.)
        state.synchronized(older, Outcome('projected'), .01)
        newer = message([detection()], 2_000_000_000)
        state.receive('detections', newer, .1)
        self.assertFalse(state.admit(older))
        state.receive('detections', older, .2)  # New bag/sim epoch may reuse the old capture key.
        self.assertTrue(state.admit(older))

    def test_partial_projection_is_ok_with_rejection_counts_for_remaining_objects(self):
        state, detections, camera = ProjectionStatus(), message([detection(), detection(y=480.)]), info()
        state.receive('detections', detections, 0.)
        state.receive('camera_info', camera, 0.)
        result = project_points(detections, camera, CAMERA, IDENTITY, Settings(ground_z_m=0.))
        state.synchronized(detections, result, .01)
        view = state.view(.02)
        self.assertEqual(view['level'], 0)
        self.assertEqual(view['values']['reason'], 'projected')
        self.assertIn('bottom_clipped', view['values']['rejected'])


class NodeBoundaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        names = ('message_filters', 'rclpy', 'rclpy.clock', 'rclpy.clock_type', 'rclpy.node', 'rclpy.qos',
                 'rclpy.time', 'diagnostic_msgs.msg', 'geometry_msgs.msg', 'kart_interfaces.msg',
                 'rcl_interfaces.msg', 'sensor_msgs.msg', 'tf2_ros', 'visualization_msgs.msg', 'vision_msgs.msg')
        modules = {name: ModuleType(name) for name in names}
        modules['rclpy.node'].Node = object
        modules['rclpy.clock'].Clock = Mock()
        modules['rclpy.clock_type'].ClockType = NS(STEADY_TIME=1)
        modules['rclpy.qos'].QoSProfile = Mock()
        modules['rclpy.qos'].ReliabilityPolicy = NS(BEST_EFFORT=1)
        modules['rclpy.qos'].DurabilityPolicy = NS(VOLATILE=1)
        modules['rclpy.qos'].qos_profile_sensor_data = object()
        modules['rclpy.time'].Time = NS(from_msg=lambda value: ('capture-time', value.sec, value.nanosec))
        modules['rcl_interfaces.msg'].ParameterDescriptor = Mock()
        modules['sensor_msgs.msg'].CameraInfo = object
        modules['vision_msgs.msg'].Detection2DArray = object
        modules['tf2_ros'].Buffer = Mock()
        modules['tf2_ros'].TransformListener = Mock()
        modules['tf2_ros'].TransformException = type('TransformException', (Exception,), {})
        class Message:
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)
        class Points(Message):
            def __init__(self, **kwargs):
                self.header, self.objects = header(0, ''), []
                super().__init__(**kwargs)
        class Marker(Message):
            ADD, DELETE, SPHERE_LIST = 0, 2, 7
            def __init__(self, **kwargs):
                self.pose = NS(orientation=NS(w=0.))
                self.scale = NS(x=0., y=0., z=0.)
                self.color = NS(r=0., g=0., b=0., a=0.)
                self.lifetime = stamp(0)
                super().__init__(**kwargs)
        class DiagnosticArray(Message):
            def __init__(self, **kwargs):
                self.header = header(0, '')
                super().__init__(**kwargs)
        modules['geometry_msgs.msg'].Point = Message
        modules['kart_interfaces.msg'].GroundDetection = Message
        modules['kart_interfaces.msg'].GroundDetections = Points
        modules['visualization_msgs.msg'].Marker = Marker
        modules['visualization_msgs.msg'].MarkerArray = Message
        modules['diagnostic_msgs.msg'].DiagnosticArray = DiagnosticArray
        modules['diagnostic_msgs.msg'].DiagnosticStatus = Message
        modules['diagnostic_msgs.msg'].KeyValue = Message
        source = Path(__file__).parents[1] / 'kart_opponent_tracker/perception_ground_node.py'
        spec = importlib.util.spec_from_file_location('kart_opponent_tracker._test_map_node', source)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        cls.module, cls.Node = module, module.PerceptionGroundNode

    def node(self):
        return NS(settings=Settings(ground_z_m=0.), status=ProjectionStatus(), output=NS(publish=Mock()),
                  markers=NS(publish=Mock()), diagnostics=NS(publish=Mock()), tf=NS(lookup_transform=Mock()),
                  get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_050_000_000, to_msg=lambda: stamp(1_050_000_000))),
                  get_logger=lambda: NS(warning=Mock()))

    def bind(self, node):
        node.publish_objects = lambda *args, **kwargs: self.Node.publish_objects(node, *args, **kwargs)
        return node

    def test_both_marker_namespaces_delete_on_failure_and_have_bounded_lifetime(self):
        node = self.node()
        obj = ProjectedDetection('frame:0', '0', 'rc_car', .9, (2., 0., 0.))
        self.Node.publish_objects(node, [obj], stamp())
        published = node.output.publish.call_args.args[0]
        self.assertEqual(published.header.frame_id, 'map')
        self.assertEqual(published.header.stamp.sec, 1)
        self.assertEqual(published.objects[0].class_name, 'rc_car')
        self.assertTrue(published.valid)
        self.assertEqual(published.reason, 'projected')
        markers = node.markers.publish.call_args.args[0].markers
        self.assertEqual([(m.ns, m.action) for m in markers], [('rc_car', 0), ('obstacle', 2)])
        self.assertTrue(all(m.lifetime.nanosec == 500_000_000 for m in markers))
        self.assertEqual((markers[0].color.r, markers[0].color.g, markers[0].color.b), COLOURS['rc_car'])
        self.Node.publish_objects(node, [])
        self.assertEqual(node.output.publish.call_args.args[0].objects, [])
        self.assertEqual([m.action for m in node.markers.publish.call_args.args[0].markers], [2, 2])

    def test_tf_lookups_use_capture_stamp_and_body_plane_then_map(self):
        node = self.bind(self.node())
        detections, camera = message([detection(), detection('1')]), info()
        node.status.receive('detections', detections)
        node.status.receive('camera_info', camera)
        node.tf.lookup_transform.side_effect = [NS(transform=CAMERA), NS(transform=IDENTITY)]
        self.Node.project(node, detections, camera)
        self.assertEqual(node.tf.lookup_transform.call_args_list[0].args,
                         ('base_link', 'camera_color_optical_frame', ('capture-time', 1, 0)))
        self.assertEqual(node.tf.lookup_transform.call_args_list[1].args,
                         ('map', 'base_link', ('capture-time', 1, 0)))
        self.assertEqual(len(node.output.publish.call_args.args[0].objects), 2)

    def test_tf_failure_and_watchdog_clear_never_replay_valid_points(self):
        node = self.bind(self.node())
        detections, camera = message([detection()]), info()
        node.status.receive('detections', detections, 0.)
        node.status.receive('camera_info', camera, 0.)
        node.tf.lookup_transform.side_effect = [NS(transform=CAMERA), self.module.TransformException('map TF missing')]
        with patch.object(self.module.time, 'monotonic', return_value=.01):
            self.Node.project(node, detections, camera)
        self.assertEqual(node.status.outcome.reason, 'target_tf_unavailable')
        self.assertEqual(node.output.publish.call_args.args[0].objects, [])
        self.assertFalse(node.output.publish.call_args.args[0].valid)
        self.assertEqual(node.output.publish.call_args.args[0].reason, 'target_tf_unavailable')
        with patch.object(self.module.time, 'monotonic', return_value=1.):
            self.Node.watchdog(node)
        self.assertEqual(node.output.publish.call_args.args[0].objects, [])
        self.assertEqual([m.action for m in node.markers.publish.call_args.args[0].markers], [2, 2])

    def test_watchdog_carries_invalid_reason_before_next_diagnostic_tick(self):
        node = self.bind(self.node())
        detections, camera = message([detection()]), info()
        node.status.receive('detections', detections, 0.)
        node.status.receive('camera_info', camera, 0.)
        node.tf.lookup_transform.side_effect = [NS(transform=CAMERA), NS(transform=IDENTITY)]
        with patch.object(self.module.time, 'monotonic', return_value=.01):
            self.Node.project(node, detections, camera)
            self.Node.publish_diagnostics(node)
        old_diagnostic = node.diagnostics.publish.call_args.args[0]
        self.assertEqual(old_diagnostic.status[0].level, 0)
        self.assertTrue(node.output.publish.call_args.args[0].valid)
        with patch.object(self.module.time, 'monotonic', return_value=1.):
            self.Node.watchdog(node)
        current = node.output.publish.call_args.args[0]
        self.assertEqual(current.objects, [])
        self.assertFalse(current.valid)
        self.assertEqual(current.reason, 'detections_stale')
        self.assertIs(node.diagnostics.publish.call_args.args[0], old_diagnostic)

    def test_received_zero_detections_are_valid_zero_without_tf(self):
        node = self.bind(self.node())
        detections, camera = message(), info()
        node.status.receive('detections', detections)
        node.status.receive('camera_info', camera)
        self.Node.project(node, detections, camera)
        current = node.output.publish.call_args.args[0]
        self.assertEqual(current.objects, [])
        self.assertTrue(current.valid)
        self.assertEqual(current.reason, 'zero_detections')
        node.tf.lookup_transform.assert_not_called()

    def test_delayed_sync_with_paused_ros_clock_cannot_publish_expired_input(self):
        node = self.bind(self.node())
        detections, camera = message([detection()]), info()
        node.status.receive('detections', detections, 0.)
        node.status.receive('camera_info', camera, 1.)
        with patch.object(self.module.time, 'monotonic', return_value=1.):
            self.Node.project(node, detections, camera)
        self.assertEqual(node.status.outcome.reason, 'detections_stale')
        node.tf.lookup_transform.assert_not_called()
        self.assertEqual(node.output.publish.call_args.args[0].objects, [])

    def test_projection_expiring_during_processing_is_cleared_before_publish(self):
        node = self.bind(self.node())
        detections, camera = message([detection()]), info()
        node.status.receive('detections', detections, 0.)
        node.status.receive('camera_info', camera, 0.)
        node.tf.lookup_transform.side_effect = [NS(transform=CAMERA), NS(transform=IDENTITY)]
        with patch.object(self.module.time, 'monotonic', side_effect=[.01, 1., 1.]):
            self.Node.project(node, detections, camera)
        self.assertEqual(node.status.outcome.reason, 'detections_stale')
        self.assertEqual(node.output.publish.call_args.args[0].objects, [])

    def test_diagnostics_identify_display_node_and_use_current_ros_stamp(self):
        node = self.node()
        self.Node.publish_diagnostics(node)
        diagnostic = node.diagnostics.publish.call_args.args[0]
        self.assertEqual(diagnostic.header.stamp.nanosec, 50_000_000)
        self.assertEqual(diagnostic.status[0].name, 'perception_ground/projection')
        self.assertEqual({value.key: value.value for value in diagnostic.status[0].values}['reason'], 'waiting_detections')


if __name__ == '__main__':
    unittest.main()
