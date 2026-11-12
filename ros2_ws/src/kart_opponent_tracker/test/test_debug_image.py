"""Exact-frame synchronization, bounded memory and real OpenCV JPEG overlays."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from kart_opponent_tracker.debug_image import (
    Box, Frame, OverlayBuffer, Result, Settings, _box_label, _caption,
    decode_image, draw_frame, encode_jpeg, placeholder, result_from_message,
)


KEY = (1_000_000_000, 'camera_color_optical_frame')
GREEN = Result(1, (Box(160., 190., 100., 80., '0', .93),))
BLUE = Result(1, (Box(430., 300., 120., 100., '2', .82, '4-7'),))


class ImageConversionTests(unittest.TestCase):
    def test_rgb_bgr_and_mono_with_row_padding_keep_exact_geometry(self):
        rgb = np.zeros((480, 640, 3), np.uint8)
        rgb[100, 200] = (255, 10, 5)
        padded = np.pad(rgb.reshape(480, 1920), ((0, 0), (0, 4)), constant_values=42)
        result = decode_image(640, 480, 'rgb8', 1924, padded.tobytes())
        self.assertEqual(result.shape, (480, 640, 3))
        self.assertEqual(result[100, 200].tolist(), [5, 10, 255])
        self.assertEqual(decode_image(640, 480, 'bgr8', 1920, result.tobytes())[100, 200].tolist(), [5, 10, 255])
        mono = np.full((480, 640), 123, np.uint8)
        self.assertTrue(np.all(decode_image(640, 480, 'mono8', 640, mono.tobytes()) == 123))

    def test_invalid_shape_encoding_step_and_truncated_data_are_rejected(self):
        for arguments in ((320, 240, 'rgb8', 960, b''), (640, 480, '16UC1', 1280, b''),
                          (640, 480, 'rgb8', 100, b''), (640, 480, 'rgb8', 1920, b'bad')):
            with self.subTest(arguments=arguments[:4]), self.assertRaises(ValueError):
                decode_image(*arguments)


class MatchingTests(unittest.TestCase):
    def setUp(self):
        self.settings = Settings()
        self.buffer = OverlayBuffer(self.settings)
        self.image = np.zeros((480, 640, 3), np.uint8)

    def test_result_before_or_after_image_matches_only_exact_source(self):
        for results_first in (False, True):
            with self.subTest(results_first=results_first):
                buffer = OverlayBuffer(self.settings)
                if not results_first:
                    buffer.add_image(KEY, self.image, 0.)
                buffer.add_result('detections', KEY, GREEN, .01)
                buffer.add_result('tracks', KEY, BLUE, .02)
                if results_first:
                    buffer.add_image(KEY, self.image, .03)
                frame = buffer.take(.04)
                self.assertEqual(frame.key, KEY)
                self.assertEqual(frame.detections, GREEN)
                self.assertEqual(frame.tracks, BLUE)

    def test_future_stamp_and_other_frame_never_overlay_the_source(self):
        self.buffer.add_image(KEY, self.image, 0.)
        self.buffer.add_result('detections', (KEY[0] + 1, KEY[1]), GREEN, .01)
        self.buffer.add_result('tracks', (KEY[0], 'other_camera'), BLUE, .02)
        self.assertIsNone(self.buffer.take(.19))
        frame = self.buffer.take(.21)
        self.assertIsNone(frame.detections)
        self.assertIsNone(frame.tracks)
        self.assertTrue(np.all(draw_frame(frame)[150, 110] == 0), 'unmatched green box must not appear')

    def test_zero_result_is_distinct_from_missing_result(self):
        self.buffer.add_image(KEY, self.image, 0.)
        self.buffer.add_result('detections', KEY, Result(0, ()), .01)
        frame = self.buffer.take(.21)
        self.assertEqual(_caption(frame.detections, 'DETECTIONS'), 'DETECTIONS: 0')
        self.assertIn('MISSING', _caption(frame.tracks, 'CAR TRACKS'))

    def test_late_result_does_not_decorate_next_frame(self):
        self.buffer.add_image(KEY, self.image, 0.)
        self.assertIsNotNone(self.buffer.take(.21))
        self.buffer.add_result('detections', KEY, GREEN, .22)
        newer = (KEY[0] + 100, KEY[1])
        self.buffer.add_image(newer, self.image, .23)
        frame = self.buffer.take(.44)
        self.assertEqual(frame.key, newer)
        self.assertIsNone(frame.detections)

    def test_high_rate_unselected_results_preserve_pending_match_and_memory_bound(self):
        self.buffer.add_image(KEY, self.image, 0.)
        for channel, result in (('detections', GREEN), ('tracks', BLUE)):
            self.buffer.add_result(channel, KEY, result, .01)
            for i in range(1, 60):
                self.buffer.add_result(channel, (KEY[0] + i, KEY[1]), Result(0, ()), .01 + i / 1000)
            self.assertLessEqual(len(self.buffer.results[channel]), self.settings.cache_size)
        frame = self.buffer.take(.20)
        self.assertEqual(frame.detections, GREEN)
        self.assertEqual(frame.tracks, BLUE)
        for i in range(50):
            self.buffer.add_image((KEY[0] + i + 100, KEY[1]), self.image, .3)
        self.assertEqual(len(self.buffer.images), self.settings.cache_size)

    def test_expired_results_and_invalid_headers_are_not_matches(self):
        self.buffer.add_result('detections', KEY, GREEN, 0.)
        self.buffer.add_image(KEY, self.image, 2.)
        self.assertIsNone(self.buffer.take(2.21).detections)
        invalid = (0, '')
        self.buffer.add_image(invalid, self.image, 3.)
        self.buffer.add_result('detections', invalid, GREEN, 3.01)
        self.assertIsNone(self.buffer.take(3.21).detections)

    def test_image_stamp_rewind_discards_previous_epoch_results(self):
        self.buffer.add_image((KEY[0] + 100, KEY[1]), self.image, 0.)
        self.buffer.add_result('detections', KEY, GREEN, .01)
        self.buffer.add_result('tracks', KEY, BLUE, .01)
        self.buffer.add_image(KEY, self.image, .02)
        frame = self.buffer.take(.23)
        self.assertIsNone(frame.detections)
        self.assertIsNone(frame.tracks)

    def test_expired_candidate_is_dropped_while_new_camera_frame_waits(self):
        self.buffer.add_image(KEY, self.image, 0.)
        newer = (KEY[0] + 1, KEY[1])
        self.buffer.add_image(newer, self.image, 1.)
        self.assertIsNone(self.buffer.take(1.1))
        self.assertEqual(list(self.buffer.images), [newer])
        self.assertEqual(self.buffer.take(1.21).key, newer)


class DrawingTests(unittest.TestCase):
    def test_labels_colors_and_jpeg_are_real_and_source_image_is_unchanged(self):
        source = np.zeros((480, 640, 3), np.uint8)
        rendered = draw_frame(Frame(KEY, source, 0., GREEN, BLUE), ['person', 'bicycle', 'car'])
        self.assertEqual(rendered[150, 110].tolist(), [0, 220, 0])
        self.assertEqual(rendered[250, 370].tolist(), [255, 100, 0])
        self.assertEqual(_box_label(GREEN.boxes[0], ['person'], False), 'person 0.93')
        self.assertEqual(_box_label(BLUE.boxes[0], ['', '', 'car'], True), 'car ID 4-7 0.82')
        self.assertTrue(np.all(source == 0))
        jpeg = encode_jpeg(rendered, 70)
        self.assertTrue(jpeg.startswith(b'\xff\xd8'))
        self.assertEqual(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR).shape, source.shape)
        self.assertLess(len(jpeg), 100_000)

    def test_placeholder_is_labeled_non_source_image(self):
        image = placeholder('RGB: MISSING - camera input stopped')
        self.assertEqual(image.shape, (480, 640, 3))
        self.assertGreater(np.count_nonzero(image), 1000)
        self.assertEqual(len(encode_jpeg(image, 70)) > 0, True)

    def test_message_extraction_is_bounded_and_keeps_real_total(self):
        def detection(score=.9):
            return SimpleNamespace(id='track', bbox=SimpleNamespace(
                center=SimpleNamespace(position=SimpleNamespace(x=20., y=30.)), size_x=10., size_y=15.),
                results=[SimpleNamespace(hypothesis=SimpleNamespace(class_id='2', score=score))])
        message = SimpleNamespace(detections=[detection(), detection(float('nan')), *[detection() for _ in range(20)]])
        result = result_from_message(message, 3)
        self.assertEqual(result.count, 22)
        self.assertEqual(len(result.boxes), 2)
        self.assertIn('2 boxes shown', _caption(result, 'DETECTIONS'))

    def test_parameter_bounds_reject_invalid_limits(self):
        for values in ({'max_fps': float('nan')}, {'jpeg_quality': True}, {'cache_size': 0},
                       {'max_boxes': 1001}, {'cache_ttl_s': .1, 'match_wait_s': .2},
                       {'image_timeout_s': .1, 'match_wait_s': .2}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Settings(**values)


class NodeBoundaryTests(unittest.TestCase):
    """Run real callbacks with message/publisher boundaries faked, without ROS."""
    @classmethod
    def setUpClass(cls):
        modules = {name: ModuleType(name) for name in ('rclpy', 'rclpy.clock', 'rclpy.clock_type',
            'rclpy.node', 'rclpy.qos', 'rcl_interfaces', 'rcl_interfaces.msg',
            'sensor_msgs', 'sensor_msgs.msg', 'vision_msgs', 'vision_msgs.msg')}
        modules['rcl_interfaces.msg'].ParameterDescriptor = object
        modules['rclpy.clock'].Clock = object
        modules['rclpy.clock_type'].ClockType = SimpleNamespace(STEADY_TIME=1)
        modules['rclpy.node'].Node = object
        modules['rclpy.qos'].QoSProfile = object
        modules['rclpy.qos'].ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
        modules['rclpy.qos'].DurabilityPolicy = SimpleNamespace(VOLATILE=1)
        class Compressed:
            def __init__(self):
                self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id='')
        modules['sensor_msgs.msg'].CompressedImage = Compressed
        modules['sensor_msgs.msg'].Image = object
        modules['vision_msgs.msg'].Detection2DArray = object
        path = Path(__file__).resolve().parents[1] / 'kart_opponent_tracker/debug_image_node.py'
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('kart_opponent_tracker.debug_image_node_boundary_test', path)
            cls.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.module)

    def node(self, subscribers=1):
        return SimpleNamespace(settings=Settings(), buffer=OverlayBuffer(Settings()), class_names=[''],
            last_image=None, last_admitted=float('-inf'), last_admitted_key=None, last_published=float('-inf'),
            source_issue='RGB: waiting for camera input',
            output=SimpleNamespace(get_subscription_count=lambda: subscribers, publish=Mock()),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=9_000_000_000)),
            get_logger=lambda: SimpleNamespace(warning=Mock()))

    def test_no_subscriber_skips_pixel_conversion_and_jpeg_encoding(self):
        node = self.node(0)
        with patch.object(self.module, 'decode_image') as decode, patch.object(self.module, 'encode_jpeg') as encode:
            self.module.DebugImageNode.image(node, object())
            self.module.DebugImageNode.result(node, 'detections', object())
            self.module.DebugImageNode.publish(node)
            decode.assert_not_called()
            encode.assert_not_called()
        node.output.publish.assert_not_called()

    def test_real_source_stamp_is_preserved_and_output_is_rate_limited(self):
        node = self.node()
        node.last_image = 0.
        node.buffer.add_image(KEY, np.zeros((480, 640, 3), np.uint8), 0.)
        with patch.object(self.module.time, 'monotonic', return_value=.21):
            self.module.DebugImageNode.publish(node)
            self.module.DebugImageNode.publish(node)
        node.output.publish.assert_called_once()
        output = node.output.publish.call_args.args[0]
        self.assertEqual((output.header.stamp.sec, output.header.stamp.nanosec), (1, 0))
        self.assertEqual(output.header.frame_id, KEY[1])
        self.assertTrue(output.data.startswith(b'\xff\xd8'))

    def test_stalled_input_uses_labeled_placeholder_not_pending_stale_pixels(self):
        node = self.node()
        node.last_image = 0.
        node.source_issue = ''
        node.buffer.add_image(KEY, np.zeros((480, 640, 3), np.uint8), 0.)
        with patch.object(self.module.time, 'monotonic', return_value=2.), \
                patch.object(self.module, 'draw_frame') as draw, \
                patch.object(self.module, 'placeholder', wraps=placeholder) as missing:
            self.module.DebugImageNode.publish(node)
        draw.assert_not_called()
        self.assertIn('MISSING', missing.call_args.args[0])
        output = node.output.publish.call_args.args[0]
        self.assertEqual(output.header.frame_id, '')
        self.assertEqual(output.header.stamp.sec, 9)
        self.assertEqual(len(node.buffer.images), 0)

    def test_high_rate_images_are_sampled_before_decoding_and_caching(self):
        node = self.node()
        rgb = SimpleNamespace(width=640, height=480, encoding='rgb8', step=1920,
            data=np.zeros((480, 640, 3), np.uint8).tobytes(),
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0), frame_id=KEY[1]))
        with patch.object(self.module, 'decode_image', wraps=decode_image) as decode:
            for i in range(60):
                rgb.header.stamp.nanosec = i * 10_000_000
                with patch.object(self.module.time, 'monotonic', return_value=i / 60):
                    self.module.DebugImageNode.image(node, rgb)
        self.assertLessEqual(decode.call_count, 5)
        self.assertGreaterEqual(decode.call_count, 4)
        self.assertEqual(len(node.buffer.images), decode.call_count)

    def test_current_camera_with_expired_pending_frame_waits_without_false_missing(self):
        node = self.node()
        node.last_image = 1.
        pixels = np.zeros((480, 640, 3), np.uint8)
        node.buffer.add_image(KEY, pixels, 0.)
        node.buffer.add_image((KEY[0] + 1, KEY[1]), pixels, 1.)
        with patch.object(self.module.time, 'monotonic', return_value=1.1), \
                patch.object(self.module, 'draw_frame') as draw, \
                patch.object(self.module, 'placeholder') as missing:
            self.module.DebugImageNode.publish(node)
        draw.assert_not_called()
        missing.assert_not_called()
        node.output.publish.assert_not_called()

    def test_repeated_source_header_does_not_refresh_stopped_image(self):
        node = self.node()
        rgb = SimpleNamespace(width=640, height=480, encoding='rgb8', step=1920,
            data=np.zeros((480, 640, 3), np.uint8).tobytes(),
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0), frame_id=KEY[1]))
        with patch.object(self.module, 'decode_image', wraps=decode_image) as decode:
            with patch.object(self.module.time, 'monotonic', return_value=0.):
                self.module.DebugImageNode.image(node, rgb)
            with patch.object(self.module.time, 'monotonic', return_value=.21):
                self.module.DebugImageNode.publish(node)
            with patch.object(self.module.time, 'monotonic', return_value=1.5):
                self.module.DebugImageNode.image(node, rgb)
                self.module.DebugImageNode.publish(node)
            self.assertEqual(decode.call_count, 1)
        self.assertEqual(node.last_image, 0.)
        self.assertEqual(node.output.publish.call_args.args[0].header.frame_id, '')
