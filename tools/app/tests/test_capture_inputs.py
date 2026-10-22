"""Reject VGL image shape mismatches before starting ROS components."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from kart_mapping.capture_snapshot import last_stereo_stamp


def frame(width, height, seconds):
    return SimpleNamespace(width=width, height=height,
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=0)))


class CaptureInputTests(unittest.TestCase):
    def check_frames(self, messages):
        class Reader:
            def __init__(self):
                self.rows = iter(messages)
                self.next_row = next(self.rows, None)

            def open(self, *_):
                pass

            def set_filter(self, *_):
                pass

            def has_next(self):
                return self.next_row is not None

            def read_next(self):
                row = self.next_row
                self.next_row = next(self.rows, None)
                return row

        with patch.dict('sys.modules', {
            'rosbag2_py': SimpleNamespace(SequentialReader=Reader,
                StorageOptions=lambda **kw: kw, ConverterOptions=lambda *args: args,
                StorageFilter=lambda **kw: kw),
            'rclpy.serialization': SimpleNamespace(deserialize_message=lambda raw, _: raw),
            'sensor_msgs.msg': SimpleNamespace(Image=object),
        }):
            return last_stereo_stamp('bag', dict(left_image='/left', right_image='/right'),
                                     [1, 3, 480, 640])

    def test_matching_stereo_size_and_tail(self):
        self.assertEqual(self.check_frames([
            ('/left', frame(640, 480, 10), 0), ('/right', frame(640, 480, 9), 0),
        ]), 9_000_000_000)

    def test_mismatch_in_either_stream_or_later_frame(self):
        for topic in ('/left', '/right'):
            with self.subTest(topic=topic), self.assertRaisesRegex(ValueError, '解像度'):
                self.check_frames([
                    ('/left', frame(640, 480, 1), 0), ('/right', frame(640, 480, 1), 0),
                    (topic, frame(424, 240, 2), 0),
                ])
