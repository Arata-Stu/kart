"""Detection display transport bounds/freshness; no ROS runtime emulation."""
import json
import math
from types import SimpleNamespace as NS
import unittest

from kart_hdmap.perception_status import PerceptionStatus


def detection(identity='generation-7', class_id='2', score=.8, width=40.):
    return NS(id=identity, bbox=NS(center=NS(position=NS(x=100., y=200.)), size_x=width, size_y=50.),
              results=[NS(hypothesis=NS(class_id=class_id, score=score))])


def message(objects=(), stamp=1_234_567_890_123_456_789):
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id='camera_rgb_optical_frame'),
              detections=list(objects))


class PerceptionStatusTests(unittest.TestCase):
    def test_no_input_is_unknown_but_fresh_empty_is_zero(self):
        channel = PerceptionStatus()
        waiting = channel.view(True, now=0)
        self.assertFalse(waiting['fresh'])
        self.assertIsNone(waiting['count'])
        self.assertIsNone(waiting['age_s'])
        channel.capture(message(), now=10)
        empty = channel.view(True, now=10.1)
        self.assertTrue(empty['fresh'])
        self.assertEqual(empty['count'], 0)
        self.assertEqual(empty['objects'], [])
        self.assertFalse(empty['truncated'])

    def test_ids_score_bbox_and_nanosecond_stamp_survive_json(self):
        channel = PerceptionStatus()
        channel.capture(message([detection(), detection('person-track', '0', .9)]), now=10)
        value = json.loads(json.dumps(channel.view(True, now=10.2), allow_nan=False))
        self.assertEqual(value['stamp_ns'], '1234567890123456789')
        self.assertEqual(value['frame_id'], 'camera_rgb_optical_frame')
        self.assertEqual(value['count'], 2)
        self.assertEqual(value['objects'][0],
                         dict(id='generation-7', class_id='2', score=.8, bbox=[100., 200., 40., 50.]))
        self.assertEqual(value['objects'][1]['class_id'], '0')
        self.assertNotIn('class_name', value['objects'][1])

    def test_disconnect_age_expiry_or_invalid_source_clears_previous_objects(self):
        for available, now in ((False, 10.1), (True, 12.01), (True, 9)):
            with self.subTest(available=available, now=now):
                channel = PerceptionStatus()
                channel.capture(message([detection()]), now=10)
                value = channel.view(available, now=now)
                self.assertFalse(value['fresh'])
                self.assertIsNone(value['count'])
                self.assertEqual(value['objects'], [])
        channel.capture(message([detection()]), valid=False, now=20)
        self.assertIsNone(channel.view(True, now=20)['count'])

    def test_source_age_is_rechecked_even_when_messages_keep_arriving(self):
        channel = PerceptionStatus()
        channel.capture(message([detection()], stamp=10_000_000_000), now=50)
        self.assertTrue(channel.view(True, now=50.1, source_now_ns=10_100_000_000)['fresh'])
        self.assertAlmostEqual(channel.view(True, now=50.1, source_now_ns=10_100_000_000)['fresh_for_s'], 1.9)
        self.assertAlmostEqual(channel.view(True, now=50.1, source_now_ns=11_990_000_000)['fresh_for_s'], .01)
        self.assertFalse(channel.view(True, now=50.2, source_now_ns=12_100_000_000)['fresh'])
        self.assertEqual(channel.view(True, now=50.2, source_now_ns=12_100_000_000)['fresh_for_s'], 0.)
        self.assertFalse(channel.view(True, now=50.2, source_now_ns=9_900_000_000)['fresh'])
        channel.capture(message(stamp=0), now=51)
        self.assertFalse(channel.view(True, now=51)['fresh'])

    def test_frame_copy_and_payload_are_bounded(self):
        channel = PerceptionStatus()
        objects = [detection(str(index)) for index in range(1000)]
        channel.capture(message(objects), now=10)
        value = channel.view(True, now=10)
        self.assertEqual(value['count'], 1000)
        self.assertEqual(len(value['objects']), 10)
        self.assertTrue(value['truncated'])
        self.assertEqual(value['objects'][-1]['id'], '9')
        self.assertLess(len(json.dumps(value)), 4096)

    def test_nonfinite_geometry_and_invalid_hypotheses_do_not_enter_json(self):
        channel = PerceptionStatus()
        objects = [detection(score=math.nan), detection(score=math.inf), detection(score=1.1),
                   detection(width=math.inf), detection(width=-1), detection(class_id=''), detection()]
        channel.capture(message(objects), now=10)
        value = channel.view(True, now=10)
        self.assertEqual(value['count'], len(objects))
        self.assertEqual(len(value['objects']), 1)
        self.assertTrue(value['truncated'])
        json.dumps(value, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
