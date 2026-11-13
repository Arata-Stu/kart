"""Map display transport bounds, explicit semantics, clocks and diagnostics."""
import json
import math
from types import SimpleNamespace as NS
import unittest

from kart_hdmap.ground_perception import GroundPerception


STAMP = 1_234_567_890_123_456_789


def header(stamp=STAMP, frame='map'):
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame)


def point(kind='rc_car', class_id='0', score=.8, x=2., identity='frame-0'):
    return NS(class_name=kind, class_id=class_id, score=score, detection_id=identity,
              point=NS(x=x, y=-1., z=.1))


def message(objects=(), stamp=STAMP, frame='map', valid=True, reason='projected'):
    return NS(header=header(stamp, frame), objects=list(objects), valid=valid, reason=reason)


def diagnostic(reason='projected', level=0, stamp=STAMP, name='perception_ground/projection'):
    status = NS(name=name, level=level, message='投影状態', values=[NS(key='reason', value=reason)])
    return NS(header=header(stamp), status=[status])


class GroundPerceptionTests(unittest.TestCase):
    def test_absent_waiting_and_real_empty_are_different(self):
        status = GroundPerception()
        self.assertFalse(status.view(False, False, now=10)['available'])
        waiting = status.view(True, True, now=10)
        self.assertFalse(waiting['fresh'])
        self.assertIsNone(waiting['count'])
        self.assertIsNone(waiting['age_s'])
        status.capture(message(), now=10)
        empty = status.view(True, True, now=10.1)
        self.assertTrue(empty['fresh'])
        self.assertEqual(empty['count'], 0)
        self.assertEqual(empty['objects'], [])

    def test_map_coordinates_canonical_names_and_source_class_ids_survive_json(self):
        status = GroundPerception()
        status.capture(message([point('rc_car', '2'), point('obstacle', '1', x=-4.)]), now=10)
        result = status.view(True, True, now=10.1, source_now_ns=STAMP + 200_000_000)
        value = json.loads(json.dumps(result, allow_nan=False))
        self.assertEqual(value['frame_id'], 'map')
        self.assertEqual(value['stamp_ns'], str(STAMP))
        self.assertEqual(value['count'], 2)
        self.assertAlmostEqual(value['fresh_for_s'], .8)
        self.assertEqual(value['objects'][0], dict(detection_id='frame-0', class_id='2', kind='rc_car',
                                                  score=.8, x=2., y=-1., z=.1))
        self.assertEqual(value['objects'][1]['kind'], 'obstacle')
        self.assertEqual(value['objects'][1]['x'], -4.)

    def test_stale_source_receipt_disconnect_and_wrong_frame_hide_old_coordinates(self):
        for source_now, now, available in ((STAMP, 11.01, True), (STAMP, 9, True),
                                           (STAMP + 1_010_000_000, 10.1, True),
                                           (STAMP - 60_000_000, 10.1, True), (STAMP, 10.1, False)):
            status = GroundPerception()
            status.capture(message([point()]), now=10)
            result = status.view(available, True, now=now, source_now_ns=source_now)
            with self.subTest(source_now=source_now, now=now, available=available):
                self.assertFalse(result['fresh'])
                self.assertIsNone(result['count'])
                self.assertEqual(result['objects'], [])
                self.assertEqual(result['fresh_for_s'], 0)
        for source in (message([point()], frame='base_link'), message([point()], stamp=0)):
            status.capture(source, now=10)
            self.assertFalse(status.view(True, True, now=10)['fresh'])
        status.capture(message([point()]), valid=False, now=10)
        self.assertFalse(status.view(True, True, now=10)['fresh'])

    def test_malformed_points_are_filtered_without_inventing_class_semantics(self):
        status = GroundPerception()
        rows = [point(kind='person', class_id='0'), point(kind=''), point(class_id=''),
                point(x=math.nan), point(x=math.inf), point(score=math.nan), point(score=1.1), point()]
        status.capture(message(rows), now=10)
        result = status.view(True, True, now=10)
        self.assertEqual(result['count'], len(rows))
        self.assertEqual(len(result['objects']), 1)
        self.assertTrue(result['truncated'])
        json.dumps(result, allow_nan=False)
        status.capture(message(rows[:-1]), now=10)
        invalid = status.view(True, True, now=10)
        self.assertFalse(invalid['fresh'])
        self.assertIsNone(invalid['count'], 'all invalid objects must not masquerade as a real zero')

    def test_points_and_metadata_are_bounded(self):
        status = GroundPerception()
        status.capture(message([point(identity='x'*1000) for _ in range(1000)]), now=10)
        value = status.view(True, True, now=10)
        self.assertEqual(value['count'], 1000)
        self.assertEqual(len(value['objects']), 100)
        self.assertEqual(len(value['objects'][0]['detection_id']), 128)
        self.assertTrue(value['truncated'])

    def test_warning_diagnostic_immediately_invalidates_previous_success(self):
        status = GroundPerception()
        status.capture(message([point()]), now=10)
        status.capture_diagnostic(diagnostic('map_tf_unavailable', 1), now=10.1)
        failed = status.view(True, True, now=10.2)
        self.assertTrue(failed['diagnostic']['fresh'])
        self.assertEqual(failed['diagnostic']['reason'], 'map_tf_unavailable')
        self.assertFalse(failed['fresh'])
        self.assertEqual(failed['objects'], [])
        status.capture_diagnostic(diagnostic('projected'), now=10.3)
        self.assertTrue(status.view(True, True, now=10.4)['fresh'])

    def test_invalid_clear_event_never_claims_zero_before_the_next_diagnostic(self):
        status = GroundPerception()
        status.capture(message([point()]), now=10)
        status.capture_diagnostic(diagnostic('projected'), now=10)
        status.capture(message(stamp=STAMP + 600_000_000, valid=False, reason='detections_stale'), now=10.6)
        value = status.view(True, True, now=10.7, source_now_ns=STAMP + 700_000_000)
        self.assertFalse(value['fresh'])
        self.assertIsNone(value['count'])
        self.assertEqual(value['objects'], [])
        self.assertTrue(value['diagnostic']['fresh'])
        self.assertEqual(value['diagnostic']['reason'], 'detections_stale')
        self.assertEqual(value['diagnostic']['source'], 'map_points')
        self.assertEqual(value['diagnostic']['level'], 1)
        # A same-stamp clear must also overwrite the preceding valid event.
        status.capture(message([point()]), now=11)
        status.capture(message(valid=False, reason='target_tf_unavailable'), now=11.01)
        self.assertIsNone(status.view(True, True, now=11.02)['count'])
        self.assertEqual(status.view(True, True, now=11.02)['diagnostic']['reason'], 'target_tf_unavailable')

    def test_diagnostic_source_and_receipt_expire_independently_at_2_5_seconds(self):
        status = GroundPerception()
        status.capture_diagnostic(diagnostic(), now=10)
        result = status.view(True, True, now=11, source_now_ns=STAMP + 2_000_000_000)
        self.assertTrue(result['diagnostic']['fresh'])
        self.assertEqual(result['diagnostic']['fresh_for_s'], .5)
        self.assertFalse(status.view(True, True, now=12.51)['diagnostic']['fresh'])
        self.assertFalse(status.view(True, True, now=11, source_now_ns=STAMP + 2_510_000_000)['diagnostic']['fresh'])
        self.assertFalse(status.view(True, False, now=11)['diagnostic']['fresh'])

    def test_unrelated_diagnostics_do_not_overwrite_projection_state(self):
        status = GroundPerception()
        status.capture_diagnostic(diagnostic('map_tf_unavailable', 1), now=10)
        status.capture_diagnostic(diagnostic('other', name='other_node'), now=10.1)
        self.assertEqual(status.view(True, True, now=10.2)['diagnostic']['reason'], 'map_tf_unavailable')


if __name__ == '__main__':
    unittest.main()
