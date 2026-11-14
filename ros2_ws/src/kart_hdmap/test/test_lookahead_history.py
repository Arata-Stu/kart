import json
import unittest
from types import SimpleNamespace as NS

from kart_hdmap.lookahead_history import LookaheadHistory


def statuses(level=0, **changes):
    values = dict(controller='pure_pursuit', lookahead_mode='speed_interpolated',
                  lookahead_m=.8, target_distance_m=.7,
                  measured_speed_mps=2.5, target_speed_mps=3.)
    values.update(changes)
    return [NS(name='lateral_controller', hardware_id='kart_control', level=level,
               values=[NS(key=k, value=str(v)) for k, v in values.items()])]


class LookaheadHistoryTests(unittest.TestCase):
    def test_actual_values_and_independent_missing_fields(self):
        h = LookaheadHistory()
        h.capture(1, statuses(measured_speed_mps=-1), True, now=0)
        self.assertEqual(h.samples[-1]['measured_speed_mps'], -1)
        h.capture(2, statuses(lookahead_mode='fixed', measured_speed_mps='nan'), True, now=.01)
        row = h.samples[-1]
        self.assertIsNone(row['measured_speed_mps'])
        self.assertEqual(row['lookahead_m'], .8)
        self.assertEqual(row['target_speed_mps'], 3.)
        h.capture(3, statuses(controller='mpc', lookahead_mode='', lookahead_m=2,
                              target_distance_m=2), True, now=.02)
        self.assertEqual(h.samples[-1]['controller'], 'mpc')
        self.assertIsNone(h.samples[-1]['lookahead_mode'])
        self.assertEqual(h.samples[-1]['target_distance_m'], 2.)
        legacy = statuses()
        legacy[0].values = [NS(key='processing_ms', value='1.5')]
        h.capture(4, legacy, True, now=.03)
        self.assertTrue(all(h.samples[-1][key] is None for key in h.FIELDS))
        self.assertIsNone(h.samples[-1]['controller'])
        json.dumps(h.view(now=.03), allow_nan=False)

    def test_invalid_level_stale_and_per_field_sanity(self):
        h = LookaheadHistory()
        h.capture(1, statuses(), True, now=0)
        h.capture(2, statuses(level=2), True, now=.01)
        self.assertTrue(all(h.samples[-1][key] is None for key in h.FIELDS))
        h.capture(3, statuses(), True, now=.02)
        h.capture(4, statuses(), False, now=.03)
        self.assertEqual(len(h.samples), 4)  # Preserve each validity transition.
        h.capture(5, statuses(lookahead_m=0, target_distance_m=-1,
                              measured_speed_mps='inf', target_speed_mps=-1), True, now=.04)
        self.assertTrue(all(h.samples[-1][key] is None for key in h.FIELDS))
        h.capture(6, statuses(lookahead_m=1001, target_distance_m=1001,
                              measured_speed_mps=101, target_speed_mps=101), True, now=.2)
        self.assertTrue(all(h.samples[-1][key] is None for key in h.FIELDS))
        h.capture(7, statuses(target_distance_m=0, target_speed_mps=0), True, now=.4)
        self.assertEqual(h.samples[-1]['target_distance_m'], 0)
        self.assertEqual(h.samples[-1]['target_speed_mps'], 0)

    def test_duplicate_stamp_never_refreshes_and_rewind_changes_stream(self):
        h = LookaheadHistory()
        h.capture(2, statuses(), True, now=0)
        stream = h.stream
        h.capture(2, statuses(lookahead_m=4), True, now=.4)
        self.assertEqual(len(h.samples), 1)
        self.assertEqual(h.samples[-1]['lookahead_m'], .8)
        h.capture(2, statuses(), True, now=.6)
        self.assertEqual(len(h.samples), 2)
        self.assertEqual(h.samples[-1]['t'], .5)
        self.assertIsNone(h.samples[-1]['lookahead_m'])
        self.assertEqual(h.stream, stream)
        h.view(now=.8)
        self.assertEqual(len(h.samples), 2)
        h.capture(1, statuses(), True, now=1)
        self.assertNotEqual(h.stream, stream)
        self.assertEqual(len(h.samples), 1)
        self.assertEqual(h.samples[-1]['t'], 1)

    def test_source_absence_and_receipt_gap_are_missing_data(self):
        h = LookaheadHistory()
        wrong = statuses()
        wrong[0].hardware_id = 'other'
        h.capture(1, wrong, True, now=0)
        h.capture(2, statuses() * 2, True, now=.1)
        self.assertEqual(len(h.samples), 0)
        h.capture(3, statuses(), True, now=.2)
        h.capture(4, statuses(), True, now=2)
        self.assertEqual(len(h.samples), 3)
        self.assertEqual(h.samples[1]['t'], .7)
        self.assertIsNone(h.samples[1]['lookahead_m'])
        self.assertEqual(h.samples[2]['lookahead_m'], .8)
        h.view(now=2.6)
        self.assertIsNone(h.samples[-1]['lookahead_m'])

    def test_bounded_sampling_keeps_mode_and_partial_validity_changes(self):
        h = LookaheadHistory()
        for i in range(10000):
            h.capture(i + 1, statuses(lookahead_m=.8 + i / 10000), True, now=i / 100)
        self.assertGreater(len(h.samples), 500)
        self.assertLessEqual(len(h.samples), 650)
        self.assertTrue(all(row['t'] >= 40 for row in h.view(now=100)['samples']))
        h.capture(10001, statuses(lookahead_mode='fixed'), True, now=100)
        self.assertEqual(h.samples[-1]['lookahead_mode'], 'fixed')
        h.capture(10002, statuses(lookahead_mode='fixed', measured_speed_mps='nan'), True, now=100.01)
        self.assertIsNone(h.samples[-1]['measured_speed_mps'])
        for i in range(2000):
            h.capture(20000 + i, statuses(level=i % 2 * 2), True, now=101 + i / 1000)
        self.assertLessEqual(len(h.samples), 650)
        self.assertEqual(h.view(now=200)['samples'], [])
