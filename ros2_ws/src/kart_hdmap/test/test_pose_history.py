import unittest
from kart_hdmap.pose_history import PoseHistory, validate_rate

P = dict(x=1., y=2., yaw=.3)


class PoseHistoryTest(unittest.TestCase):
    def test_90hz_batches_and_cursor_are_lossless(self):
        h = PoseHistory()
        for i in range(90):
            h.add(1_000_000_000 + i * 11_111_112, P, True)
        batch = h.batch()
        self.assertEqual(len(batch['pose_samples']), 90)
        self.assertEqual(h.batch(batch['pose_stream'], batch['pose_cursor'])['pose_samples'], [])
        h.add(h.last_seen, P, True)
        self.assertEqual(h.sequence, 90)
        h.add(h.last_seen + 11_111_112, P, True)
        self.assertEqual(len(h.batch(batch['pose_stream'], batch['pose_cursor'])['pose_samples']), 1)

    def test_rate_changes_follow_source_without_synthetic_points(self):
        h = PoseHistory()
        for i in range(90):
            h.add(1_000_000_000 + i * 11_111_112, P, True, 30.)
        self.assertEqual(len(h.samples), 30)
        for i in range(30):
            h.add(2_000_000_000 + i * 33_333_334, P, True, 0.)
        self.assertEqual(len(h.samples), 60)

    def test_overflow_clock_reset_and_invalid_pose_breaks(self):
        h = PoseHistory(3)
        for i in range(5): h.add(1_000_000_000+i*10_000_000, P, True)
        first = h.batch()
        self.assertEqual(len(first['pose_samples']), 3)
        self.assertTrue(h.batch(first['pose_stream'], 1)['pose_reset'])
        h.add(0, {}, False)
        h.add(1_100_000_000, P, True)
        self.assertTrue(h.samples[-1]['break_before'])
        h.add(500_000_000, P, True)
        after = h.batch(first['pose_stream'], first['pose_cursor'])
        self.assertTrue(after['pose_reset'])
        self.assertEqual(len(after['pose_samples']), 1)
        self.assertNotEqual(after['pose_stream'], first['pose_stream'])

    def test_invalid_rate_and_cursor(self):
        for value in (-1, float('nan'), float('inf'), True, 1001):
            with self.assertRaises(ValueError): validate_rate(value)
        for value in (-1, True, '1'):
            with self.assertRaises(ValueError): PoseHistory().batch(after=value)
