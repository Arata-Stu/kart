import unittest
from kart_e2e.evs_alignment import infer_alignment, WRAP_US


def row(seq, epoch, sensor, offset):
    return dict(stream_id='camera', sequence=seq, sensor_epoch=epoch,
                latest_event_sensor_us=sensor, window_end_sensor_us=sensor+1,
                tensor_stamp_ros_ns=sensor*1000+offset, sensor_to_ros_offset_ns=offset)


class AlignmentTests(unittest.TestCase):
    def test_recording_1422_four_wraps(self):
        rows = [row(0, 0, 75064000, 10**18), row(1, 0, 126228000, 10**18)]
        offsets, evidence = infer_alignment(rows, 7903419, 59159135)
        self.assertEqual(offsets, {'0': -67108864})
        self.assertLess(abs(evidence['last_edge_residual_us']), 50000)

    def test_old_recording_reset_two_wraps(self):
        rows = [row(2442, 0, 10031685, 1734475431480),
                row(11557, 0, 46559999, 1734474243624),
                row(11558, 2, 13061567, 1768028637192),
                row(15444, 2, 28633566, 1768028216592)]
        offsets, _ = infer_alignment(rows, 10081667, 62187998)
        self.assertEqual(offsets, {'0': 0, '2': 33554432})

    def test_zero_offset_and_metadata(self):
        rows = [row(0, 0, 100000, 10**12), row(1, 0, 5100000, 10**12)]
        offsets, _ = infer_alignment(rows, 110000, 5110000, 10**12+100000000)
        self.assertEqual(offsets, {'0': 0})
        with self.assertRaisesRegex(ValueError, 'metadata'):
            infer_alignment(rows, 110000, 5110000, 10**12+10000000000)

    def test_reject_different_duration_or_arbitrary_shift(self):
        rows = [row(0, 0, 100000, 10**12), row(1, 0, 5100000, 10**12)]
        for first, last in [(100000, 25100000), (3100000, 8100000)]:
            with self.assertRaisesRegex(ValueError, '一意'):
                infer_alignment(rows, first, last)

    def test_reject_offset_drift_and_hidden_wrap(self):
        for change in (2000000, WRAP_US):
            rows = [row(0, 0, 100000, 10**12), row(1, 0, 5100000, 10**12+change*1000)]
            with self.assertRaises(ValueError):
                infer_alignment(rows, 100000, 5100000)

    def test_reject_reversal_multiple_streams_and_empty(self):
        rows = [row(0, 0, 100000, 10**12), row(1, 0, 5100000, 10**12)]
        for bad in ([], list(reversed(rows)), [rows[0], dict(rows[1], stream_id='other')]):
            with self.assertRaises(ValueError):
                infer_alignment(bad, 100000, 5100000)
