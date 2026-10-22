import unittest
from tools.pipeline_audit.evs_clock import progress


def row(bag, sensor, offset, epoch=0):
    return dict(topic='/timing', bag_ns=bag, payload={'json': dict(
        stream_id='camera', sensor_epoch=epoch, window_end_sensor_us=sensor,
        sensor_to_ros_offset_ns=offset)})


class ClockTests(unittest.TestCase):
    def test_receipt_mapping_can_hide_sensor_progress_deficit(self):
        # Recomputed offsets make header age zero even when sensor progress lags.
        rows = [row(1_000_000_000, 100_000, 900_000_000),
                row(3_000_000_000, 1_100_000, 1_900_000_000)]
        segments, series = progress(rows, '/timing')
        self.assertEqual(segments[0]['recorded_elapsed_s'], 2)
        self.assertEqual(segments[0]['sensor_elapsed_s'], 1)
        self.assertEqual(series[-1]['recorded_minus_sensor_elapsed_ms'], 1000)
        self.assertEqual(series[-1]['offset_change_ms'], 1000)

    def test_epoch_is_reanchored_not_combined(self):
        segments, series = progress([row(100, 0, 100), row(200, 0, 200, epoch=1)], '/timing')
        self.assertEqual(len(segments), 2)
        self.assertTrue(all(s['offset_change_ms'] == 0 for s in series))

    def test_missing_and_invalid_metadata_are_not_zero_drift(self):
        broken = row(100, 0, 100)
        broken['payload']['json'].pop('sensor_to_ros_offset_ns')
        self.assertEqual(progress([broken], '/timing'), ([], []))
        self.assertEqual(progress([row(100, 0, 100)], '/other'), ([], []))
