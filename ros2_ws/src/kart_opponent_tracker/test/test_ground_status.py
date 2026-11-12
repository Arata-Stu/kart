"""Projection input contracts and live diagnostic state, without ROS emulation."""
import copy
import math
from types import SimpleNamespace as NS
import unittest

from kart_opponent_tracker.ground_status import GroundStatus, projection_issue, stamp_ns


STAMP = 1_234_567_890_123_456_789


def header(stamp=STAMP, frame='camera_color_optical_frame'):
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame)


def calibration():
    return NS(header=header(), width=640, height=480, binning_x=0, binning_y=0,
              roi=NS(x_offset=0, y_offset=0, width=0, height=0),
              k=[500., 0., 320., 0., 500., 240., 0., 0., 1.],
              d=[0., 0., 0., 0., 0.], distortion_model='plumb_bob')


class GroundInputTests(unittest.TestCase):
    def test_original_fixed_rgb_contract_and_exact_stamp_are_preserved(self):
        tracks, info = NS(header=header()), calibration()
        self.assertIsNone(projection_issue(tracks, info))
        self.assertEqual(stamp_ns(tracks.header.stamp), STAMP)
        info.roi.width, info.roi.height = 640, 480
        self.assertIsNone(projection_issue(tracks, info))
        info.header.stamp.nanosec += 1
        self.assertEqual(projection_issue(tracks, info), 'not_synchronized')

    def test_invalid_stamp_frame_shape_and_calibration_have_distinct_reasons(self):
        good_tracks, good_info = NS(header=header()), calibration()
        cases = ('stamp', 'frame', 'size', 'binning', 'roi_offset', 'roi_size',
                 'model', 'focal', 'nonfinite_k', 'k_length', 'nonfinite_d', 'd_length')
        for case in cases:
            tracks, info = copy.deepcopy(good_tracks), copy.deepcopy(good_info)
            expected = 'invalid_calibration'
            if case == 'stamp':
                tracks.header = header(stamp=0)
                expected = 'invalid_stamp'
            elif case == 'frame':
                info.header.frame_id = 'other_camera'
                expected = 'invalid_frame'
            elif case == 'size':
                info.width = 848
                expected = 'invalid_image_shape'
            elif case == 'binning':
                info.binning_x = 2
                expected = 'invalid_image_shape'
            elif case == 'roi_offset':
                info.roi.x_offset = 1
                expected = 'invalid_image_shape'
            elif case == 'roi_size':
                info.roi.width, info.roi.height = 320, 240
                expected = 'invalid_image_shape'
            elif case == 'model':
                info.distortion_model = 'equidistant'
            elif case == 'focal':
                info.k[4] = 0.
            elif case == 'nonfinite_k':
                info.k[0] = math.nan
            elif case == 'k_length':
                info.k.pop()
            elif case == 'nonfinite_d':
                info.d[0] = math.inf
            else:
                info.d = [0., 0.]
            with self.subTest(case=case):
                self.assertEqual(projection_issue(tracks, info), expected)


class GroundStatusTests(unittest.TestCase):
    def setUp(self):
        self.status = GroundStatus()
        self.frame = 'camera_color_optical_frame'

    def reason(self, now):
        return self.status.view(now)['values']['reason']

    def receive(self, count=1, now=10, stamp=STAMP):
        self.status.tracks(stamp, self.frame, count, now)
        self.status.camera_info(stamp, self.frame, now)

    def test_raw_input_counts_explain_missing_camera_info_and_exact_sync(self):
        self.assertEqual(self.reason(10), 'waiting_tracks')
        self.status.tracks(STAMP, self.frame, 1, now=10)
        self.assertEqual(self.reason(10), 'waiting_camera_info')
        self.status.camera_info(STAMP+1, self.frame, now=10.1)
        result = self.status.view(10.1)
        self.assertEqual(result['values']['reason'], 'not_synchronized')
        self.assertEqual(result['values']['tracks_received'], '1')
        self.assertEqual(result['values']['camera_info_received'], '1')
        self.assertEqual(result['values']['synchronized_received'], '0')
        self.assertEqual(result['values']['tracks_stamp_ns'], str(STAMP))
        self.assertEqual(result['values']['camera_info_stamp_ns'], str(STAMP+1))

    def test_zero_tracks_is_healthy_but_projection_rejection_and_tf_are_warnings(self):
        self.receive(count=0)
        self.status.synchronized(STAMP, 'zero_tracks', now=10)
        zero = self.status.view(10.1)
        self.assertEqual(zero['level'], 0)
        self.assertEqual(zero['values']['last_tracks_count'], '0')
        self.assertEqual(zero['values']['last_projection_count'], '0')
        for reason in ('tf_unavailable', 'all_rejected', 'invalid_stamp', 'invalid_frame',
                       'invalid_image_shape', 'invalid_calibration'):
            with self.subTest(reason=reason):
                self.status.synchronized(STAMP, reason, detail='explanation', now=10.2)
                result = self.status.view(10.3)
                self.assertEqual(result['level'], 1)
                self.assertEqual(result['values']['reason'], reason)
                self.assertEqual(result['values']['detail'], 'explanation')

    def test_success_counts_points_and_next_unsynchronized_track_cannot_keep_success(self):
        self.receive(count=3)
        self.status.synchronized(STAMP, 'projected', projected=2, now=10)
        success = self.status.view(10.1)
        self.assertEqual(success['level'], 0)
        self.assertEqual(success['values']['last_projection_count'], '2')
        self.assertEqual(success['values']['projected_points'], '2')
        self.assertEqual(success['values']['synchronized_received'], '1')
        self.status.tracks(STAMP+1, self.frame, 1, now=10.2)
        pending = self.status.view(10.3)
        self.assertEqual(pending['level'], 1)
        self.assertEqual(pending['values']['reason'], 'not_synchronized')

    def test_stopped_inputs_and_stale_sync_clear_old_success_using_steady_age(self):
        self.receive()
        self.status.synchronized(STAMP, 'projected', projected=1, now=10)
        self.assertEqual(self.reason(11.01), 'tracks_stale')
        self.status.tracks(STAMP+1, self.frame, 1, now=11.1)
        self.assertEqual(self.reason(11.1), 'camera_info_stale')
        self.status.camera_info(STAMP+1, self.frame, now=11.2)
        self.assertEqual(self.reason(11.2), 'not_synchronized')
        self.status.synchronized(STAMP+1, 'projected', projected=1, now=11.2)
        # CameraInfo can arrive ahead of slow inference; newer CameraInfo must
        # not invalidate a matching synchronized track from the same camera.
        self.status.camera_info(STAMP+2, self.frame, now=11.3)
        self.assertEqual(self.reason(11.3), 'projected')
        self.assertEqual(self.reason(9), 'tracks_stale')

    def test_invalid_raw_header_is_visible_without_a_synchronized_callback(self):
        self.status.tracks(0, self.frame, 1, now=10)
        self.assertEqual(self.reason(10), 'invalid_stamp')
        self.status.tracks(STAMP, '', 1, now=10)
        self.assertEqual(self.reason(10), 'invalid_frame')
        self.receive(now=10)
        self.status.camera_info(STAMP, 'different_camera', now=10)
        self.assertEqual(self.reason(10), 'invalid_frame')

    def test_details_and_frame_names_are_bounded(self):
        self.status.tracks(STAMP, 'x'*1000, 1, now=10)
        self.status.camera_info(STAMP, 'x'*1000, now=10)
        self.status.synchronized(STAMP, 'tf_unavailable', detail='y'*1000, now=10)
        values = self.status.view(10)['values']
        self.assertEqual(len(values['detail']), 256)
        self.assertEqual(len(values['tracks_frame']), 256)
        with self.assertRaises(ValueError):
            self.status.synchronized(STAMP, 'unknown', now=10)


if __name__ == '__main__':
    unittest.main()
