from dataclasses import replace
import math
import unittest

from kart_lap_manager.core import LapDetector, RequestState, Settings


SQUARE = [(0., 0.), (10., 0.), (10., 10.), (0., 10.)]


def position(s):
    s %= 40.
    if s < 10:
        return s, 0.
    if s < 20:
        return 10., s - 10
    if s < 30:
        return 30 - s, 10.
    return 0., 40 - s


def drive(detector, start, end, time0=10.):
    events = []
    steps = round((end-start)/.2)
    for i in range(steps+1):
        s = start + i * .2
        result = detector.sample(*position(s), time0 + i * .1)
        if result is not None:
            events.append(result)
    return events


class LapTests(unittest.TestCase):
    def detector(self):
        detector = LapDetector()
        detector.set_reference(SQUARE)
        return detector

    def test_arbitrary_start_partial_lap_is_not_counted_then_each_complete_lap_is(self):
        detector = self.detector()
        events = drive(detector, 17., 122.)
        self.assertEqual([e['lap'] for e in events], [1, 2])
        self.assertTrue(all(abs(e['lap_time_s']-20.) < .11 for e in events))

    def test_start_on_finish_counts_first_full_circuit(self):
        detector = self.detector()
        events = drive(detector, 0., 80.)
        self.assertEqual([e['lap'] for e in events], [1,2])
        self.assertTrue(all(abs(e['lap_time_s']-20.) < .11 for e in events))

    def test_full_distance_threshold_handles_roundoff(self):
        detector = LapDetector(replace(Settings(), min_lap_fraction=1.0))
        detector.set_reference(SQUARE)
        self.assertEqual(len(drive(detector, 0., 40.)), 1)

    def test_same_line_refresh_keeps_progress_but_changed_line_or_open_line_drops_it(self):
        detector = self.detector()
        drive(detector, 35., 65.)
        self.assertIsNotNone(detector.armed_at)
        self.assertFalse(detector.set_reference(SQUARE))
        self.assertIsNotNone(detector.armed_at)
        detector.set_reference([(x+1,y) for x,y in SQUARE])
        self.assertIsNone(detector.armed_at)
        with self.assertRaises(ValueError):
            detector.set_reference(SQUARE, False)
        self.assertEqual(detector.length, 0.)

    def test_finish_jitter_does_not_emit_or_accumulate_an_entire_lap(self):
        detector = self.detector()
        for i in range(500):
            detector.sample(*position(39.98 if i % 2 == 0 else .02), 10+i*.1)
        self.assertEqual(detector.count, 0)

    def test_reverse_and_shortcut_and_tracking_loss_cannot_complete_lap(self):
        for fault in ('reverse', 'shortcut', 'stale', 'rewind', 'lost', 'off_track'):
            detector = self.detector()
            drive(detector, 35., 60.)
            self.assertIsNotNone(detector.armed_at)
            if fault == 'reverse':
                detector.sample(*position(19.), 22.6)
            elif fault == 'shortcut':
                detector.sample(*position(39.9), 22.6)
            elif fault == 'stale':
                detector.sample(*position(20.2), 24.)
            elif fault == 'rewind':
                detector.sample(*position(20.2), 5.)
            elif fault == 'lost':
                detector.invalidate('tracking_lost')
            else:
                detector.sample(5., 5., 22.6)
            self.assertIsNone(detector.armed_at, fault)
            self.assertEqual(detector.count, 0)

    def test_nonfinite_and_duplicate_timestamp(self):
        detector = self.detector()
        detector.sample(1.,0.,1.)
        detector.sample(2.,0.,1.)
        self.assertEqual(detector.last[0], 1.)
        detector.sample(math.nan,0.,2.)
        self.assertIsNone(detector.last)
        for points in ([*SQUARE, SQUARE[0]], [(0,0),(1,0),(math.inf,0)]):
            with self.assertRaises(ValueError):
                detector.set_reference(points)
        with self.assertRaises(ValueError):
            LapDetector(replace(Settings(), max_sample_gap_s=math.inf))


class RequestTests(unittest.TestCase):
    def test_acceptance_is_not_completion_and_hint_needs_later_diagnostic(self):
        state = RequestState(needs_hint=True)
        self.assertTrue(state.start(1.))
        self.assertFalse(state.start(2.))
        state.reply(True, 'accepted')
        state.healthy_diagnostic(2.)
        self.assertTrue(state.busy)
        state.hint(3.)
        state.healthy_diagnostic(2.9)
        self.assertTrue(state.busy)
        state.healthy_diagnostic(3.1)
        self.assertFalse(state.busy)
        self.assertEqual((state.requests,state.completed,state.skipped), (1,1,1))

    def test_timeout_latches_until_late_hint_and_diagnostic_never_automatic_retry(self):
        state = RequestState(needs_hint=True)
        state.start(1.)
        state.reply(True)
        state.check_timeout(32.,3.,30.)
        self.assertEqual(state.state, 'suppressed_unknown_result')
        self.assertFalse(state.start(40.))
        state.hint(41.)
        state.healthy_diagnostic(42.)
        self.assertFalse(state.busy)
        self.assertEqual(state.requests, 1)

    def test_unknown_service_result_cannot_release_busy_without_late_reply(self):
        cache = RequestState()
        cache.start(1.)
        cache.check_timeout(5.,3.,15.)
        self.assertTrue(cache.busy)
        self.assertFalse(cache.start(6.))
        cache.reply(True)
        self.assertFalse(cache.busy)
        cache.start(8.)
        cache.reply(False, 'denied')
        self.assertFalse(cache.busy)


if __name__ == '__main__':
    unittest.main()
