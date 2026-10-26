"""Speed trials preserve original profiles, stop points and real-default opt-out."""
import unittest
from kart_hdmap.speed import validate_speed_override, override_speeds, centerline_speed


class SpeedTests(unittest.TestCase):
    def test_center_speed_follows_map_and_explicit_override(self):
        self.assertEqual(centerline_speed({}, -1), 1.5)
        self.assertEqual(centerline_speed({'centerline_speed_mps': .7}, -1), .7)
        self.assertEqual(centerline_speed({'centerline_speed_mps': 0.}, -1), 0.)
        self.assertEqual(centerline_speed({'centerline_speed_mps': .7}, 0.), 0.)
        self.assertEqual(centerline_speed({'centerline_speed_mps': .7}, 1.2), 1.2)
        for value in (-.1, -2, True, float('nan')):
            with self.assertRaises(ValueError):
                centerline_speed({}, value)

    def test_opt_in_and_numeric_bounds(self):
        validate_speed_override(-1., False)
        for value in (.05, .8, 1.5):
            validate_speed_override(value, True)
            with self.assertRaises(ValueError):validate_speed_override(value, False)
        for value in (True, None, '0.5', 0, -.5, 1.51, float('nan'), float('inf')):
            with self.assertRaises(ValueError):validate_speed_override(value, True)

    def test_reversible_uniform_speed_keeps_stop_points(self):
        profile = [1.2, .4, 0., .8]
        self.assertEqual(override_speeds(profile, .9, .45), [.9, .9, 0., .9])
        self.assertEqual(override_speeds(profile, -1., .45), [.45, .4, 0., .45])
        self.assertEqual(override_speeds(profile, -1.), profile)
        self.assertEqual(profile, [1.2, .4, 0., .8])
        self.assertEqual(override_speeds([], .9), [])
