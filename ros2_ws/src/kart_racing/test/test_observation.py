from collections import deque
import unittest

from kart_racing.observation import Course, velocity


class ObservationTests(unittest.TestCase):
    def test_static_map_target_with_approaching_ego(self):
        course = Course([(0, 0), (20, 0), (20, 5), (0, 5)], [2] * 4)
        samples = deque()
        for i in range(24):
            t = i / 30
            ego_x = 2*t
            relative_x = 8 - ego_x
            ego, _, _ = course.project((ego_x, 0))
            other, d, _ = course.project((ego_x+relative_x, .1))
            samples.append((t, ego, other))
            self.assertAlmostEqual(d, .1)
        self.assertAlmostEqual(velocity(samples, 1, .03)[0], 2)
        speed, variance = velocity(samples, 2, .03)
        self.assertAlmostEqual(speed, 0)
        self.assertLess(speed + 2*variance**.5, .12)

    def test_seam_and_ambiguous_hairpin(self):
        c = Course([(0, 0), (20, 0), (20, .3), (0, .3)], [2]*4)
        self.assertAlmostEqual(c.unwrap(.2, c.length-.1), c.length+.2)
        with self.assertRaisesRegex(ValueError, "ambiguous"):
            c.project((10, .12))
