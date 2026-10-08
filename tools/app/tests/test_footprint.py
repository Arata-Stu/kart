import math
import unittest

from kart_studio import footprint
from kart_studio.lines import settings


class FootprintTest(unittest.TestCase):
    def setUp(self):
        self.p = settings({"margin": 0})

    def region(self, x, y, half=0.01):
        return [
            {
                "id": "block",
                "polygon": [
                    [x - half, y - half],
                    [x + half, y - half],
                    [x + half, y + half],
                    [x - half, y + half],
                ],
            }
        ]

    def test_rear_axle_reference_and_rotation(self):
        rect = footprint.rectangle([0, 0], 0, self.p)
        self.assertAlmostEqual(min(p[0] for p in rect), -0.1065)
        self.assertAlmostEqual(max(p[0] for p in rect), 0.3635)
        rect = footprint.rectangle([0, 0], math.pi / 2, self.p)
        self.assertAlmostEqual(max(p[1] for p in rect), 0.3635)

    def test_front_and_rear_overhang(self):
        for x in (-0.09, 1.35):
            with self.assertRaises(ValueError):
                footprint.validate_line(
                    [[0, 0], [1, 0]], False, self.p, obstacles=self.region(x, 0)
                )
        footprint.validate_line(
            [[0, 0], [1, 0]], False, self.p, obstacles=self.region(1.5, 0)
        )

    def test_obstacle_between_sparse_samples(self):
        with self.assertRaises(ValueError):
            footprint.validate_line(
                [[0, 0], [3, 0]], False, self.p, obstacles=self.region(1.5, 0)
            )

    def test_rotating_corner_sweep(self):
        # Obstacle lies near a front corner halfway through a 90 degree turn.
        x, y = 0.01 + 0.3635 / math.sqrt(2), 0.3635 / math.sqrt(2)
        with self.assertRaises(ValueError):
            footprint.validate_line(
                [[0, 0], [0.02, 0]],
                False,
                self.p,
                yaws=[0, math.pi / 2],
                obstacles=self.region(x, y),
            )

    def test_narrow_corridor_and_open_ends(self):
        c = footprint.corridor_for(
            [[0, 0.2], [2, 0.2]], [[0, -0.2], [2, -0.2]], False, self.p
        )
        footprint.validate_line([[0, 0], [2, 0]], False, self.p, corridor=c)
        reversed_right = footprint.corridor_for(
            [[0, 0.2], [2, 0.2]], [[2, -0.2], [0, -0.2]], False, self.p
        )
        footprint.validate_line(
            [[0, 0], [2, 0]], False, self.p, corridor=reversed_right
        )
        with self.assertRaises(ValueError):
            footprint.validate_line(
                [[0, 0], [2, 0]], False, self.p, corridor=c, yaws=[math.pi / 2] * 2
            )

    def test_invalid_dimensions(self):
        with self.assertRaises(ValueError):
            settings({"vehicle_length": 0.2, "rear_axle_to_rear": 0.3})
