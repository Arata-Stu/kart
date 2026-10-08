import math
import unittest

from kart_mapping.frames import FinalFrames


class FinalFrameTests(unittest.TestCase):
    def test_later_tf_wins_even_with_out_of_order_delivery(self):
        frames = FinalFrames()

        def record(t, x):
            return dict(
                parent="map",
                child="odom",
                stamp_ns=str(t),
                translation=[x, 0, 0],
                rotation=[0, 0, 0, 1],
            )

        frames.update(record(10, 1))
        frames.update(record(100, 8))
        frames.update(record(50, 4))
        points, tf = frames.convert([[1, 2, 3]], "odom")
        self.assertEqual(points, [[9, 2, 3]])
        self.assertEqual(tf["stamp_ns"], "100")

    def test_map_not_transformed_twice(self):
        frames = FinalFrames()
        frames.update(
            dict(
                parent="map",
                child="odom",
                stamp_ns="100",
                translation=[8, 0, 0],
                rotation=[0, 0, 0, 1],
            )
        )
        points, tf = frames.convert([[1, 2, 3]], "map")
        self.assertEqual(points, [[1, 2, 3]])
        self.assertIsNone(tf)

    def test_rotation_and_height(self):
        frames = FinalFrames()
        s = math.sqrt(0.5)
        frames.update(
            dict(
                parent="map",
                child="odom",
                stamp_ns="100",
                translation=[0, 0, 2],
                rotation=[s, 0, 0, s],
            )
        )
        points, _ = frames.convert([[0, 1, 0]], "odom")
        self.assertAlmostEqual(points[0][2], 3)
        self.assertAlmostEqual(points[0][1], 0)

    def test_no_identity_fallback(self):
        with self.assertRaises(ValueError):
            FinalFrames().convert([[0, 0, 0]], "odom")


if __name__ == "__main__":
    unittest.main()
