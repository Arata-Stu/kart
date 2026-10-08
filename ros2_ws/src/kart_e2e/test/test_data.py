import unittest
from types import SimpleNamespace as NS

from kart_e2e.data import decode_image, previous, valid_label


class DataTests(unittest.TestCase):
    def test_causal_alignment(self):
        records = [(100, "a"), (200, "b")]
        self.assertEqual(previous(records, 150, 60), "a")
        self.assertIsNone(previous(records, 90, 60))
        self.assertIsNone(previous(records, 180, 60))

    def test_brake_reverse_and_nonfinite_excluded(self):
        base = dict(steering=0.2, throttle=0.1, brake=0.0, reverse=0.0)
        self.assertTrue(valid_label(NS(**base)))
        for key, value in [
            ("brake", 0.1),
            ("reverse", 0.1),
            ("steering", float("nan")),
            ("throttle", 2),
        ]:
            self.assertFalse(valid_label(NS(**{**base, key: value})))

    def test_bgr_stride(self):
        msg = NS(
            encoding="bgr8",
            width=1,
            height=2,
            step=4,
            data=bytes([0, 0, 255, 9, 255, 0, 0, 9]),
        )
        self.assertEqual(list(decode_image(msg).getdata()), [(255, 0, 0), (0, 0, 255)])
        msg.step = 2
        with self.assertRaises(ValueError):
            decode_image(msg)
