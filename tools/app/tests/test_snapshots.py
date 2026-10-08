import base64
import struct
import unittest

from kart_studio.snapshots import decode_cloud, normalize


def cloud(frame="odom", big=False):
    order = ">" if big else "<"
    return dict(
        header=dict(frame_id=frame),
        width=1,
        height=2,
        point_step=16,
        row_step=20,
        fields=[
            dict(name=k, offset=i * 4, datatype=7, count=1)
            for i, k in enumerate(("x", "y", "z"))
        ],
        is_bigendian=big,
        data=base64.b64encode(
            struct.pack(order + "fff", 1, 2, 3)
            + b"\0" * 8
            + struct.pack(order + "fff", 4, 5, 6)
            + b"\0" * 8
        ).decode(),
    )


class SnapshotTests(unittest.TestCase):
    def test_row_padding_and_endianness(self):
        for big in (False, True):
            self.assertEqual(decode_cloud(cloud(big=big)), [[1, 2, 3], [4, 5, 6]])

    def test_legacy_final_transform_and_height(self):
        snapshot = dict(
            landmarks=cloud(),
            localization=dict(
                map_from_frame=dict(
                    odom=dict(
                        translation=dict(x=10, y=0, z=2),
                        rotation=dict(x=0, y=0, z=0, w=1),
                    )
                )
            ),
        )
        result = normalize(snapshot)
        self.assertEqual(result["points"], [[11, 2, 5], [14, 5, 8]])
        self.assertIn("TF時刻", result["provenance"]["warning"])
        self.assertEqual(snapshot["landmarks"]["header"]["frame_id"], "odom")

    def test_missing_tf_is_not_silently_identity(self):
        with self.assertRaisesRegex(ValueError, "Missing final"):
            normalize(dict(landmarks=cloud()))
        result = normalize(dict(landmarks=cloud(frame="map")))
        self.assertEqual(result["points"][0], [1, 2, 3])

    def test_corrupt_cloud_size(self):
        bad = cloud()
        bad["row_step"] = 8
        with self.assertRaises(ValueError):
            decode_cloud(bad)


if __name__ == "__main__":
    unittest.main()
