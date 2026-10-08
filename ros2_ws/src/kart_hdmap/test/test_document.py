import copy
import json
import math
import tempfile
import unittest
from pathlib import Path

from kart_hdmap.document import load, path_samples, select


def example():
    xy = [[1.0, 2.0], [3.0, 2.0], [3.0, 4.0]]
    lane = {
        "id": "main",
        "closed": True,
        "left": xy,
        "right": xy,
        "lines": {
            "centerline": {"points": xy},
            "raceline": {
                "points": xy,
                "profile": [[i * 2, *p, 0, 0, 1.5, 0] for i, p in enumerate(xy)],
            },
        },
    }
    return {"schema": "kart.hdmap.v2", "frame": "map", "revision": 3, "lanes": [lane]}


class DocumentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.file = Path(self.temp.name) / "map.json"
        self.doc = example()

    def read(self):
        self.file.write_text(json.dumps(self.doc))
        return load(self.file)

    def test_preserves_map_coordinates_ids_speeds(self):
        second = copy.deepcopy(self.doc["lanes"][0])
        second["id"] = "shortcut"
        self.doc["lanes"].append(second)
        d = self.read()
        self.assertEqual(d["lanes"], self.doc["lanes"])
        self.assertIsNone(select(d, ""))
        self.assertEqual(select(d, "shortcut")["id"], "shortcut")
        with self.assertRaises(ValueError):
            select(d, "unknown")
        samples = path_samples(d["lanes"][0]["lines"]["centerline"], True)
        self.assertEqual(samples[0], samples[-1])
        self.assertEqual(samples[0][:2], (1.0, 2.0))
        for p in samples:
            self.assertAlmostEqual(math.hypot(p[2], p[3]), 1)

    def test_obstacles_preserved_and_invalid_rejected(self):
        self.doc["obstacles"] = [{"id": "block", "polygon": [[1, 2], [3, 2], [3, 4]]}]
        self.assertEqual(self.read()["obstacles"], self.doc["obstacles"])
        self.doc["obstacles"].append(copy.deepcopy(self.doc["obstacles"][0]))
        with self.assertRaises(ValueError):
            self.read()

    def test_frame_relabel_and_pending_rejected(self):
        self.doc["frame"] = "odom"
        with self.assertRaises(ValueError):
            self.read()
        self.doc["frame"] = "map"
        self.doc["snapshot_status"] = "pending"
        with self.assertRaises(ValueError):
            self.read()

    def test_bad_coordinates_profile_and_duplicate_rejected(self):
        for mutate in [
            lambda d: d["lanes"][0]["left"][0].__setitem__(0, float("nan")),
            lambda d: d["lanes"].append(copy.deepcopy(d["lanes"][0])),
            lambda d: d["lanes"][0]["lines"]["raceline"]["profile"][0].__setitem__(
                1, 99
            ),
        ]:
            self.doc = example()
            mutate(self.doc)
            with self.assertRaises(ValueError):
                self.read()

    def test_export_csv_and_traversal(self):
        lane = self.doc["lanes"][0]
        self.doc = {
            "schema": "kart.hdmap.v2",
            "frame_id": "map",
            "revision": 3,
            "line_role": "offline_reference",
            "lanes": [
                {
                    "id": "main",
                    "closed_loop": True,
                    "left_bound": lane["left"],
                    "right_bound": lane["right"],
                    "centerline": lane["lines"]["centerline"]["points"],
                    "offline_lines": {"centerline": "centerline.csv"},
                }
            ],
        }
        (self.file.parent / "centerline.csv").write_text(
            "x_m,y_m,width_right_m,width_left_m\n1,2,1,1\n3,2,1,1\n3,4,1,1\n"
        )
        self.assertEqual(
            self.read()["lanes"][0]["lines"]["centerline"]["points"], lane["left"]
        )
        self.doc["lanes"][0]["offline_lines"]["centerline"] = str(
            self.file.parent / "centerline.csv"
        )
        with self.assertRaises(ValueError):
            self.read()

    def test_open_path_not_closed(self):
        line = {"points": [[0, 0], [0, 2]]}
        samples = path_samples(line, False)
        self.assertEqual(len(samples), 2)
        self.assertAlmostEqual(samples[-1][2], math.sqrt(0.5))
