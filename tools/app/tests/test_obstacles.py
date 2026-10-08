import copy
import tempfile
import unittest
from pathlib import Path

from kart_studio import obstacles, registration
from kart_studio.maps import Maps
from kart_studio.storage import atomic_json, read_json
from test_studio import track


class ObstaclesTests(unittest.TestCase):
    def setUp(self):
        self.region = dict(id="block", polygon=[[1, -1], [2, -1], [2, 1], [1, 1]])

    def test_crossing_containment_touch_and_clearance(self):
        for line, clearance in [
            ([[0, 0], [3, 0]], 0),
            ([[1.2, 0], [1.8, 0]], 0),
            ([[0, 1], [3, 1]], 0),
            ([[0, 1.1], [3, 1.1]], 0.15),
        ]:
            with self.assertRaises(ValueError):
                obstacles.check_line(line, False, [self.region], clearance)
        obstacles.check_line([[0, 2], [3, 2]], False, [self.region], 0.15)
        # Closing edge crosses the polygon even though the open chain avoids it.
        line = [[0, 0], [0, 3], [3, 3], [3, 0]]
        obstacles.check_line(line, False, [self.region])
        with self.assertRaises(ValueError):
            obstacles.check_line(line, True, [self.region])

    def test_invalid_polygon(self):
        for polygon in [[], [[0, 0], [1, 0], [2, 0]], [[0, 0], [2, 2], [0, 2], [2, 0]]]:
            with self.assertRaises(ValueError):
                obstacles.validate([dict(id="bad", polygon=polygon)])
        with self.assertRaises(ValueError):
            obstacles.validate([self.region, self.region])

    def test_save_generate_export_registration(self):
        with tempfile.TemporaryDirectory() as root:
            maps = Maps(Path(root))
            atomic_json(
                maps.root / "course/map.json",
                dict(
                    track(),
                    title="course",
                    revision=1,
                    lines={},
                    point_count=0,
                    updated=0,
                ),
            )
            maps.generate("course", "centerline", {}, 1, lambda: None)
            doc = maps.load("course")
            doc["obstacles"] = [dict(id="far", polygon=[[90, 90], [91, 90], [91, 91]])]
            saved = maps.save("course", doc)
            self.assertEqual(saved["lanes"][0]["lines"], {})
            maps.generate("course", "centerline", {}, saved["revision"], lambda: None)
            exported = read_json(maps.export("course") / "hd_map.yaml")
            self.assertEqual(exported["obstacles"], doc["obstacles"])
            atomic_json(maps.root / "target/map.json", maps.load("course"))
            proposal = registration.preview(
                maps,
                "target",
                dict(
                    source="course",
                    revision=maps.load("target")["revision"],
                    x=2,
                    y=3,
                    yaw=0,
                ),
            )
            self.assertEqual(
                proposal["document"]["obstacles"][0]["polygon"][0], [92, 93]
            )
            doc = maps.load("course")
            p = doc["lanes"][0]["lines"]["centerline"]["points"][0]
            doc["obstacles"] = [
                dict(
                    id="on_track",
                    polygon=[
                        [p[0] - 1, p[1] - 1],
                        [p[0] + 1, p[1] - 1],
                        [p[0] + 1, p[1] + 1],
                        [p[0] - 1, p[1] + 1],
                    ],
                )
            ]
            saved = maps.save("course", doc)
            before = copy.deepcopy(saved)
            with self.assertRaisesRegex(ValueError, "on_track"):
                maps.generate(
                    "course", "centerline", {}, saved["revision"], lambda: None
                )
            self.assertEqual(before, maps.load("course"))
