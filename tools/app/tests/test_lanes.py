import copy
import tempfile
import unittest
from pathlib import Path

from kart_studio import registration
from kart_studio.lanes import empty
from kart_studio.maps import Maps
from kart_studio.storage import atomic_json, read_json
from test_studio import track


class LanesTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.maps = Maps(Path(self.temp.name) / "map")
        self.legacy = dict(
            track(), title="legacy", revision=4, lines={}, point_count=0, updated=0
        )
        atomic_json(self.maps.root / "course/map.json", self.legacy)

    def test_legacy_load_is_lossless_and_does_not_write(self):
        doc = self.maps.load("course")
        self.assertEqual(doc["schema"], "kart.hdmap.v2")
        self.assertEqual(doc["lanes"][0]["id"], "lane_001")
        self.assertEqual(doc["lanes"][0]["left"], self.legacy["left"])
        self.assertEqual(
            read_json(self.maps.folder("course") / "map.json"), self.legacy
        )
        saved = self.maps.save("course", doc)
        self.assertEqual(saved["revision"], 5)
        self.assertNotIn("left", saved)

    def two_lanes(self):
        doc = self.maps.load("course")
        extra = dict(
            empty("shortcut"),
            left=[[0, 1], [8, 1]],
            right=[[0, -1], [8, -1]],
            custom=[[0, 0], [8, 0]],
            closed=False,
        )
        doc["lanes"].append(extra)
        return self.maps.save("course", doc)

    def generate(self, lane_id, kind="centerline"):
        doc = self.maps.load("course")
        self.maps.generate(
            "course", kind, {"spacing": 0.5}, doc["revision"], lambda: None, lane_id
        )

    def test_generation_and_invalidation_are_per_lane(self):
        self.two_lanes()
        self.generate("lane_001")
        self.generate("shortcut")
        self.generate("shortcut", "customline")
        doc = self.maps.load("course")
        first = copy.deepcopy(doc["lanes"][0])
        doc["lanes"][1]["custom_speeds"] = [dict(start=2, end=3, speed=0.4)]
        doc = self.maps.save("course", doc)
        self.assertEqual(doc["lanes"][0], first)
        self.assertIn("centerline", doc["lanes"][1]["lines"])
        self.assertNotIn("customline", doc["lanes"][1]["lines"])
        doc["lanes"][1]["left"][0][1] += 0.1
        doc = self.maps.save("course", doc)
        self.assertEqual(doc["lanes"][0], first)
        self.assertEqual(doc["lanes"][1]["lines"], {})
        with self.assertRaises(ValueError):
            self.maps.generate(
                "course", "centerline", {}, doc["revision"], lambda: None
            )
        with self.assertRaises(ValueError):
            self.maps.generate(
                "course", "centerline", {}, doc["revision"], lambda: None, "missing"
            )

    def test_ids_validation_and_generated_results_not_client_owned(self):
        doc = self.two_lanes()
        for lane_id in ("lane_001", "LANE_001", "../bad"):
            bad = copy.deepcopy(doc)
            bad["lanes"][1]["id"] = lane_id
            with self.assertRaises(ValueError):
                self.maps.save("course", bad)
        bad = copy.deepcopy(doc)
        bad["lanes"] = []
        with self.assertRaises(ValueError):
            self.maps.save("course", bad)
        doc["lanes"][1]["lines"] = {"raceline": {"points": [[99, 99]]}}
        doc = self.maps.save("course", doc)
        self.assertEqual(doc["lanes"][1]["lines"], {})

    def test_export_lane_ids_paths_and_drafts(self):
        self.two_lanes()
        self.generate("lane_001")
        with self.assertRaisesRegex(ValueError, "shortcut"):
            self.maps.export("course")
        self.generate("shortcut")
        folder = self.maps.export("course")
        hd = read_json(folder / "hd_map.yaml")
        self.assertEqual(hd["line_role"], "offline_reference")
        self.assertEqual([lane["id"] for lane in hd["lanes"]], ["lane_001", "shortcut"])
        for lane in hd["lanes"]:
            self.assertTrue((folder / lane["offline_lines"]["centerline"]).is_file())
        self.assertFalse((folder / "centerline.csv").exists())
        self.assertEqual(
            set(read_json(folder / "metadata.json")["lanes"]), {"lane_001", "shortcut"}
        )
        doc = self.maps.load("course")
        doc["lanes"].pop(1)
        self.maps.save("course", doc)
        self.assertTrue(folder.is_dir())
        self.assertTrue((self.maps.export("course") / "centerline.csv").is_file())

    def test_registration_transforms_every_lane_and_preserves_ids(self):
        source = self.two_lanes()
        self.generate("shortcut")
        source = self.maps.load("course")
        atomic_json(self.maps.root / "target/map.json", dict(source, title="target"))
        body = dict(source="course", revision=source["revision"], x=3, y=4, yaw=0)
        preview = registration.preview(self.maps, "target", body)
        result = registration.apply(
            self.maps, "target", dict(body, token=preview["token"])
        )
        for old, new in zip(source["lanes"], result["lanes"]):
            self.assertEqual(old["id"], new["id"])
            self.assertAlmostEqual(new["left"][0][0], old["left"][0][0] + 3)
            self.assertAlmostEqual(new["left"][0][1], old["left"][0][1] + 4)
        self.assertEqual(self.maps.load("course"), source)
