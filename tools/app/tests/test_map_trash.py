import tempfile
import unittest
from pathlib import Path

from kart_studio.jobs import Jobs
from kart_studio.lanes import upgrade
from kart_studio.maps import Maps
from kart_studio.server import Studio
from kart_studio.storage import atomic_json


class MapTrashTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.maps = Maps(self.root / "map")
        self.doc = dict(title="test", revision=3, lines={}, point_count=0, updated=0)
        atomic_json(self.maps.root / "course/map.json", self.doc)
        (self.maps.root / "course/cuvslam_map").mkdir()
        (self.maps.root / "course/cuvslam_map/data").write_bytes(b"map binary")
        self.studio = Studio.__new__(Studio)
        self.studio.maps = self.maps
        self.studio.jobs = Jobs(self.root / "jobs")

    def test_roundtrip_preserves_folder(self):
        result = self.studio.post("/api/delete-map", dict(id="course", revision=3))
        self.assertEqual(self.maps.list(), [])
        self.assertEqual(len(self.maps.trashed()), 1)
        self.studio.post("/api/restore-map", result)
        self.assertEqual(self.maps.load("course"), upgrade(self.doc))
        self.assertEqual(
            (self.maps.folder("course") / "cuvslam_map/data").read_bytes(),
            b"map binary",
        )
        self.assertEqual(self.maps.trashed(), [])

    def test_stale_revision_and_busy_map_rejected(self):
        with self.assertRaises(ValueError):
            self.maps.trash("course", 2)
        with self.studio.jobs.guard("map:course"):
            with self.assertRaises(ValueError):
                self.studio.post("/api/delete-map", dict(id="course", revision=3))
        self.assertEqual(self.maps.load("course"), upgrade(self.doc))

    def test_restore_collision_and_path_escape_rejected(self):
        token = self.maps.trash("course", 3)["token"]
        atomic_json(self.maps.root / "course/map.json", dict(self.doc, revision=4))
        with self.assertRaises(ValueError):
            self.maps.restore(token)
        self.assertEqual(self.maps.load("course")["revision"], 4)
        self.assertEqual(len(self.maps.trashed()), 1)
        with self.assertRaises(ValueError):
            self.maps.restore("../course")
        with self.assertRaises(ValueError):
            self.maps.trash("../course", 3)
