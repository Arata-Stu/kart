import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from kart_mapping.build_map import run
from kart_studio.jobs import Job
from kart_studio.mapping import Mapping
from kart_studio.maps import Maps
from kart_studio.copies import copy_map
from kart_studio.server import Studio
from kart_studio.storage import atomic_json


class OfficialMappingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo = Path(__file__).resolve().parents[3]
        self.maps = Maps(self.root / "map")
        self.mapping = Mapping(self.repo, self.root / "record", self.maps)
        self.bag = self.root / "record/bag"
        self.bag.mkdir(parents=True)
        (self.bag / "metadata.yaml").write_text("test")
        self.output = self.root / "output"
        self.output.mkdir()
        self.job = dict(
            bag=str(self.bag), output=str(self.output), workflow=self.mapping.defaults
        )

    def test_official_cli_and_artifact(self):
        def execute(command, **kwargs):
            if command[0] == "ros2":
                self.assertIn("create_map_offline.py", command)
                self.assertEqual(
                    command[-3:], ["--steps_to_run", "edex", "compute_poses"]
                )
                self.assertIn("--use_raw_image=False", command)
                artifact = self.output / "official/run/cuvslam_map"
                artifact.mkdir(parents=True)
                (artifact / "map.mdb").write_bytes(b"test map")
            return SimpleNamespace(stdout="5.0", returncode=0)

        with (
            patch.dict(os.environ, ROS_DOMAIN_ID="92"),
            patch.dict(
                "sys.modules",
                {
                    "kart_mapping.bag_inputs": SimpleNamespace(
                        inspect_bag=lambda *args: None
                    )
                },
            ),
            patch("kart_mapping.build_map.subprocess.run", side_effect=execute),
        ):
            run(self.job)
        self.assertTrue((self.output / "cuvslam_map/map.mdb").is_file())
        self.assertFalse((self.output / "snapshot.json").exists())
        result = json.loads((self.output / "mapping_result.json").read_text())
        self.assertEqual(result["snapshot_status"], "pending")

    def test_missing_official_map_is_failure(self):
        with (
            patch.dict(os.environ, ROS_DOMAIN_ID="92"),
            patch.dict(
                "sys.modules",
                {
                    "kart_mapping.bag_inputs": SimpleNamespace(
                        inspect_bag=lambda *args: None
                    )
                },
            ),
            patch("kart_mapping.build_map.subprocess.run"),
            self.assertRaises(ValueError),
        ):
            run(self.job)

    def test_publish_pending_and_copy_without_cloud(self):
        class Worker:
            def log(inner, text):
                pass

            def check(inner):
                pass

            def run(inner, command, **kwargs):
                stage = Path(command[-1]).parent
                atomic_json(stage / "mapping_result.json", {"status": "vslam_ready"})
                (stage / "cuvslam_map").mkdir()
                (stage / "cuvslam_map/map.mdb").write_bytes(b"test map")

        self.mapping.build(Worker(), "course", self.bag, self.mapping.defaults)
        doc = self.maps.load("course")
        self.assertEqual(doc["snapshot_status"], "pending")
        self.assertEqual(self.maps.list()[0]["snapshot_status"], "pending")
        for action in (
            lambda: self.maps.save("course", doc),
            lambda: self.maps.export("course"),
        ):
            with self.assertRaisesRegex(ValueError, "点群"):
                action()
        logs = self.root / "jobs"
        logs.mkdir()
        copy_map(self.maps, Job(logs, "copy", []), "course", "course_v2", 1)
        self.assertEqual(self.maps.load("course_v2")["snapshot_status"], "pending")
        studio = Studio.__new__(Studio)
        studio.maps = self.maps
        self.assertEqual(studio.get("/api/cloud", {"id": "course"})["points"], [])

    def test_failed_build_not_published(self):
        class Worker:
            def log(inner, text):
                pass

            def run(inner, *args, **kwargs):
                raise RuntimeError("official failed")

        with self.assertRaises(RuntimeError):
            self.mapping.build(Worker(), "course", self.bag, self.mapping.defaults)
        self.assertEqual(list(self.maps.root.iterdir()), [])
