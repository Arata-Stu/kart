import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from kart_studio.capture import capture, validate
from kart_studio.maps import Maps
from kart_studio.storage import atomic_json


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.maps = Maps(self.root / "map")
        self.folder = self.maps.root / "course"
        self.folder.mkdir()
        self.bag = self.root / "record/bag"
        self.bag.mkdir(parents=True)
        (self.bag / "metadata.yaml").write_text("original bag")
        atomic_json(
            self.folder / "mapping_result.json",
            {
                "status": "vslam_ready",
                "bag_metadata_sha256": hashlib.sha256(
                    (self.bag / "metadata.yaml").read_bytes()
                ).hexdigest(),
            },
        )
        self.maps.initialize_pending(self.folder, "course")
        atomic_json(self.folder / "job.json", {"workflow": {"ros_domain_id": 92}})
        (self.folder / "cuvslam_map").mkdir()
        (self.folder / "cuvslam_map/map.mdb").write_bytes(b"original map")
        self.studio = SimpleNamespace(
            maps=self.maps,
            records=self.root / "record",
            repo=Path(__file__).resolve().parents[3],
        )
        self.snapshot = {
            "schema": "kart.snapshot.v1",
            "frame": "map",
            "points": [[1, 2, 3]],
            "trajectory": [[1, 2, 0]],
            "provenance": {"localized_in_existing_map": True},
        }

    def worker(self, result=None, failure=False):
        snapshot = self.snapshot if result is None else result

        class Worker:
            def check(inner):
                pass

            def log(inner, message):
                pass

            def run(inner, args, **kwargs):
                stage = Path(args[-1]).parent
                (stage / "cuvslam_map/map.mdb").write_bytes(b"working map changed")
                if failure:
                    raise ValueError("replay failed")
                atomic_json(stage / "snapshot.json", snapshot)

        return Worker()

    def run_capture(self, worker):
        return capture(
            self.studio, worker, "course", 1, self.bag, {"ros_domain_id": 92}, {}
        )

    def test_pending_to_ready_preserves_original_map(self):
        result = self.run_capture(self.worker())
        self.assertEqual(result["revision"], 2)
        self.assertEqual(self.maps.load("course")["snapshot_status"], "ready")
        self.assertEqual(
            json.loads((self.folder / "cloud.json").read_text())["points"], [[1, 2, 3]]
        )
        self.assertEqual(
            (self.folder / "cuvslam_map/map.mdb").read_bytes(), b"original map"
        )
        self.assertEqual(list(self.maps.root.glob(".capture-*")), [])
        with self.assertRaises(ValueError):
            self.run_capture(self.worker())

    def test_failure_or_unlocalized_snapshot_not_published(self):
        bad = dict(self.snapshot, provenance={})
        for worker in [self.worker(failure=True), self.worker(result=bad)]:
            with self.assertRaises(ValueError):
                self.run_capture(worker)
            self.assertEqual(self.maps.load("course")["snapshot_status"], "pending")
            self.assertFalse((self.folder / "snapshot.json").exists())
            self.assertEqual(list(self.maps.root.glob(".capture-*")), [])

    def test_cancel_during_copy_keeps_pending(self):
        from kart_studio.jobs import Cancelled

        worker = self.worker()

        def cancel():
            raise Cancelled("cancel")

        worker.check = cancel
        with self.assertRaises(Cancelled):
            self.run_capture(worker)
        self.assertEqual(self.maps.load("course")["snapshot_status"], "pending")
        self.assertEqual(
            (self.folder / "cuvslam_map/map.mdb").read_bytes(), b"original map"
        )
        self.assertEqual(list(self.maps.root.glob(".capture-*")), [])

    def test_validate_origin_and_revision(self):
        with patch("kart_studio.capture.shutil.which", return_value="ros2"):
            bag, _workflow, options = validate(
                self.studio, "course", {"revision": 1, "bag": "bag"}
            )
            self.assertEqual(bag, self.bag.resolve())
            self.assertEqual(options["replay_rate"], 1.0)
            with self.assertRaises(ValueError):
                validate(self.studio, "course", {"revision": 2, "bag": "bag"})
            (self.bag / "metadata.yaml").write_text("different bag")
            with self.assertRaises(ValueError):
                validate(self.studio, "course", {"revision": 1, "bag": "bag"})
