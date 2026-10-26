"""Exercise the SCP workflow against temp files; never connects to a Jetson."""

import copy
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from kart_studio.jobs import Cancelled
from kart_studio.maps import Maps
from kart_studio.mapping import local_records
from kart_studio.server import Studio
from types import SimpleNamespace
from unittest.mock import Mock
from kart_studio.remote_agent import execute
from kart_studio.storage import atomic_json, read_json
from kart_studio.transfers import Transfers


class LocalScpJob:
    def __init__(self, cancel=False):
        self.cancel = cancel
        self.transferred = False

    def log(self, message):
        pass

    def check(self):
        if self.cancel and self.transferred:
            raise Cancelled()

    def run(self, args):
        source, target = args[-2:]
        if ":" in source:
            source = source.split(":", 1)[1]
        else:
            target = str(Path(target.split(":", 1)[1]) / Path(source).name)
        shutil.copytree(source, target)
        self.transferred = True


def local_request(profile, action, root, relative="", **extra):
    return execute(dict(action=action, root=root, relative=relative, **extra))


class TransferTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.remote = self.root / "remote"
        self.remote.mkdir()
        (self.remote / "record" / "session" / "bag").mkdir(parents=True)
        (self.remote / "map").mkdir()
        bag = self.remote / "record" / "session" / "bag"
        (bag / "metadata.yaml").write_text("metadata")
        (bag / "bag.mcap").write_bytes(b"data")
        self.maps = Maps(self.root / "maps")
        self.transfer = Transfers(self.root / "records", self.maps)
        self.profile = dict(
            user="tester",
            host="jetson.local",
            port=22,
            record_root=str(self.remote / "record"),
            map_root=str(self.remote / "map"),
        )

    def tearDown(self):
        self.temp.cleanup()

    @patch("kart_studio.transfers.request", side_effect=local_request)
    def test_pull_commits_after_validation(self, request):
        result = self.transfer.pull(LocalScpJob(), self.profile, "session/bag", "run1")
        self.assertEqual(result["record"], "session/run1")
        self.assertEqual((self.root / "records/session/run1/bag.mcap").read_bytes(), b"data")
        with self.assertRaises(ValueError):
            self.transfer.pull(LocalScpJob(), self.profile, "session/bag", "run1")

    @patch("kart_studio.transfers.request", side_effect=local_request)
    def test_pull_preserves_date_hierarchy_and_duplicate_leaf_names(self, request):
        source = self.remote / "record/session/bag"
        for date in ("2026-10-25", "2026-10-26"):
            relative = date + "/session/bag"
            shutil.copytree(source, self.remote / "record" / relative)
            result = self.transfer.pull(LocalScpJob(), self.profile, relative)
            self.assertEqual(result["record"], relative)
            self.assertEqual((self.transfer.records / relative / "bag.mcap").read_bytes(), b"data")
        self.assertEqual({r["id"] for r in local_records(self.transfer.records)},
                         {"2026-10-25/session/bag", "2026-10-26/session/bag"})
        with self.assertRaisesRegex(ValueError, "同名"):
            self.transfer.pull(LocalScpJob(), self.profile, relative)

    def test_pull_refuses_escape_empty_and_symlink(self):
        for relative in ("", ".", "../bag", "/bag"):
            with self.assertRaises(ValueError):
                self.transfer.pull_target(relative)
        (self.transfer.records / "linked").symlink_to(self.remote, target_is_directory=True)
        with self.assertRaises(ValueError):
            self.transfer.pull_target("linked/bag")

    def test_batch_api_reserves_paths_in_selection_order(self):
        jobs = Mock()
        jobs.start_many.return_value = [{"id": "a"}, {"id": "b"}]
        studio = SimpleNamespace(transfers=self.transfer, jobs=jobs)
        paths = ["2026-10-26/bag", "2026-10-25/bag"]
        result = Studio.post(studio, "/api/pull", dict(connection=self.profile, relatives=paths))
        self.assertEqual(result, {"jobs": [{"id": "a"}, {"id": "b"}]})
        tasks = jobs.start_many.call_args.args[0]
        self.assertEqual([t[1] for t in tasks], [["record:" + p] for p in paths])
        self.assertEqual([t[2].args[1] for t in tasks], paths)
        self.assertEqual(jobs.start_many.call_args.kwargs, {"queue": "scp"})
        jobs.reset_mock()
        for paths in ([], ["valid/bag", "../bad"], "not-a-list", [None]):
            with self.assertRaises(ValueError):
                Studio.post(studio, "/api/pull", dict(connection=self.profile, relatives=paths))
        jobs.start_many.assert_not_called()

    @patch("kart_studio.transfers.request", side_effect=local_request)
    def test_cancel_does_not_publish_partial_bag(self, request):
        with self.assertRaises(Cancelled):
            self.transfer.pull(
                LocalScpJob(cancel=True), self.profile, "session/bag", "run1"
            )
        self.assertEqual(list((self.root / "records").iterdir()), [])

    def test_changing_remote_bag_is_rejected(self):
        calls = 0

        def change(p, action, root, relative="", **extra):
            nonlocal calls
            result = local_request(p, action, root, relative, **extra)
            calls += 1
            if calls == 2:
                result["signature"] = "changed"
            return result

        with (
            patch("kart_studio.transfers.request", side_effect=change),
            self.assertRaisesRegex(ValueError, "更新"),
        ):
            self.transfer.pull(LocalScpJob(), self.profile, "session/bag", "run1")
        self.assertFalse((self.root / "records/session/run1").exists())

    @patch("kart_studio.transfers.request", side_effect=local_request)
    def test_push_contains_current_export_and_vslam(self, request):
        snapshot = self.root / "snapshot.json"
        atomic_json(
            snapshot,
            dict(schema="kart.snapshot.v1", frame="map", points=[[0, 0, 0], [1, 0, 1]]),
        )
        self.maps.import_snapshot("course", snapshot)
        doc = self.maps.load("course")
        doc["lanes"][0].update(
            left=[[0, 1], [5, 1]], right=[[0, -1], [5, -1]], closed=False
        )
        doc["lanes"].append(dict(copy.deepcopy(doc["lanes"][0]), id="shortcut"))
        doc = self.maps.save("course", doc)
        for lane_id in ("lane_001", "shortcut"):
            self.maps.generate(
                "course", "centerline", {}, doc["revision"], lambda: None, lane_id
            )
            doc = self.maps.load("course")
        folder = self.maps.folder("course") / "cuvslam_map"
        folder.mkdir()
        (folder / "map.db").write_bytes(b"map")
        result = self.transfer.push(LocalScpJob(), self.profile, "course")
        target = Path(result["path"])
        self.assertTrue((target / "export/hd_map.yaml").is_file())
        self.assertEqual(
            [lane["id"] for lane in read_json(target / "map.json")["lanes"]],
            ["lane_001", "shortcut"],
        )
        for lane_id in ("lane_001", "shortcut"):
            self.assertTrue(
                (target / "export/lanes" / lane_id / "centerline.csv").is_file()
            )
        self.assertEqual((target / "cuvslam_map/map.db").read_bytes(), b"map")
        self.assertFalse(
            any(
                p.name.startswith(".kart-upload-")
                for p in (self.remote / "map").iterdir()
            )
        )


if __name__ == "__main__":
    unittest.main()
