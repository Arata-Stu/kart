"""Portable VGL staging: association, corruption and cancellation boundaries."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from kart_bringup.vgl_assets import CONFIG_FILES, digest, map_files
from kart_studio.storage import atomic_json
from kart_studio.vgl_transfer import copy_vgl_bundles
from kart_studio.jobs import Cancelled


class Job:
    def __init__(self):
        self.messages = []

    def log(self, message):
        self.messages.append(message)

    def check(self):
        pass


class VglTransferTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.maps = self.root / "map"
        self.source = self.maps / "course"
        (self.source / "cuvslam_map").mkdir(parents=True)
        (self.source / "cuvslam_map/map.mdb").write_bytes(b"course map")
        self.output = self.root / "output"
        self.job = Job()

    def bundle(self, name, data=b"course map", status="complete"):
        root = self.maps / "vgl" / name
        (root / "cuvslam_map").mkdir(parents=True)
        (root / "cuvslam_map/map.mdb").write_bytes(data)
        config = root / "vgl_runtime_config"
        config.mkdir()
        for filename in CONFIG_FILES:
            (config / filename).write_text("config")
        (config / CONFIG_FILES[0]).write_text(
            "aliked_detector { tensorrt_config { input_dimension { "
            + " ".join(
                f"{key}: {v}" for key in ("min", "opt", "max") for v in (1, 3, 240, 424)
            )
            + " } } }"
        )
        for filename in (
            "keyframes/frames_meta.json",
            "bow_index.pb",
            "vocabulary/data",
        ):
            path = root / "cuvgl_map" / filename
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"data")
        (root / "unused.engine").write_bytes(b"not portable")
        atomic_json(
            root / "vgl_profile.json",
            dict(
                schema="kart.vgl.v1",
                status=status,
                input_shape=[1, 3, 240, 424],
                cuvslam_sha256=map_files(root),
                config_sha256={f: digest(config / f) for f in CONFIG_FILES},
            ),
        )
        return root

    def copy(self):
        return copy_vgl_bundles(self.job, self.maps, self.source, self.output)

    def test_matching_versions_only_and_no_engine(self):
        self.bundle("course-v1")
        self.bundle("renamed-v2")
        self.bundle("other", b"other map")
        self.bundle("unfinished", status="building")
        self.assertEqual(self.copy(), ["course-v1", "renamed-v2"])
        self.assertEqual(
            sorted(p.name for p in (self.output / "vgl").iterdir()),
            ["course-v1", "renamed-v2"],
        )
        self.assertTrue((self.output / "vgl/renamed-v2/vgl_profile.json").is_file())
        self.assertFalse(list(self.output.rglob("*.engine")))

    def test_actual_map_must_match_manifest(self):
        bundle = self.bundle("broken")
        (bundle / "cuvslam_map/map.mdb").write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "元地図"):
            self.copy()

    def test_corrupt_config_rejected(self):
        bundle = self.bundle("broken")
        (bundle / "vgl_runtime_config" / CONFIG_FILES[1]).write_text("changed")
        with self.assertRaisesRegex(ValueError, "ハッシュ"):
            self.copy()

    def test_missing_bow_rejected(self):
        bundle = self.bundle("broken")
        (bundle / "cuvgl_map/bow_index.pb").unlink()
        with self.assertRaises(ValueError):
            self.copy()

    def test_symlink_rejected(self):
        bundle = self.bundle("linked")
        (bundle / "link").symlink_to(self.source / "cuvslam_map/map.mdb")
        with self.assertRaisesRegex(ValueError, "symlink"):
            self.copy()

    def test_missing_vgl_logs_and_preserves_normal_transfer(self):
        self.assertEqual(self.copy(), [])
        self.assertIn("VGL bundleなし", self.job.messages[0])

    def test_cancel_before_copy(self):
        self.bundle("course")
        with (
            patch.object(self.job, "check", side_effect=Cancelled()),
            self.assertRaises(Cancelled),
        ):
            self.copy()
        self.assertFalse(self.output.exists())

    def test_push_publishes_bundle_discoverable_by_bringup(self):
        from kart_bringup.mission import discover
        from kart_studio.transfers import Transfers
        from test_transfers import LocalScpJob, local_request

        self.bundle("course-v1")
        exported = self.root / "export"
        exported.mkdir()
        atomic_json(exported / "metadata.json", {})
        for filename in ("snapshot.json", "cloud.json"):
            atomic_json(self.source / filename, {})
        maps = Mock(root=self.maps)
        maps.folder.return_value = self.source
        maps.export.return_value = exported
        maps.load.return_value = {"revision": 1}
        remote = self.root / "remote"
        remote.mkdir()
        profile = dict(
            user="kart",
            host="jetson.local",
            port=22,
            map_root=str(remote),
            record_root=str(remote),
        )
        with patch("kart_studio.transfers.request", side_effect=local_request):
            result = Transfers(self.root / "records", maps).push(
                LocalScpJob(), profile, "course"
            )
        self.assertEqual(
            discover(remote, "bundle"), [(Path(result["path"]) / "vgl/course-v1").resolve()]
        )


if __name__ == "__main__":
    unittest.main()
