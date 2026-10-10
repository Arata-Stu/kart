import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from kart_studio import vgl
from kart_studio.jobs import Cancelled
from kart_studio.maps import Maps
from kart_studio.storage import atomic_json


class VglTests(unittest.TestCase):
    def test_publish_only_completed_bundle_and_preserve_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve()
            source = root / "course"
            (source / "cuvslam_map").mkdir(parents=True)
            (source / "cuvslam_map/map.mdb").write_bytes(b"original")
            output = root / "vgl/course-v1"

            def run(args):
                stage = Path(args[args.index("--output") + 1])
                stage.mkdir()
                atomic_json(
                    stage / "vgl_profile.json",
                    dict(schema="kart.vgl.v1", status="complete"),
                )

            job = Mock()
            job.run.side_effect = run
            result = vgl.build(job, source, root / "models", output, [424, 240])
            self.assertEqual(result["bundle"], str(output))
            self.assertEqual((source / "cuvslam_map/map.mdb").read_bytes(), b"original")
            job.check.side_effect = Cancelled("cancel")
            with self.assertRaises(Cancelled):
                vgl.build(
                    job, source, root / "models", root / "vgl/cancelled", [424, 240]
                )
            self.assertFalse((root / "vgl/cancelled").exists())
            self.assertFalse(list((root / "vgl").glob(".vgl-ui-*")))

    def test_prepare_missing_frames_and_unsafe_name(self):
        with tempfile.TemporaryDirectory() as tmp:
            maps = Maps(Path(tmp) / "map")
            source = maps.root / "course"
            source.mkdir()
            app = SimpleNamespace(maps=maps)
            with self.assertRaises(ValueError):
                vgl.prepare(app, dict(id="../escape"))
            with self.assertRaises(ValueError):
                vgl.prepare(app, dict(id="course"))
