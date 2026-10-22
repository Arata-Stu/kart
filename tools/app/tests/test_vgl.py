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


class ModelOptionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        # Deliberately misleading folder name: resolution must come from metadata.
        self.models = self.root / '1280x720/runtime_models'
        files = self.models / 'aliked_lightglue'
        files.mkdir(parents=True)
        for name in ('aliked.onnx', 'aliked_test.engine', 'lightglue_aliked_test.engine'):
            (files / name).write_bytes(b'model')
        from kart_bringup.vgl_assets import digest
        self.manifest = self.models.parent / 'manifest.json'
        self.data = dict(input_shape=[1, 3, 480, 640],
                         onnx_sha256=digest(files / 'aliked.onnx'))
        atomic_json(self.manifest, self.data)

    def test_size_from_hash_matched_manifest_not_folder_name(self):
        option = vgl.model_option(self.models)
        self.assertEqual((option['width'], option['height']), (640, 480))
        self.assertEqual(option['path'], str(self.models))
        self.assertFalse(option['error'])

    def test_bad_hash_shape_or_incomplete_engines_are_not_candidates(self):
        for change in (dict(onnx_sha256='wrong'), dict(input_shape=[1, 3, 0, 640]),
                       dict(input_shape='640x480')):
            atomic_json(self.manifest, {**self.data, **change})
            self.assertTrue(vgl.model_option(self.models)['error'])
        atomic_json(self.manifest, self.data)
        (self.models / 'aliked_lightglue/aliked_test.engine').unlink()
        self.assertTrue(vgl.model_option(self.models)['error'])

    def test_no_manifest_keeps_legacy_model_as_unknown(self):
        self.manifest.unlink()
        option = vgl.model_option(self.models)
        self.assertIsNone(option['width'])
        self.assertFalse(option['error'])

    def test_prepare_rejects_wrong_resolution_before_job(self):
        source = self.root / 'map/course'
        source.mkdir(parents=True)
        app = SimpleNamespace(maps=Maps(self.root / 'map'))
        body = dict(id='course', name='course-vgl-v1', model_dir=str(self.models),
                    width=424, height=240)
        with patch('kart_bringup.prepare_vgl_map.source_frames'), patch(
                'kart_studio.vgl.shutil.which', return_value='ros2'):
            with self.assertRaisesRegex(ValueError, '一致'):
                vgl.prepare(app, body)
            body.update(width=640, height=480)
            self.assertEqual(vgl.prepare(app, body)[-1], [640, 480])
            del body['width'], body['height']
            self.assertEqual(vgl.prepare(app, body)[-1], [640, 480])
