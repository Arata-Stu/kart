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

    def bundle_fixture(self):
        from kart_bringup.vgl_assets import CONFIG_FILES, digest, map_files
        source = self.root / 'source'
        bundle = self.root / 'bundle'
        for root in (source, bundle):
            (root / 'cuvslam_map').mkdir(parents=True)
            (root / 'cuvslam_map/map.mdb').write_bytes(b'map')
        config = bundle / 'vgl_runtime_config'
        config.mkdir()
        for filename in CONFIG_FILES:
            (config / filename).write_text('setting: true')
        (config / CONFIG_FILES[0]).write_text('aliked_detector { tensorrt_config { input_dimension {' +
            ' '.join(f'{key}: {v}' for key in ('min', 'opt', 'max') for v in (1, 3, 480, 640)) + '} } }')
        for filename in ('keyframes/frames_meta.json', 'bow_index.pb', 'vocabulary/vocab.pb'):
            path = bundle / 'cuvgl_map' / filename
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('data')
        atomic_json(bundle / 'vgl_profile.json', dict(schema='kart.vgl.v1', status='complete',
            input_shape=[1, 3, 480, 640], onnx_sha256=self.data['onnx_sha256'],
            cuvslam_sha256=map_files(source), config_sha256={f: digest(config / f) for f in CONFIG_FILES},
            source_map='/another-PC/original-map'))
        return source, bundle

    def test_capture_default_matches_hash_and_shape_not_pc_path(self):
        source, bundle = self.bundle_fixture()
        correct = vgl.model_option(self.models)
        wrong_hash = {**correct, 'path': '/models/wrong-weights', 'onnx_sha256': 'different'}
        wrong_size = {**correct, 'path': '/models/wrong-size', 'width': 424, 'height': 240}
        result = vgl.capture_option(source, bundle, [wrong_hash, wrong_size, correct])
        self.assertEqual(result['models'], [str(self.models)])
        self.assertEqual(result['default_model'], str(self.models))
        self.assertFalse(result['error'])
        missing = vgl.capture_option(source, bundle, [wrong_hash, wrong_size])
        self.assertFalse(missing['default_model'])
        self.assertTrue(missing['error'])

    def test_capture_default_rejects_wrong_source_and_corrupted_bundle(self):
        source, bundle = self.bundle_fixture()
        model = vgl.model_option(self.models)
        (bundle / 'vgl_runtime_config/matching_config.pb.txt').write_text('modified')
        self.assertTrue(vgl.capture_option(source, bundle, [model])['error'])
        (source / 'cuvslam_map/map.mdb').write_bytes(b'other map')
        self.assertIn('VSLAM', vgl.capture_option(source, bundle, [model])['error'])

    def test_size_from_hash_matched_manifest_not_folder_name(self):
        option = vgl.model_option(self.models)
        self.assertEqual((option['width'], option['height']), (640, 480))
        self.assertEqual(option['path'], str(self.models))
        self.assertFalse(option['error'])

    def test_848_by_100_model_shape_is_available_without_rounding_height(self):
        self.data['input_shape'] = [1, 3, 100, 848]
        atomic_json(self.manifest, self.data)
        option = vgl.model_option(self.models)
        self.assertFalse(option['error'])
        self.assertEqual((option['width'], option['height']), (848, 100))
        app = SimpleNamespace(maps=Maps(self.root / 'map'))
        (app.maps.root / 'course').mkdir()
        body = dict(id='course', name='course-vgl-848x100', model_dir=str(self.models))
        with patch('kart_bringup.prepare_vgl_map.source_frames'), patch('kart_studio.vgl.shutil.which', return_value='ros2'):
            self.assertEqual(vgl.prepare(app, body)[-1], [848, 100])
            with self.assertRaisesRegex(ValueError, '一致'):
                vgl.prepare(app, dict(body, height=128))

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
