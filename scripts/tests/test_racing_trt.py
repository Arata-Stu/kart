"""Official COCO and custom opponent ONNX contracts and target-engine caching."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


if importlib.util.find_spec('onnx') is None:
    raise unittest.SkipTest('ONNX is required for racing model conversion tests')

import onnx
from onnx import TensorProto, helper

ROOT = Path(__file__).resolve().parents[2]
for relative in ('scripts', 'ros2_ws/src/kart_bringup', 'ros2_ws/src/kart_e2e'):
    sys.path.insert(0, str(ROOT / relative))
import build_racing_engine as builder
from kart_bringup.racing_models import get_profile
from prepare_racing_onnx import prepare


def detect_model(path, classes=80, dtype=TensorProto.FLOAT):
    graph = helper.make_graph([
        helper.make_node('ReduceMean', ['images'], ['mean'], keepdims=0),
        helper.make_node('Mul', ['mean', 'weight'], ['scaled']),
        helper.make_node('Expand', ['scaled', 'shape'], ['output0']),
    ], 'detect-contract',
        [helper.make_tensor_value_info('images', dtype, [1, 3, 480, 640])],
        [helper.make_tensor_value_info('output0', dtype, [1, classes + 4, 6300])],
        [helper.make_tensor('weight', dtype, [], [0]),
         helper.make_tensor('shape', TensorProto.INT64, [3], [1, classes + 4, 6300])])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)])
    onnx.checker.check_model(model)
    onnx.save(model, path)


class PrepareTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.source = Path(self.temp.name) / 'source.onnx'
        self.output = Path(self.temp.name) / 'ros.onnx'

    def test_official_fp32_coco_export_is_accepted_without_retraining(self):
        detect_model(self.source)
        prepare(self.source, self.output)
        original, result = onnx.load(self.source), onnx.load(self.output)
        self.assertEqual(result, original)
        self.assertEqual(result.graph.output[0].type.tensor_type.shape.dim[1].dim_value, 84)

    def test_fp16_exports_gain_fp32_boundary_bindings_for_both_profiles(self):
        for classes in (1, 80):
            with self.subTest(classes=classes):
                detect_model(self.source, classes, TensorProto.FLOAT16)
                before = self.source.read_bytes()
                prepare(self.source, self.output, classes)
                model = onnx.load(self.output)
                onnx.checker.check_model(model)
                self.assertEqual(self.source.read_bytes(), before)
                self.assertEqual(model.graph.input[0].type.tensor_type.elem_type, TensorProto.FLOAT)
                self.assertEqual(model.graph.output[0].type.tensor_type.elem_type, TensorProto.FLOAT)
                self.assertEqual(model.graph.node[0].op_type, 'Cast')
                self.assertEqual(model.graph.node[-1].op_type, 'Cast')

    def test_class_count_shape_nms_and_type_mismatches_are_rejected(self):
        for change in ('classes', 'nms', 'dynamic', 'dtype'):
            with self.subTest(change=change):
                detect_model(self.source)
                model = onnx.load(self.source)
                if change == 'classes':
                    model.graph.output[0].type.tensor_type.shape.dim[1].dim_value = 5
                elif change == 'nms':
                    dims = model.graph.output[0].type.tensor_type.shape.dim
                    dims[1].dim_value, dims[2].dim_value = 300, 6
                elif change == 'dynamic':
                    model.graph.input[0].type.tensor_type.shape.dim[0].dim_param = 'batch'
                else:
                    model.graph.input[0].type.tensor_type.elem_type = TensorProto.UINT8
                onnx.save(model, self.source)
                with self.assertRaises(ValueError):
                    prepare(self.source, self.output)


class BuilderTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.hardware = dict(gpu_uuid='gpu-test', tensorrt='11.0', driver=1)
        self.commands = []
        for profile in ('coco', 'opponent'):
            profile = get_profile(profile)
            detect_model(self.root / profile['source_name'], profile['num_classes'])

    def run_trtexec(self, command, **_kwargs):
        self.commands.append(command)
        output = next(v.split('=', 1)[1] for v in command if v.startswith('--saveEngine='))
        Path(output).write_bytes(b'validated-target-engine')
        return SimpleNamespace(returncode=0)

    def test_profiles_are_isolated_and_reuse_checks_class_count_and_hash(self):
        with patch.object(builder, 'fingerprint', return_value=self.hardware), \
                patch.object(builder.subprocess, 'run', side_effect=self.run_trtexec), \
                patch.object(builder, 'check_bindings') as check:
            coco = builder.build(self.root, get_profile('coco'))
            self.assertEqual(builder.build(self.root, get_profile('coco')), coco)
            self.assertEqual(len(self.commands), 1)
            self.assertEqual(check.call_args.args, (coco, 80))
            opponent = builder.build(self.root, get_profile('opponent'))
            self.assertNotEqual(coco.parent, opponent.parent)
            self.assertEqual(len(self.commands), 2)
            for name, path in (('coco', coco), ('opponent', opponent)):
                report = json.loads((path.parent / 'build.json').read_text())
                self.assertEqual(report['model_profile'], name)
                self.assertEqual(report['num_classes'], get_profile(name)['num_classes'])
                self.assertEqual(report['engine_sha256'], builder.digest(path))
            self.assertTrue(all('--fp16' not in command for command in self.commands))

            # Corrupted engine bytes cannot be reused just because its path exists.
            coco.write_bytes(b'corrupt')
            builder.build(self.root, get_profile('coco'))
            self.assertEqual(len(self.commands), 3)

    def test_failed_rebuild_preserves_old_engine_and_manifest(self):
        with patch.object(builder, 'fingerprint', return_value=self.hardware), \
                patch.object(builder.subprocess, 'run', side_effect=self.run_trtexec), \
                patch.object(builder, 'check_bindings'):
            path = builder.build(self.root, get_profile('coco'))
            before = path.read_bytes(), (path.parent / 'build.json').read_bytes()
        with patch.object(builder, 'fingerprint', return_value=self.hardware), \
                patch.object(builder.subprocess, 'run', return_value=SimpleNamespace(returncode=1)), \
                patch.object(builder, 'check_bindings'):
            with self.assertRaises(RuntimeError):
                builder.build(self.root, get_profile('coco'), force=True)
        self.assertEqual((path.read_bytes(), (path.parent / 'build.json').read_bytes()), before)

    def test_trt10_uses_typed_build_and_rejects_older_versions(self):
        hardware = dict(self.hardware, tensorrt='10.7')
        with patch.object(builder, 'fingerprint', return_value=hardware), \
                patch.object(builder.subprocess, 'run', side_effect=self.run_trtexec), \
                patch.object(builder, 'check_bindings'):
            builder.build(self.root, get_profile('coco'))
        self.assertIn('--stronglyTyped', self.commands[-1])
        with patch.object(builder, 'fingerprint', return_value=dict(hardware, tensorrt='9.0')):
            with self.assertRaisesRegex(ValueError, 'TensorRT 10'):
                builder.build(self.root, get_profile('coco'))

    def test_cli_defaults_to_official_model_and_allows_custom_model(self):
        for argv, name in (([], 'coco'), (['--model', 'opponent', '--force'], 'opponent')):
            with self.subTest(profile=name), patch.object(sys, 'argv', ['build_racing_engine.py', *argv]), \
                    patch.object(builder, 'build', return_value=Path('model.plan')) as build, \
                    contextlib.redirect_stdout(io.StringIO()):
                builder.main()
                self.assertEqual(build.call_args.args[1]['name'], name)
                self.assertEqual(build.call_args.args[2], name == 'opponent')

    def test_cli_accepts_a_transferred_model_directory_without_copying_over_existing_weights(self):
        target = self.root / 'finetuned' / 'version-a'
        with patch.object(sys, 'argv', ['build_racing_engine.py', '--model', 'opponent', '--model-root', str(target)]), \
                patch.object(builder, 'build', return_value=Path('model.plan')) as build, \
                contextlib.redirect_stdout(io.StringIO()):
            builder.main()
        self.assertEqual(build.call_args.args[0], target.resolve())
        self.assertEqual(build.call_args.args[1]['name'], 'opponent')


class BindingTests(unittest.TestCase):
    def test_engine_output_must_match_selected_profile_and_have_fp32_bindings(self):
        path = Path(self.enterContext(tempfile.TemporaryDirectory())) / 'model.plan'
        path.write_bytes(b'engine')
        shapes = {'images': (1, 3, 480, 640), 'output0': (1, 84, 6300)}
        engine = SimpleNamespace(num_io_tensors=2, get_tensor_mode=lambda n: n,
                                 get_tensor_shape=shapes.get, get_tensor_dtype=lambda _n: 'fp32')
        trt = SimpleNamespace(Logger=lambda _level: None,
                              Runtime=lambda _logger: SimpleNamespace(deserialize_cuda_engine=lambda _raw: engine),
                              TensorIOMode=SimpleNamespace(INPUT='images', OUTPUT='output0'), float32='fp32')
        trt.Logger = type('Logger', (), {'WARNING': 0, '__init__': lambda self, _level: None})
        with patch.dict(sys.modules, tensorrt=trt):
            builder.check_bindings(path, 80)
            with self.assertRaisesRegex(ValueError, 'output0'):
                builder.check_bindings(path, 1)
            engine.get_tensor_dtype = lambda _n: 'fp16'
            with self.assertRaisesRegex(ValueError, 'images'):
                builder.check_bindings(path, 80)
