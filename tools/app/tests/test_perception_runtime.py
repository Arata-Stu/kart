"""Training boundary contracts; no model downloads or GPU required."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from kart_studio import perception_runtime as runtime


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.weights = self.root / 'yolov8s.pt'
        self.weights.write_bytes(b'test checkpoint; model boundary is injected')
        self.source_hash = hashlib.sha256(self.weights.read_bytes()).hexdigest()
        self.addCleanup(patch.stopall)
        patch.object(runtime, 'OFFICIAL_WEIGHTS_SHA256', self.source_hash).start()
        patch.object(runtime, '_device', return_value='cpu').start()

    def dataset(self):
        root = self.root / 'dataset'
        for split in ('train', 'val'):
            folder = root / 'images' / split
            folder.mkdir(parents=True)
            (folder / f'{split}.jpg').write_bytes(b'image data inspected by the real trainer, not this boundary test')
        yaml = root / 'data.yaml'
        yaml.write_text(f'path: {json.dumps(str(root))}\ntrain: images/train\nval: images/val\nnc: 2\nnames:\n  0: rc_car\n  1: obstacle\n')
        meta = dict(schema='kart.perception.dataset.v2', class_names=['rc_car', 'obstacle'], annotation_revision=7,
                    annotation_sha256='labels-sha', split_bag_ids=dict(train=['bag-train'], val=['bag-val']),
                    frames=[dict(reviewed=True, split=split, boxes=[[1, 2, 20, 30], [30, 2, 40, 30]],
                                 class_ids=[0, 1]) for split in ('train', 'val')])
        (root / 'snapshot.json').write_text(json.dumps(meta))
        return yaml

    def trainer(self, epoch=0):
        return SimpleNamespace(epoch=epoch, tloss=[1, 2, 3], lr={'lr/pg0': .001},
            metrics={'metrics/precision(B)': .6, 'metrics/recall(B)': .7,
                     'metrics/mAP50(B)': .8, 'metrics/mAP50-95(B)': .4, 'val/box_loss': 2},
            label_loss_items=lambda _loss, prefix: {f'{prefix}/box_loss': 1, f'{prefix}/cls_loss': 2})

    def test_import_does_not_load_learning_libraries_in_server(self):
        code = ('import sys; import kart_studio.perception_runtime; '
                'assert not any(x in sys.modules for x in ("torch","ultralytics","onnx","PIL"))')
        subprocess.run([sys.executable, '-c', code], check=True)

    def test_train_explicit_optimizer_metrics_and_existing_artifact_directory(self):
        dataset = self.dataset()
        output = self.root / 'run'
        output.mkdir()
        (output / 'artifact.json').write_text('{"status":"running"}')
        model = SimpleNamespace(names={i: f'class{i}' for i in range(80)}, task='detect')
        callbacks = {}
        model.add_callback = lambda name, callback: callbacks.update({name: callback})
        def train(**kwargs):
            model.names = {0: 'rc_car', 1: 'obstacle'}
            (output / 'weights').mkdir()
            for name in ('best.pt', 'last.pt'):
                (output / 'weights' / name).write_bytes(b'original-format training checkpoint')
            callbacks['on_train_epoch_start'](self.trainer())
            callbacks['on_fit_epoch_end'](self.trainer())
        model.train = Mock(side_effect=train)
        with patch.object(runtime, '_load_yolo', return_value=model):
            report = runtime.run(dict(action='train', weights=str(self.weights), dataset_yaml=str(dataset),
                output=str(output), epochs=1, batch=2, imgsz=320, lr0=.001, device='cpu', seed=0, workers=0))
        kwargs = model.train.call_args.kwargs
        self.assertEqual(kwargs['optimizer'], 'AdamW')
        self.assertEqual(kwargs['lr0'], .001)
        self.assertIs(kwargs['amp'], False)
        self.assertIs(kwargs['resume'], False)
        self.assertEqual(kwargs['pretrained'], str(self.weights.resolve()))
        self.assertEqual(report['annotation_revision'], 7)
        self.assertEqual(report['source_weights_sha256'], self.source_hash)
        self.assertEqual(report['class_names'], ['rc_car', 'obstacle'])
        self.assertEqual(report['model_profile'], 'racing')
        self.assertEqual(report['num_classes'], 2)
        self.assertEqual(report['class_boxes'], {'train': [1, 1], 'val': [1, 1]})
        rows = [json.loads(v) for v in (output / 'metrics.jsonl').read_text().splitlines()]
        self.assertEqual((rows[0]['epoch'], rows[0]['loss'], rows[0]['map50'], rows[0]['map50_95']), (1, 3., .8, .4))
        self.assertEqual(self.weights.read_bytes(), b'test checkpoint; model boundary is injected')
        with patch.object(runtime, '_load_yolo') as load, self.assertRaises(ValueError):
            runtime.run(dict(action='train', weights=str(self.weights), dataset_yaml=str(dataset), output=str(output)))
        load.assert_not_called()

    def test_unreviewed_and_same_bag_validation_are_rejected_before_model_load(self):
        dataset = self.dataset()
        path = dataset.parent / 'snapshot.json'
        original = json.loads(path.read_text())
        for key in ('unreviewed', 'same_bag'):
            value = json.loads(json.dumps(original))
            if key == 'unreviewed':
                value['frames'][0]['reviewed'] = False
            else:
                value['split_bag_ids']['val'] = ['bag-train']
            path.write_text(json.dumps(value))
            with self.subTest(key=key), patch.object(runtime, '_load_yolo') as load, self.assertRaises(ValueError):
                runtime.run(dict(action='train', weights=str(self.weights), dataset_yaml=str(dataset), output=str(self.root / key)))
            load.assert_not_called()

    def test_remote_dataset_scripts_wrong_base_and_invalid_parameters_are_rejected(self):
        dataset = self.dataset()
        with dataset.open('a') as stream:
            stream.write('download: echo unsafe\n')
        with patch.object(runtime, '_load_yolo') as load, self.assertRaises(ValueError):
            runtime.run(dict(action='train', weights=str(self.weights), dataset_yaml=str(dataset), output=str(self.root / 'run')))
        load.assert_not_called()
        with patch.object(runtime, 'OFFICIAL_WEIGHTS_SHA256', 'different'), self.assertRaises(ValueError):
            runtime.run(dict(action='prelabel', weights=str(self.weights), frames=[], output=str(self.root / 'proposals')))
        for key, value in (('epochs', 0), ('imgsz', 641), ('lr0', float('nan')), ('batch', True)):
            with self.subTest(key=key), self.assertRaises(ValueError):
                runtime.run(dict(action='train', weights=str(self.weights), dataset_yaml=str(dataset),
                    output=str(self.root / 'invalid'), **{key: value}))

    def test_new_training_requires_v2_two_class_snapshot_with_class_coverage(self):
        dataset = self.dataset()
        snapshot = dataset.parent / 'snapshot.json'
        original = json.loads(snapshot.read_text())
        for case in ('legacy_schema', 'wrong_names', 'mismatched_ids', 'missing_obstacle', 'bool_class'):
            metadata = json.loads(json.dumps(original))
            if case == 'legacy_schema':
                metadata['schema'] = 'kart.perception.dataset.v1'
            elif case == 'wrong_names':
                metadata['class_names'] = ['obstacle', 'rc_car']
            elif case == 'mismatched_ids':
                metadata['frames'][0]['class_ids'] = [0]
            elif case == 'missing_obstacle':
                metadata['frames'][1]['class_ids'] = [0, 0]
            else:
                metadata['frames'][0]['class_ids'] = [False, 1]
            snapshot.write_text(json.dumps(metadata))
            with self.subTest(case=case), patch.object(runtime, '_load_yolo') as load:
                with self.assertRaises(ValueError):
                    runtime.run(dict(action='train', weights=str(self.weights), dataset_yaml=str(dataset),
                                     output=str(self.root / case)))
                load.assert_not_called()
        dataset.write_text(dataset.read_text().replace('nc: 2', 'nc: 1'))
        with self.assertRaisesRegex(ValueError, '2クラス'):
            runtime._dataset(dataset)

    def test_metrics_ignore_nonfinite_values_and_duplicate_epoch_callbacks(self):
        path = self.root / 'metrics.jsonl'
        logger = runtime.Metrics(path, 2)
        trainer = self.trainer()
        trainer.metrics['metrics/mAP50(B)'] = float('nan')
        trainer.label_loss_items = lambda *_args, **_kwargs: {'train/box_loss': float('inf')}
        logger.start_epoch(trainer)
        logger(trainer)
        logger(trainer)
        trainer.epoch = 1  # final_eval's best-model callback must not count as a second training epoch.
        logger(trainer)
        rows = path.read_text().splitlines()
        self.assertEqual(len(rows), 1)
        self.assertIsNone(json.loads(rows[0])['map50'])
        self.assertIsNone(json.loads(rows[0])['loss'])

    def test_prelabel_uses_all_coco_classes_and_keeps_zero_proposals_unreviewed(self):
        from PIL import Image
        frames = []
        for identity in ('positive', 'negative'):
            path = self.root / f'{identity}.png'
            Image.new('RGB', (32, 24)).save(path)
            frames.append(dict(id=identity, path=str(path)))
        xyxy = SimpleNamespace(cpu=lambda: SimpleNamespace(tolist=lambda: [[-3, 2, 40, 21], [5, 5, 5, 9]]))
        model = SimpleNamespace(names={i: str(i) for i in range(80)}, predict=Mock(side_effect=[
            [SimpleNamespace(orig_shape=(24, 32), boxes=SimpleNamespace(xyxy=xyxy))],
            [SimpleNamespace(orig_shape=(24, 32), boxes=None)]]))
        output = self.root / 'prelabel'
        with patch.object(runtime, '_load_yolo', return_value=model):
            report = runtime.run(dict(action='prelabel', output=str(output), weights=str(self.weights),
                frames=frames, device='cpu', confidence=.1))
        self.assertEqual(json.loads((output / 'suggestions.json').read_text()),
                         {'positive': [[0., 2., 32, 21.]], 'negative': []})
        self.assertIs(model.predict.call_args.kwargs['classes'], None)
        self.assertIs(report['reviewed'], False)
        self.assertIs(report['automatic_ground_truth'], False)
        self.assertIn('rc_car / obstacle', report['note'])

    def test_empty_proposal_job_needs_no_torch_and_excess_images_are_rejected(self):
        with patch.object(runtime, '_load_yolo') as load:
            output = self.root / 'empty'
            runtime.run(dict(action='prelabel', weights=str(self.weights), output=str(output), frames=[]))
            load.assert_not_called()
        self.assertEqual(json.loads((output / 'suggestions.json').read_text()), {})
        with self.assertRaises(ValueError):
            runtime.run(dict(action='prelabel', weights=str(self.weights), output=str(self.root / 'too-many'),
                             frames=[{}] * (runtime.MAX_FRAMES + 1)))

    def test_export_checks_one_class_and_copies_checkpoint_before_fixed_onnx_export(self):
        import onnx
        from onnx import TensorProto, helper
        calls = []
        def load(weights, output):
            self.assertNotEqual(weights, self.weights)
            self.assertEqual(weights.read_bytes(), self.weights.read_bytes())
            def export(**kwargs):
                calls.append(kwargs)
                path = weights.with_suffix('.onnx')
                graph = helper.make_graph([helper.make_node('Constant', [], ['output0'], value=helper.make_tensor(
                    'values', TensorProto.FLOAT, [1, 5, 6300], [0.] * (5 * 6300)))], 'raw-detect',
                    [helper.make_tensor_value_info('images', TensorProto.FLOAT, [1, 3, 480, 640])],
                    [helper.make_tensor_value_info('output0', TensorProto.FLOAT, [1, 5, 6300])])
                onnx.save(helper.make_model(graph, opset_imports=[helper.make_opsetid('', 17)]), path)
                return str(path)
            return SimpleNamespace(names={0: 'rc_car'}, task='detect', export=export)
        output = self.root / 'export'
        with patch.object(runtime, '_load_yolo', side_effect=load):
            report = runtime.run(dict(action='export', weights=str(self.weights), output=str(output)))
        self.assertEqual(calls[0]['imgsz'], (480, 640))
        self.assertIs(calls[0]['dynamic'], False)
        self.assertIs(calls[0]['nms'], False)
        self.assertIs(calls[0]['half'], False)
        self.assertEqual(report['output']['shape'], [1, 5, 6300])
        self.assertEqual(runtime._sha(self.weights), self.source_hash)
        onnx.checker.check_model(onnx.load(output / 'opponent.onnx'))
        with patch.object(runtime, '_load_yolo', return_value=SimpleNamespace(names={i: str(i) for i in range(80)})), \
                self.assertRaisesRegex(ValueError, 'class names/order'):
            runtime.run(dict(action='export', weights=str(self.weights), output=str(self.root / 'wrong-nc')))

    def test_export_two_class_racing_shape_and_strict_class_name_order(self):
        import onnx
        from onnx import TensorProto, helper
        def load(weights, output):
            def export(**kwargs):
                path = weights.with_suffix('.onnx')
                graph = helper.make_graph([helper.make_node('Constant', [], ['output0'], value=helper.make_tensor(
                    'values', TensorProto.FLOAT, [1, 6, 6300], [0.] * (6 * 6300)))], 'raw-detect',
                    [helper.make_tensor_value_info('images', TensorProto.FLOAT, [1, 3, 480, 640])],
                    [helper.make_tensor_value_info('output0', TensorProto.FLOAT, [1, 6, 6300])])
                onnx.save(helper.make_model(graph, opset_imports=[helper.make_opsetid('', 17)]), path)
                return str(path)
            # Dict insertion order differs, but numeric IDs are authoritative.
            return SimpleNamespace(names={1: 'obstacle', 0: 'rc_car'}, task='detect', export=export)
        output = self.root / 'racing-export'
        with patch.object(runtime, '_load_yolo', side_effect=load):
            report = runtime.run(dict(action='export', weights=str(self.weights), output=str(output)))
        self.assertEqual(report['model_profile'], 'racing')
        self.assertEqual(report['class_names'], ['rc_car', 'obstacle'])
        self.assertEqual(report['num_classes'], 2)
        self.assertEqual(report['output']['shape'], [1, 6, 6300])
        onnx.checker.check_model(onnx.load(output / 'opponent.onnx'))
        invalid = ({0: 'obstacle', 1: 'rc_car'}, {0: 'rc_car', 1: 'cone'}, {0: 'opponent'},
                   {1: 'rc_car'}, {0: 'rc_car', 1: 'obstacle', 2: 'other'})
        for index, names in enumerate(invalid):
            model = SimpleNamespace(names=names, task='detect', export=Mock())
            with self.subTest(names=names), patch.object(runtime, '_load_yolo', return_value=model):
                with self.assertRaises(ValueError):
                    runtime.run(dict(action='export', weights=str(self.weights), output=str(self.root / f'invalid-{index}')))
                model.export.assert_not_called()

    def test_export_rejects_raw_output_shape_disagreeing_with_two_class_names(self):
        import onnx
        from onnx import TensorProto, helper
        def load(weights, output):
            def export(**kwargs):
                source = weights.with_suffix('.onnx')
                graph = helper.make_graph([], 'wrong-class-count',
                    [helper.make_tensor_value_info('images', TensorProto.FLOAT, [1, 3, 480, 640])],
                    [helper.make_tensor_value_info('output0', TensorProto.FLOAT, [1, 5, 6300])])
                onnx.save(helper.make_model(graph, opset_imports=[helper.make_opsetid('', 17)]), source)
                return str(source)
            return SimpleNamespace(names={0: 'rc_car', 1: 'obstacle'}, task='detect', export=export)
        output = self.root / 'wrong-class-shape'
        with patch.object(runtime, '_load_yolo', side_effect=load), self.assertRaises(ValueError):
            runtime.run(dict(action='export', weights=str(self.weights), output=str(output)))
        self.assertFalse((output / 'opponent.onnx').exists())
        self.assertFalse((output / 'report.json').exists())

    def test_setup_script_rejects_extra_arguments_before_environment_changes(self):
        result = subprocess.run(['bash', str(runtime.REPO / 'scripts/perception_setup.sh'), '--inject'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('Usage:', result.stderr)
