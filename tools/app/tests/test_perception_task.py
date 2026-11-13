"""Artifact publication keeps actual export class contracts and legacy assets."""
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from kart_studio import perception_task as task
from kart_studio.remote_agent import perception_manifest
from kart_studio.storage import atomic_json, read_json


class TaskTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.root = self.base / 'perception'
        for folder in ('runs', 'models'):
            (self.root / folder).mkdir(parents=True)

    def plan(self, action, key='new-model', **values):
        return dict(action=action, base=str(self.base), records=str(self.base / 'records'),
                    settings=dict(weights=str(self.base / 'official.pt'), device='cpu'),
                    key=key, title='RCカーと障害物', project='project-a', revision=7, created=1, **values)

    def export_source(self, key, classes):
        source = self.root / 'runs' / key
        (source / 'weights').mkdir(parents=True)
        (source / 'weights/best.pt').write_bytes(b'checkpoint; the runtime boundary is injected')
        atomic_json(source / 'artifact.json', dict(status='succeeded', project_revision=7, class_names=classes))

    def export_runtime(self, classes, invalid=False):
        def run(plan):
            output = Path(plan['output'])
            payload = output / 'opponent.onnx'
            payload.write_bytes(b'ONNX payload fixture; no inference performed')
            count = len(classes)
            atomic_json(output / 'report.json', dict(schema='kart.perception.export.v1',
                class_names=classes, model_profile='racing' if count == 2 else 'opponent', num_classes=count,
                onnx_sha256=hashlib.sha256(payload.read_bytes()).hexdigest(),
                input=dict(name='images', shape=[1, 3, 480, 640], dtype='float32'),
                output=dict(name='output0', shape=[1, 5 if invalid else 4 + count, 6300], dtype='float32')))
        return run

    def test_export_preserves_legacy_opponent_and_new_racing_contracts(self):
        for index, classes in enumerate((['rc_car'], ['rc_car', 'obstacle'])):
            source, key = f'run-{index}', f'model-{index}'
            self.export_source(source, classes)
            with patch('kart_studio.perception_runtime.run', side_effect=self.export_runtime(classes)):
                task.execute(self.plan('export', key, run=source))
            model = self.root / 'models' / key
            artifact = read_json(model / 'artifact.json')
            self.assertEqual(artifact['class_names'], classes)
            self.assertEqual(artifact['num_classes'], len(classes))
            self.assertEqual(artifact['model_profile'], 'opponent' if len(classes) == 1 else 'racing')
            self.assertEqual(artifact['source_revision'], 7)
            self.assertEqual(set(perception_manifest(model, transfer_required=False)),
                             {'artifact.json', 'report.json', 'opponent.onnx'})

    def test_mismatched_export_report_is_not_published(self):
        self.export_source('run-a', ['rc_car', 'obstacle'])
        with patch('kart_studio.perception_runtime.run', side_effect=self.export_runtime(['rc_car', 'obstacle'], invalid=True)):
            with self.assertRaisesRegex(ValueError, 'shape'):
                task.execute(self.plan('export', run='run-a'))
        self.assertFalse((self.root / 'models/new-model').exists())
        self.assertEqual(list((self.root / 'models').iterdir()), [])

    def test_new_training_artifact_is_two_class_and_retains_report(self):
        output = self.root / 'runs/new-run'
        def run(plan):
            self.assertEqual(plan['action'], 'train')
            (output / 'weights').mkdir()
            (output / 'weights/best.pt').write_bytes(b'checkpoint fixture')
            atomic_json(output / 'report.json', dict(class_names=['rc_car', 'obstacle'], model_profile='racing', num_classes=2))
        with patch.object(task.DatasetStore, 'snapshot', return_value=dict(dataset_yaml=output / 'dataset/data.yaml')), \
                patch('kart_studio.perception_runtime.run', side_effect=run):
            task.execute(self.plan('train', 'new-run', epochs=1, batch=2, imgsz=320, lr0=.001, seed=0, workers=0))
        artifact = read_json(output / 'artifact.json')
        self.assertEqual(artifact['status'], 'succeeded')
        self.assertEqual(artifact['class_names'], ['rc_car', 'obstacle'])
        self.assertEqual(artifact['model_profile'], 'racing')
        self.assertEqual(artifact['num_classes'], 2)
        self.assertEqual(artifact['report']['class_names'], artifact['class_names'])

    def test_failed_training_does_not_claim_completed_two_class_weights(self):
        output = self.root / 'runs/failed-run'
        with patch.object(task.DatasetStore, 'snapshot', side_effect=ValueError('obstacle不足')):
            with self.assertRaisesRegex(ValueError, 'obstacle不足'):
                task.execute(self.plan('train', 'failed-run'))
        artifact = read_json(output / 'artifact.json')
        self.assertEqual(artifact['status'], 'failed')
        self.assertEqual(artifact['class_names'], ['rc_car', 'obstacle'])
        self.assertFalse((output / 'weights/best.pt').exists())


if __name__ == '__main__':
    unittest.main()
