"""UI service contracts and durable task routing, without GPU or ROS."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from kart_studio.perception import Perception
from kart_studio.storage import atomic_json
from kart_studio.server import Studio, Handler
from kart_studio.perception_task import execute


REPO = Path(__file__).resolve().parents[3]


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        (self.base / 'record').mkdir()
        (self.base / '.kart-studio').mkdir()
        self.service = Perception(REPO, self.base, self.base / 'record', self.base / '.kart-studio')
        self.weights = self.base / 'official.pt'
        self.weights.write_bytes(b'fixture, not a checkpoint')
        self.service.save_settings(dict(python=sys.executable, weights=str(self.weights), device='cpu'))
        self.project = self.service.create(dict(name='RCカーとオレンジダクト'))

    def selection_project(self, seeds=False):
        document = self.service.datasets.load(self.project['id'])
        frame = dict(id='frame-a', source_id='bag-a', width=100, height=80, stamp_ns='1000',
                     split='train', reviewed=False, boxes=[], suggestions=[], revision=1, image='images/frame-a.jpg')
        document['frames'] = [frame]
        if seeds:
            document['frames'].extend([
                dict(frame, id='positive', image='images/positive.jpg', reviewed=True, boxes=[[10, 10, 30, 30]]),
                dict(frame, id='negative', image='images/negative.jpg', reviewed=True),
            ])
        document['sources'] = [dict(id='bag-a', content_fingerprint='hash', split='train', topic='/rgb')]
        folder = self.service.datasets.folder(document['id'])
        for row in document['frames']:
            image = folder / row['image']
            image.parent.mkdir(exist_ok=True)
            image.write_bytes(b'path fixture; inference is mocked')
        atomic_json(folder / 'project.json', document)
        return self.service.datasets.load(document['id'])

    def test_display_name_and_storage_id_are_separate_and_annotation_conflicts_preserve_data(self):
        self.assertEqual(self.project['name'], 'RCカーとオレンジダクト')
        self.assertTrue(self.project['id'].startswith('rc-car-'))
        self.assertEqual(self.project['class_names'], ['rc_car', 'obstacle'])
        self.assertEqual(self.service.datasets.load(self.project['id']), self.project)
        with self.assertRaises(ValueError):
            self.service.datasets.load('../outside')

    def test_annotation_requires_two_class_client_and_saves_mixed_labels(self):
        project = self.selection_project()
        request = dict(id=project['id'], frame='frame-a', revision=project['revision'], frame_revision=1,
                       boxes=[[10, 10, 30, 30], [50, 10, 80, 30]], reviewed=True)
        with self.assertRaisesRegex(ValueError, '再読み込み'):
            self.service.annotate(request)
        with self.assertRaisesRegex(ValueError, '再読み込み'):
            self.service.annotate(dict(request, class_names=['rc_car', 'obstacle'], class_ids=None))
        self.assertEqual(self.service.datasets.load(project['id']), project)
        saved = self.service.annotate(dict(request, class_names=['rc_car', 'obstacle'], class_ids=[0, 1]))
        self.assertEqual(saved['frames'][0]['class_ids'], [0, 1])
        self.assertEqual(saved['frames'][0]['boxes'], request['boxes'])
        with self.assertRaisesRegex(ValueError, '更新'):
            self.service.annotate(dict(request, class_names=['rc_car', 'obstacle'], class_ids=[1, 0]))

    def test_train_parameters_are_bounded_and_revisions_are_captured(self):
        plan = self.service.prepare('train', dict(id=self.project['id'], epochs=2, batch=1, imgsz=320))
        self.assertEqual(plan['revision'], self.project['revision'])
        self.assertEqual(plan['settings']['weights'], str(self.weights))
        self.assertIn('perception:compute', plan['resources'])
        self.assertIn('read:perception:environment', plan['resources'])
        for body in (dict(epochs=True), dict(batch=0), dict(imgsz=321), dict(lr0=float('nan'))):
            with self.subTest(body=body), self.assertRaises(ValueError):
                self.service.prepare('train', dict(id=self.project['id'], **body))

    def test_settings_cannot_turn_into_shell_arguments_or_unknown_devices(self):
        for values in (dict(python=sys.executable, weights='relative.pt', device='cpu'),
                       dict(python=sys.executable, weights=str(self.weights), device='cuda;echo bad'),
                       dict(python=sys.executable, weights=str(self.weights), device='cpu', extra=True)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.service.save_settings(values)

    def test_extract_rejects_paths_and_limits_before_starting_a_job(self):
        for body in (dict(bag='../elsewhere'), dict(bag='/absolute'), dict(bag='missing')):
            with self.subTest(body=body), self.assertRaises(ValueError):
                self.service.prepare('extract', dict(id=self.project['id'], **body))

    def test_extract_uses_studio_python_without_training_environment_or_weights(self):
        missing_python = str(self.base / 'not-installed/bin/python')
        self.service.save_settings(dict(python=missing_python, weights=str(self.base / 'missing.pt'), device='cpu'))
        before = self.service.settings()
        bag = self.base / 'record' / 'day' / 'session' / 'bag'
        bag.mkdir(parents=True)
        (bag / 'metadata.yaml').write_text('bag path fixture; extraction is not executed')
        with patch('kart_studio.perception.shutil.which', side_effect=lambda value: None if value == missing_python else sys.executable), \
                patch('kart_studio.perception.subprocess.run', return_value=Mock(returncode=0)) as probe:
            plan = self.service.prepare('extract', dict(id=self.project['id'], bag='day/session/bag'))
        self.assertEqual(plan['settings']['python'], sys.executable)
        self.assertEqual(plan['bag_key'], 'day/session/bag')
        self.assertEqual(self.service.settings(), before, 'extract does not rewrite training settings')
        self.assertIn('rosbags.highlevel', probe.call_args.args[0][-1])
        self.assertIn('PIL', probe.call_args.args[0][-1])
        self.assertNotIn('ultralytics', probe.call_args.args[0][-1])
        job = Mock()
        self.assertEqual(self.service.execute(job, plan), dict(project=self.project['id']))
        self.assertEqual(job.run.call_args.args[0][:3], [sys.executable, '-m', 'kart_studio.perception_task'])
        self.assertFalse(Path(job.run.call_args.args[0][-1]).exists())

    def test_extract_prefers_configured_python_then_falls_back_when_imports_fail(self):
        configured = str(self.base / 'training/bin/python')
        self.service.save_settings(dict(self.service.settings(), python=configured))
        for codes, expected in (([0], configured), ([1, 0], sys.executable)):
            with self.subTest(codes=codes), \
                    patch('kart_studio.perception.shutil.which', side_effect=lambda value: value), \
                    patch('kart_studio.perception.subprocess.run', side_effect=[Mock(returncode=code) for code in codes]) as probe:
                self.assertEqual(self.service.extraction_python(), expected)
                self.assertEqual(probe.call_count, len(codes))

    def test_missing_extraction_dependencies_explain_setup_location(self):
        with patch('kart_studio.perception.shutil.which', side_effect=lambda value: value), \
                patch('kart_studio.perception.subprocess.run', return_value=Mock(returncode=1)):
            with self.assertRaisesRegex(ValueError, 'rosbags・Pillow.*3 学習・ONNX.*Notebookの学習環境'):
                self.service.extraction_python()

    def test_training_still_requires_its_selected_python(self):
        self.service.save_settings(dict(self.service.settings(), python=str(self.base / 'missing/bin/python')))
        with self.assertRaisesRegex(ValueError, '学習環境をセットアップ'):
            self.service.prepare('train', dict(id=self.project['id']))

    def test_catalog_only_offers_completed_model_downloads_and_confines_files(self):
        folder = self.service.root / 'models' / 'model-a'
        folder.mkdir()
        atomic_json(folder / 'artifact.json', dict(status='succeeded', created=1, name='RCモデル'))
        (folder / 'opponent.onnx').write_bytes(b'fixture, not an ONNX model')
        atomic_json(folder / 'report.json', dict(num_classes=1))
        with patch.object(self.service, 'environment', return_value=dict(ready=True)):
            catalog = self.service.catalog()
        self.assertEqual(catalog['projects'][0]['frames'], 0)
        self.assertEqual(len(catalog['models'][0]['downloads']), 3)
        self.assertEqual(self.service.artifact('model-a', 'opponent.onnx'), folder / 'opponent.onnx')
        for filename in ('../../outside', 'weights/best.pt', 'request.json'):
            with self.subTest(filename=filename), self.assertRaises(ValueError):
                self.service.artifact('model-a', filename)

    def test_job_uses_selected_python_fixed_module_and_cleans_request(self):
        plan = self.service.prepare('prelabel', dict(id=self.project['id']))
        job = Mock()
        result = self.service.execute(job, plan)
        argv = job.run.call_args.args[0]
        self.assertEqual(argv[:3], [sys.executable, '-m', 'kart_studio.perception_task'])
        self.assertFalse(Path(argv[-1]).exists())
        self.assertEqual(result, dict(project=self.project['id']))
        self.assertEqual(list(self.service.root.glob('.request-*')), [])

    def test_export_rejects_unfinished_training(self):
        run = self.service.root / 'runs' / 'unfinished'
        run.mkdir()
        atomic_json(run / 'artifact.json', dict(status='failed'))
        with self.assertRaisesRegex(ValueError, '完了'):
            self.service.prepare('export', dict(run='unfinished'))

    def test_selection_is_bounded_and_dino_requires_train_seeds(self):
        with self.assertRaisesRegex(ValueError, '未確認'):
            self.service.prepare('select', dict(id=self.project['id']))
        document = self.selection_project()
        plan = self.service.prepare('select', dict(id=document['id'], limit=10, max_images=20))
        self.assertEqual(plan['method'], 'yolo')
        self.assertIn('images_signature', plan)
        self.assertEqual(plan['weights'], str(self.weights))
        with self.assertRaisesRegex(ValueError, '以下'):
            self.service.prepare('select', dict(id=document['id'], limit=21, max_images=20))
        with self.assertRaisesRegex(ValueError, 'それぞれ'):
            self.service.prepare('select', dict(id=document['id'], method='dinov3'))

    def test_yolo_review_threshold_defaults_custom_values_and_invalid_inputs(self):
        document = self.selection_project()
        keys = ('review_confidence', 'expected_max_detections', 'large_bbox_area_ratio')
        plan = self.service.prepare('select', dict(id=document['id']))
        self.assertEqual({key: plan[key] for key in keys}, dict(
            review_confidence=.10, expected_max_detections=3, large_bbox_area_ratio=.40))
        for values in (dict(review_confidence=.35, expected_max_detections=0, large_bbox_area_ratio=.55),
                       dict(review_confidence=.05, expected_max_detections=100, large_bbox_area_ratio=1.)):
            with self.subTest(values=values):
                plan = self.service.prepare('select', dict(id=document['id'], **values))
                self.assertEqual({key: plan[key] for key in keys}, values)
                self.assertIs(type(plan['expected_max_detections']), int)
        invalid = (dict(review_confidence=.049), dict(review_confidence=1.01),
                   dict(review_confidence=float('nan')), dict(review_confidence=True),
                   dict(expected_max_detections=-1), dict(expected_max_detections=101),
                   dict(expected_max_detections=1.5), dict(expected_max_detections=True),
                   dict(large_bbox_area_ratio=.009), dict(large_bbox_area_ratio=1.01),
                   dict(large_bbox_area_ratio=float('inf')), dict(large_bbox_area_ratio=False))
        for values in invalid:
            with self.subTest(invalid=values), self.assertRaises(ValueError):
                self.service.prepare('select', dict(id=document['id'], **values))

    def test_selection_task_forwards_yolo_review_options_and_publishes_only_advisory_results(self):
        document = self.selection_project(seeds=True)
        options = dict(review_confidence=.35, expected_max_detections=0, large_bbox_area_ratio=.55)
        encoder = self.base / 'dinov3'
        encoder.mkdir()
        encoder_weights = self.base / 'dinov3.pth'
        encoder_weights.write_bytes(b'fixture, not encoder weights')
        for method in ('yolo', 'dinov3'):
            with self.subTest(method=method):
                body = dict(id=document['id'], method=method, limit=1, max_images=10,
                            encoder_repo=str(encoder), encoder_weights=str(encoder_weights), **options)
                plan = self.service.prepare('select', body)
                if method == 'dinov3':
                    self.assertFalse(set(options) & set(plan), 'YOLO thresholds do not enter DINO plans')
                    # Even a restored legacy plan carrying these fields must not forward them to DINO.
                    plan.update(options)
                result = dict(method=method, ranked=[dict(frame_id='frame-a', rank=1, score=.8,
                    reason='重点確認の目安（誤りの断定ではありません）')])
                if method == 'yolo':
                    result['ranked'][0].update(detections=8, review_count=5, max_bbox_area_ratio=.7,
                                              review_flags=['many_detections', 'large_bbox'], hard_review=True)
                before = self.service.datasets.load(document['id'])
                with patch('kart_studio.perception_selection.run', return_value=result) as select_images:
                    execute(plan)
                forwarded = select_images.call_args.args[0]
                self.assertEqual(forwarded['method'], method)
                self.assertEqual(forwarded['device'], 'cpu')
                self.assertEqual(len(forwarded['frames']), 3)
                self.assertTrue(all(Path(row['path']).is_file() for row in forwarded['frames']))
                if method == 'yolo':
                    self.assertEqual({key: forwarded[key] for key in options}, options)
                else:
                    self.assertFalse(set(options) & set(forwarded))
                published = self.service.selections.get(document['id'])
                self.assertEqual(published['ranked'], result['ranked'])
                self.assertFalse(published['automatic_ground_truth'])
                self.assertFalse(published['stale'])
                self.assertEqual(self.service.datasets.load(document['id']), before,
                                 'selection cannot change human boxes, negatives or reviewed flags')

    def test_tools_preferences_are_separate_from_training_environment(self):
        before = self.service.settings()
        values = self.service.save_tools_settings(dict(container_model_root='/workspaces/custom/models'))
        self.assertEqual(values['container_model_root'], '/workspaces/custom/models')
        self.assertEqual(self.service.settings(), before)
        with self.assertRaises(ValueError):
            self.service.save_tools_settings(dict(shell_command='unexpected'))


class StudioRouteTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.studio = Studio(REPO, self.base)
        self.addCleanup(self.studio.instance_lock.close)

    def test_project_and_setup_routes_use_existing_durable_jobs(self):
        project = self.studio.post('/api/perception/create', dict(name='RC実験'))
        self.assertEqual(self.studio.get('/api/perception/project', dict(id=project['id'])), project)
        with patch.object(self.studio.jobs, 'start', return_value=dict(id='job-a')) as start:
            self.assertEqual(self.studio.post('/api/perception/setup', {}), dict(id='job-a'))
            title, resources, task = start.call_args.args
            self.assertEqual(task.action, 'perception')
            self.assertEqual(task.args[0]['action'], 'setup')
            self.assertIn('perception:environment', resources)
        self.assertEqual(self.studio.get('/api/perception/selection', dict(id=project['id']))['ranked'], [])

    def test_onnx_push_uses_the_shared_scp_queue_and_never_schedules_a_build(self):
        plan = dict(id='model-a', connection=dict(user='kart', host='jetson.example'),
                    model_root='/models/rc-car')
        with patch('kart_studio.perception_transfer.PerceptionTransfer') as transfer, \
                patch.object(self.studio.jobs, 'start', return_value=dict(id='push-a', status='queued')) as start:
            transfer.return_value.prepare.return_value = plan
            result = self.studio.post('/api/perception/push', dict(id='model-a', model_root='/models/rc-car'))
        self.assertEqual(result['status'], 'queued')
        self.assertEqual(start.call_args.kwargs, dict(queue='scp'))
        title, resources, task = start.call_args.args
        self.assertEqual(task.action, 'perception_push')
        self.assertEqual(task.args, [plan])
        self.assertIn('read:perception:model:model-a', resources)

    def test_missing_image_cannot_read_arbitrary_local_file(self):
        handler = Mock()
        handler.path = '/perception-image?id=../../elsewhere&frame=frame'
        handler.server.studio = self.studio
        Handler.do_GET(handler)
        self.assertEqual(handler.reply.call_args.args[0], 400)

    def test_artifact_download_only_accepts_the_model_allowlist(self):
        handler = Mock()
        handler.path = '/perception-file?id=example&file=../../.kart-studio/connection.json'
        handler.server.studio = self.studio
        Handler.do_GET(handler)
        self.assertEqual(handler.reply.call_args.args[0], 400)


if __name__ == '__main__':
    unittest.main()
