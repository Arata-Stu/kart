"""Exercise fixed SSH/SCP publication locally, without a Jetson or model build."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from kart_studio.jobs import Cancelled
from kart_studio.perception_transfer import PerceptionTransfer
from kart_studio.remote_agent import execute, perception_manifest
from kart_studio.storage import atomic_json, read_json


class LocalScpJob:
    def __init__(self, cancel=False, mutate=None):
        self.cancel, self.mutate, self.copied = cancel, mutate, False
        self.commands, self.messages = [], []

    def check(self):
        if self.cancel and self.copied:
            raise Cancelled('cancelled after scp')

    def log(self, message):
        self.messages.append(message)

    def run(self, command):
        self.commands.append(command)
        if command[0] != 'scp':
            raise AssertionError('Transfer must never run a model, engine build, or arbitrary command')
        source = Path(command[-2])
        destination = Path(command[-1].split(':', 1)[1]) / source.name
        shutil.copytree(source, destination)
        if self.mutate:
            self.mutate(destination)
        self.copied = True


class TransferTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name).resolve()
        self.local = self.base / 'local/perception'
        self.model = self.local / 'models/rc-model'
        self.model.mkdir(parents=True)
        (self.model / 'opponent.onnx').write_bytes(b'ONNX payload fixture; no inference is simulated')
        self.onnx_hash = hashlib.sha256((self.model / 'opponent.onnx').read_bytes()).hexdigest()
        atomic_json(self.model / 'report.json', dict(schema='kart.perception.export.v1',
            model_profile='opponent', num_classes=1, class_names=['rc_car'], onnx_sha256=self.onnx_hash,
            input=dict(name='images', shape=[1, 3, 480, 640], dtype='float32'),
            output=dict(name='output0', shape=[1, 5, 6300], dtype='float32')))
        atomic_json(self.model / 'artifact.json', dict(id='rc-model', status='succeeded',
            class_names=['rc_car'], model_profile='opponent', num_classes=1, source_revision=7))
        self.remote_root = self.base / 'remote/weights/racing/yolov8s/finetuned'
        self.remote_root.parent.mkdir(parents=True)
        (self.remote_root.parent / 'yolov8s.onnx').write_bytes(b'preserve official COCO')
        self.connection = dict(user='kart', host='jetson.local', port=2222,
                               record_root='/record', map_root='/map')
        self.transfer = PerceptionTransfer(self.base, self.local)
        self.body = dict(id='rc-model', connection=self.connection, model_root=str(self.remote_root),
                         container_model_root='/workspaces/weights/racing/yolov8s/finetuned')
        self.ssh_commands = []
        self.actions = []
        self.ssh = self.enterContext(patch('kart_studio.transfers.subprocess.run', side_effect=self.local_ssh))

    def local_ssh(self, command, **kwargs):
        self.assertEqual(command[0], 'ssh')
        self.assertIn('kart@jetson.local', command)
        self.assertEqual(command[command.index('-p')+1], '2222')
        self.assertTrue(command[-1].startswith('python3 -c '))
        self.ssh_commands.append(command)
        data = json.loads(kwargs['input'])
        self.actions.append(data['action'])
        try:
            result, code = execute(data), 0
        except Exception as exc:
            result, code = dict(error=str(exc)), 1
        return subprocess.CompletedProcess(command, code, json.dumps(result), '')

    def assert_no_staging(self):
        self.assertFalse(list(self.remote_root.glob('.kart-upload-*')))

    def two_classes(self):
        report = read_json(self.model / 'report.json')
        report.update(model_profile='racing', num_classes=2, class_names=['rc_car', 'obstacle'])
        report['output']['shape'] = [1, 6, 6300]
        atomic_json(self.model / 'report.json', report)
        artifact = read_json(self.model / 'artifact.json')
        artifact.update(model_profile='racing', num_classes=2, class_names=['rc_car', 'obstacle'])
        atomic_json(self.model / 'artifact.json', artifact)

    def test_success_publishes_all_verified_payloads_and_only_returns_a_build_command(self):
        plan = self.transfer.prepare(self.body)
        job = LocalScpJob()
        result = self.transfer.push(job, plan)
        target = Path(result['host_path'])
        self.assertEqual(result['remote_path'], result['host_path'])
        self.assertEqual(target, self.remote_root / plan['destination'])
        self.assertEqual(result['source_sha256'], self.onnx_hash)
        self.assertFalse(result['engine_built'])
        self.assertEqual(result['hashes'], perception_manifest(target))
        self.assertEqual(set(result['hashes']), {'opponent.onnx', 'report.json', 'artifact.json', 'transfer.json'})
        self.assertEqual(read_json(target / 'transfer.json')['payload_sha256'], plan['source_hashes'])
        self.assertEqual(result['container_path'], self.body['container_model_root'] + '/' + plan['destination'])
        self.assertEqual(result['build_command'], 'bash scripts/racing_trt.sh --model opponent --model-root ' + result['container_path'])
        self.assertEqual([command[0] for command in job.commands], ['scp'])
        self.assertEqual(self.actions, ['reserve-perception', 'publish-perception'])
        self.assert_no_staging()
        self.assertEqual((self.remote_root.parent / 'yolov8s.onnx').read_bytes(), b'preserve official COCO')

    def test_container_path_is_never_inferred_from_host_path(self):
        body = dict(self.body)
        del body['container_model_root']
        result = self.transfer.push(LocalScpJob(), self.transfer.prepare(body))
        self.assertIsNone(result['container_path'])
        self.assertIsNone(result['build_command'])

    def test_two_class_contract_uses_racing_build_and_ignores_client_profile_claim(self):
        self.two_classes()
        plan = self.transfer.prepare(dict(self.body, model_profile='opponent', num_classes=1))
        self.assertEqual(plan['model_profile'], 'racing')
        self.assertEqual(plan['num_classes'], 2)
        result = self.transfer.push(LocalScpJob(), plan)
        self.assertEqual(result['class_names'], ['rc_car', 'obstacle'])
        self.assertEqual(result['model_profile'], 'racing')
        self.assertEqual(result['build_command'],
                         'bash scripts/racing_trt.sh --model racing --model-root ' + result['container_path'])
        manifest = read_json(Path(result['host_path']) / 'transfer.json')
        self.assertEqual(manifest['model_profile'], 'racing')
        self.assertEqual(manifest['num_classes'], 2)
        self.assertEqual(manifest['class_names'], ['rc_car', 'obstacle'])
        self.assertEqual(result['hashes'], perception_manifest(Path(result['host_path'])))

    def test_legacy_transfer_manifest_without_profile_fields_still_validates(self):
        result = self.transfer.push(LocalScpJob(), self.transfer.prepare(self.body))
        target = Path(result['host_path'])
        manifest = read_json(target / 'transfer.json')
        for field in ('model_profile', 'num_classes', 'class_names'):
            del manifest[field]
        atomic_json(target / 'transfer.json', manifest)
        self.assertEqual(set(perception_manifest(target)), {'opponent.onnx', 'report.json', 'artifact.json', 'transfer.json'})

    def test_class_name_profile_shape_and_artifact_mismatches_are_rejected(self):
        self.two_classes()
        original = read_json(self.model / 'report.json')
        for change in (dict(class_names=['obstacle', 'rc_car']), dict(class_names=['rc_car', 'cone']),
                       dict(model_profile='opponent'), dict(num_classes=1), dict(num_classes=True),
                       dict(output=dict(name='output0', shape=[1, 5, 6300], dtype='float32'))):
            atomic_json(self.model / 'report.json', dict(original, **change))
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.transfer.prepare(self.body)
        atomic_json(self.model / 'report.json', original)
        artifact = read_json(self.model / 'artifact.json')
        artifact.update(class_names=['rc_car'], model_profile='opponent', num_classes=1)
        atomic_json(self.model / 'artifact.json', artifact)
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            self.transfer.prepare(self.body)
        self.assertEqual(self.actions, [])

    def test_tampered_profile_in_plan_or_remote_manifest_never_publishes(self):
        self.two_classes()
        plan = self.transfer.prepare(self.body)
        with self.assertRaisesRegex(ValueError, 'profile'):
            self.transfer.push(LocalScpJob(), dict(plan, model_profile='opponent'))
        self.assertEqual(self.actions, [])
        def tamper(bundle):
            manifest = read_json(bundle / 'transfer.json')
            manifest['model_profile'] = 'opponent'
            atomic_json(bundle / 'transfer.json', manifest)
        with self.assertRaisesRegex(ValueError, 'class/profile'):
            self.transfer.push(LocalScpJob(mutate=tamper), plan)
        self.assertFalse((self.remote_root / plan['destination']).exists())
        self.assert_no_staging()

    def test_known_local_runtime_caches_are_not_transferred(self):
        for directory in ('.ultralytics', '.matplotlib'):
            cache = self.model / directory
            cache.mkdir()
            (cache / 'settings.json').write_text('{}')
        result = self.transfer.push(LocalScpJob(), self.transfer.prepare(self.body))
        self.assertEqual({path.name for path in Path(result['host_path']).iterdir()},
                         {'opponent.onnx', 'report.json', 'artifact.json', 'transfer.json'})

    def test_unknown_local_entries_and_symlinks_inside_known_caches_are_refused(self):
        unexpected = self.model / 'extra.plan'
        unexpected.write_bytes(b'not an export payload')
        with self.assertRaisesRegex(ValueError, 'unexpected'):
            self.transfer.prepare(self.body)
        unexpected.unlink()
        cache = self.model / '.ultralytics'
        cache.mkdir()
        (cache / 'linked.json').symlink_to(self.model / 'report.json')
        with self.assertRaisesRegex(ValueError, 'symlink'):
            self.transfer.prepare(self.body)

    def test_cancel_cleans_staging_without_publishing(self):
        plan = self.transfer.prepare(self.body)
        with self.assertRaises(Cancelled):
            self.transfer.push(LocalScpJob(cancel=True), plan)
        self.assertFalse((self.remote_root / plan['destination']).exists())
        self.assertEqual(self.actions, ['reserve-perception', 'discard-perception'])
        self.assert_no_staging()

    def test_tampered_or_unexpected_payload_is_rejected_and_removed(self):
        mutations = [lambda bundle: (bundle / 'opponent.onnx').write_bytes(b'corrupt'),
                     lambda bundle: (bundle / 'extra.plan').write_bytes(b'no engines allowed')]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                plan = self.transfer.prepare(self.body)
                with self.assertRaises(ValueError):
                    self.transfer.push(LocalScpJob(mutate=mutate), plan)
                self.assertFalse((self.remote_root / plan['destination']).exists())
                self.assert_no_staging()

    def test_source_and_destination_symlinks_are_refused_without_touching_targets(self):
        source = self.model / 'opponent.onnx'
        external = self.base / 'external.onnx'
        external.write_bytes(source.read_bytes())
        source.unlink()
        source.symlink_to(external)
        with self.assertRaises(ValueError):
            self.transfer.prepare(self.body)
        source.unlink()
        source.write_bytes(external.read_bytes())
        plan = self.transfer.prepare(self.body)

        def link(bundle):
            (bundle / 'opponent.onnx').unlink()
            (bundle / 'opponent.onnx').symlink_to(external)

        with self.assertRaises(ValueError):
            self.transfer.push(LocalScpJob(mutate=link), plan)
        self.assertTrue(external.is_file())
        self.assertEqual(hashlib.sha256(external.read_bytes()).hexdigest(), self.onnx_hash)
        self.assert_no_staging()

    def test_existing_destination_and_model_files_are_never_overwritten(self):
        plan = self.transfer.prepare(self.body)
        target = self.remote_root / plan['destination']
        target.mkdir(parents=True)
        old = target / 'opponent.onnx'
        old.write_bytes(b'old model remains')
        with self.assertRaisesRegex(ValueError, 'destination exists'):
            self.transfer.push(LocalScpJob(), plan)
        self.assertEqual(old.read_bytes(), b'old model remains')
        self.assert_no_staging()

    def test_source_change_after_prepare_fails_before_any_remote_write(self):
        plan = self.transfer.prepare(self.body)
        (self.model / 'opponent.onnx').write_bytes(b'changed')
        with self.assertRaises(ValueError):
            self.transfer.push(LocalScpJob(), plan)
        self.assertEqual(self.actions, [])
        self.assertFalse(self.remote_root.exists())

    def test_remote_root_symlink_is_refused_before_scp(self):
        outside = self.base / 'outside-remote'
        outside.mkdir()
        self.remote_root.symlink_to(outside, target_is_directory=True)
        job = LocalScpJob()
        with self.assertRaisesRegex(ValueError, 'symlink'):
            self.transfer.push(job, self.transfer.prepare(self.body))
        self.assertEqual(job.commands, [])
        self.assertEqual(list(outside.iterdir()), [])

    def test_shape_status_and_path_contracts_are_checked_before_transfer(self):
        report = read_json(self.model / 'report.json')
        report['output']['shape'] = [1, 84, 6300]
        atomic_json(self.model / 'report.json', report)
        with self.assertRaisesRegex(ValueError, 'shape'):
            self.transfer.prepare(self.body)
        report['output']['shape'] = [1, 5, 6300]
        atomic_json(self.model / 'report.json', report)
        artifact = read_json(self.model / 'artifact.json')
        artifact['status'] = 'running'
        atomic_json(self.model / 'artifact.json', artifact)
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            self.transfer.prepare(self.body)
        for change in (dict(id='../escape'), dict(model_root='/tmp/../escape'),
                       dict(container_model_root='/path with spaces'), dict(model_root='/')):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.transfer.prepare(dict(self.body, **change))
        self.assertEqual(self.actions, [])

    def test_leftover_stage_after_rename_rolls_publication_back(self):
        plan = self.transfer.prepare(self.body)
        rmdir = Path.rmdir

        def refuse_stage(path):
            if path.name == plan['stage']:
                raise OSError('cannot remove staging')
            return rmdir(path)

        with patch('kart_studio.remote_agent.Path.rmdir', autospec=True, side_effect=refuse_stage):
            with self.assertRaisesRegex(ValueError, 'cannot remove staging'):
                self.transfer.push(LocalScpJob(), plan)
        self.assertFalse((self.remote_root / plan['destination']).exists())
        self.assert_no_staging()

    def test_missing_publication_ack_is_not_reported_as_success(self):
        plan = self.transfer.prepare(self.body)
        original = self.local_ssh

        def lost_ack(command, **kwargs):
            result = original(command, **kwargs)
            if json.loads(kwargs['input'])['action'] == 'publish-perception':
                return subprocess.CompletedProcess(command, 255, '', 'connection lost after request')
            return result

        with patch('kart_studio.transfers.subprocess.run', side_effect=lost_ack):
            with self.assertRaisesRegex(ValueError, 'SSH応答'):
                self.transfer.push(LocalScpJob(), plan)
        # Publication can have committed before a connection loss. Its verified
        # files stay intact; the caller receives an error, never a guessed success.
        self.assertTrue((self.remote_root / plan['destination'] / 'opponent.onnx').is_file())
        self.assert_no_staging()


if __name__ == '__main__':
    unittest.main()
