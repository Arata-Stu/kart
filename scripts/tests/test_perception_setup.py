"""Notebook-only Docker setup and checked model publication, without downloads/GPU."""
import hashlib
import importlib.util
import io
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('perception_prepare', ROOT / 'docker/python/perception/prepare.py')
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


class PerceptionSetupTests(unittest.TestCase):
    def test_docker_executes_setup_only_for_target_amd64(self):
        dockerfile = (ROOT / 'docker/Dockerfile.kart').read_text()
        block = dockerfile.split('# Notebook object-detection training', 1)[1]
        commands = block.split('    set -eux;', 1)[1].replace('\\\n', ' ')
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            calls = root / 'calls'
            for name in ('uv', 'python', 'chmod', 'dpkg'):
                script = root / name
                script.write_text('#!/bin/sh\n' + ('echo "$KART_TEST_ARCH"\n' if name == 'dpkg' else
                                  'printf "%s\\n" "' + name + ' $*" >> "$KART_TEST_CALLS"\n'))
                script.chmod(0o755)
            commands = commands.replace('/opt/perception/bin/python', str(root / 'python'))
            for architecture in ('amd64', 'arm64'):
                with self.subTest(architecture=architecture):
                    calls.write_text('')
                    result = subprocess.run(['bash', '-euc', commands], capture_output=True, text=True,
                        env=dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'],
                                 KART_TEST_ARCH=architecture, KART_TEST_CALLS=str(calls)))
                    self.assertEqual(result.returncode, 0, result.stderr)
                    if architecture == 'amd64':
                        self.assertIn('uv sync --project /opt/kart/python/perception', calls.read_text())
                        self.assertIn('--python /usr/bin/python3 --locked --no-dev', calls.read_text())
                        self.assertIn('prepare.py /opt/kart/models/yolov8s.pt', calls.read_text())
                    else:
                        self.assertEqual(calls.read_text(), '', 'arm64 cannot install dependencies or download weights')

    def test_checked_download_and_existing_weight_preservation(self):
        payload = b'synthetic official weight fixture'
        with tempfile.TemporaryDirectory() as tmp, patch.object(prepare, 'WEIGHTS_SHA256', hashlib.sha256(payload).hexdigest()):
            weights = Path(tmp) / 'models/yolov8s.pt'
            with patch.object(prepare.urllib.request, 'urlopen', return_value=io.BytesIO(payload)) as download:
                prepare.ensure_weights(weights)
                download.assert_called_once_with(prepare.WEIGHTS_URL, timeout=60)
            self.assertEqual(weights.read_bytes(), payload)
            with patch.object(prepare.urllib.request, 'urlopen') as download:
                prepare.ensure_weights(weights)
                download.assert_not_called()
                weights.write_bytes(b'custom weights must never be replaced')
                with self.assertRaisesRegex(ValueError, '既存の重みは変更しません'):
                    prepare.ensure_weights(weights)
                self.assertEqual(weights.read_bytes(), b'custom weights must never be replaced')
                download.assert_not_called()

    def test_bad_download_is_never_published(self):
        with tempfile.TemporaryDirectory() as tmp:
            weights = Path(tmp) / 'models/yolov8s.pt'
            with patch.object(prepare.urllib.request, 'urlopen', return_value=io.BytesIO(b'wrong payload')):
                with self.assertRaisesRegex(ValueError, 'SHA-256'):
                    prepare.ensure_weights(weights)
            self.assertFalse(weights.exists())
            self.assertEqual(list(weights.parent.iterdir()), [])

    def test_setup_rejects_arguments_before_installation(self):
        result = subprocess.run(['bash', str(ROOT / 'scripts/perception_setup.sh'), '--inject'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('Usage:', result.stderr)


if __name__ == '__main__':
    unittest.main()
