"""Read-only paths must not switch the host's network; use a fake nmcli."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'hotspot.sh'


class HotspotTest(unittest.TestCase):
    def test_help_and_invalid_action(self):
        for arg, code in [('--help', 0), ('invalid', 2)]:
            run = subprocess.run(['bash', str(SCRIPT), arg], capture_output=True)
            self.assertEqual(run.returncode, code)

    def test_status_read_only_and_missing_interface(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            mock = root / 'nmcli'
            mock.write_text('#!/bin/bash\nprintf "%s\\n" "$*" >> "$CALLS"\nexit "${MOCK_EXIT:-0}"\n')
            mock.chmod(0o755)
            log = root / 'calls'
            env = dict(os.environ, PATH=folder+':'+os.environ['PATH'], CALLS=str(log))
            run = subprocess.run(['bash', str(SCRIPT), 'status'], env=env, capture_output=True)
            self.assertEqual(run.returncode, 0)
            calls = log.read_text().splitlines()
            self.assertEqual(len(calls), 3)
            self.assertTrue(all('device show wlP1p1s0' in c or 'connection show uuid d2c82d15-9fd1-4174-9f30-aac3c61e76c2' in c for c in calls))
            env['MOCK_EXIT'] = '10'
            run = subprocess.run(['bash', str(SCRIPT), 'status'], env=env, capture_output=True)
            self.assertNotEqual(run.returncode, 0)


if __name__ == '__main__':
    unittest.main()
