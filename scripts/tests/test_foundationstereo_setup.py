"""Portable setup and early mapping preflight tests; no GPU/model downloads."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/kart_mapping'))
from kart_mapping.build_map import run
from kart_mapping.dense_outputs import require_engine


class FoundationStereoSetupTests(unittest.TestCase):
    def test_missing_engine_stops_before_bag_or_subprocess(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, ISAAC_ROS_WS=tmp), patch('subprocess.run') as process:
            with self.assertRaisesRegex(ValueError, 'scripts/setup/foundationstereo.sh'):
                run({'workflow': {'dense_reconstruction': True}})
            process.assert_not_called()

    def test_empty_missing_and_present_engines(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, ISAAC_ROS_WS=tmp):
            for resolution, size in [('low_res', '320x736'), ('high_res', '576x960')]:
                engine = Path(tmp) / 'isaac_ros_assets/models/foundationstereo/deployable_v2.0' / f'foundationstereo_{size}.engine'
                engine.parent.mkdir(parents=True, exist_ok=True)
                with self.assertRaises(ValueError):
                    require_engine(resolution)
                engine.touch()
                with self.assertRaises(ValueError):
                    require_engine(resolution)
                engine.write_bytes(b'fake engine')
                self.assertEqual(require_engine(resolution), engine)
                result = subprocess.run(['bash', str(ROOT / 'scripts/setup/foundationstereo.sh'), '--model-res', resolution], env=os.environ.copy(), capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('スキップ', result.stdout)

    def test_invalid_resolution(self):
        result = subprocess.run(['bash', str(ROOT / 'scripts/setup/foundationstereo.sh'), '--model-res', 'invalid'], capture_output=True)
        self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main()
