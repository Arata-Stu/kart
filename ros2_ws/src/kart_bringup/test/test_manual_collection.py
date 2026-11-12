"""Bringup MANUAL fixed-throttle selection. No ROS, serial or vehicle access."""
import contextlib
import importlib.util
import io
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts/lib'))


class ManualCollectionTests(unittest.TestCase):
    def cli(self, *args):
        return subprocess.run([sys.executable, str(ROOT / 'scripts/lib/bringup.py'),
                               '--no-bridge', '--dry-run', *args], capture_output=True, text=True)

    def test_collect_fixed_manual_default_and_disable(self):
        for value in ('0.1', '0', '-1'):
            with self.subTest(value=value):
                result = self.cli('--mode', 'collect', '--auto-fixed-throttle', value)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(f'auto_fixed_throttle:={float(value)}', result.stdout)
                self.assertIn('mission.launch.py', result.stdout)
                self.assertNotIn('operation_mode/request', result.stdout)
        result = self.cli('--mode', 'collect')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('auto_fixed_throttle:=', result.stdout)
        self.assertIn('AUTOスロットル: 通常AUTO', result.stdout)

    def test_invalid_and_offline_options_rejected(self):
        for value in ('nan', 'inf', '-0.1', '0.31'):
            with self.subTest(value=value):
                result = self.cli('--mode', 'collect', '--auto-fixed-throttle', value)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('auto_fixed_throttle', result.stderr)
        result = self.cli('--mode', 'eval', '--auto-fixed-throttle', '0.1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--auto-fixed-throttle', result.stderr)

    def test_r2_gated_collection_is_forwarded_and_can_be_disabled(self):
        for mode in ('collect', 'environment'):
            with self.subTest(mode=mode):
                result = self.cli('--mode', mode, '--auto-fixed-throttle', '0.10', '--auto-fixed-throttle-r2-gate')
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('auto_fixed_throttle_r2_gate:=true', result.stdout)
                self.assertIn('R2で固定駆動／惰行', result.stdout)
        result = self.cli('--mode', 'collect', '--auto-fixed-throttle', '0.10', '--no-auto-fixed-throttle-r2-gate')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('auto_fixed_throttle_r2_gate:=false', result.stdout)
        result = self.cli('--mode', 'collect', '--auto-fixed-throttle-r2-gate')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('requires auto_fixed_throttle', result.stderr)
        result = self.cli('--mode', 'collect', '--record-profile', 'vslam-map',
                          '--auto-fixed-throttle', '0.1', '--auto-fixed-throttle-r2-gate')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('standard record profile', result.stderr)
        result = self.cli('--mode', 'collect', '--infra-profile', '0',
                          '--auto-fixed-throttle', '0.1', '--auto-fixed-throttle-r2-gate')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('stereo Infra', result.stderr)
        for mode in ('drive', 'e2e', 'eval', 'e2e-latency'):
            result = self.cli('--mode', mode, '--auto-fixed-throttle-r2-gate')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('auto_fixed_throttle_r2_gate', result.stderr)

    def test_tui_prompts_and_forwards_fixed_value(self):
        spec = importlib.util.spec_from_file_location('manual_collection_cli', ROOT / 'scripts/lib/bringup.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        answers = iter(('record', 'control', 'fixed', 'start'))
        output = io.StringIO()
        with (patch.object(sys, 'argv', ['bringup.py', '--no-bridge', '--dry-run', '--run-name', 'manual_fixed']),
              patch.object(sys.stdin, 'isatty', return_value=True),
              patch.object(module, 'choose', side_effect=lambda *a, **k: next(answers)),
              patch('builtins.input', return_value='0.12'),
              contextlib.redirect_stdout(output)):
            module.main()
        self.assertIn('auto_fixed_throttle:=0.12', output.getvalue())
        self.assertIn('△で通常MANUALへ直接切替', output.getvalue())
        self.assertIn('run_name:=manual_fixed', output.getvalue())

    def test_tui_identification_then_fixed_selection_clears_gate(self):
        spec = importlib.util.spec_from_file_location('r2_collection_cli', ROOT / 'scripts/lib/bringup.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        from bringup_tui import Editor
        args = module.argument_parser().parse_args(['--mode', 'collect'])
        editor = Editor(args, ROOT / 'ros2_ws/src/kart_bringup/config', ROOT,
                        lambda *a, **k: 'identification')
        with patch('builtins.input', return_value='0.125'):
            editor.edit_control()
        self.assertTrue(args.auto_fixed_throttle_r2_gate)
        self.assertEqual(args.auto_fixed_throttle, 0.125)
        self.assertIn('車両同定', editor.control_summary())
        editor.choose = lambda *a, **k: 'fixed'
        with patch('builtins.input', return_value=''):
            editor.edit_control()
        self.assertFalse(args.auto_fixed_throttle_r2_gate)
        self.assertEqual(args.auto_fixed_throttle, 0.125)
        args.mode = 'drive'
        args.auto_fixed_throttle_r2_gate = True
        def choose_drive(title, values, label=str, default=0):
            self.assertNotIn('identification', values)
            self.assertEqual(values[default], 'fixed')
            return 'fixed'
        editor.choose = choose_drive
        with patch('builtins.input', return_value=''):
            editor.edit_control()
        self.assertFalse(args.auto_fixed_throttle_r2_gate)
