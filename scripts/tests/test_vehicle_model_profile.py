"""Vehicle-model profile user flows; no ROS, hardware or saved settings."""
from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import importlib.util
import io
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/lib'))
spec = importlib.util.spec_from_file_location('vehicle_model_entry', ROOT / 'scripts/lib/bringup.py')
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)
from bringup_tui import Editor

CONFIG = ROOT / 'ros2_ws/src/kart_bringup/config'
REVIEW = '設定一覧（変更する項目を選択）'


def scripted(steps):
    remaining = iter(steps)

    def choose(title, values, label=str, default=0):
        expected, selected = next(remaining)
        if title != expected or selected not in values:
            raise AssertionError((title, expected, selected, list(values)))
        for candidate in values:
            if not isinstance(label(candidate), str):
                raise AssertionError('Menu labels must be text')
        return selected

    return choose


class VehicleModelProfileTests(unittest.TestCase):
    def cli(self, *args):
        return subprocess.run([sys.executable, str(ROOT / 'scripts/lib/bringup.py'),
            '--mode', 'collect', '--no-bridge', '--dry-run',
            '--record-profile', 'vehicle-model', *args], capture_output=True, text=True)

    def editor(self, *options, choices=()):
        args = entry.argument_parser().parse_args(['--mode', 'collect', *options])
        return Editor(args, CONFIG, ROOT, scripted(choices))

    def test_cli_profile_alone_resolves_control_sensor_bag_and_vo_defaults(self):
        result = self.cli()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('record_profile:=vehicle-model', result.stdout)
        self.assertIn('auto_fixed_throttle:=0.1', result.stdout)
        self.assertIn('auto_fixed_throttle_r2_gate:=true', result.stdout)
        self.assertIn('realsense_vehicle_model.yaml', result.stdout)
        self.assertIn('bag_manager_vehicle_model.yaml', result.stdout)
        self.assertIn('地図なしVO', result.stdout)
        self.assertIn('run_name:=vehicle_model_', result.stdout)
        self.assertNotIn('ros2 topic pub', result.stdout)

    def test_cli_keeps_explicit_fixed_value_and_record_name(self):
        result = self.cli('--auto-fixed-throttle', '0.125', '--run-name', 'repeat_003')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('auto_fixed_throttle:=0.125', result.stdout)
        self.assertIn('run_name:=repeat_003', result.stdout)

    def test_cli_rejects_incompatible_control_and_sensor_overrides(self):
        invalid = (
            ('--auto-fixed-throttle', '-1'),
            ('--no-auto-fixed-throttle-r2-gate',),
            ('--infra-profile', '0'),
            ('--imu-profile', 'off'),
            ('--rgb-profile', '640x480x30'),
            ('--depth-profile', '640x480x90'),
            ('--evs',),
        )
        for options in invalid:
            with self.subTest(options=options):
                result = self.cli(*options)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn('vehicle-model', result.stderr)
                self.assertNotIn('ros2 launch', result.stdout)

    def test_tui_profile_selection_replaces_conflicting_choices_and_restores_them(self):
        editor = self.editor('--rgb-fps', '30', '--infra-profile', '0',
            '--depth-profile', '640x480x30', '--imu-profile', 'off',
            '--evs', '--evs-backend', 'cuda_async', '--evs-aggregation', 'events',
            '--evs-image', '--perception', '/old/engine.plan',
            '--run-name', 'keep', '--joy-max-throttle', '0.2',
            choices=[('記録profile', 'vehicle-model'), ('記録profile', 'standard')])
        before = deepcopy(vars(editor.args))
        with patch('builtins.input', return_value='0.125'):
            editor.edit('record_profile')
        self.assertEqual(editor.args.record_profile, 'vehicle-model')
        self.assertEqual(editor.args.auto_fixed_throttle, .125)
        self.assertIs(editor.args.auto_fixed_throttle_r2_gate, True)
        self.assertIsNone(editor.args.rgb_fps)
        self.assertIsNone(editor.args.infra_fps)
        self.assertEqual(editor.args.rgb_profile, '')
        self.assertEqual(editor.args.infra_profile, '')
        self.assertEqual(editor.args.depth_profile, '')
        self.assertEqual(editor.args.imu_profile, '')
        self.assertFalse(editor.args.evs)
        self.assertTrue(editor.args.no_evs)
        self.assertEqual(editor.args.perception, 'off')
        camera = editor.sensors()
        self.assertFalse(camera['enable_color'] or camera['enable_depth'])
        self.assertTrue(camera['enable_infra1'] and camera['enable_infra2'])
        self.assertTrue(camera['enable_accel'] and camera['enable_gyro'])
        self.assertEqual(camera['depth_module.infra_profile'], '640x480x90')
        self.assertEqual((camera['accel_fps'], camera['gyro_fps']), (200, 400))
        self.assertEqual(editor.args.run_name, 'keep')
        self.assertEqual(editor.args.joy_max_throttle, .2)
        # User edits to metadata and limits are independent of profile rollback.
        editor.args.run_name = 'updated'
        editor.args.joy_max_throttle = .15
        editor.edit('record_profile')
        expected = dict(before, record_profile='standard', run_name='updated', joy_max_throttle=.15)
        self.assertEqual(vars(editor.args), expected)
        self.assertIsNone(editor.profile_restore)

    def test_tui_model_control_edit_only_prompts_for_fixed_value(self):
        editor = self.editor(choices=[('記録profile', 'vehicle-model')])
        with patch('builtins.input', return_value=''):
            editor.edit('record_profile')
        self.assertEqual(editor.args.auto_fixed_throttle, .1)
        with patch('builtins.input', return_value='0.15'):
            # The scripted menu is exhausted: opening a gate/normal menu would fail.
            editor.edit('control')
        self.assertEqual(editor.args.auto_fixed_throttle, .15)
        self.assertIs(editor.args.auto_fixed_throttle_r2_gate, True)
        self.assertIn('車両同定', editor.control_summary())

    def test_invalid_profile_input_rolls_back_before_cancel(self):
        for value in ('nan', 'inf', '-0.01', '0.201', 'bad'):
            with self.subTest(value=value):
                editor = self.editor('--joy-max-throttle', '0.2', '--run-name', 'keep',
                    choices=[(REVIEW, 'record_profile'), ('記録profile', 'vehicle-model'),
                             (REVIEW, 'cancel')])
                before = deepcopy(vars(editor.args))
                output = io.StringIO()
                with patch('builtins.input', return_value=value), redirect_stdout(output):
                    self.assertFalse(editor.review())
                self.assertIn('変更できません', output.getvalue())
                self.assertEqual(vars(editor.args), before)
                self.assertIsNone(editor.profile_restore)

    def test_leaving_model_profile_does_not_leak_gate_into_minimal_profile(self):
        for original in ((), ('--auto-fixed-throttle', '0.125', '--auto-fixed-throttle-r2-gate')):
            with self.subTest(original=original):
                editor = self.editor(*original,
                    choices=[('記録profile', 'vehicle-model'), ('記録profile', 'vslam-map')])
                with patch('builtins.input', return_value=''):
                    editor.edit('record_profile')
                editor.edit('record_profile')
                self.assertEqual(editor.args.record_profile, 'vslam-map')
                self.assertEqual(editor.args.auto_fixed_throttle, -1)
                self.assertIs(editor.args.auto_fixed_throttle_r2_gate, False)
                self.assertFalse(editor.sensors()['enable_gyro'])

    def test_end_to_end_tui_profile_dry_run_never_executes_ros(self):
        output, errors = io.StringIO(), io.StringIO()
        steps = [('用途', 'record'), (REVIEW, 'record_profile'),
                 ('記録profile', 'vehicle-model'), (REVIEW, 'start')]
        with (patch.object(sys, 'argv', ['bringup.py', '--dry-run', '--no-bridge']),
              patch.object(sys.stdin, 'isatty', return_value=True),
              patch.object(entry, 'choose', side_effect=scripted(steps)),
              patch('builtins.input', return_value=''),
              patch.object(entry, 'run_command', side_effect=AssertionError('dry-run executed ROS')),
              patch.object(entry.os, 'execvp', side_effect=AssertionError('dry-run executed process')),
              redirect_stdout(output), redirect_stderr(errors)):
            entry.main()
        self.assertFalse(errors.getvalue(), errors.getvalue())
        self.assertIn('record_profile:=vehicle-model', output.getvalue())
        self.assertIn('auto_fixed_throttle_r2_gate:=true', output.getvalue())
        self.assertIn('run_name:=vehicle_model_', output.getvalue())
        self.assertNotIn('operation_mode/request', output.getvalue())


if __name__ == '__main__':
    unittest.main()
