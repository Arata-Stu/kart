"""CLI forwarding and mode validation; never starts ROS or recording."""
import argparse
import contextlib
import importlib.util
import io
import shlex
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('bringup_propagation', ROOT / 'scripts/lib/bringup.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PropagationTests(unittest.TestCase):
    def run_cli(self, *options):
        output = io.StringIO()
        with patch.object(sys, 'argv', ['bringup.py', '--dry-run', '--no-bridge', *options]), contextlib.redirect_stdout(output):
            module.main()
        return output.getvalue()

    def test_e2e_model_throttle_forwarding_and_conflicts(self):
        for modality in ('rgb', 'evs'):
            for selection in ('fixed', 'model'):
                with (self.subTest(modality=modality, selection=selection),
                      patch('kart_e2e.contract.routing_settings', return_value=None),
                      patch('kart_e2e.contract.input_contract', return_value=(modality, 'input', [1, 20, 120, 160])),
                      patch('kart_e2e.contract.model_contract', return_value=({}, 'steer_throttle')),
                      patch('kart_e2e.contract.runtime_settings', return_value={'fixed_throttle': .1, 'max_throttle': .2}),
                      patch('kart_e2e.contract.validate_runtime'),
                      patch('kart_bringup.e2e.validate_evs_tensor'),
                      patch('kart_bringup.e2e.select_evs_tensor', return_value='')):
                    output = self.run_cli('--mode', 'e2e', '--e2e-model-dir', '/model',
                                          '--evs', '--e2e-throttle-mode', selection)
                    self.assertIn('e2e_throttle_mode:=' + selection, output)
                    self.assertIn('モデルの可変スロットル' if selection == 'model' else '固定スロットル 0.1', output)
        with self.assertRaises(ValueError):
            self.run_cli('--mode', 'collect', '--e2e-throttle-mode', 'model')

    def test_e2e_tui_model_mode_does_not_prompt_fixed_value(self):
        prompts = []
        edits = iter(["control", "start"])
        def choose(title, values, label=str, default=0):
            prompts.append(title)
            if title.startswith('設定一覧'):
                return next(edits)
            if title == '用途':
                return 'e2e'
            if title == 'E2Eスロットル':
                self.assertIn('model', values)
                self.assertIn('可変', label('model'))
                return 'model'
            return values[default]
        with (patch.object(sys.stdin, 'isatty', return_value=True),
              patch.object(module, 'choose', side_effect=choose),
              patch('builtins.input', side_effect=AssertionError('unexpected fixed throttle prompt')),
              patch('kart_e2e.contract.routing_settings', return_value=None),
              patch('kart_e2e.contract.input_contract', return_value=('rgb', 'image', [1, 3, 120, 212])),
              patch('kart_e2e.contract.model_contract', return_value=({}, 'steer_throttle')),
              patch('kart_e2e.contract.runtime_settings', return_value={'fixed_throttle': .1, 'max_throttle': .2})):
            output = self.run_cli('--e2e-model-dir', '/model', '--no-evs', '--recovery', 'off',
                                  '--rgb-profile', '640x480x30', '--infra-profile', '0',
                                  '--imu-profile', 'off', '--run-name', 'test')
        self.assertIn('E2Eスロットル', prompts)
        self.assertIn('e2e_throttle_mode:=model', output)

    def test_collect_all_sensor_overrides_reach_ros_argv(self):
        output = self.run_cli('--mode', 'collect', '--rgb-profile', '0', '--infra-profile', '0',
                              '--imu-profile', 'off', '--evs-backend', 'cuda_async',
                              '--evs-aggregation', 'evt3_runs_grouped_gpu', '--no-evs-image',
                              '--evs-serial', '000123', '--evs-bias-file', '@default',
                              '--record-dir', '/tmp/records with space', '--run-name', '0000',
                              '--auto-fixed-throttle', '0', '--foxglove', '--device', '/dev/ttyACM9')
        command = next(shlex.split(line) for line in output.splitlines() if line.startswith('ros2 launch '))
        values = dict(token.split(':=', 1) for token in command[4:])
        expected = dict(mode='collect', rgb_profile='0', infra_profile='0', imu_profile='off',
                        evs_backend='cuda_async', evs_aggregation='evt3_runs_grouped_gpu',
                        evs_image='false', evs_serial='000123', evs_bias_file='@default',
                        enable_evs='true', enable_foxglove='true', enable_bridge='false',
                        record_dir='/tmp/records with space', run_name='0000',
                        auto_fixed_throttle='0.0', device='/dev/ttyACM9')
        self.assertEqual(values, expected)

    def test_unsupported_options_fail_before_assets_or_ros_are_touched(self):
        cases = [
            ('collect', '--bag', '/tmp/bag'), ('collect', '--rate', '.5'),
            ('collect', '--map-file', '/tmp/map.json'), ('collect', '--model-dir', '/tmp/model'),
            ('collect', '--e2e-model-dir', '/tmp/model'), ('collect', '--eval-localization', 'vgl'),
            ('eval', '--rgb-profile', '0'), ('eval', '--infra-fps', '0'),
            ('eval', '--device', '/dev/ttyACM9'), ('eval', '--run-name', 'ignored'),
            ('eval', '--foxglove'), ('eval', '--model-dir', '/tmp/model'),
            ('e2e-latency', '--rgb-profile', '0'), ('e2e-latency', '--tracking-mode', '0'),
            ('e2e-latency', '--multicam-mode', '0'), ('e2e-latency', '--no-tuning'),
        ]
        for mode, *options in cases:
            with self.subTest(mode=mode, options=options), self.assertRaises(ValueError):
                self.run_cli('--mode', mode, *options)

    def test_848_by_100_infra_profile_reaches_mission_without_rgb(self):
        output = self.run_cli('--mode', 'collect', '--rgb-profile', '0', '--infra-profile', '848x100x100')
        self.assertIn('infra_profile:=848x100x100', output)
        self.assertIn('rgb_profile:=0', output)

    def test_every_registered_option_has_an_audit_classification(self):
        parsers = []
        original = argparse.ArgumentParser.parse_args
        def parse(parser, *args, **kwargs):
            parsers.append(parser)
            return original(parser, *args, **kwargs)
        with patch.object(argparse.ArgumentParser, 'parse_args', parse):
            self.run_cli('--mode', 'collect')
        # Search roots and dry_run are local selector behavior, not ROS parameters.
        selector = {'help', 'map_root', 'model_root', 'bias_root', 'dry_run'}
        forwarded = {'mode', 'rgb_fps', 'infra_fps', 'rgb_profile', 'infra_profile', 'imu_profile',
                     'device', 'map_file', 'map_dir', 'model_dir', 'lane_id', 'line_type',
                     'tuning', 'record_dir', 'evs', 'no_evs', 'evs_backend', 'evs_aggregation',
                     'evs_image', 'evs_serial', 'evs_tensor_config', 'evs_bias_file', 'foxglove',
                     'no_bridge', 'joy_max_throttle', 'auto_fixed_throttle', 'e2e_fixed_throttle', 'e2e_throttle_mode', 'recovery',
                     'run_name', 'e2e_model_dir', 'eval_localization', 'multicam_mode', 'parking_site', 'parking_slot',
                     'tracking_mode', 'base_frame', 'imu_frame', 'latency_output', 'bag', 'rate'}
        actual = {action.dest for action in parsers[0]._actions}
        self.assertEqual(actual, selector | forwarded)


if __name__ == '__main__':
    unittest.main()
