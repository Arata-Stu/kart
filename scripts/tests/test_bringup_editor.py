"""User flows for the editable TUI; no hardware, ROS or saved settings."""
from contextlib import redirect_stdout, redirect_stderr
import importlib.util
import io
import json
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

spec = importlib.util.spec_from_file_location('editor_entry', Path(__file__).parents[1] / 'lib/bringup.py')
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)
from bringup_tui import Editor

CONFIG = entry.ROOT / 'ros2_ws/src/kart_bringup/config'
REVIEW = '設定一覧（変更する項目を選択）'


def scripted(steps):
    steps = iter(steps)
    def choose(title, values, label=str, default=0):
        expected, value = next(steps)
        assert title == expected, (title, expected)
        assert value in values, (title, value, values)
        # Render every visible row, as fzf does.
        for candidate in values:
            assert isinstance(label(candidate), str)
        return value
    return choose


def run_tui(steps, *options, inputs=()):
    out, err = io.StringIO(), io.StringIO()
    with (patch.object(sys, 'argv', ['bringup.py', '--dry-run', '--no-bridge', *options]),
          patch.object(sys.stdin, 'isatty', return_value=True),
          patch.object(entry, 'choose', side_effect=scripted(steps)),
          patch('builtins.input', side_effect=inputs),
          patch.object(entry.os, 'execvp', side_effect=AssertionError('dry run must not execute')),
          redirect_stdout(out), redirect_stderr(err)):
        entry.main()
    return out.getvalue(), err.getvalue()


def test_record_defaults_only_ask_purpose_and_review():
    out, err = run_tui([('用途', 'record'), ('記録内容', 'collect'), (REVIEW, 'start')])
    assert not err
    assert 'mode:=collect' in out
    assert 'run_name:=run' in out
    assert 'rgb_profile:=' not in out  # YAML preserved, not copied into overrides.
    assert 'auto_fixed_throttle:=' not in out


def test_cancel_does_not_build_or_launch():
    with patch.object(entry, 'build_command', side_effect=AssertionError('cancel must not validate/launch')):
        out, err = run_tui([('用途', 'record'), ('記録内容', 'collect'), (REVIEW, 'cancel')])
    assert 'ros2 launch' not in out


def test_repeated_edits_keep_other_fields_and_only_last_camera_choice():
    steps = [('用途', 'record'), ('記録内容', 'collect'), (REVIEW, 'rgb'),
             ('RGB 解像度 × FPS（USB3 D455候補）', '640x480x30'), (REVIEW, 'imu'),
             ('IMU 取得レート', 'off'), (REVIEW, 'rgb'),
             ('RGB 解像度 × FPS（USB3 D455候補）', '640x360x60'), (REVIEW, 'start')]
    out, err = run_tui(steps, '--infra-profile', '0', '--run-name', 'keep')
    command = next(line for line in out.splitlines() if line.startswith('ros2 launch'))
    assert 'rgb_profile:=640x360x60' in command
    assert '640x480x30' not in command
    assert 'infra_profile:=0' in command and 'imu_profile:=off' in command and 'run_name:=keep' in command
    assert not err


def test_validation_error_returns_to_review_and_can_be_fixed():
    steps = [('用途', 'record'), ('記録内容', 'collect'), (REVIEW, 'start'), (REVIEW, 'record'),
             ('記録設定', 'name'), ('記録設定', 'back'), (REVIEW, 'start')]
    out, err = run_tui(steps, '--run-name', '../invalid', inputs=['fixed'])
    assert '設定エラー' in err
    assert 'run_name:=fixed' in out
    assert out.count('ros2 launch') == 1


def test_conflicting_camera_options_can_be_repaired():
    out, err = run_tui([('用途', 'record'), ('記録内容', 'collect'), (REVIEW, 'rgb'),
                       ('RGB 解像度 × FPS（USB3 D455候補）', '640x480x30'), (REVIEW, 'start')],
                      '--rgb-profile', '640x480x60', '--rgb-fps', '30')
    assert 'rgb_profile:=640x480x30' in out
    assert 'rgb_fps:=' not in out
    assert not err


def test_evs_detail_only_changes_requested_setting():
    steps = [('用途', 'record'), ('記録内容', 'collect'), (REVIEW, 'evs'),
             ('EVS 詳細設定', 'enabled'), ('EVS', True), ('EVS 詳細設定', 'image'),
             ('EVS event image可視化', False), ('EVS 詳細設定', 'back'), (REVIEW, 'start')]
    out, err = run_tui(steps)
    assert 'enable_evs:=true' in out and 'evs_image:=false' in out
    assert 'evs_backend:=' not in out and 'evs_aggregation:=' not in out
    assert not err


def test_disabling_evs_clears_inactive_overrides():
    steps = [('用途', 'record'), ('記録内容', 'collect'), (REVIEW, 'evs'),
             ('EVS 詳細設定', 'enabled'), ('EVS', False), ('EVS 詳細設定', 'back'), (REVIEW, 'start')]
    out, err = run_tui(steps, '--evs-aggregation', 'events', '--evs-image')
    command = next(line for line in out.splitlines() if line.startswith('ros2 launch'))
    assert 'enable_evs:=false' in command
    assert 'evs_aggregation:=' not in command and 'evs_image:=' not in command
    assert not err


def test_evs_model_selected_before_settings_infers_input_without_prompting_sensors():
    steps = [('用途', 'e2e'), ('E2E ONNXモデル（metadata付き）', Path('/model')), (REVIEW, 'start')]
    with (patch('kart_bringup.mission.discover', return_value=[Path('/model')]),
          patch('kart_e2e.contract.input_contract', return_value=('evs', 'input', [1, 20, 120, 160])),
          patch('kart_e2e.contract.routing_settings', return_value=None),
          patch('kart_e2e.contract.model_contract', return_value=({}, 'steer')),
          patch('kart_e2e.contract.runtime_settings', return_value={'fixed_throttle': .1, 'max_throttle': .2}),
          patch('kart_bringup.e2e.select_evs_tensor', return_value='/model/openeb_tensor.yaml'),
          patch('kart_bringup.e2e.validate_evs_tensor'),
          patch('kart_bringup.evs.pipeline_configuration', return_value=([], {}))):
        out, err = run_tui(steps, '--recovery', 'off')
    assert 'enable_evs:=true' in out and 'evs_tensor_config:=/model/openeb_tensor.yaml' in out
    assert not err


def make_editor(*options, choose=None):
    args = entry.argument_parser().parse_args(['--mode', 'drive', *options])
    return Editor(args, CONFIG, entry.ROOT, choose or scripted([]))


def test_bundle_inference_keeps_fps_and_does_not_override_explicit_camera(tmp_path):
    bundle = tmp_path / 'bundle'
    bundle.mkdir()
    (bundle / 'vgl_profile.json').write_text(json.dumps({'input_shape': [1, 3, 480, 640]}))
    with patch('kart_bringup.mission.hdmap_choices', return_value=[('lane', 'centerline')]):
        editor = make_editor('--map-file', '/map.json', '--map-dir', str(bundle), '--model-dir', '/model')
        fps = editor.sensors()['depth_module.infra_profile'].split('x')[-1]
        editor.maps()
        assert editor.args.infra_profile == '640x480x' + fps
        editor.reset_inferred()
        assert editor.args.infra_profile == ''
        editor = make_editor('--map-file', '/map.json', '--map-dir', str(bundle), '--model-dir', '/model',
                             '--infra-profile', '424x240x60')
        editor.maps()
        assert editor.args.infra_profile == '424x240x60'
        assert not editor.inferred


def test_bad_edit_rolls_back_and_cancel_keeps_previous_record_name():
    editor = make_editor('--mode', 'collect', '--run-name', 'keep',
                         choose=scripted([(REVIEW, 'control'), ('AUTOスロットル', 'fixed'), (REVIEW, 'cancel')]))
    with patch('builtins.input', return_value='not-a-number'):
        assert not editor.review()
    assert editor.args.auto_fixed_throttle is None
    assert editor.args.run_name == 'keep'


def test_launch_only_after_successful_validation():
    steps = [('用途', 'record'), ('記録内容', 'collect'), (REVIEW, 'start'),
             (REVIEW, 'record'), ('記録設定', 'name'), ('記録設定', 'back'), (REVIEW, 'start')]
    with (patch.object(sys, 'argv', ['bringup.py', '--no-bridge', '--run-name', '../bad']),
          patch.object(sys.stdin, 'isatty', return_value=True),
          patch.object(entry, 'choose', side_effect=scripted(steps)),
          patch('builtins.input', return_value='fixed'),
          patch.object(entry.shutil, 'which', return_value='/usr/bin/ros2'),
          patch.object(entry.os, 'execvp') as execute,
          redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO())):
        entry.main()
    execute.assert_called_once()
    assert execute.call_args.args[0] == 'ros2'
    assert 'run_name:=fixed' in execute.call_args.args[1]


def test_explicit_no_evs_survives_model_input_inference():
    editor = make_editor('--mode', 'e2e', '--e2e-model-dir', '/model', '--no-evs')
    with (patch('kart_e2e.contract.input_contract', return_value=('evs', 'input', [1, 20, 120, 160])),
          patch('kart_e2e.contract.routing_settings', return_value=None),
          patch('kart_bringup.e2e.select_evs_tensor', return_value='/model/tensor.yaml')):
        editor.assets()
    assert editor.args.no_evs and not editor.args.evs


def test_changed_bundle_updates_only_inferred_camera_values(tmp_path):
    bundles = []
    for width, height in [(640, 480), (424, 240)]:
        bundle = tmp_path / str(width)
        bundle.mkdir()
        (bundle / 'vgl_profile.json').write_text(json.dumps({'input_shape': [1, 3, height, width]}))
        bundles.append(bundle)
    editor = make_editor('--map-file', '/map.json', '--map-dir', str(bundles[0]), '--model-dir', '/model',
                         choose=scripted([('対応するVGL bundle', bundles[1])]))
    with (patch('kart_bringup.mission.hdmap_choices', return_value=[('lane', 'centerline')]),
          patch('kart_bringup.mission.discover', return_value=bundles),
          patch('bringup_tui.paired_maps', return_value=bundles),
          patch('bringup_tui.select_runtime_model', return_value='/model')):
        editor.maps()
        assert editor.args.infra_profile.startswith('640x480x')
        editor.edit('bundle')
        assert editor.args.infra_profile.startswith('424x240x')
        assert editor.args.map_dir == str(bundles[1])


def test_evaluation_can_switch_without_retaining_incompatible_options():
    editor = make_editor('--mode', 'eval', '--eval-localization', 'vgl', '--map-file', '/hdmap',
                         '--map-dir', '/bundle', '--model-dir', '/model', '--bag', '/bag',
                         choose=scripted([('オフライン解析', 'fresh'), ('追跡方式', '1')]))
    editor.edit('evaluation')
    assert editor.args.eval_localization == 'fresh' and editor.args.tracking_mode == '1'
    assert editor.args.bag == '/bag'
    assert not editor.args.map_file and not editor.args.map_dir and not editor.args.model_dir
    rows = editor.rows()
    assert 'assets' not in rows and 'rgb' not in rows and 'control' not in rows
