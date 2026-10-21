from pathlib import Path
from unittest.mock import patch
import sys
import types
import yaml
from kart_bringup.recovery import configuration, route

ROOT = Path(__file__).resolve().parents[1]


def test_disabled_route_does_not_import_ros_launch():
    assert configuration(ROOT / 'config/recovery')[0] is False
    assert route(ROOT / 'config/recovery') == ([], [])


def test_enabled_route_uses_single_gate_and_preserves_yaml(tmp_path):
    (tmp_path / 'launch.yaml').write_text((ROOT / 'config/recovery/launch.yaml').read_text().replace('enabled: false', 'enabled: true'))
    (tmp_path / 'recovery.yaml').write_text((ROOT / 'config/recovery/recovery.yaml').read_text())
    class Action:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
    modules = {'launch': types.ModuleType('launch'), 'launch.actions': types.SimpleNamespace(LogInfo=Action),
               'launch_ros': types.ModuleType('launch_ros'), 'launch_ros.actions': types.SimpleNamespace(Node=Action)}
    with patch.dict(sys.modules, modules):
        remaps, actions = route(tmp_path, 'true')
    assert remaps == [('auto/control_cmd', '/recovery/input_cmd')]
    assert actions[1].kwargs['package'] == 'kart_recovery'
    assert actions[1].kwargs['parameters'] == [str(tmp_path / 'recovery.yaml'), {'use_sim_time': True}]


def test_all_defaults_and_recording_routes_match():
    sys.path.insert(0, str(ROOT.parent / 'kart_recovery'))
    from dataclasses import asdict
    from kart_recovery.core import Settings
    import ast
    source = ast.parse((ROOT.parent / 'kart_recovery/kart_recovery/node.py').read_text())
    defaults = next(ast.literal_eval(n.value) for n in source.body if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == 'NODE_DEFAULTS' for t in n.targets))
    params = configuration(ROOT / 'config/recovery')[1]
    assert params == {**defaults, **asdict(Settings()), 'use_sim_time': False}
    assert yaml.safe_load((ROOT.parent / 'kart_recovery/config/recovery.yaml').read_text())['/**/recovery']['ros__parameters'] == params
    for name in ('bag_manager.yaml', 'bag_manager_openeb.yaml'):
        topics = yaml.safe_load((ROOT / 'config/recording' / name).read_text())['/**/kart_bag_manager']['ros__parameters']['topics']
        assert all(t in topics for t in ('/recovery/input_cmd', '/recovery/active', '/recovery/status'))


def test_explicit_selection_enables_and_changes_only_profile():
    enabled, original = configuration(ROOT / 'config/recovery')
    assert not enabled
    enabled, selected = configuration(ROOT / 'config/recovery', 'rgb_imu')
    assert enabled and selected['sensor_profile'] == 'rgb_imu'
    assert selected['image_topic'] == '/realsense/color/image_raw'
    assert {k: v for k, v in selected.items() if k not in ('sensor_profile', 'image_topic')} == {
        k: v for k, v in original.items() if k not in ('sensor_profile', 'image_topic')}
    assert not configuration(ROOT / 'config/recovery', 'off')[0]


def test_sensor_and_fixed_auto_mismatches_rejected():
    import pytest
    from kart_bringup.recovery import validate_inputs
    sensors = dict(enable_color=True, enable_infra1=True, enable_infra2=True, enable_gyro=True)
    for selection, evs, changed, fixed in [('rgb_imu', False, {'enable_color': False}, -1),
                                          ('infra_imu', False, {'enable_infra1': False}, -1),
                                          ('infra_imu', False, {'enable_gyro': False}, -1),
                                          ('evs_assisted', False, {}, -1),
                                          ('infra_imu', False, {}, .1)]:
        _, params = configuration(ROOT / 'config/recovery', selection)
        with pytest.raises(ValueError):
            validate_inputs(params, dict(sensors, **changed), evs, fixed)
    _, params = configuration(ROOT / 'config/recovery', 'evs_assisted')
    validate_inputs(params, sensors, True)


def test_bringup_cli_and_tui_forward_selection_without_editing_yaml(monkeypatch, capsys):
    import importlib.util
    from unittest.mock import patch
    script = ROOT.parents[2] / 'scripts/lib/bringup.py'
    spec = importlib.util.spec_from_file_location('recovery_cli', script)
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    before = (ROOT / 'config/recovery/recovery.yaml').read_bytes()
    common = ['bringup.py', '--no-bridge', '--dry-run', '--run-name', 'recovery_test',
              '--e2e-model-dir', '/model', '--e2e-fixed-throttle', '0.1',
              '--rgb-fps', '30', '--infra-fps', '30', '--no-evs', '--foxglove']
    with (patch('kart_e2e.contract.input_contract', return_value=('rgb', 'image', [1,3,120,212])),
          patch('kart_e2e.contract.model_contract', return_value=({}, 'steer_only')),
          patch('kart_e2e.contract.routing_settings', return_value=None),
          patch('kart_e2e.contract.runtime_settings', return_value={'fixed_throttle': .1, 'max_throttle': .2})):
        monkeypatch.setattr(sys, 'argv', common + ['--mode', 'e2e', '--recovery', 'rgb_imu'])
        cli.main()
        assert 'recovery:=rgb_imu' in capsys.readouterr().out
        choices = iter(['e2e', 'infra_imu'])
        monkeypatch.setattr(sys.stdin, 'isatty', lambda: True)
        monkeypatch.setattr(cli, 'choose', lambda *a, **k: next(choices))
        monkeypatch.setattr(sys, 'argv', common)
        cli.main()
        assert 'recovery:=infra_imu' in capsys.readouterr().out
    assert (ROOT / 'config/recovery/recovery.yaml').read_bytes() == before


def test_e2e_inference_routes_selection_only_when_drive_enabled():
    import importlib.util
    from unittest.mock import Mock
    for modality, input_name in [('rgb', 'image'), ('evs', 'input_tensor')]:
        lu, lut = types.ModuleType('isaac_ros_launch_utils'), types.ModuleType('isaac_ros_launch_utils.all_types')
        lu.get_path = lambda package, suffix: ROOT / suffix
        lu.log_info = lambda message: message
        lu.load_composable_nodes = Mock()
        lut.ComposableNode = lambda **kwargs: kwargs
        with patch.dict(sys.modules, {'isaac_ros_launch_utils': lu, 'isaac_ros_launch_utils.all_types': lut}):
            spec = importlib.util.spec_from_file_location('recovery_inference', ROOT / 'launch/modules/e2e/inference.launch.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        args = types.SimpleNamespace(model_dir='/model', container_name='/container', use_sim_time='',
                                     fixed_throttle='', use_fixed_throttle='', force_engine_update='',
                                     input_topic='', drive_enabled='true', recovery='infra_imu')
        with (patch.object(module, 'model_contract', return_value=({}, 'steer_only')),
              patch.object(module, 'input_contract', return_value=(modality, input_name, [])),
              patch.object(module, 'routing_settings', return_value=None),
              patch.object(module, 'runtime_settings', return_value={'fixed_throttle': .1, 'max_throttle': .2}),
              patch.object(module, 'require_engine'),
              patch('kart_bringup.recovery.route', return_value=([('auto/control_cmd', '/recovery/input_cmd')], ['gate'])) as routing):
            assert module.add_nodes(args)[-1] == 'gate'
            assert routing.call_args.kwargs == {'selection': 'infra_imu'}
            decoder = lu.load_composable_nodes.call_args.args[1][-1]
            assert decoder['remappings'] == [('auto/control_cmd', '/recovery/input_cmd')]
            routing.reset_mock()
            args.drive_enabled = 'false'
            module.add_nodes(args)
            routing.assert_not_called()
            assert lu.load_composable_nodes.call_args.args[1][-1]['remappings'] == []
