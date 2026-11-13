"""Live behavior preflight and actual launch routing; no ROS or engine execution."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock, patch

from kart_bringup.mission import composition, validate_overtake

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
sys.path.insert(0, str(ROOT / 'scripts/lib'))
from bringup import argument_parser
from bringup_tui import Editor


class OvertakeTests(unittest.TestCase):
    def setUp(self):
        self.folder = self.enterContext(tempfile.TemporaryDirectory())
        self.engine = Path(self.folder) / 'racing.plan'
        self.engine.write_bytes(b'preflight fixture, not a TensorRT engine')
        self.manifest('racing')

    def manifest(self, profile):
        from kart_bringup.racing_models import get_profile
        (self.engine.parent / 'build.json').write_text(json.dumps(dict(
            model_profile=profile, num_classes=get_profile(profile)['num_classes'],
            engine_sha256=hashlib.sha256(self.engine.read_bytes()).hexdigest())))

    def test_off_default_and_live_prerequisites(self):
        self.assertEqual(composition(PACKAGE / 'config/mission.yaml', 'drive')['overtake_mode'], 'off')
        validate_overtake('off', 'collect', '', 'detect')
        for change in ({'mode': 'collect'}, {'mode': 'e2e'}, {'engine': ''},
                       {'engine': '/missing/engine.plan'}, {'perception_mode': 'detect'}):
            values = dict(selection='shadow', mode='drive', engine=str(self.engine), perception_mode='track')
            values.update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_overtake(**values)
        with self.assertRaisesRegex(ValueError, 'drive-only'):
            composition(PACKAGE / 'config/mission.yaml', 'e2e', {'overtake_mode': 'active'})

    def test_active_requires_two_classes_and_exclusive_driving_ownership(self):
        for profile in ('coco', 'opponent', 'racing'):
            self.manifest(profile)
            validate_overtake('shadow', 'drive', str(self.engine), 'track')
            if profile != 'racing':
                with self.assertRaisesRegex(ValueError, 'two-class'):
                    validate_overtake('active', 'drive', str(self.engine), 'track')
            else:
                validate_overtake('active', 'drive', str(self.engine), 'track')
        for conflict in ({'parking': True}, {'fixed_throttle': 0.}, {'recovery_enabled': True}):
            with self.subTest(conflict=conflict), self.assertRaisesRegex(ValueError, 'cannot use'):
                validate_overtake('active', 'drive', str(self.engine), 'track', **conflict)

    def editor(self, choices):
        args = argument_parser().parse_args(['--mode', 'drive', '--perception', str(self.engine),
                                             '--perception-mode', 'detect', '--recovery', 'infra_imu'])
        selected = iter(choices)
        return Editor(args, PACKAGE / 'config', ROOT, lambda *_args, **_kwargs: next(selected))

    def test_tui_active_enables_tracking_disables_recovery_and_records_adjustments(self):
        editor = self.editor(['active'])
        editor.edit('overtake')
        self.assertEqual(editor.args.overtake, 'active')
        self.assertEqual(editor.args.perception_mode, 'track')
        self.assertEqual(editor.args.recovery, 'off')
        self.assertTrue(any('自動復帰' in text and 'OFF' in text for text in editor.adjustments))

    def test_tui_disabling_perception_or_selecting_detection_turns_behavior_off(self):
        for action in ('perception', 'perception_mode'):
            editor = self.editor(['off' if action == 'perception' else 'detect'])
            editor.args.overtake = 'active'
            editor.edit(action)
            self.assertEqual(editor.args.overtake, 'off')

    def test_tracking_launch_routes_single_reference_owner_and_applies_active_timeout_only(self):
        lu = MagicMock()
        lu.get_path.side_effect = lambda _package, relative: PACKAGE / relative
        launch_arguments = ModuleType('kart_bringup.launch_arguments')
        launch_arguments.LaunchArguments = MagicMock()
        modules = {'isaac_ros_launch_utils': lu, 'isaac_ros_launch_utils.all_types': MagicMock(),
                   'kart_bringup.launch_arguments': launch_arguments}
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('overtake_tracking_test', PACKAGE / 'launch/tracking.launch.py')
            launch = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(launch)
            for mode in ('off', 'shadow', 'active'):
                lu.include.reset_mock()
                args = NS(container_name='', create_container='', map_file='map.json', lane_id='main',
                          line_type='raceline', use_sim_time='', recovery='off', overtake_mode=mode,
                          perception_engine=str(self.engine), perception_mode='track', auto_fixed_throttle='-1')
                launch.add_tracking(args)
                calls = {call.args[1]: call.kwargs['launch_arguments'] for call in lu.include.call_args_list}
                expected = 'planning/course_reference' if mode == 'active' else 'planning/reference_line'
                self.assertEqual(calls['launch/hdmap.launch.py']['reference_topic'], expected)
                self.assertEqual(calls['launch/modules/control/pure_pursuit.launch.py']['overtake_mode'], mode)
                self.assertEqual('launch/racing_behavior.launch.py' in calls, mode != 'off')
                if mode != 'off':
                    self.assertEqual(calls['launch/racing_behavior.launch.py']['mode'], mode)

    def test_pure_pursuit_only_reads_short_reference_timeouts_when_active(self):
        import yaml
        lu = MagicMock()
        lu.get_path.side_effect = lambda _package, relative: PACKAGE / relative
        launch_arguments = ModuleType('kart_bringup.launch_arguments')
        launch_arguments.LaunchArguments = MagicMock()
        modules = {'isaac_ros_launch_utils': lu, 'isaac_ros_launch_utils.all_types': MagicMock(),
                   'kart_bringup.launch_arguments': launch_arguments}
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('overtake_pp_test', PACKAGE / 'launch/modules/control/pure_pursuit.launch.py')
            launch = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(launch)
            launch.lut.ComposableNode.side_effect = lambda **kwargs: kwargs
            for mode in ('off', 'shadow', 'active'):
                launch.add_node(NS(container_name='/control', use_sim_time='', overtake_mode=mode))
                parameters = lu.load_composable_nodes.call_args.args[1][0]['parameters']
                files = [Path(value) for value in parameters if isinstance(value, str)]
                self.assertEqual(len(files), 2 if mode == 'active' else 1)
                if mode == 'active':
                    self.assertEqual(files[-1].name, 'control_overrides.yaml')
                    overrides = yaml.safe_load(files[-1].read_text())['/**/pure_pursuit']['ros__parameters']
                    self.assertEqual(overrides['line_timeout_s'], .4)
                    self.assertEqual(overrides['wall_timeout_s'], .4)

    def test_behavior_launch_starts_confirmed_projection_and_remaps_baseline_only(self):
        modules = {name: ModuleType(name) for name in (
            'ament_index_python', 'ament_index_python.packages', 'launch', 'launch.actions',
            'launch.substitutions', 'launch_ros', 'launch_ros.actions', 'launch_ros.parameter_descriptions')}
        modules['ament_index_python.packages'].get_package_share_directory = lambda _name: str(PACKAGE)
        modules['launch'].LaunchDescription = lambda values: values
        for name in ('DeclareLaunchArgument', 'LogInfo', 'OpaqueFunction'):
            setattr(modules['launch.actions'], name, lambda *args, **kwargs: kwargs)
        modules['launch.substitutions'].LaunchConfiguration = lambda name: NS(perform=lambda context: context[name])
        modules['launch_ros.actions'].Node = lambda **kwargs: kwargs
        modules['launch_ros.parameter_descriptions'].ParameterValue = lambda value, value_type: NS(value=value, value_type=value_type)
        map_file = Path(self.folder) / 'map.json'
        map_file.write_text('{}')  # ROS behavior node independently validates map contents.
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('overtake_behavior_test', PACKAGE / 'launch/racing_behavior.launch.py')
            launch = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(launch)
            for mode in ('shadow', 'active'):
                actions = launch.behavior(dict(mode=mode, map_file=str(map_file),
                                               perception_engine=str(self.engine), use_sim_time=''))
                ground, behavior = actions[-2:]
                self.assertEqual(ground['executable'], 'opponent_ground')
                self.assertEqual(ground['namespace'], '/racing')
                self.assertEqual(behavior['name'], 'racing_behavior')
                self.assertEqual(behavior['namespace'], '/')
                self.assertEqual(behavior['parameters'][1]['mode'].value, mode)
                self.assertEqual(set(behavior['parameters'][1]), {'mode', 'map_file'})
                baseline = '/planning/course_reference' if mode == 'active' else '/planning/reference_line'
                self.assertEqual(behavior['remappings'], [('baseline_reference', baseline)])


if __name__ == '__main__':
    unittest.main()
