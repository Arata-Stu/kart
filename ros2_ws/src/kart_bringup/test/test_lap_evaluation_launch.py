"""Execute offline launch composition with action stubs; no ROS, GPU or bag playback."""
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / 'config'


class ProcessAction:
    def __init__(self, **kwargs):
        self.options = kwargs


class LapEvaluationLaunchTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        for folder in ('evaluation', 'localization'):
            shutil.copytree(CONFIG / folder, self.root / folder)
        self.document = dict(frame_id='map', lanes=[dict(id='main', closed=True,
            lines={'raceline': {'points': [[0,0], [2,0], [0,2]]}})])

    def plan(self, *, lap_vgl='', enable_vgl='true', record=False, missing_package=False,
             base_frame='', rviz='false'):
        lu = ModuleType('isaac_ros_launch_utils')
        lu.get_path = lambda package, suffix: self.root
        lu.log_info = lambda message: ('log', message)
        lu.include = Mock(side_effect=lambda *args, **kwargs: ('include', args, kwargs))
        lut = ModuleType('isaac_ros_launch_utils.all_types')
        lut.LaunchDescription = lambda actions: actions
        index = ModuleType('ament_index_python.packages')
        index.get_package_share_directory = Mock(side_effect=RuntimeError('package missing') if missing_package else None,
                                                  return_value='/mock/kart_lap_manager')
        ros_actions = ModuleType('launch_ros.actions')
        ros_actions.Node = Mock(side_effect=ProcessAction)
        modules = {'isaac_ros_launch_utils': lu, 'isaac_ros_launch_utils.all_types': lut,
                   'ament_index_python.packages': index, 'launch_ros.actions': ros_actions}
        selected_record = yaml.safe_load((self.root / 'evaluation/recording.yaml').read_text())
        selected_record['output'] = '/new/offline-result'
        recorder = Mock(return_value=[('record',)])
        self.recorder, self.node_action = recorder, ros_actions.Node
        args = SimpleNamespace(lap_vgl=lap_vgl, enable_vgl=enable_vgl, base_frame=base_frame,
            tracking_mode='', imu_frame='', multicam_mode='', bag='/fixture/bag', rate='',
            map_dir='/fixture/bundle', model_dir='/fixture/models', map_file='/fixture/hdmap.json',
            lane_id='main', line_type='raceline', rviz=rviz, eval_record='', eval_output='')
        def localization(root, map_dir, model_dir, overrides, **kwargs):
            return ({'cuvslam': dict(tracking_mode=0, base_frame=overrides['base_frame'],
                map_frame='map', odom_frame='replay_odom')}, {}, {}, {'input_shape': [1,3,480,640]})
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('lap_evaluation_launch', PACKAGE / 'launch/evaluation.launch.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            with (patch.object(module, 'resolve', side_effect=localization),
                  patch.object(module, 'playback'), patch.object(module, 'validate_bag_shape'),
                  patch.object(module, 'hdmap_choices', return_value=[('main', 'raceline')]),
                  patch('kart_hdmap.document.load', return_value=self.document),
                  patch('kart_bringup.evaluation_recording.recording_plan', return_value=selected_record if record else None),
                  patch('kart_bringup.evaluation_recording.start_recording', recorder)):
                actions = module.add_evaluation(args)
        return SimpleNamespace(actions=actions, processes=[a.options for a in actions if isinstance(a, ProcessAction)],
                               includes=lu.include, packages=index.get_package_share_directory,
                               module=module, record=selected_record, recorder=recorder)

    def test_default_off_and_explicit_off_keep_existing_composition(self):
        for option in ('', 'false'):
            with self.subTest(option=option):
                plan = self.plan(lap_vgl=option)
                self.assertEqual(plan.processes, [])
                plan.packages.assert_not_called()
                includes = [call.args[1] for call in plan.includes.call_args_list]
                self.assertEqual(includes, ['launch/localization.launch.py', 'launch/hdmap.launch.py',
                                            'launch/modules/evaluation/replay.launch.py'])

    def test_on_adds_only_lap_with_effective_clock_and_frames(self):
        plan = self.plan(lap_vgl='true')
        self.assertEqual(len(plan.processes), 1)
        lap = plan.processes[0]
        self.assertEqual((lap['package'], lap['executable'], lap['name']),
                         ('kart_lap_manager', 'lap_manager', 'lap_manager'))
        parameters = lap['parameters'][0]
        self.assertTrue(parameters['use_sim_time'])
        self.assertTrue(parameters['vgl_enabled'])
        self.assertFalse(parameters['cache_clear_enabled'])
        self.assertEqual((parameters['base_frame'], parameters['odom_frame'], parameters['map_frame']),
                         ('camera_link', 'replay_odom', 'map'))
        plan.packages.assert_called_once_with('kart_lap_manager')
        replay = plan.includes.call_args_list[-1].kwargs['launch_arguments']
        self.assertEqual(replay['lap_node'], 'lap_manager')
        self.assertEqual(replay['lap_status_topic'], '/localization/lap_status')
        self.assertEqual(json.loads(replay['lap_input_topics_json']),
                         ['/planning/reference_line', '/visual_slam/tracking/odometry', '/localization/vslam/diagnostics'])
        plan = self.plan(lap_vgl='true', base_frame='rear_axle')
        self.assertEqual(plan.processes[0]['parameters'][0]['base_frame'], 'rear_axle')

    def test_yaml_default_is_preserved_and_explicit_false_overrides_it(self):
        path = self.root / 'evaluation/replay.yaml'
        settings = yaml.safe_load(path.read_text())
        settings['lap_vgl'] = True
        path.write_text(yaml.safe_dump(settings))
        self.assertEqual(len(self.plan().processes), 1)
        self.assertEqual(self.plan(lap_vgl='false').processes, [])

    def test_invalid_selection_stops_before_recording_or_process_creation(self):
        for options, reason in (({'enable_vgl': 'false'}, 'enable_vgl'),
                                ({'missing_package': True}, 'package missing'),
                                ({'rviz': 'bad'}, 'true/false')):
            with self.subTest(options=options), self.assertRaisesRegex((ValueError, RuntimeError), reason):
                self.plan(lap_vgl='true', record=True, **options)
            self.recorder.assert_not_called()
            self.node_action.assert_not_called()
        self.document['lanes'][0]['closed'] = False
        with self.assertRaisesRegex(ValueError, 'closed'):
            self.plan(lap_vgl='true', record=True)
        self.recorder.assert_not_called()
        self.node_action.assert_not_called()

    def test_result_manifest_and_topics_record_effective_lap_selection(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                plan = self.plan(lap_vgl=str(enabled).lower(), record=True)
                manifest = plan.recorder.call_args.args[2]
                self.assertIs(manifest['lap_vgl'], enabled)
                self.assertEqual(manifest['lap_parameters'], plan.processes[0]['parameters'][0] if enabled else None)
                for topic in ('/localization/lap_events', '/localization/lap_status'):
                    self.assertEqual(plan.record['topics'].count(topic), 1)
                replay = plan.includes.call_args_list[-1].kwargs['launch_arguments']
                self.assertIn('/localization/lap_status', json.loads(replay['record_topics_json']))

    def test_launch_argument_default_is_empty(self):
        module = self.plan().module
        arguments = SimpleNamespace(add_arg=Mock(), add_opaque_function=Mock(), get_launch_actions=lambda: [])
        with patch.object(module, 'LaunchArguments', return_value=arguments):
            module.generate_launch_description()
        arguments.add_arg.assert_any_call('lap_vgl', '', cli=True)

    def test_replay_module_forwards_lap_graph_readiness_only_when_enabled(self):
        plan = self.plan(lap_vgl='true')
        modules = {'isaac_ros_launch_utils': plan.module.lu,
                   'isaac_ros_launch_utils.all_types': ModuleType('isaac_ros_launch_utils.all_types')}
        for name in ('launch.actions', 'launch.event_handlers', 'launch.events', 'launch_ros.actions'):
            modules[name] = ModuleType(name)
        for name in ('ExecuteProcess', 'RegisterEventHandler', 'TimerAction', 'EmitEvent'):
            setattr(modules['launch.actions'], name, ProcessAction)
        modules['launch.event_handlers'].OnProcessExit = ProcessAction
        modules['launch.events'].Shutdown = ProcessAction
        modules['launch_ros.actions'].Node = ProcessAction
        replay_args = plan.includes.call_args_list[-1].kwargs['launch_arguments']
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('lap_replay_module', PACKAGE / 'launch/modules/evaluation/replay.launch.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            with patch.object(module, 'playback', return_value=['ros2', 'bag', 'play', '/fixture/bag']):
                actions = module.add_replay(SimpleNamespace(**replay_args))
                ready = actions[-1].options['parameters'][1]
                self.assertEqual(ready['lap_node'], 'lap_manager')
                self.assertEqual(ready['lap_input_topics'], ['/planning/reference_line',
                    '/visual_slam/tracking/odometry', '/localization/vslam/diagnostics'])
                self.assertEqual(ready['lap_status_topic'], '/localization/lap_status')
                replay_args['lap_node'] = ''
                ready = module.add_replay(SimpleNamespace(**replay_args))[-1].options['parameters'][1]
                self.assertNotIn('lap_node', ready)
                self.assertNotIn('lap_input_topics', ready)
                self.assertNotIn('lap_status_topic', ready)


if __name__ == '__main__':
    unittest.main()
