"""Execute launch callbacks with ROS substitutions stubbed; no ROS or device I/O."""
import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from kart_bringup.launch_arguments import LaunchArguments
from kart_bringup.configuration import load, RUNTIME_KEYS

PACKAGE = Path(__file__).resolve().parents[1]


class Configuration:
    def __init__(self, name, default=None):
        self.name, self.default = name, default

    def perform(self, context):
        value = context.get(self.name, self.default)
        if value is None:
            raise ValueError('missing argument: ' + self.name)
        return str(value)


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class Callback(Action):
    def execute(self, context):
        return self.kwargs['function'](context)


class LaunchArgumentTests(unittest.TestCase):
    def setUp(self):
        modules = {name: types.ModuleType(name) for name in (
            'launch', 'launch.actions', 'launch.substitutions', 'launch_ros',
            'launch_ros.actions', 'launch_ros.descriptions', 'isaac_ros_launch_utils',
            'isaac_ros_launch_utils.all_types', 'ament_index_python',
            'ament_index_python.packages')}
        self.lu = modules['isaac_ros_launch_utils']
        self.lu.__path__ = []
        self.lu.get_path = lambda pkg, suffix: PACKAGE / suffix
        self.lu.include = lambda pkg, file, **kw: SimpleNamespace(
            file=file, values={k: str(v) for k, v in kw.get('launch_arguments', {}).items()})
        self.lu.log_info = lambda message: Action(message)
        self.lu.component_container = Mock(return_value=Action())
        self.lu.load_composable_nodes = Mock(return_value=Action())
        lut = modules['isaac_ros_launch_utils.all_types']
        lut.Action = Action
        lut.ComposableNode = Action
        lut.LaunchDescription = list
        self.lu.all_types = lut
        actions = modules['launch.actions']
        actions.DeclareLaunchArgument = Action
        actions.OpaqueFunction = Callback
        actions.LogInfo = Action
        modules['launch.substitutions'].LaunchConfiguration = Configuration
        modules['launch_ros.actions'].Node = Action
        modules['launch_ros.actions'].ComposableNodeContainer = Action
        modules['launch_ros.descriptions'].ComposableNode = Action
        modules['ament_index_python.packages'].get_package_share_directory = lambda pkg: str(PACKAGE)
        modules['launch'].LaunchDescription = list
        patcher = patch.dict(sys.modules, modules)
        patcher.start()
        self.addCleanup(patcher.stop)

    def module(self, file):
        spec = importlib.util.spec_from_file_location('audit_' + file.replace('/', '_'), PACKAGE / 'launch' / file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_text_is_preserved_and_empty_is_the_only_unset_sentinel(self):
        args = LaunchArguments()
        values = dict(zero='0', serial='123456', run_name='0000', boolean='False',
                      number='1e2', json='[1, 2]', unset='')
        for key in values:
            args.add_arg(key, '', cli=True)
        callback = Mock(return_value=[])
        action = args.add_opaque_function(callback)
        action.execute(values)
        self.assertEqual(vars(callback.call_args.args[0]), values)
        self.assertEqual(len(args.get_launch_actions()), len(values) + 1)

    def test_collect_command_reaches_camera_driver_and_bag_parameters(self):
        with tempfile.TemporaryDirectory() as folder:
            bias = Path(folder).resolve() / 'selected.bias'
            bias.write_text('178 % bias_diff_off\n')
            mission = self.module('mission.launch.py')
            context = dict(mode='collect', enable_bridge='false', enable_evs='true',
                           rgb_profile='0', infra_profile='0', imu_profile='off',
                           evs_serial='123456', evs_bias_file=str(bias),
                           evs_image='false', evs_aggregation='evt3_runs_grouped_gpu',
                           record_dir=str(Path(folder) / 'record'), run_name='0000',
                           auto_fixed_throttle='0')
            actions = mission.generate_launch_description()[-1].execute(context)
            includes = {a.file: a.values for a in actions if hasattr(a, 'file')}
            camera = self.module('modules/sensors/realsense.launch.py')
            camera.generate_launch_description()[-1].execute(includes['launch/modules/sensors/realsense.launch.py'])
            params = self.lu.load_composable_nodes.call_args.args[1][0].kwargs['parameters'][0]
            for key in ('enable_color', 'enable_infra1', 'enable_infra2', 'enable_accel', 'enable_gyro'):
                self.assertIs(params[key], False)
            evs = self.module('modules/sensors/openeb.launch.py')
            node = evs.setup(includes['launch/modules/sensors/openeb.launch.py'])[-1]
            self.assertNotIn('name', node.kwargs)  # Dict overrides must reach every internal node.
            self.assertEqual(node.kwargs['parameters'][-1], dict(
                tensor_backend='cuda_async', event_aggregation='evt3_runs_grouped_gpu',
                event_image_enabled=False, serial='123456', bias_file=str(bias)))
            vehicle = self.module('vehicle.launch.py')
            results = vehicle.start(dict.fromkeys(RUNTIME_KEYS, '') |
                                    {'config': str(PACKAGE / 'config/bringup.yaml')} |
                                    includes['launch/vehicle.launch.py'])
            bag = next(a for a in results if getattr(a, 'kwargs', {}).get('name') == 'kart_bag_manager')
            self.assertEqual(bag.kwargs['parameters'][-1]['recording_name'], '0000')
            self.assertEqual(bag.kwargs['parameters'][-1]['output_dir'], context['record_dir'])
            _, modules = load(PACKAGE / 'config/bringup.yaml', includes['launch/vehicle.launch.py'])
            self.assertEqual(modules['joy_manager']['effective']['auto_fixed_throttle'], 0.)
            self.assertIs(modules['command_mux']['effective']['auto_use_teleop'], True)

    def test_all_isaac_based_launches_use_text_arguments(self):
        for file in (PACKAGE / 'launch').rglob('*.py'):
            self.assertNotIn('lu.ArgumentContainer()', file.read_text(), str(file))

    def test_rgb_and_evs_e2e_keep_explicit_zero_throttle_and_false(self):
        for modality, file, input_name in (('rgb', 'e2e.launch.py', 'image'),
                                           ('evs', 'evs_e2e.launch.py', 'input_tensor')):
            with self.subTest(modality=modality):
                pipeline = self.module(file)
                context = dict(model_dir='/model', container_name='/event_camera/tensor_container' if modality == 'evs' else '/external', create_container='false',
                               evs_tensor_config='/tensor.yaml', use_fixed_throttle='true',
                               fixed_throttle='0', drive_enabled='false', use_sim_time='false', recovery='off')
                with (patch('kart_e2e.engine_cache.require_engine'),
                      patch.object(pipeline, 'require_engine', create=True),
                      patch.object(pipeline, 'runtime_settings', return_value={}),
                      patch.object(pipeline, 'validate_evs_tensor', create=True),
                      patch.object(pipeline, 'model_contract', return_value=({}, 'steer_only'), create=True),
                      patch.object(pipeline, 'input_contract', return_value=(modality, input_name, []), create=True)):
                    actions = pipeline.generate_launch_description()[-1].execute(context)
                inference_args = next(a.values for a in actions if getattr(a, 'file', '').endswith('inference.launch.py'))
                inference = self.module('modules/e2e/inference.launch.py')
                with (patch.object(inference, 'require_engine'),
                      patch.object(inference, 'model_contract', return_value=({}, 'steer_only')),
                      patch.object(inference, 'input_contract', return_value=(modality, input_name, [])),
                      patch.object(inference, 'runtime_settings', return_value=dict(fixed_throttle=.1, max_throttle=.2)),
                      patch.object(inference, 'routing_settings', return_value=None)):
                    inference.generate_launch_description()[-1].execute(inference_args)
                decoder = self.lu.load_composable_nodes.call_args.args[1][1].kwargs['parameters'][0]
                self.assertEqual(decoder['fixed_throttle'], 0.)
                self.assertIs(decoder['use_fixed_throttle'], True)
                self.assertIs(decoder['drive_enabled'], False)
                self.assertIs(decoder['use_sim_time'], False)

    def test_offline_zero_modes_frames_and_rate_reach_child_launches(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'map.mdb').write_bytes(b'fixture')
            evaluation = self.module('evaluation.launch.py')
            context = dict(map_dir=str(root), map_file='/hdmap.yaml', lane_id='main', line_type='centerline',
                           bag='/bag', rate='.5', enable_vgl='false', tracking_mode='0', multicam_mode='0',
                           base_frame='custom_base', imu_frame='custom_imu', rviz='false')
            with (patch.object(evaluation, 'playback'),
                  patch.object(evaluation, 'hdmap_choices', return_value=[('main', 'centerline')])):
                actions = evaluation.generate_launch_description()[-1].execute(context)
            includes = {a.file: a.values for a in actions if hasattr(a, 'file')}
            replay = includes['launch/modules/evaluation/replay.launch.py']
            self.assertEqual(replay['rate'], '.5')
            localization = self.module('localization.launch.py')
            result = localization.generate_launch_description()[-1].execute(includes['launch/localization.launch.py'])
            vslam_args = next(a.values for a in result if getattr(a, 'file', '').endswith('/vslam.launch.py'))
            vslam = self.module('modules/localization/vslam.launch.py')
            vslam.generate_launch_description()[-1].execute(vslam_args)
            params = self.lu.load_composable_nodes.call_args.args[1][0].kwargs['parameters'][0]
            self.assertEqual(params['tracking_mode'], 0)
            self.assertEqual(params['multicam_mode'], 0)
            self.assertEqual(params['base_frame'], 'custom_base')
            self.assertEqual(params['imu_frame'], 'custom_imu')
            self.assertIs(params['use_sim_time'], True)

    def test_tracking_off_reaches_hdmap_and_control_parameters(self):
        tracking = self.module('tracking.launch.py')
        context = dict(map_file='/maps/course/map.json', lane_id='main', line_type='customline',
                       tuning_enabled='false', tuning_map_dir='/maps/course',
                       create_container='false', use_sim_time='false', recovery='off')
        result = tracking.generate_launch_description()[-1].execute(context)
        includes = {a.file: a.values for a in result if hasattr(a, 'file')}
        hdmap = self.module('hdmap.launch.py')
        node = hdmap.setup(includes['launch/hdmap.launch.py'])[0]
        params = node.kwargs['parameters'][-1]
        self.assertIs(params['tuning_enabled'], False)
        self.assertIs(params['use_sim_time'], False)
        self.assertEqual(params['tuning_map_dir'], '/maps/course')
        self.assertEqual(params['map_file'], context['map_file'])
        self.assertEqual(params['lane_id'], 'main')
        self.assertEqual(params['line_type'], 'customline')
        for kind in ('pure_pursuit', 'speed_controller'):
            file = f'modules/control/{kind}.launch.py'
            control = self.module(file)
            control.generate_launch_description()[-1].execute(includes['launch/' + file])
            params = self.lu.load_composable_nodes.call_args.args[1][0].kwargs['parameters'][-1]
            self.assertIs(params['use_sim_time'], False)


if __name__ == '__main__':
    unittest.main()
