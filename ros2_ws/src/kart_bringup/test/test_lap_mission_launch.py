"""Execute mission composition with action stubs; no ROS, sockets or GPU work."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / 'config'


class ProcessAction:
    def __init__(self, **kwargs):
        self.options = kwargs


class LapMissionLaunchTests(unittest.TestCase):
    def plan(self, *, lap_vgl='', lap_cache_clear=''):
        lu = ModuleType('isaac_ros_launch_utils')
        lut = ModuleType('isaac_ros_launch_utils.all_types')
        lu.get_path = lambda package, suffix: PACKAGE / suffix
        lu.log_info = lambda message: ('log', message)
        lu.component_container = lambda *args, **kwargs: ('container', args, kwargs)
        lu.include = Mock(side_effect=lambda *args, **kwargs: ('include', args, kwargs))
        index = ModuleType('ament_index_python.packages')
        index.get_package_share_directory = Mock(side_effect=lambda name: '/mock/' + name)
        ros_actions = ModuleType('launch_ros.actions')
        ros_actions.Node = ProcessAction
        modules = {'isaac_ros_launch_utils': lu,
                   'isaac_ros_launch_utils.all_types': lut,
                   'ament_index_python.packages': index,
                   'launch_ros.actions': ros_actions}

        # Replace host/HDMap readiness with the effective operational parameters.
        # The assertions below verify that mission passes this result unchanged.
        effective = yaml.safe_load((CONFIG / 'localization/lap_manager.yaml').read_text())[
            '/**/lap_manager']['ros__parameters']
        effective.update(vgl_enabled=lap_vgl == 'true', cache_clear_enabled=lap_cache_clear == 'true')
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location(
                'lap_mission_composition', PACKAGE / 'launch/mission.launch.py')
            mission = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mission)
            fields = (*mission.OVERRIDES, 'device', 'record_dir', 'run_name',
                      'rgb_fps', 'infra_fps', 'map_dir', 'model_dir', 'map_file',
                      'lane_id', 'line_type', 'multicam_mode')
            args = SimpleNamespace(**{key: '' for key in fields})
            args.mode, args.recovery, args.enable_bridge = 'drive', 'off', 'false'
            args.map_file, args.map_dir, args.model_dir = '/fixture/hdmap.json', '/fixture/map', '/fixture/models'
            args.lane_id, args.line_type, args.infra_profile = 'main', 'raceline', '640x480x60'
            args.lap_vgl, args.lap_cache_clear = lap_vgl, lap_cache_clear
            with (patch.object(mission, 'load_transforms'),
                  patch.object(mission, 'resolve', return_value=(
                      {'cuvslam': {'tracking_mode': 0}}, {}, {}, {'input_shape': [1, 3, 480, 640]})),
                  patch.object(mission, 'hdmap_choices', return_value=[('main', 'raceline')]),
                  patch('kart_bringup.lap_maintenance.maintenance_parameters', return_value=effective) as maintenance):
                actions = mission.add_mission(args)
        return SimpleNamespace(
            processes=[action.options for action in actions if isinstance(action, ProcessAction)],
            maintenance=maintenance, effective=effective, includes=lu.include,
            packages=index.get_package_share_directory)

    def assert_maintenance_selection(self, plan, *, vgl, cache):
        plan.maintenance.assert_called_once()
        root, composition, map_file, lane_id, line_type = plan.maintenance.call_args.args
        self.assertEqual(root, CONFIG)
        self.assertEqual((map_file, lane_id, line_type), ('/fixture/hdmap.json', 'main', 'raceline'))
        self.assertEqual(plan.maintenance.call_args.kwargs, {'parking': False})
        self.assertEqual((composition['lap_vgl'], composition['lap_cache_clear']), (vgl, cache))
        lap = next(process for process in plan.processes if process['name'] == 'lap_manager')
        self.assertEqual((lap['package'], lap['executable']), ('kart_lap_manager', 'lap_manager'))
        self.assertEqual(lap['parameters'], [plan.effective])
        self.assertIs(lap['parameters'][0], plan.effective)
        self.assertIs(lap['parameters'][0]['vgl_enabled'], vgl)
        self.assertIs(lap['parameters'][0]['cache_clear_enabled'], cache)
        plan.packages.assert_any_call('kart_lap_manager')

    def test_default_off_adds_no_lap_or_cache_process(self):
        plan = self.plan()
        self.assertEqual(plan.processes, [])
        plan.maintenance.assert_not_called()
        checked = [call.args[0] for call in plan.packages.call_args_list]
        self.assertNotIn('kart_lap_manager', checked)
        self.assertNotIn('kart_jetson_cache', checked)

    def test_vgl_only_adds_one_lap_process_with_operational_parameters(self):
        plan = self.plan(lap_vgl='true')
        self.assertEqual([process['name'] for process in plan.processes], ['lap_manager'])
        self.assert_maintenance_selection(plan, vgl=True, cache=False)
        checked = [call.args[0] for call in plan.packages.call_args_list]
        self.assertNotIn('kart_jetson_cache', checked)
        includes = {call.args[1]: call.kwargs['launch_arguments'] for call in plan.includes.call_args_list}
        self.assertEqual(includes['launch/localization.launch.py']['enable_vgl'], 'true')
        self.assertIn('launch/tracking.launch.py', includes)

    def test_cache_only_adds_one_lap_and_one_cache_process(self):
        plan = self.plan(lap_cache_clear='true')
        self.assertEqual([process['name'] for process in plan.processes], ['lap_manager', 'jetson_cache_service'])
        self.assert_maintenance_selection(plan, vgl=False, cache=True)
        cache = next(process for process in plan.processes if process['name'] == 'jetson_cache_service')
        self.assertEqual((cache['package'], cache['executable']), ('kart_jetson_cache', 'jetson_cache_service'))
        self.assertEqual(cache['parameters'], [str(CONFIG / 'system/jetson_cache.yaml')])
        plan.packages.assert_any_call('kart_jetson_cache')


if __name__ == '__main__':
    unittest.main()
