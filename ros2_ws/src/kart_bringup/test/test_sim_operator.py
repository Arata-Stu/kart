"""Selection, launch isolation and full sim parameter contracts; no ROS/GPU."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import ModuleType, SimpleNamespace
import yaml
from kart_bringup.sim_operator import validate_selection, pipeline_command

PACKAGE=Path(__file__).resolve().parents[1]


class OperatorTests(unittest.TestCase):
    def selection(self):
        return dict(map_file='/maps/a b/map.json',map_dir='/maps/vslam',model_dir='',enable_vgl='false',lane_id='lane_1',line_type='centerline')
    def test_selection_requires_discovered_assets_and_lane(self):
        assets=dict(hdmaps=['/maps/a b/map.json'],vslam_maps=['/maps/vslam'],vgl_bundles=['/maps/vgl'],models=['/models/gpu'])
        with patch('kart_bringup.sim_operator.hdmap_choices',return_value=[('lane_1','centerline')]):
            selection=self.selection();self.assertEqual(validate_selection(selection,assets),selection)
            command=pipeline_command(selection)
            self.assertIn('map_file:=/maps/a b/map.json',command)
            for key,value in [('map_file','/outside/map.json'),('lane_id','wrong'),('enable_vgl','yes'),('model_dir','/models/gpu')]:
                invalid={**selection,key:value}
                with self.assertRaises(ValueError):validate_selection(invalid,assets)
            selection.update(enable_vgl='true',map_dir='/maps/vgl',model_dir='/models/gpu')
            self.assertEqual(validate_selection(selection,assets),selection)
    def test_sim_configs_are_complete_and_leave_real_gains_zero(self):
        for name,folder in [('pure_pursuit','control'),('speed_controller','control'),('hdmap','hdmap'),('operation_mode_manager','system'),('command_mux','system')]:
            original=yaml.safe_load((PACKAGE/f'config/{folder}/{name}.yaml').read_text())
            sim=yaml.safe_load((PACKAGE/f'config/sim/drive_{name}.yaml').read_text())
            old=next(iter(original.values()))['ros__parameters'];new=next(iter(sim.values()))['ros__parameters']
            self.assertEqual(set(old),set(new));self.assertTrue(new['use_sim_time'])
        old=yaml.safe_load((PACKAGE/'config/control/speed_controller.yaml').read_text())['/**/speed_controller']['ros__parameters']
        self.assertEqual(old['kp'],0.)
    def test_drive_launch_owns_only_sim_controller_and_saved_localization(self):
        names=('launch','launch.actions','launch.launch_description_sources','launch.substitutions','launch_ros','launch_ros.actions','launch_ros.descriptions','ament_index_python','ament_index_python.packages')
        modules={name:ModuleType(name) for name in names}
        def capture(kind):return lambda *a,**k:SimpleNamespace(kind=kind,args=a,kwargs=k)
        modules['launch'].LaunchDescription=capture('LaunchDescription')
        for kind in ('DeclareLaunchArgument','OpaqueFunction','IncludeLaunchDescription','Shutdown'):setattr(modules['launch.actions'],kind,capture(kind))
        modules['launch.launch_description_sources'].PythonLaunchDescriptionSource=lambda p:p
        modules['launch.substitutions'].LaunchConfiguration=lambda name:SimpleNamespace(perform=lambda ctx:ctx[name])
        for kind in ('ComposableNodeContainer','Node'):setattr(modules['launch_ros.actions'],kind,capture(kind))
        modules['launch_ros.descriptions'].ComposableNode=capture('component')
        modules['ament_index_python.packages'].get_package_share_directory=lambda _:str(PACKAGE)
        with patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('drive_test',PACKAGE/'launch/sim_drive.launch.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with patch.object(module,'hdmap_choices',return_value=[('lane_1','centerline')]),patch.object(module,'resolve') as resolve:
            actions=module.build(self.selection())
            self.assertEqual(resolve.call_args.kwargs['enable_vgl'],False)
        node=next(a for a in actions if a.kind=='Node')
        self.assertEqual(node.kwargs['package'],'kart_hdmap')
        container=next(a for a in actions if a.kind=='ComposableNodeContainer')
        components=container.kwargs['composable_node_descriptions']
        self.assertEqual({c.kwargs['name'] for c in components},{'operation_mode_manager','command_mux','pure_pursuit','speed_controller'})
        include=next(a for a in actions if a.kind=='IncludeLaunchDescription')
        self.assertEqual(dict(include.kwargs['launch_arguments'])['use_sim_time'],'true')


if __name__=='__main__':unittest.main()
