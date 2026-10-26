import importlib.util
from pathlib import Path
import shutil
import tempfile
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import patch
import sys
import yaml
from kart_bringup.tuning import collect_configuration, fixed_configuration

ROOT=Path(__file__).resolve().parents[1]/'config'


class TuningCollectionTest(unittest.TestCase):
    def test_launch_loads_into_collect_container_without_control(self):
        lu=ModuleType('isaac_ros_launch_utils');lut=ModuleType('isaac_ros_launch_utils.all_types')
        lu.get_path=lambda *args:ROOT
        lu.log_info=lambda text:('log',text)
        lu.include=lambda package,file,**kwargs:('include',file,kwargs)
        actions_module=ModuleType('launch.actions');ros_actions=ModuleType('launch_ros.actions')
        ros_actions.Node=lambda **kwargs:('node',kwargs)
        actions_module.Shutdown=lambda:('shutdown',)
        with patch.dict(sys.modules,{'isaac_ros_launch_utils':lu,'isaac_ros_launch_utils.all_types':lut,
                'launch':ModuleType('launch'),'launch.actions':actions_module,
                'launch_ros':ModuleType('launch_ros'),'launch_ros.actions':ros_actions}):
            spec=importlib.util.spec_from_file_location('tuning_collect_test',ROOT.parent/'launch/tuning_collect.launch.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            actions=module.build(SimpleNamespace(**{key:'' for key in module.KEYS}))
            fixed=module.build(SimpleNamespace(**{key:'' for key in module.KEYS},
                fixed_command='true',fixed_axes='throttle',throttle='.12',steering='-.05'))
        includes=[a for a in actions if a[0]=='include']
        self.assertEqual([a[1] for a in includes],['launch/mission.launch.py','launch/modules/localization/vslam.launch.py'])
        self.assertEqual(includes[0][2]['launch_arguments']['mode'],'collect')
        self.assertEqual(includes[1][2]['launch_arguments']['container_name'],'/kart_sensor_container')
        nodes=[a[1] for a in fixed if a[0]=='node']
        self.assertEqual(len(nodes),1)
        self.assertEqual(nodes[0]['executable'],'fixed_command')
        self.assertEqual(nodes[0]['parameters'][0]['throttle'],.12)
        self.assertEqual(nodes[0]['parameters'][0]['fixed_axes'],'throttle')
        self.assertEqual(nodes[0]['on_exit'],('shutdown',))

    def test_fixed_values_opt_in_and_ranges(self):
        self.assertFalse(fixed_configuration(ROOT)[0])
        with self.assertRaisesRegex(ValueError,'require fixed_command'):
            fixed_configuration(ROOT,'',{'throttle':'.1'})
        for values in ({'fixed_axes':'invalid'},{'throttle':'1.01'},{'steering':'nan'},{'duration_s':'-1'},
                       {'throttle':'.1','brake':'.1'}):
            with self.assertRaises(ValueError): fixed_configuration(ROOT,True,values)
        on,params=fixed_configuration(ROOT,True,{'throttle':'.1','steering':'-.1','duration_s':'3'})
        self.assertTrue(on);self.assertEqual(params['duration_s'],3.)
        reference=yaml.safe_load((ROOT.parents[1]/'kart_control/config/fixed_command.yaml').read_text())
        self.assertEqual(set(params),set(reference['/**/fixed_command']['ros__parameters']))

    def test_manual_mapless_shared_container(self):
        mission,params,remaps=collect_configuration(ROOT)
        self.assertFalse(mission['tracking'])
        self.assertFalse(mission['e2e'])
        self.assertFalse(mission['localization'])
        self.assertFalse(params['use_sim_time'])
        self.assertFalse(params['enable_localization_n_mapping'])
        self.assertFalse(params['publish_map_to_odom_tf'])
        self.assertFalse(params['publish_odom_to_base_tf'])
        self.assertEqual(params['base_frame'],'base_link')
        self.assertIn(('/diagnostics','/localization/vslam/diagnostics'),remaps)
        full=yaml.safe_load((ROOT/'localization/cuvslam.yaml').read_text())['/**/visual_slam']['ros__parameters']
        self.assertEqual(set(params),set(full))

    def test_missing_stereo_or_tf_rejected_and_vio_explicit(self):
        with self.assertRaisesRegex(ValueError,'stereo'):
            collect_configuration(ROOT,{'infra_fps':'0'})
        _,params,_=collect_configuration(ROOT,{'tracking_mode':'1','multicam_mode':'2'})
        self.assertEqual(params['tracking_mode'],1)
        self.assertEqual(params['multicam_mode'],2)
        with tempfile.TemporaryDirectory() as folder:
            copy_root=Path(folder)/'config';shutil.copytree(ROOT,copy_root)
            path=copy_root/'vehicle/transforms.yaml'
            data=yaml.safe_load(path.read_text());data['transforms']['camera_mount']['xyz']=None
            path.write_text(yaml.safe_dump(data))
            with self.assertRaisesRegex(ValueError,'未設定'):
                collect_configuration(copy_root)


if __name__=='__main__': unittest.main()
