import importlib.util
from pathlib import Path
import shutil
import tempfile
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import patch
import sys
import yaml
from kart_bringup.tuning import collect_configuration

ROOT=Path(__file__).resolve().parents[1]/'config'


class TuningCollectionTest(unittest.TestCase):
    def test_launch_loads_into_collect_container_without_control(self):
        lu=ModuleType('isaac_ros_launch_utils');lut=ModuleType('isaac_ros_launch_utils.all_types')
        lu.get_path=lambda *args:ROOT
        lu.log_info=lambda text:('log',text)
        lu.include=lambda package,file,**kwargs:('include',file,kwargs)
        with patch.dict(sys.modules,{'isaac_ros_launch_utils':lu,'isaac_ros_launch_utils.all_types':lut}):
            spec=importlib.util.spec_from_file_location('tuning_collect_test',ROOT.parent/'launch/tuning_collect.launch.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            actions=module.build(SimpleNamespace(**{key:'' for key in module.KEYS}))
        includes=[a for a in actions if a[0]=='include']
        self.assertEqual([a[1] for a in includes],['launch/mission.launch.py','launch/modules/localization/vslam.launch.py'])
        self.assertEqual(includes[0][2]['launch_arguments']['mode'],'collect')
        self.assertEqual(includes[1][2]['launch_arguments']['container_name'],'/kart_sensor_container')

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
