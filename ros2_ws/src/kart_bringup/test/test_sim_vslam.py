"""Sim bag diagnostic launch validation, without ROS/CUDA execution."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch
import yaml

PACKAGE=Path(__file__).resolve().parents[1]


class SimVslamTests(unittest.TestCase):
    def load(self):
        names=('ament_index_python','ament_index_python.packages','launch','launch.actions',
               'launch.substitutions','launch_ros','launch_ros.actions','launch_ros.descriptions')
        modules={name:ModuleType(name) for name in names}
        modules['ament_index_python.packages'].get_package_share_directory=lambda _:str(PACKAGE)
        def capture(kind):
            return lambda *args,**kwargs:SimpleNamespace(kind=kind,args=args,kwargs=kwargs)
        modules['launch'].LaunchDescription=capture('description')
        for name in ('DeclareLaunchArgument','OpaqueFunction','LogInfo','Shutdown','ExecuteProcess'):
            setattr(modules['launch.actions'],name,capture(name))
        class Configuration:
            def __init__(self,name):self.name=name
            def perform(self,context):return context.get(self.name,'')
        modules['launch.substitutions'].LaunchConfiguration=Configuration
        for name in ('ComposableNodeContainer','Node'):
            setattr(modules['launch_ros.actions'],name,capture(name))
        modules['launch_ros.descriptions'].ComposableNode=capture('component')
        with patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('sim_vslam_test',PACKAGE/'launch/sim_vslam.launch.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module

    def bag(self,folder,imu=False):
        flow=json.loads((PACKAGE/'config/localization/workflow.json').read_text())
        topics=[flow[k] for k in ('left_image','right_image','left_info','right_info')]+['/tf_static','/tf','/clock']
        if imu:topics.append('/realsense/imu')
        rows=[{'topic_metadata':{'name':name},'message_count':1} for name in topics]
        (Path(folder)/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{'topics_with_message_count':rows}}))

    def test_mapless_visual_review_filters_clock_and_tf(self):
        module=self.load()
        with tempfile.TemporaryDirectory() as folder:
            self.bag(folder)
            actions=module.build({'bag':folder,'visualize':'true','rate':'.5'})
        container=next(a for a in actions if a.kind=='ComposableNodeContainer')
        component=container.kwargs['composable_node_descriptions'][0]
        config,overrides=component.kwargs['parameters']
        params=yaml.safe_load(Path(config).read_text())['/**/visual_slam']['ros__parameters']
        self.assertEqual(params['load_map_folder_path'],'')
        self.assertFalse(params['localize_on_startup'])
        self.assertTrue(all(overrides[k] for k in ('enable_slam_visualization','enable_landmarks_view','enable_observations_view')))
        rviz=next(a for a in actions if a.kind=='Node' and a.kwargs['package']=='rviz2')
        rviz_config=yaml.safe_load(Path(rviz.kwargs['arguments'][1]).read_text())
        displays=rviz_config['Visualization Manager']['Displays']
        self.assertTrue(any(d['Class']=='rviz_default_plugins/Image' for d in displays))
        self.assertTrue(any(d['Name']=='Landmarks' for d in displays))
        player=next(a for a in actions if a.kind=='ExecuteProcess').kwargs['cmd']
        topics=player[player.index('--topics')+1:]
        self.assertNotIn('/tf',topics);self.assertNotIn('/clock',topics)
        self.assertIn('--start-paused',player)
        self.assertIn('--clock',player)
        ready=next(a for a in actions if a.kind=='Node' and a.kwargs['executable']=='replay_ready')
        self.assertEqual(ready.kwargs['parameters'][1]['subscriber_nodes'],['visual_slam'])

    def test_vio_requires_and_replays_imu_and_live_keeps_defaults(self):
        module=self.load()
        with tempfile.TemporaryDirectory() as folder:
            self.bag(folder)
            with self.assertRaisesRegex(ValueError,'/realsense/imu'):
                module.build({'bag':folder,'tracking_mode':'1'})
            self.bag(folder,imu=True)
            actions=module.build({'bag':folder,'tracking_mode':'1','rviz':'false'})
            player=next(a for a in actions if a.kind=='ExecuteProcess').kwargs['cmd']
            self.assertIn('/realsense/imu',player[player.index('--topics')+1:])
        actions=module.build({})
        self.assertFalse(any(a.kind in ('ExecuteProcess','Node') for a in actions))
        component=next(a for a in actions if a.kind=='ComposableNodeContainer').kwargs['composable_node_descriptions'][0]
        self.assertEqual(component.kwargs['parameters'][1],{})
        for key,value in [('visualize','bad'),('rviz','bad'),('tracking_mode','2'),('rate','1')]:
            with self.assertRaises(ValueError):module.build({key:value})


if __name__=='__main__':unittest.main()
