"""Execute RGB launch builders with ROS action stubs to verify process wiring."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

PACKAGE = Path(__file__).resolve().parents[1]


class CompositionTests(unittest.TestCase):
    def test_rgb_camera_encoder_inference_share_external_container(self):
        lu, lut = ModuleType('isaac_ros_launch_utils'), ModuleType('isaac_ros_launch_utils.all_types')
        lu.get_path = lambda package, suffix: PACKAGE / suffix
        lu.log_info = lambda message: message
        lu.component_container = Mock()
        lu.include = Mock()
        lu.load_composable_nodes = Mock()
        lut.ComposableNode = lambda **kwargs: kwargs
        modules = {'isaac_ros_launch_utils': lu, 'isaac_ros_launch_utils.all_types': lut}

        def load(relative):
            spec = importlib.util.spec_from_file_location('composition_'+relative.replace('/', '_'), PACKAGE/'launch'/relative)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module

        with patch.dict(sys.modules, modules):
            pipeline = load('e2e.launch.py')
            encoder = load('modules/e2e/image_encoder.launch.py')
            inference = load('modules/e2e/inference.launch.py')
            camera = load('modules/sensors/realsense.launch.py')
        container = '/kart_sensor_container'
        args = SimpleNamespace(container_name=container, create_container='false', model_dir='/model',
                               image_topic='', camera_info_topic='', use_sim_time='', fixed_throttle='0.1',
                               use_fixed_throttle='true', drive_enabled='true', force_engine_update='')
        with (patch.object(pipeline, 'model_contract'), patch.object(pipeline, 'runtime_settings'),
              patch.object(pipeline, 'input_contract', return_value=('rgb','image',[1,3,120,212])),
              patch('kart_e2e.engine_cache.require_engine')):
            pipeline.add_pipeline(args)
        lu.component_container.assert_not_called()
        includes = {call.args[1]:call.kwargs['launch_arguments'] for call in lu.include.call_args_list}
        enc = includes['launch/modules/e2e/image_encoder.launch.py']
        inf = includes['launch/modules/e2e/inference.launch.py']
        self.assertEqual(enc['container_name'], container)
        self.assertEqual(inf['container_name'], container)
        self.assertEqual(inf['use_fixed_throttle'], 'true')
        camera.add_camera(SimpleNamespace(container_name=container, rgb_fps='', infra_fps=''))
        encoder.add_nodes(SimpleNamespace(**enc))
        with (patch.object(inference, 'model_contract', return_value=({},'steer_only')),
              patch.object(inference, 'routing_settings', return_value=None),
              patch.object(inference, 'runtime_settings', return_value={'fixed_throttle':0.1,'max_throttle':0.2}),
              patch.object(inference, 'input_contract', return_value=('rgb','image',[1,3,120,212])),
              patch.object(inference, 'require_engine')):
            inference.add_nodes(SimpleNamespace(**inf, input_topic=''))
        calls = lu.load_composable_nodes.call_args_list
        self.assertEqual([call.args[0] for call in calls], [container]*3)
        self.assertEqual([len(call.args[1]) for call in calls], [1,6,2])
        nodes = [node for call in calls for node in call.args[1]]
        self.assertEqual(len({node['name'] for node in nodes}), 9)
        self.assertTrue(all(node['extra_arguments']==[{'use_intra_process_comms':True}] for node in nodes))
        self.assertIn(('reshaped_tensor','/e2e/tensor_input'), nodes[6]['remappings'])
        self.assertIn(('tensor_pub','/e2e/tensor_input'), nodes[7]['remappings'])
        self.assertIn(('tensor_sub','/e2e/tensor_output'), nodes[7]['remappings'])

    def test_section_routing_loads_selector_and_keeps_trt_in_container(self):
        import json
        lu, lut = ModuleType('isaac_ros_launch_utils'), ModuleType('isaac_ros_launch_utils.all_types')
        lu.get_path = lambda package, suffix: PACKAGE / suffix
        lu.log_info = str
        lu.load_composable_nodes = Mock()
        lut.ComposableNode = lambda **kwargs: kwargs
        actions = ModuleType('launch_ros.actions')
        actions.Node = Mock(return_value='selector')
        modules = {'isaac_ros_launch_utils':lu,'isaac_ros_launch_utils.all_types':lut,
                   'launch_ros.actions':actions}
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('section_inference',PACKAGE/'launch/modules/e2e/inference.launch.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            routing=json.loads((PACKAGE.parent/'kart_e2e/config/examples/section_assignments.json').read_text())
            routing.update(schema='kart.section_heads.v1',frame='map',closed=False,points=[[0,0],[20,0]])
            args=SimpleNamespace(model_dir='/model',use_sim_time='',fixed_throttle='',use_fixed_throttle='true',
                                 drive_enabled='true',force_engine_update='',input_topic='',container_name='/same',recovery='off')
            with (patch.object(module,'routing_settings',return_value=routing),
                  patch.object(module,'model_contract',return_value=({},'steer_only')),
                  patch.object(module,'input_contract',return_value=('rgb','image',[1,3,120,212])),
                  patch.object(module,'runtime_settings',return_value=dict(fixed_throttle=.1,max_throttle=.2)),
                  patch.object(module,'require_engine'),patch('kart_bringup.recovery.route',return_value=([],[]))):
                result=module.add_nodes(args)
            self.assertIn('selector',result)
            container,nodes=lu.load_composable_nodes.call_args.args
            self.assertEqual(container,'/same')
            self.assertEqual(nodes[1]['parameters'][0]['head_count'],2)
            self.assertTrue(nodes[1]['parameters'][0]['routing_sha256'])
            self.assertEqual(actions.Node.call_args.kwargs['parameters'][1]['model_dir'],'/model')

    def test_section_mission_adds_saved_map_localization_without_tracking(self):
        import json
        lu, lut = ModuleType('isaac_ros_launch_utils'), ModuleType('isaac_ros_launch_utils.all_types')
        lu.get_path=lambda pkg,suffix:PACKAGE/suffix
        lu.log_info=str;lu.component_container=Mock();lu.include=Mock()
        index=ModuleType('ament_index_python.packages')
        index.get_package_share_directory=lambda name:'/mock/'+name
        with patch.dict(sys.modules,{'isaac_ros_launch_utils':lu,'isaac_ros_launch_utils.all_types':lut,
                                     'ament_index_python.packages':index}):
            spec=importlib.util.spec_from_file_location('section_mission',PACKAGE/'launch/mission.launch.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        routing=json.loads((PACKAGE.parent/'kart_e2e/config/examples/section_assignments.json').read_text())
        routing.update(frame='map')
        fields=(*module.OVERRIDES,'device','record_dir','run_name','auto_fixed_throttle','e2e_model_dir',
                'evs_backend','evs_serial','evs_bias_file','evs_tensor_config','rgb_fps','infra_fps',
                'map_dir','model_dir','map_file','lane_id','line_type','multicam_mode','recovery')
        args=SimpleNamespace(**{k:'' for k in fields})
        args.mode='e2e';args.recovery='off';args.enable_bridge='false'
        with (patch('kart_e2e.contract.routing_settings',return_value=routing),
              patch('kart_e2e.contract.input_contract',return_value=('rgb','image',[1,3,120,212])),
              patch('kart_e2e.sections.validate_maps'),patch('kart_e2e.contract.model_contract'),
              patch('kart_e2e.contract.runtime_settings'),patch('kart_e2e.engine_cache.require_engine'),
              patch.object(module,'load_transforms'),patch.object(module,'resolve',return_value=({}, {}, {}, {'input_shape':None}))):
            module.add_mission(args)
        calls={call.args[1]:call.kwargs['launch_arguments'] for call in lu.include.call_args_list}
        self.assertIn('launch/localization.launch.py',calls)
        self.assertNotIn('launch/tracking.launch.py',calls)
        self.assertEqual(calls['launch/localization.launch.py']['enable_vgl'],'false')
        self.assertEqual(calls['launch/localization.launch.py']['vslam_container'],
                         calls['launch/e2e.launch.py']['container_name'])
