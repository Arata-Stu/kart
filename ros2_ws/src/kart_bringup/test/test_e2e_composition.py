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
