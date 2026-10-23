"""EVS startup contract and launch callbacks, with no ROS/GPU dependencies."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

import yaml
from kart_bringup.e2e import validate_evs_tensor

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]


class EvsE2ETests(unittest.TestCase):
    def test_full_driver_config_validation(self):
        base = PACKAGE / 'config/sensors/openeb_tensor_cuda_async.yaml'
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'tensor.yaml'
            config = yaml.safe_load(base.read_text())
            params = config['/**/event_tensor']['ros__parameters']
            self.assertEqual((params['width'],params['height']), (160,120))
            with patch('kart_e2e.contract.input_contract', return_value=('evs', 'input_tensor', [1,20,120,160])):
                path.write_text(yaml.safe_dump(config))
                self.assertEqual(validate_evs_tensor(path, 'model')['width'], 160)
                for key, value in [('width',212), ('channel_mean',[0.1]), ('temporal_interpolation','linear'),
                                   ('polarity_layout','time_major'), ('tensor_name','image')]:
                    previous = params[key]
                    params[key] = value
                    path.write_text(yaml.safe_dump(config))
                    with self.assertRaises(ValueError):
                        validate_evs_tensor(path, 'model')
                    params[key] = previous
                with self.assertRaises(ValueError):
                    validate_evs_tensor('', 'model')

    def launch_module(self, filename, lu, lut):
        with patch.dict(sys.modules, {'isaac_ros_launch_utils': lu,
                                     'isaac_ros_launch_utils.all_types': lut}):
            spec = importlib.util.spec_from_file_location('evs_test_launch', PACKAGE / filename)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        return module

    def test_consumer_loads_into_openeb_without_camera_or_container_creation(self):
        lu = types.ModuleType('isaac_ros_launch_utils')
        lut = types.ModuleType('isaac_ros_launch_utils.all_types')
        lu.all_types = lut
        lu.get_path = lambda pkg, suffix: PACKAGE / suffix
        lu.log_info = lambda msg: msg
        lu.include = Mock()
        module = self.launch_module('launch/evs_e2e.launch.py', lu, lut)
        args = types.SimpleNamespace(container_name='', input_topic='', model_dir='/model',
                                     evs_tensor_config='/model/openeb.yaml', fixed_throttle='', use_fixed_throttle='true',
                                     drive_enabled='', use_sim_time='')
        with patch.object(module, 'validate_evs_tensor'), patch.object(module, 'runtime_settings'), patch.object(module, 'require_engine'):
            module.add_pipeline(args)
        self.assertEqual(lu.include.call_count, 1)
        self.assertEqual(lu.include.call_args.args[1], 'launch/modules/e2e/inference.launch.py')
        values = lu.include.call_args.kwargs['launch_arguments']
        self.assertEqual(values['container_name'], '/event_camera/tensor_container')
        self.assertEqual(values['input_topic'], '/event_camera/tensor')
        self.assertEqual(values['use_fixed_throttle'], 'true')

    def test_tensor_rt_ev_bindings_and_remap_keep_existing_decoder(self):
        lu = types.ModuleType('isaac_ros_launch_utils')
        lut = types.ModuleType('isaac_ros_launch_utils.all_types')
        lu.all_types = lut
        lu.get_path = lambda pkg, suffix: PACKAGE / suffix
        lu.log_info = lambda msg: msg
        lu.load_composable_nodes = Mock()
        lut.ComposableNode = lambda **kw: kw
        module = self.launch_module('launch/modules/e2e/inference.launch.py', lu, lut)
        args = types.SimpleNamespace(container_name='/event_camera/tensor_container', model_dir='/model',
                                     use_sim_time='', fixed_throttle='0.12', drive_enabled='', use_fixed_throttle='true',
                                     force_engine_update='', input_topic='/event_camera/tensor')
        with (patch.object(module, 'model_contract', return_value=({'model_file_path':'/model/model.onnx', 'engine_file_path':'/model/model.plan'}, 'steer_only')),
              patch.object(module, 'input_contract', return_value=('evs','input_tensor',[1,20,120,160])),
              patch.object(module, 'runtime_settings', return_value={'fixed_throttle':0.1,'max_throttle':0.2}),
              patch.object(module, 'routing_settings', return_value=None),
              patch.object(module, 'require_engine')):
            module.add_nodes(args)
        container, nodes = lu.load_composable_nodes.call_args.args
        self.assertEqual(container, '/event_camera/tensor_container')
        self.assertEqual(len(nodes), 2)
        self.assertIn(('tensor_pub', '/event_camera/tensor'), nodes[0]['remappings'])
        self.assertEqual(nodes[0]['parameters'][0]['input_binding_names'], ['input_tensor'])
        self.assertFalse(nodes[1]['parameters'][0]['drive_enabled'])
        self.assertEqual(nodes[1]['parameters'][0]['fixed_throttle'], 0.12)
        self.assertTrue(nodes[1]['parameters'][0]['use_fixed_throttle'])

    def test_generated_config_and_cli_support_evs_without_rgb(self):
        import hashlib
        from kart_e2e.contract import evs_spec
        fixture_path = PACKAGE.parent / 'kart_e2e/test/evs_fixtures.py'
        spec = importlib.util.spec_from_file_location('evs_test_metadata', fixture_path)
        fixture = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(fixture)
        with tempfile.TemporaryDirectory() as tmp:
            model = Path(tmp) / 'evs'
            model.mkdir()
            (model / 'model.onnx').write_bytes(b'metadata fixture')
            meta = fixture.metadata()
            data = dict(schema=1, modality='evs', evs_metadata=meta, model_spec=evs_spec(meta),
                        input_name='input_tensor', input_shape=[1,20,120,160],
                        output_name='control', output_shape=[1,1], output_mode='steer_only',
                        runtime={'fixed_throttle':0.1,'max_throttle':0.2},
                        onnx_sha256=hashlib.sha256(b'metadata fixture').hexdigest())
            (model / 'metadata.json').write_text(json.dumps(data))
            config = model / 'openeb_tensor.yaml'
            subprocess.run([sys.executable, '-m', 'kart_e2e.evs_contract',
                            '--model-json', str(model / 'metadata.json'),
                            '--base-config', str(PACKAGE / 'config/sensors/openeb_tensor_cuda_async.yaml'),
                            '--output', str(config)], check=True, capture_output=True, text=True)
            generated = validate_evs_tensor(config, model)
            original = yaml.safe_load((PACKAGE / 'config/sensors/openeb_tensor_cuda_async.yaml').read_text())['/**/event_tensor']['ros__parameters']
            self.assertEqual(set(generated), set(original))
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/lib/bringup.py'),
                                     '--mode', 'e2e', '--evs', '--no-bridge', '--dry-run',
                                     '--e2e-model-dir', str(model), '--evs-tensor-config', str(config),
                                     '--rgb-fps', '0', '--infra-fps', '0'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('evs_tensor_config:=', result.stdout)
            self.assertIn('RGB: OFF', result.stdout)
            self.assertIn('E2E AUTO: モデルのステア + 固定スロットル 0.1', result.stdout)
            self.assertNotIn('auto_fixed_throttle:=', result.stdout)
            command = [sys.executable, str(ROOT / 'scripts/lib/bringup.py'),
                       '--mode', 'e2e', '--evs', '--no-bridge', '--dry-run',
                       '--e2e-model-dir', str(model), '--evs-tensor-config', str(config)]
            overridden = subprocess.run(command+['--e2e-fixed-throttle', '0.12'], capture_output=True, text=True)
            self.assertEqual(overridden.returncode, 0, overridden.stderr)
            self.assertIn('e2e_fixed_throttle:=0.12', overridden.stdout)
            for value in ('nan', 'inf', '-0.1', '0.21'):
                invalid = subprocess.run(command+['--e2e-fixed-throttle', value], capture_output=True, text=True)
                self.assertNotEqual(invalid.returncode, 0)
            invalid = subprocess.run(command+['--auto-fixed-throttle', '0.1'], capture_output=True, text=True)
            self.assertNotEqual(invalid.returncode, 0)
            self.assertIn('--e2e-fixed-throttle', invalid.stderr)
