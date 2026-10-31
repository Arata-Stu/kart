import shutil
import tempfile
import unittest
from pathlib import Path
import yaml
from kart_bringup.configuration import load

class ConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'config'
        shutil.copytree(Path(__file__).parents[1] / 'config', self.root)
        self.path = self.root / 'bringup.yaml'

    def edit(self, relative, change):
        path = self.root / relative
        data = yaml.safe_load(path.read_text())
        change(data)
        path.write_text(yaml.safe_dump(data))

    def test_static_values_are_not_overridden_by_empty_arguments(self):
        self.edit('bringup.yaml', lambda d:d['launch'].update(container_name='test_container', enable_joy=False))
        self.edit('recording/bag_manager.yaml', lambda d:d['/**/kart_bag_manager']['ros__parameters'].update(output_dir='/data/my_bags'))
        settings, modules = load(self.path, {'container_name':'', 'record_dir':'', 'enable_joy':''})
        self.assertEqual(settings['container_name'],'test_container')
        self.assertFalse(settings['enable_joy'])
        self.assertEqual(modules['bag_manager']['effective']['output_dir'],'/data/my_bags')
        self.assertFalse(modules['bag_manager']['overrides'])

    def test_joy_limit_override(self):
        _, modules = load(self.path, {'joy_max_throttle': '0.5', 'auto_fixed_throttle': '0.4'})
        self.assertEqual(modules['joy_manager']['overrides']['speed_scale'], 0.5)
        self.assertEqual(modules['joy_manager']['effective']['speed_scale'], 0.5)
        for value in ('nan', 'inf', '0', '1.1', '-0.1', True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                load(self.path, {'joy_max_throttle': value})

    def test_explicit_overrides(self):
        settings, modules = load(self.path, {'container_name':'runtime', 'record_dir':'/data/run',
                                            'enable_bridge':'true', 'device':'/dev/test', 'namespace':'/kart'})
        self.assertEqual(settings['container_name'],'runtime')
        self.assertTrue(settings['enable_bridge'])
        self.assertEqual(modules['bridge']['effective']['device'],'/dev/test')
        self.assertEqual(modules['bag_manager']['effective']['output_dir'],'/data/run')
        self.assertEqual(modules['bag_manager']['overrides'],{'output_dir':'/data/run'})

    def test_recording_runtime_overrides(self):
        _, modules = load(self.path, {'run_name': 'trial', 'session_layout': 'false'})
        self.assertEqual(modules['bag_manager']['effective']['recording_name'], 'trial')
        self.assertFalse(modules['bag_manager']['effective']['session_layout'])
        with self.assertRaises(ValueError):
            load(self.path, {'session_layout': 'yes'})

    def test_device_from_yaml_is_used(self):
        self.edit('vehicle/bridge.yaml',lambda d:d['/**/kart_bridge']['ros__parameters'].update(device='/dev/configured'))
        _, modules = load(self.path,{'enable_bridge':'true','device':''})
        self.assertEqual(modules['bridge']['effective']['device'],'/dev/configured')

    def test_invalid_configuration(self):
        self.edit('vehicle/bridge.yaml', lambda d:d['/**/kart_bridge']['ros__parameters'].update(device=''))
        for overrides in ({'enable_bridge':'true'}, {'enable_bridge':'true','enable_control':'false','device':'/dev/test'},
                          {'enable_joy':'yes'}, {'unknown':'x'}):
            with self.assertRaises(ValueError): load(self.path,overrides)
        self.edit('bringup.yaml',lambda d:d['launch'].update(unknown=True))
        with self.assertRaises(ValueError): load(self.path)

    def test_no_cross_package_fallback(self):
        (self.root/'input/joy_manager.yaml').unlink()
        with self.assertRaises(FileNotFoundError): load(self.path)

    def test_manual_fixed_preserves_yaml_and_explicit_disable(self):
        self.edit('input/joy_manager.yaml', lambda d: d['/**/kart_joy_manager']['ros__parameters'].update(auto_fixed_throttle=0.1))
        _, modules = load(self.path, {'auto_fixed_throttle':''})
        self.assertEqual(modules['joy_manager']['effective']['auto_fixed_throttle'], 0.1)
        self.assertEqual(modules['joy_manager']['overrides'], {})
        _, modules = load(self.path, {'auto_fixed_throttle':'-1'})
        self.assertEqual(modules['joy_manager']['overrides']['auto_fixed_throttle'], -1.0)
        _, modules = load(self.path, {'auto_fixed_throttle':'0.20'})
        self.assertEqual(modules['joy_manager']['effective']['auto_fixed_throttle'], 0.2)
        self.assertTrue(modules['command_mux']['effective']['auto_use_teleop'])
        self.assertTrue(modules['command_mux']['overrides']['auto_use_teleop'])
        self.assertIsInstance(modules['joy_manager']['overrides']['auto_fixed_throttle'], float)

    def test_manual_fixed_invalid_values_fail_before_launch(self):
        for value in ('nan', 'inf', '-0.1', '0.31', 'not-a-number', True):
            with self.subTest(value=value), self.assertRaises((ValueError, TypeError)):
                load(self.path, {'auto_fixed_throttle':value})
        for key in ('enable_joy', 'enable_control'):
            with self.assertRaises(ValueError):
                load(self.path, {'auto_fixed_throttle':'0.1', key:'false'})
        self.edit('input/joy_manager.yaml', lambda d: d['/**/kart_joy_manager']['ros__parameters'].update(auto_fixed_throttle=0.4))
        with self.assertRaises(ValueError):
            load(self.path)

if __name__=='__main__': unittest.main()
