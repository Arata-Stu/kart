"""Model selection and cached-engine metadata; no ROS or GPU runtime."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType
import unittest
from unittest.mock import patch

import yaml

from kart_bringup.racing_models import engine_profile, get_profile


class RacingModelTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.engine = self.root / 'test.plan'
        self.engine.write_bytes(b'profile fixture, not a TensorRT engine')

    def manifest(self, profile):
        value = dict(model_profile=profile, num_classes=get_profile(profile)['num_classes'],
                     engine_sha256=hashlib.sha256(self.engine.read_bytes()).hexdigest())
        (self.root / 'build.json').write_text(json.dumps(value))
        return value

    def test_official_classes_are_distinct_from_opponent_labels(self):
        coco, opponent = get_profile('coco'), get_profile('opponent')
        self.assertEqual(coco['num_classes'], 80)
        self.assertEqual(len(set(coco['class_names'])), 80)
        self.assertEqual(coco['class_names'][0], 'person')
        self.assertEqual(coco['class_names'][int(coco['track_class_id'])], 'car')
        self.assertEqual(opponent['class_names'], ['opponent'])
        self.assertEqual(opponent['track_class_id'], '0')
        coco['class_names'][0] = 'edited'
        self.assertEqual(get_profile('coco')['class_names'][0], 'person')

    def test_manifest_selects_each_model_without_manual_launch_flags(self):
        for name in ('coco', 'opponent'):
            with self.subTest(name=name):
                self.manifest(name)
                self.assertEqual(engine_profile(self.engine), get_profile(name))

    def test_external_engine_defaults_to_coco_or_explicit_opponent(self):
        self.assertEqual(engine_profile(self.engine)['name'], 'coco')
        self.assertEqual(engine_profile(self.engine, 'opponent')['name'], 'opponent')

    def test_conflicting_profile_and_class_count_fail_before_launch(self):
        value = self.manifest('coco')
        with self.assertRaisesRegex(ValueError, 'profiles differ'):
            engine_profile(self.engine, 'opponent')
        value['num_classes'] = 1
        (self.root / 'build.json').write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, 'class count'):
            engine_profile(self.engine)

    def test_changed_engine_cannot_reuse_profile_metadata(self):
        self.manifest('coco')
        self.engine.write_bytes(b'different engine')
        with self.assertRaisesRegex(ValueError, 'hash'):
            engine_profile(self.engine)

    def test_legacy_manifest_requires_explicit_profile_or_rebuild(self):
        value = self.manifest('opponent')
        del value['model_profile']
        del value['num_classes']
        (self.root / 'build.json').write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, 'Legacy'):
            engine_profile(self.engine)
        self.assertEqual(engine_profile(self.engine, 'opponent')['num_classes'], 1)

    def test_invalid_profile_and_manifest_fail(self):
        with self.assertRaises(ValueError):
            get_profile('yolov8-seg')
        for value in ('[]', '{', '{"model_profile":"unknown"}'):
            with self.subTest(value=value):
                (self.root / 'build.json').write_text(value)
                with self.assertRaises(ValueError):
                    engine_profile(self.engine)

    def test_operational_defaults_match_official_model(self):
        root = Path(__file__).resolve().parents[1] / 'config/racing'
        decoder = yaml.safe_load((root / 'perception.yaml').read_text())['decoder']
        tracker = yaml.safe_load((root / 'tracker.yaml').read_text())['/**/opponent_tracker']['ros__parameters']
        self.assertEqual(decoder['num_classes'], get_profile('coco')['num_classes'])
        self.assertEqual(tracker['tracked_class_id'], get_profile('coco')['track_class_id'])


class PerceptionLaunchTests(unittest.TestCase):
    """Inspect actual launch parameter wiring with ROS launch actions stubbed."""
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.root / 'config/racing'
        self.config.mkdir(parents=True)
        package = Path(__file__).resolve().parents[1]
        for filename in ('perception.yaml', 'tracker.yaml', 'debug_image.yaml'):
            (self.config / filename).write_text((package / 'config/racing' / filename).read_text())
        self.engine = self.root / 'yolov8s.plan'
        self.engine.write_bytes(b'launch fixture, not a TensorRT engine')
        self.launch_file = package / 'launch/racing_perception.launch.py'

    def pipeline(self, override, debug='false'):
        modules = {name: ModuleType(name) for name in (
            'ament_index_python', 'ament_index_python.packages', 'launch', 'launch.actions',
            'launch.substitutions', 'launch_ros', 'launch_ros.actions', 'launch_ros.descriptions',
            'launch_ros.parameter_descriptions')}
        modules['ament_index_python.packages'].get_package_share_directory = lambda _name: str(self.root)
        modules['launch'].LaunchDescription = lambda values: values
        actions = modules['launch.actions']
        actions.DeclareLaunchArgument = lambda *args, **kwargs: (args, kwargs)
        actions.LogInfo = lambda **kwargs: kwargs
        actions.OpaqueFunction = lambda **kwargs: kwargs
        class Configuration:
            def __init__(self, name): self.name = name
            def perform(self, context): return context[self.name]
        class Value:
            def __init__(self, value, value_type):
                self.value, self.value_type = value, value_type
        modules['launch.substitutions'].LaunchConfiguration = Configuration
        modules['launch_ros.actions'].ComposableNodeContainer = lambda **kwargs: kwargs
        modules['launch_ros.actions'].LoadComposableNodes = lambda **kwargs: kwargs
        modules['launch_ros.actions'].Node = lambda **kwargs: kwargs
        modules['launch_ros.descriptions'].ComposableNode = lambda **kwargs: kwargs
        modules['launch_ros.parameter_descriptions'].ParameterValue = Value
        context = dict(engine=str(self.engine), model_profile='', tracking_enabled=override,
                       image_topic='/camera/rgb', target_container='', debug_image=debug)
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('racing_mode_launch_test', self.launch_file)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self.actions = module.pipeline(context)
            container = self.actions[1]
        return {node['name']: node['parameters'][0] for node in container['composable_node_descriptions']}

    def test_debug_image_only_starts_when_enabled_and_uses_same_image_source(self):
        self.pipeline('true')
        self.assertEqual(len(self.actions), 2)
        self.pipeline('true', debug='true')
        self.assertEqual(len(self.actions), 3)
        preview = self.actions[2]
        self.assertEqual(preview['executable'], 'opponent_debug_image')
        self.assertEqual(preview['namespace'], '/racing')
        self.assertEqual(preview['remappings'], [('rgb', '/camera/rgb')])
        self.assertEqual(preview['parameters'][1]['class_names'], get_profile('coco')['class_names'])

    def test_explicit_modes_are_ros_bools_and_keep_all_class_detections(self):
        for raw, expected in (('false', False), ('true', True)):
            with self.subTest(selection=raw):
                nodes = self.pipeline(raw)
                tracking = nodes['opponent_tracker']['tracking_enabled']
                self.assertIs(tracking.value, expected)
                self.assertIs(tracking.value_type, bool)
                self.assertEqual(nodes['decoder']['num_classes'], 80)
                selected = nodes['opponent_tracker']['tracked_class_id']
                self.assertEqual(selected.value, '2')
                self.assertIs(selected.value_type, str)
        with self.assertRaisesRegex(ValueError, 'true/false'):
            self.pipeline('0')

    def test_unspecified_tracking_preserves_tracker_yaml(self):
        for configured in (False, True):
            data = yaml.safe_load((self.config / 'tracker.yaml').read_text())
            data['/**/opponent_tracker']['ros__parameters']['tracking_enabled'] = configured
            (self.config / 'tracker.yaml').write_text(yaml.safe_dump(data))
            self.assertIs(self.pipeline('')['opponent_tracker']['tracking_enabled'], configured)


if __name__ == '__main__':
    unittest.main()
