"""Human steering is teaching only in the intended driving mode of each bag."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS, ModuleType
import unittest
from unittest.mock import patch

from kart_e2e.teacher_selection import TeacherSelection, parse_config, AUTO_SCHEMA, LEGACY_SCHEMA

SECOND = 1_000_000_000


def config(**values):
    return dict(schema=AUTO_SCHEMA, enabled=True, armed=True, throttle=0.1,
                speed_scale=0.3, entry_neutral_s=0.3, mode=1, **values)


def command(**values):
    return NS(**dict(dict(steering=0.7, throttle=0.1, brake=0, reverse=0), **values))


def modes(mode=1):
    return [(SECOND+i*100_000_000, NS(mode=mode)) for i in range(6)]


class SelectionTests(unittest.TestCase):
    def test_bag_intent_includes_enabled_stop_and_excludes_all_manual_recovery(self):
        # Even an enabled STOP config later in the bag changes the whole bag's intent.
        row = dict(config(), mode=3, armed=False)
        select = TeacherSelection([(2*SECOND, row)], modes())
        self.assertEqual(select.metadata()['policy'], 'fixed_auto_only')
        self.assertIsNone(select.select(command(), NS(mode=2), SECOND))
        self.assertIsNone(select.select(command(), NS(mode=2), 3*SECOND))

    def test_fixed_only_fresh_causal_config_and_human_steering(self):
        select = TeacherSelection([(SECOND+350_000_000, config())], modes())
        stamp = SECOND+400_000_000
        self.assertEqual(select.select(command(), NS(mode=1), stamp), 'fixed_auto')
        self.assertIsNone(select.select(command(), NS(mode=1), stamp, SECOND-1))
        self.assertIsNone(select.select(command(), NS(mode=2), stamp))
        self.assertIsNone(select.select(command(throttle=0), NS(mode=1), stamp))
        self.assertIsNone(select.select(command(brake=1), NS(mode=1), stamp))
        self.assertIsNone(select.select(command(reverse=0.1), NS(mode=1), stamp))
        self.assertIsNone(select.select(command(), NS(mode=1), SECOND+340_000_000))
        self.assertIsNone(select.select(command(), NS(mode=1), 2*SECOND))
        for changes in ({'enabled': False, 'throttle': -1}, {'armed': False}, {'mode': 2}):
            row = dict(config(), **changes)
            select = TeacherSelection([(SECOND+350_000_000, config()), (stamp, row)], modes())
            self.assertIsNone(select.select(command(), NS(mode=1), stamp))

    def test_neutral_entry_zero_throttle_and_new_auto_entry(self):
        row = dict(config(), throttle=0)
        select = TeacherSelection([(SECOND, row)], modes())
        self.assertIsNone(select.select(command(throttle=0), NS(mode=1), SECOND+100_000_000))
        self.assertEqual(select.select(command(throttle=0), NS(mode=1), SECOND+300_000_000), 'fixed_auto')
        transitions = modes()+[(SECOND+550_000_000, NS(mode=2)), (SECOND+600_000_000, NS(mode=1))]
        select = TeacherSelection([(SECOND+600_000_000, config())], transitions)
        self.assertIsNone(select.select(command(), NS(mode=1), SECOND+600_000_000))

    def test_ordinary_and_legacy_manual_bags_keep_manual_policy(self):
        for configs in ([], [(SECOND, dict(config(), enabled=False, throttle=-1))],
                        [(SECOND, dict(config(), schema=LEGACY_SCHEMA, mode=2))]):
            select = TeacherSelection(configs, modes(2))
            self.assertEqual(select.select(command(), NS(mode=2), SECOND+400_000_000), 'manual')
            self.assertIsNone(select.select(command(), NS(mode=1), SECOND+400_000_000))
            self.assertEqual(select.metadata()['policy'], 'manual_only')

    def test_malformed_metadata_cannot_fall_back_to_manual(self):
        self.assertEqual(parse_config(json.dumps(config())), config())
        for changes in ({'schema': 'unknown'}, {'enabled': 1}, {'throttle': float('nan')},
                        {'throttle': .5}, {'armed': 'true'}, {'entry_neutral_s': -1}, {'mode': True}):
            with self.assertRaises(ValueError):
                parse_config(json.dumps(dict(config(), **changes)))

    def test_rgb_extraction_selects_only_fixed_auto_and_preserves_steering(self):
        from kart_e2e.preprocess_bag import main
        from kart_e2e.teacher_selection import read_fixed_configs
        cmd, mode, image, fixed = '/teleop/control_cmd', '/operation_mode/state', '/realsense/color/image_raw', '/teleop/fixed_config'
        topic_types = {cmd:'kart_interfaces/msg/ControlCommand', mode:'kart_interfaces/msg/OperationModeState',
                       image:'sensor_msgs/msg/Image', fixed:'std_msgs/msg/String'}
        rows = [(mode, NS(mode=1), t) for t, _ in modes()]
        stamp = SECOND+400_000_000
        rows += [(fixed, NS(data=json.dumps(config())), SECOND+350_000_000),
                 (cmd, command(), stamp), (image, NS(), stamp),
                 (mode, NS(mode=2), SECOND+500_000_001),
                 (cmd, command(steering=-0.9), SECOND+550_000_000),
                 (image, NS(), SECOND+550_000_000)]
        class Reader:
            def open(self, *args): pass
            def get_all_topics_and_types(self):
                return [NS(name=k, type=v) for k,v in topic_types.items()]
            def set_filter(self, value):
                self.rows = iter(sorted((r for r in rows if r[0] in value.topics), key=lambda r:r[2]))
                self.next = next(self.rows, None)
            def has_next(self): return self.next is not None
            def read_next(self):
                result = self.next
                self.next = next(self.rows, None)
                return result
        modules = {n:ModuleType(n) for n in ('rosbag2_py','rclpy.serialization','rosidl_runtime_py.utilities')}
        modules['rosbag2_py'].SequentialReader = Reader
        for key in ('StorageOptions', 'ConverterOptions', 'StorageFilter'):
            setattr(modules['rosbag2_py'], key, lambda *a, **kw:NS(**kw))
        modules['rclpy.serialization'].deserialize_message = lambda raw, _:raw
        modules['rosidl_runtime_py.utilities'].get_message = lambda _:None
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules, modules):
            target = Path(tmp)/'dataset'
            with patch.object(sys, 'argv', ['extract', 'bag', str(target)]), \
                    patch('kart_e2e.preprocess_bag.decode_image', return_value=NS(save=lambda _:None)), \
                    contextlib.redirect_stdout(io.StringIO()):
                main()
            samples = [json.loads(line) for line in (target/'samples.jsonl').read_text().splitlines()]
            self.assertEqual(len(samples), 1)
            self.assertEqual(samples[0]['steering'], .7)
            self.assertEqual(samples[0]['teacher_source'], 'fixed_auto')
            meta = json.loads((target/'metadata.json').read_text())
            self.assertEqual(meta['teacher_selection']['policy'], 'fixed_auto_only')
            del topic_types[fixed]
            self.assertEqual(read_fixed_configs('old-bag', fixed), [])
