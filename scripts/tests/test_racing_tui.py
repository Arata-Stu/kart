"""Official, two-class and legacy model caches are selectable in bringup."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/lib'))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/kart_bringup'))
from bringup_tui import Editor
from bringup import argument_parser

CONFIG = ROOT / 'ros2_ws/src/kart_bringup/config'


class RacingMenuTests(unittest.TestCase):
    def test_bbox_jpeg_is_opt_in_and_does_not_change_detection_or_tracking(self):
        args = argument_parser().parse_args(['--mode', 'drive', '--perception', '/models/yolov8s.plan',
                                             '--perception-mode', 'track'])
        def choose(title, values, label, default):
            self.assertEqual(values[default], False)
            return True
        editor = Editor(args, CONFIG, ROOT, choose)
        self.assertIn('無効', editor.rows()['perception_debug_image'])
        editor.edit('perception_debug_image')
        self.assertTrue(args.perception_debug_image)
        self.assertIn('有効', editor.rows()['perception_debug_image'])
        self.assertEqual(args.perception, '/models/yolov8s.plan')
        self.assertEqual(args.perception_mode, 'track')

    def test_drive_mode_can_switch_detection_and_car_tracking_without_changing_engine(self):
        args = argument_parser().parse_args(['--mode', 'drive', '--perception', '/models/yolov8s.plan',
                                             '--perception-mode', 'detect'])
        choices = []
        def choose(title, values, label, default):
            choices.append((title, list(values), values[default]))
            self.assertIn('車両追跡', label('track'))
            return 'track'
        editor = Editor(args, CONFIG, ROOT, choose)
        self.assertIn('物体検出のみ', editor.rows()['perception_mode'])
        editor.edit('perception_mode')
        self.assertEqual(args.perception_mode, 'track')
        self.assertEqual(args.perception, '/models/yolov8s.plan')
        self.assertIn('車両追跡', editor.rows()['perception_mode'])
        self.assertEqual(choices, [('物体認識モード', ['detect', 'track'], 'detect')])

    def test_coco_custom_and_legacy_engines_are_discovered_without_staging(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            cache = root / 'weights/racing/yolov8s/engines'
            relative = ('coco/gpu-11/yolov8s.plan', 'opponent/gpu-11/opponent.plan',
                        'old-gpu-10/opponent.plan', 'coco/gpu-11/.staging/yolov8s.plan',
                        'coco/gpu-11/empty.plan')
            for name in relative:
                path = cache / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b'' if name.endswith('empty.plan') else b'fixture')
            tuned = root / 'weights/racing/yolov8s/finetuned/model-a/engines/racing/gpu-11/racing.plan'
            tuned.parent.mkdir(parents=True)
            tuned.write_bytes(b'fine-tuned engine fixture')
            target = str(cache / relative[0])
            args = SimpleNamespace(perception='', record_profile='')

            def choose(title, values, label, default):
                self.assertEqual(title, '物体検出用engine')
                self.assertEqual(set(values), {'off', 'custom', str(tuned), *(str(cache / name) for name in relative[:3])})
                self.assertEqual(values[default], 'off')
                return target

            editor = Editor(args, root, root, choose)
            editor.edit('perception')
            self.assertEqual(args.perception, target)


if __name__ == '__main__':
    unittest.main()
