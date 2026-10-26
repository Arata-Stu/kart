"""Portable bringup selection and content-pairing checks; no Jetson access."""
import contextlib
import importlib.util
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from kart_bringup.studio_tuning import map_root

ROOT = Path(__file__).resolve().parents[4]


class TuningSelectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.bundle = self.root / 'course'
        self.hdmap = self.bundle / 'export/hd_map.yaml'
        self.hdmap.parent.mkdir(parents=True)
        self.hdmap.write_text('{}')
        (self.bundle / 'cuvslam_map').mkdir()
        (self.bundle / 'cuvslam_map/map.mdb').write_bytes(b'same-map')
        self.vgl = self.bundle / 'vgl/gpu'
        (self.vgl / 'cuvslam_map').mkdir(parents=True)
        (self.vgl / 'cuvslam_map/map.mdb').write_bytes(b'same-map')

    def test_selected_nested_vgl_bundle_reuses_hdmap_root_by_content(self):
        self.assertEqual(map_root(self.hdmap, self.vgl), str(self.bundle))
        self.assertEqual(map_root(self.hdmap, self.bundle/'cuvslam_map'), str(self.bundle))
        (self.vgl / 'cuvslam_map/map.mdb').write_bytes(b'another-map')
        with self.assertRaisesRegex(ValueError, '一致'):
            map_root(self.hdmap, self.vgl)

    def test_tui_and_cli_forward_selection_without_another_map_prompt(self):
        spec = importlib.util.spec_from_file_location('kart_bringup_tuning_cli', ROOT/'scripts/lib/bringup.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        base = ['bringup.py', '--dry-run', '--no-bridge', '--no-evs', '--foxglove',
                '--rgb-profile', '424x240x30', '--infra-profile', '424x240x60',
                '--imu-profile', 'high', '--multicam-mode', '0', '--recovery', 'off', '--auto-fixed-throttle', '-1',
                '--run-name', 'tuning', '--map-file', str(self.hdmap), '--map-dir', str(self.vgl),
                '--model-dir', str(self.root/'models')]
        for enabled, interactive in ((True, True), (False, True), (True, False), (False, False)):
            prompts = []
            def choose(title, values, *args, **kwargs):
                prompts.append(title)
                if title == '用途': return 'drive'
                if title == 'lane / 追従ライン': return ('main', 'customline')
                if title == 'Map Studio 動的調整': return enabled
                self.fail('Unexpected prompt: '+title)
            argv = base if interactive else base + ['--mode', 'drive', '--lane-id', 'main', '--line-type', 'customline', '--tuning' if enabled else '--no-tuning']
            output = io.StringIO()
            with self.subTest(enabled=enabled, interactive=interactive), \
                 patch.object(sys, 'argv', argv), patch.object(sys.stdin, 'isatty', return_value=True), \
                 patch.object(module, 'choose', side_effect=choose), \
                 patch('kart_bringup.mission.hdmap_choices', return_value=[('main','customline')]), \
                 patch('kart_bringup.localization.resolve', return_value=({'cuvslam': {'tracking_mode': 0}}, {}, {}, {'input_shape':[1,3,240,424]})), \
                 contextlib.redirect_stdout(output):
                module.main()
            self.assertIn('tuning_enabled:='+str(enabled).lower(), output.getvalue())
            self.assertEqual(prompts.count('Map Studio 動的調整'), int(interactive))
            self.assertNotIn('HDMap',prompts)
            if enabled: self.assertIn('tuning_map_dir:='+str(self.bundle),output.getvalue())
            else: self.assertNotIn('tuning_map_dir:=',output.getvalue())

    def test_tuning_is_drive_only(self):
        import subprocess
        result = subprocess.run([sys.executable, str(ROOT/'scripts/lib/bringup.py'), '--mode','collect','--tuning','--dry-run'], capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('--tuning',result.stderr)
