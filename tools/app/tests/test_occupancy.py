import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from kart_studio.dense import overlay


class OccupancyTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.maps = SimpleNamespace(folder=lambda key: self.root)

    def prepare(self):
        (self.root / 'mapping_result.json').write_text(json.dumps({
            'dense': {'files': []}, 'official_output': 'official/run'}))
        run = self.root / 'official/run'
        run.mkdir(parents=True)
        (run / 'occupancy_map_before_adjustment.png').write_bytes(b'png')
        meta = run / 'occupancy_map_before_adjustment.yaml'
        meta.write_text('image: occupancy_map_before_adjustment.png\nresolution: 0.05\norigin: [1, 2, 0.3]\n')
        return run, meta

    def test_no_dense_and_no_untransformed_raster(self):
        self.assertFalse(overlay(self.maps, 'm')['available'])
        run, meta = self.prepare()
        meta.unlink()
        (run / 'occupancy_map.yaml').write_text('resolution: 1')
        self.assertFalse(overlay(self.maps, 'm')['available'])

    def test_origin_and_source(self):
        self.prepare()
        result = overlay(self.maps, 'm')
        self.assertEqual(result['origin'], [1, 2, 0.3])
        self.assertEqual(result['resolution'], 0.05)
        self.assertIn('before_adjustment.png', result['url'])

    def test_invalid_geometry_and_mismatched_image(self):
        run, meta = self.prepare()
        for content in ['image: other.png\nresolution: 1\norigin: [0,0,0]',
                        'image: occupancy_map_before_adjustment.png\nresolution: -1\norigin: [0,0,0]',
                        'image: occupancy_map_before_adjustment.png\nresolution: 1\norigin: [0,.nan,0]']:
            meta.write_text(content)
            with self.assertRaises(ValueError):
                overlay(self.maps, 'm')

    def test_symlink_not_served(self):
        run, meta = self.prepare()
        image = run / 'occupancy_map_before_adjustment.png'
        image.unlink()
        image.symlink_to(meta)
        self.assertFalse(overlay(self.maps, 'm')['available'])
