import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from kart_studio.dense import mesh_overlay
from kart_studio.storage import atomic_json


class MeshOverlayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.run = self.root / 'dense_runs/one'
        self.run.mkdir(parents=True)
        self.mapping = dict(backend='nvidia.create_map_offline', official_output='official/run',
                            dense_output='dense_runs/one', dense={'files': []})
        atomic_json(self.root / 'mapping_result.json', self.mapping)
        self.doc = {'frame': 'map'}
        self.maps = SimpleNamespace(folder=lambda key: self.root, load=lambda key: self.doc)
        self.mesh = self.run / 'mesh_before_adjustment.ply'
        self.mesh.write_bytes(b'ply\nformat ascii 1.0\nelement vertex 3\nelement face 1\nend_header\n')

    def test_original_frame_and_versioned_output(self):
        result = mesh_overlay(self.maps, 'm')
        self.assertEqual(result['frame'], 'map')
        self.assertEqual(result['source'], 'before_adjustment')
        self.assertEqual(result['face'], 1)
        self.assertIn('mesh_before_adjustment.ply', result['url'])

    def test_never_fallback_to_ground_mesh(self):
        self.mesh.rename(self.run / 'mesh.ply')
        with self.assertRaises(ValueError): mesh_overlay(self.maps, 'm')

    def test_unknown_frame_and_origin_rejected(self):
        self.doc['frame'] = 'odom'
        with self.assertRaises(ValueError): mesh_overlay(self.maps, 'm')
        self.doc['frame'] = 'map'
        self.mapping['backend'] = 'unknown'
        atomic_json(self.root / 'mapping_result.json', self.mapping)
        with self.assertRaises(ValueError): mesh_overlay(self.maps, 'm')

    def test_empty_and_symlink_rejected(self):
        self.mesh.write_bytes(b'ply\nelement vertex 0\nelement face 0\nend_header\n')
        with self.assertRaises(ValueError): mesh_overlay(self.maps, 'm')
        self.mesh.unlink()
        self.mesh.symlink_to(self.root / 'mapping_result.json')
        with self.assertRaises(ValueError): mesh_overlay(self.maps, 'm')
