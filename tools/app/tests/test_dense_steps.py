import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from kart_studio import dense_steps
from kart_studio.storage import atomic_json, read_json
from kart_studio.jobs import Cancelled


class DenseStepsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.run = self.root / 'official/run'
        rect = self.run / 'map_frames/rectified'
        rect.mkdir(parents=True)
        (rect / 'left.png').write_bytes(b'image')
        atomic_json(rect / 'frames_meta.json', {'image': '/old/map_frames/rectified/left.png'})
        self.mapping = {'official_output': 'official/run', 'steps': ['edex', 'compute_poses']}
        atomic_json(self.root / 'mapping_result.json', self.mapping)
        atomic_json(self.root / 'map.json', {'revision': 1, 'lanes': ['keep'], 'mapping': self.mapping})
        self.maps = SimpleNamespace(folder=lambda key: self.root, load=lambda key: read_json(self.root / 'map.json'))
        self.studio = SimpleNamespace(maps=self.maps, repo=self.root,
                                      mapping=SimpleNamespace(defaults={'ros_domain_id': 92}))

    def job(self, step, fail=False):
        def execute(command, **kwargs):
            self.assertEqual(command[command.index('--steps_to_run') + 1], step)
            self.assertNotIn('compute_poses', command)
            target = Path(next(v.split('=', 1)[1] for v in command if v.startswith('--map_dir=')))
            meta = read_json(target / 'map_frames/rectified/frames_meta.json')
            self.assertEqual(meta['image'], str((target / 'map_frames/rectified/left.png').resolve()))
            if fail:
                raise Cancelled('cancel')
            if step == 'depth':
                depth = target / 'map_frames/depth'
                depth.mkdir()
                (depth / 'left.png').write_bytes(b'depth')
            else:
                self.assertTrue((target / 'map_frames/depth/left.png').exists())
                (target / 'mesh.ply').write_bytes(b'ply\nelement vertex 3\nelement face 1\nend_header\n')
                for name in ['occupancy_map.png', 'occupancy_map.yaml', 'ground_plane.yaml']:
                    (target / name).write_bytes(b'output')
        return SimpleNamespace(check=lambda: None, log=lambda msg: None, run=execute)

    def test_stages_and_preserved_hdmap(self):
        self.assertTrue(dense_steps.status(self.maps, 'm')['can_depth'])
        self.assertFalse(dense_steps.status(self.maps, 'm')['can_occupancy'])
        with patch('kart_studio.dense_steps.require_engine'):
            dense_steps.build(self.studio, self.job('depth'), 'm', 1, 'depth', 'low_res')
        self.assertTrue(dense_steps.status(self.maps, 'm')['can_occupancy'])
        dense_steps.build(self.studio, self.job('occupancy'), 'm', 2, 'occupancy', 'low_res')
        doc = self.maps.load('m')
        self.assertEqual(doc['lanes'], ['keep'])
        self.assertEqual(doc['revision'], 3)
        self.assertEqual(doc['mapping']['official_output'], 'official/run')
        self.assertTrue(doc['mapping']['dense_output'].startswith('dense_runs/'))
        self.assertFalse((self.run / 'map_frames/depth').exists())

    def test_cancel_preserves_manifest_and_cleans_copy(self):
        before = (self.root / 'mapping_result.json').read_bytes()
        with patch('kart_studio.dense_steps.require_engine'), self.assertRaises(Cancelled):
            dense_steps.build(self.studio, self.job('depth', fail=True), 'm', 1, 'depth', 'low_res')
        self.assertEqual(before, (self.root / 'mapping_result.json').read_bytes())
        self.assertEqual(list((self.root / 'dense_runs').iterdir()), [])

    def test_missing_inputs_disable_stage(self):
        (self.run / 'map_frames/rectified/frames_meta.json').unlink()
        result = dense_steps.status(self.maps, 'm')
        self.assertFalse(result['can_depth'])
        self.assertFalse(result['can_occupancy'])

    def test_stale_revision_refused(self):
        with self.assertRaises(ValueError):
            dense_steps.build(self.studio, self.job('depth'), 'm', 0, 'depth', 'low_res')
