import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from kart_mapping.build_map import run
from kart_mapping.dense_outputs import collect, options
from kart_studio import dense
from kart_studio.storage import atomic_json


class DenseMappingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.run_dir = self.root / "official/run"
        self.run_dir.mkdir(parents=True)

    def artifacts(self):
        (self.run_dir / "mesh.ply").write_bytes(b"ply\nformat ascii 1.0\nelement vertex 3\nelement face 1\nend_header\n0 0 0\n1 0 0\n0 1 0\n3 0 1 2\n")
        for name in ("occupancy_map.png", "occupancy_map.yaml", "ground_plane.yaml"):
            (self.run_dir / name).write_bytes(b"data")

    def test_output_summary_and_download_confinement(self):
        self.artifacts()
        result = collect(self.run_dir)
        self.assertEqual((result['vertices'], result['faces']), (3, 1))
        atomic_json(self.root / 'mapping_result.json', dict(dense=result, official_output='official/run'))
        maps = SimpleNamespace(folder=lambda key: self.root)
        self.assertEqual(len(dense.catalog(maps, 'course')['files']), 4)
        self.assertEqual(dense.artifact(maps, 'course', 'mesh.ply'), self.run_dir.resolve() / 'mesh.ply')
        with self.assertRaises(ValueError):
            dense.artifact(maps, 'course', '../job.json')
        (self.run_dir / 'mesh.ply').unlink()
        (self.run_dir / 'mesh.ply').symlink_to(self.root / 'mapping_result.json')
        with self.assertRaises(ValueError):
            dense.artifact(maps, 'course', 'mesh.ply')

    def test_http_mesh_download_and_traversal_rejected(self):
        import threading
        from http.server import ThreadingHTTPServer
        from urllib.request import urlopen
        from urllib.error import HTTPError
        from kart_studio.server import Handler
        self.artifacts()
        atomic_json(self.root / 'mapping_result.json', dict(dense=collect(self.run_dir), official_output='official/run'))
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        server.studio = SimpleNamespace(maps=SimpleNamespace(folder=lambda key: self.root))
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            base = f'http://127.0.0.1:{server.server_port}/dense-file?id=course&file='
            with urlopen(base + 'mesh.ply') as response:
                self.assertIn('attachment', response.headers['Content-Disposition'])
                self.assertEqual(response.read(), (self.run_dir / 'mesh.ply').read_bytes())
            with self.assertRaises(HTTPError) as caught:
                urlopen(base + '../mapping_result.json')
            self.assertEqual(caught.exception.code, 400)
            caught.exception.close()
        finally:
            server.shutdown()
            server.server_close()
            worker.join()

    def test_missing_and_empty_mesh_fail(self):
        with self.assertRaises(ValueError):
            collect(self.run_dir)
        self.artifacts()
        (self.run_dir / 'mesh.ply').write_bytes(b'ply\nelement vertex 0\nelement face 0\nend_header\n')
        with self.assertRaises(ValueError):
            collect(self.run_dir)

    def test_options_strict_and_legacy_default(self):
        self.assertEqual(options({}), (False, 'low_res'))
        for data in ({'dense_reconstruction': 'true'}, {'fs_model_res': 'invalid'}):
            with self.assertRaises(ValueError):
                options(data)

    def test_dense_official_command_and_manifest(self):
        repo = Path(__file__).resolve().parents[3]
        workflow = json.loads((repo / 'ros2_ws/src/kart_bringup/config/mapping/workflow.json').read_text())
        workflow.update(dense_reconstruction=True, fs_model_res='high_res')
        output = self.root / 'output'
        output.mkdir()
        bag = self.root / 'bag'
        bag.mkdir()
        (bag / 'metadata.yaml').write_text('bag')
        self.run_dir = output / 'official/run'
        def execute(command, **kwargs):
            if command[0] == 'ros2':
                self.assertIn('--depth_model=foundationstereo', command)
                self.assertIn('--fs_model_res=high_res', command)
                self.assertEqual(command[-5:], ['--steps_to_run', 'edex', 'compute_poses', 'depth', 'occupancy'])
                (self.run_dir / 'cuvslam_map').mkdir(parents=True)
                (self.run_dir / 'cuvslam_map/map.mdb').write_bytes(b'map')
                self.artifacts()
            return SimpleNamespace(stdout='5.0', returncode=0)
        with patch.dict(os.environ, ROS_DOMAIN_ID='92'), patch.dict('sys.modules', {'kart_mapping.bag_inputs': SimpleNamespace(inspect_bag=lambda *args: None)}), patch('kart_mapping.build_map.subprocess.run', side_effect=execute):
            run(dict(workflow=workflow, bag=str(bag), output=str(output)))
        result = json.loads((output / 'mapping_result.json').read_text())
        self.assertEqual(result['dense']['faces'], 1)
        self.assertEqual(result['steps'][-2:], ['depth', 'occupancy'])
        self.assertTrue((output / 'cuvslam_map/map.mdb').is_file())


if __name__ == '__main__':
    unittest.main()
