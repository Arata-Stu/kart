import json
from pathlib import Path
import tempfile
import unittest
from urllib.request import Request, urlopen
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from kart_sim.core import Simulation
from kart_sim.session import SimSession
from kart_sim.monitor import SensorMonitor
from kart_sim.sensors import SensorFrame

PACKAGE = Path(__file__).resolve().parents[1]
MAPS = PACKAGE.parents[2]/'maps/sim'


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.sim = Simulation(MAPS/'minicar_2026.json', PACKAGE/'assets', camera=True)
        self.addCleanup(self.sim.close)
        self.session = SimSession(self.sim, self.tmp.name)
        self.addCleanup(self.session.close)

    def samples(self):
        frames = {}
        for name in self.sim.rig.offsets:
            image = np.zeros((240, 424) if name.startswith('infra') else (240, 424, 3), np.uint8)
            frames[name] = SensorFrame(float(self.sim.data.time), self.sim.rig.frames[name],
                {'image': image, 'encoding': 'mono8' if name.startswith('infra') else 'rgb8', 'K': self.sim.rig.K[name]})
        return {'images': [frames], 'imu': [self.sim.imu_state()]}

    def test_bag_roundtrip_and_reset_guard(self):
        self.session.request('record_start'); self.session.drain(self.sim)
        self.assertTrue(self.session.recorder.writer)
        self.sim.step(steps=20)
        self.session.append(self.sim, self.samples())
        self.session.request('reset'); self.session.drain(self.sim)
        self.assertGreater(self.sim.data.time, 0)
        self.assertIn('Stop recording', self.session.error)
        self.session.request('record_stop'); self.session.drain(self.sim)
        self.assertTrue(self.session.recorder.status()['finalized'])
        first = self.session.recorder.path
        store = get_typestore(Stores.ROS2_HUMBLE)
        with Reader(first) as reader:
            messages = {c.topic: store.deserialize_cdr(raw, c.msgtype) for c, _, raw in reader.messages()}
            left = messages['/realsense/infra1/image_rect_raw']
            right = messages['/realsense/infra2/image_rect_raw']
            self.assertEqual(left.header.stamp, right.header.stamp)
            self.assertEqual((left.width, left.height, left.encoding), (424, 240, 'mono8'))
            np.testing.assert_allclose(messages['/realsense/infra2/camera_info'].p.reshape(3, 4), self.sim.rig.projection('infra2'))
            static = messages['/tf_static']
            self.assertTrue(any(t.child_frame_id == 'camera_infra1_optical_frame' for t in static.transforms))
            self.assertFalse(any(t.header.frame_id in ('map', 'odom') or t.child_frame_id in ('map', 'odom') for t in static.transforms))
            connection = next(c for c in reader.connections if c.topic == '/tf_static')
            self.assertEqual(connection.ext.offered_qos_profiles[0].durability.name, 'TRANSIENT_LOCAL')
            self.assertIn('/sim/ground_truth/pose', messages)
            self.assertIn('/realsense/imu', messages)
        self.session.request('reset'); self.session.drain(self.sim)
        self.assertEqual(self.sim.data.time, 0)
        self.session.request('record_start'); self.session.drain(self.sim)
        self.assertNotEqual(first, self.session.recorder.path)
        self.session.close()
        self.assertTrue((Path(self.session.recorder.path)/'metadata.yaml').is_file())

    def test_http_queues_controls_and_rejects_foreign_origin(self):
        monitor = SensorMonitor(self.session)
        self.addCleanup(monitor.close)
        url = f'http://127.0.0.1:{monitor.server.server_port}'
        request = Request(url+'/command', data=json.dumps({'action': 'auto_start'}).encode(), headers={'Content-Type': 'application/json'})
        with urlopen(request) as response:
            self.assertEqual(response.status, 202)
        self.assertFalse(self.session.controller.enabled)
        self.session.drain(self.sim)
        self.assertTrue(self.session.controller.enabled)
        monitor.update(self.sim)
        with urlopen(url+'/state') as response:
            self.assertTrue(json.load(response)['autodrive']['enabled'])
        request.add_header('Origin', 'http://example.com')
        from urllib.error import HTTPError
        with self.assertRaises(HTTPError) as caught:
            urlopen(request)
        self.assertEqual(caught.exception.code, 403)

    def test_stalled_controller_stops(self):
        self.session.request('auto_start'); self.session.drain(self.sim)
        self.sim.data.time = 9
        self.assertEqual(self.session.output(self.sim), (0., 0., 1., 0.))
        self.assertFalse(self.session.controller.enabled)

    def test_two_laps_without_wall_collision(self):
        self.sim.step(steps=1000)
        self.session.request('auto_start'); self.session.drain(self.sim)
        wall_contacts = set()
        for _ in range(14000):
            self.sim.step(*self.session.output(self.sim), steps=20)
            for contact in self.sim.data.contact:
                for geom in (contact.geom1, contact.geom2):
                    name = self.sim.model.geom(geom).name or ''
                    if name.startswith(('barrier_', 'black_curtain_', 'room_wall_', 'gate_post_', 'joint_post_')):
                        wall_contacts.add(name)
            if not self.session.controller.enabled or self.session.controller.laps >= 2:
                break
        self.assertTrue(self.session.controller.enabled, self.session.controller.error)
        self.assertEqual(self.session.controller.laps, 2)
        self.assertFalse(wall_contacts, wall_contacts)


if __name__ == '__main__':
    unittest.main()
