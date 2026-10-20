import copy
import json
import tempfile
import unittest
from pathlib import Path

from kart_hdmap.tuning import StopGate, digest, map_identity, validate


def snapshot():
    result = dict(map_id='map-a', lane_id='a', line_type='customline', centerline_speed_mps=.3,
                  document=dict(schema='kart.hdmap.v2', frame='map', revision=1, lanes=[
                      dict(id='a', closed=False, left=[[0,1],[2,1]], right=[[0,-1],[2,-1]], lines={
                          'customline': dict(points=[[0,0],[2,0]], profile=[[0,0,0,0,0,.2,0],[2,2,0,0,0,0,0]])})]))
    result['revision'] = digest(result)
    return result


def resign(value):
    value['revision'] = digest({k:v for k,v in value.items() if k != 'revision'})
    return value


class TuningTests(unittest.TestCase):
    def test_stop_requires_continuous_fresh_samples_and_one_second(self):
        now = [0.]
        gate = StopGate(lambda: now[0])
        for i in range(12):
            now[0] = i * .1
            gate.update_mode(3)
            gate.update_speed(0.)
            if i < 10:
                self.assertTrue(gate.issue())
        self.assertEqual(gate.issue(), '')
        now[0] += .31
        self.assertTrue(gate.issue())
        gate.update_mode(3)
        now[0] += .3
        gate.update_mode(3)
        gate.update_speed(0.)
        self.assertTrue(gate.issue())  # receipt gap resets stopped interval
        gate.update_speed(.1)
        self.assertIsNone(gate.stopped_at)
        gate.update_speed(float('nan'))
        self.assertIsNone(gate.speed)
        gate.update_mode(1)
        self.assertTrue(gate.issue())

    def test_snapshot_map_revision_and_geometry_rejections(self):
        good = snapshot()
        document = validate(good, 'map-a')
        self.assertEqual(document['lanes'][0]['lines']['customline']['profile'][-1][5], 0)
        for change in ('map', 'hash', 'lane', 'line', 'profile', 'frame', 'speed', 'paths'):
            bad = copy.deepcopy(good)
            if change == 'map': bad['map_id'] = 'another'
            if change == 'hash': bad['revision'] = 'invalid'
            if change == 'lane': bad['lane_id'] = ''
            if change == 'line': bad['line_type'] = 'raceline'
            if change == 'profile': bad['document']['lanes'][0]['lines']['customline']['profile'][0][1] = 1
            if change == 'frame': bad['document']['frame'] = 'odom'
            if change == 'speed': bad['centerline_speed_mps'] = -1
            if change == 'paths': bad['document']['frame_id'] = 'map'
            if change != 'hash': resign(bad)
            with self.subTest(change=change), self.assertRaises((ValueError, KeyError)):
                validate(bad, 'map-a')

    def test_localization_identity_survives_copy_but_not_content_changes(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root/'snapshot.json').write_text('{}')
            first = map_identity(root)
            (root/'map.json').write_text('{"revision":2}')
            self.assertEqual(map_identity(root), first)  # tuning geometry is separate
            (root/'snapshot.json').write_text('{"frame":"map"}')
            self.assertNotEqual(map_identity(root), first)
            (root/'cuvslam_map').mkdir()
            (root/'cuvslam_map'/'link').symlink_to(root/'snapshot.json')
            with self.assertRaises(ValueError): map_identity(root)


class TransactionTests(unittest.TestCase):
    """Executor request contracts with injected state storage; no ROS emulation."""
    def setUp(self):
        from kart_hdmap.tuning_bridge import TuningBridge
        from types import SimpleNamespace
        from unittest.mock import Mock
        self.bridge = TuningBridge.__new__(TuningBridge)
        b = self.bridge
        b.identity = 'map-a'
        b.previous = None
        b.pose = None
        b.tracking = None
        b.localized = False
        b.localized_at = float('-inf')
        b.gate = StopGate()
        initial = validate(snapshot(), 'map-a')
        b.node = SimpleNamespace(document=initial, selected='a', line_type='customline',
                                 runtime_centerline_speed=.3, settings={'speed_override_mps':-1.0},
                                 count_publishers=lambda _:1)
        b.issue = Mock(return_value='')
        def store(candidate):
            n=b.node
            n.document=copy.deepcopy(candidate['document'])
            n.selected=candidate['lane_id']
            n.line_type=candidate['line_type']
            n.runtime_centerline_speed=candidate['centerline_speed_mps']
            n.settings['speed_override_mps']=candidate['speed_override_mps']
        b.replace = Mock(side_effect=store)

    def call(self, action, **body):
        import time
        return self.bridge.handle(action, {'map_id':'map-a', **body}, time.monotonic()+4)

    def test_atomic_apply_and_rollback_keep_complete_old_version(self):
        b=self.bridge
        original=copy.deepcopy(b.active())
        candidate=snapshot()
        candidate['centerline_speed_mps']=.5
        candidate['document']['lanes'][0]['lines']['customline']['profile'][0][5]=.4
        resign(candidate)
        first=self.call('/apply',snapshot=candidate,expected_revision=b.revision())
        self.assertNotIn('error',first)
        self.assertEqual(b.node.runtime_centerline_speed,.5)
        self.assertEqual(b.previous,original)
        second=self.call('/rollback',expected_revision=first['revision'])
        self.assertNotIn('error',second)
        self.assertEqual(b.active(),original)

    def test_stale_conflicting_or_expired_requests_never_mutate(self):
        import time
        b=self.bridge
        for body in ({'expected_revision':'stale'}, {'map_id':'other','expected_revision':b.revision()}):
            result=self.call('/apply',snapshot=snapshot(),**body)
            self.assertIn('error',result)
        result=b.handle('/apply',{},time.monotonic()-1)
        self.assertIn('error',result)
        b.issue.return_value='STOP required'
        self.assertIn('error',self.call('/apply',snapshot=snapshot(),expected_revision=b.revision()))
        b.replace.assert_not_called()

    def test_status_does_not_resend_map_and_rechecks_gate_after_validation(self):
        b=self.bridge
        result=self.call('/status',known_revision=b.revision())
        self.assertNotIn('active',result)
        self.assertLess(len(json.dumps(result)),600)
        b.issue.side_effect=['','aged stop']
        result=self.call('/apply',snapshot=snapshot(),expected_revision=b.revision())
        self.assertIn('error',result)
        b.replace.assert_not_called()


class TrackingReferenceTests(unittest.TestCase):
    def test_only_valid_finite_map_frame_and_fresh_reference_is_forwarded(self):
        from kart_hdmap.tuning_bridge import TuningBridge
        from types import SimpleNamespace as NS
        from unittest.mock import patch
        bridge = TuningBridge.__new__(TuningBridge)
        bridge.node = NS(settings={'frame_id':'map'})
        bridge.tracking = None
        bridge.fresh = lambda stamp, timeout: stamp == 'fresh'
        def message(**changes):
            values = dict(valid=True,header=NS(frame_id='map',stamp='fresh'),
                          vehicle_position=NS(x=1.,y=2.,z=0.),target=NS(x=1.8,y=2.,z=0.),
                          lookahead_m=.8,target_distance_m=.8)
            return NS(**(values | changes))
        with patch('kart_hdmap.tuning_bridge.time.monotonic',return_value=10):
            bridge.reference(message())
            view = bridge.tracking_status()
            self.assertEqual(view['target'],[1.8,2.])
            self.assertEqual(view['lookahead_m'],.8)
        with patch('kart_hdmap.tuning_bridge.time.monotonic',return_value=10.6):
            self.assertIsNone(bridge.tracking_status())
        for msg in (message(valid=False),message(header=NS(frame_id='odom',stamp='fresh')),
                    message(header=NS(frame_id='map',stamp='old')),message(lookahead_m=float('nan')),
                    message(target_distance_m=-1),message(lookahead_m=0)):
            with self.subTest(msg=msg):
                bridge.reference(msg)
                self.assertIsNone(bridge.tracking_status())
