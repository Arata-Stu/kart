import json
import unittest
from kart_hdmap.behavior_status import BehaviorStatus


def payload(**changes):
    return json.dumps(dict(mode='shadow',state='FOLLOW',frame_id='map',valid=True,reason='following_moving_rc',
                          target_speed=.4,stamp_ns='1000000000',published_at_ns='10000000000',
                          path=[[0.,0.],[1.,0.]],candidates=[],**changes))


class BehaviorStatusTests(unittest.TestCase):
    def test_diagnostic_freshness_uses_publication_clock_but_keeps_observation_stamp(self):
        status=BehaviorStatus()
        status.capture(payload(),now=20)
        value=status.view(True,10_100_000_000,now=20.1)
        self.assertTrue(value['fresh'])
        self.assertEqual(value['status']['stamp_ns'],'1000000000')
        self.assertAlmostEqual(value['fresh_for_s'],.4)
        self.assertFalse(status.view(True,10_600_000_000,now=20.1)['fresh'])
        self.assertFalse(status.view(True,10_100_000_000,now=20.6)['fresh'])
        self.assertFalse(status.view(False,10_100_000_000,now=20.1)['fresh'])

    def test_fresh_fault_remains_visible_without_replaying_a_previous_route(self):
        status=BehaviorStatus()
        status.capture(payload(),now=20)
        value=json.loads(payload())
        value.update(valid=False,state='FAULT',reason='perception_timeout',path=[])
        status.capture(json.dumps(value),now=20.1)
        result=status.view(True,10_100_000_000,now=20.1)
        self.assertTrue(result['fresh'])
        self.assertEqual(result['status']['path'],[])
        self.assertEqual(result['status']['reason'],'perception_timeout')

    def test_committed_pass_states_preserve_diagnostics_and_still_expire(self):
        for state, reason in (('COAST_PASS', 'committed_prediction'), ('HOLD_SIDE', 'merge_clearance_pending')):
            with self.subTest(state=state):
                status = BehaviorStatus()
                value = json.loads(payload())
                value.update(state=state, reason=reason)
                status.capture(json.dumps(value), now=20)
                result = status.view(True, 10_100_000_000, now=20.1)
                self.assertTrue(result['fresh'])
                self.assertEqual(result['status'], value)
                self.assertIsNone(status.view(True, 10_600_000_000, now=20.1)['status'])

    def test_invalid_payload_clears_previous_status_and_bounds_paths(self):
        status=BehaviorStatus()
        original=json.loads(payload())
        for change in ({'mode':'other'},{'state':'UNKNOWN'},{'frame_id':'base_link'},{'valid':1},{'target_speed':float('nan')},
                       {'path':[[0,0]]*301},{'published_at_ns':0},{'candidates':[dict(side='left')]*3}):
            status.capture(payload(),now=20)
            status.capture(json.dumps(dict(original,**change)),now=20.1)
            self.assertFalse(status.view(True,10_100_000_000,now=20.1)['fresh'])


if __name__=='__main__':
    unittest.main()
