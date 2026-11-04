import unittest
from types import SimpleNamespace as NS
from kart_hdmap.timing_history import TimingHistory


def status(state='tracking', level=0, **values):
    return [NS(name='lateral_controller', hardware_id='kart_control', message=state,
               level=level, values=[NS(key=k, value=str(v)) for k,v in values.items()])]


class TimingHistoryTests(unittest.TestCase):
    def test_independent_fields_and_valid_negative_age(self):
        h=TimingHistory('lateral_controller','kart_control',('pose_age_ms',))
        h.capture(1,status(processing_ms=2,pose_age_ms=-1),True,now=0)
        h.capture(2,status(processing_ms='nan',pose_age_ms=20),True,now=1)
        self.assertEqual(h.samples[0]['pose_age_ms'],-1)
        self.assertIsNone(h.samples[1]['processing_ms'])
        self.assertEqual(h.samples[1]['pose_age_ms'],20)

    def test_invalid_stale_missing_and_reset(self):
        h=TimingHistory('lateral_controller','kart_control',('pose_age_ms',))
        h.capture(1,status(processing_ms=2,pose_age_ms=20),True,now=0)
        h.capture(2,status(level=2,processing_ms=-1,pose_age_ms=-1),True,now=.01)
        self.assertIsNone(h.samples[-1]['pose_age_ms'])
        self.assertIsNone(h.samples[-1]['processing_ms'])
        self.assertEqual(len(h.samples),2)  # validity change is not sampled away
        h.capture(3,status(processing_ms=2,pose_age_ms=20),False,now=1)
        self.assertIsNone(h.samples[-1]['pose_age_ms'])
        h.capture(1,status(),True,now=2)
        self.assertEqual(len(h.samples),1)
        self.assertIsNone(h.samples[-1]['pose_age_ms'])
        self.assertEqual(h.view(now=63)['samples'],[])

    def test_bounded_sampling_and_wrong_source(self):
        h=TimingHistory('lateral_controller','kart_control',('pose_age_ms',))
        for i in range(10000):
            h.capture(i,status(processing_ms=2,pose_age_ms=20),True,now=i/100)
        self.assertLessEqual(len(h.samples),650)
        self.assertTrue(all(r['t']>=40 for r in h.view(now=100)['samples']))
        before=len(h.samples)
        wrong=status();wrong[0].hardware_id='other'
        h.capture(20000,wrong,True,now=101)
        self.assertEqual(len(h.samples),before)
