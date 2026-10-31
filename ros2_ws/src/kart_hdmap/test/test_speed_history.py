import math
import unittest
from types import SimpleNamespace as NS
from kart_hdmap.speed_history import SpeedHistory


def status(target='2', measured='1', state='active'):
    return [NS(name='speed_controller', hardware_id='speed_controller', message=state,
               values=[NS(key='target_mps', value=target), NS(key='measured_mps', value=measured)])]


class SpeedHistoryTests(unittest.TestCase):
    def test_valid_invalid_inactive_and_stale(self):
        h=SpeedHistory()
        for i, (s,f) in enumerate([(status(),True),(status('nan'),True),(status(state='inactive'),True),(status(),False)]):
            h.capture(i+1,s,f,now=i)
        rows=h.view(now=4)['samples']
        self.assertEqual(rows[0]['target'],2)
        self.assertEqual(rows[0]['measured'],1)
        self.assertTrue(all(r['target'] is None and r['measured'] is None for r in rows[1:]))

    def test_bounded_age_clock_reset_and_wrong_source(self):
        h=SpeedHistory()
        for i in range(1000): h.capture(i+1,status(),True,now=i/10)
        self.assertLessEqual(len(h.samples),650)
        self.assertTrue(all(r['t']>=40 for r in h.view(now=100)['samples']))
        stream=h.stream
        h.capture(1,status(),True,now=101)
        self.assertNotEqual(h.stream,stream)
        self.assertEqual(len(h.samples),1)
        wrong=status();wrong[0].hardware_id='other'
        h.capture(2,wrong,True,now=102)
        self.assertEqual(len(h.samples),1)
        self.assertEqual(h.view(now=162)['samples'],[])
