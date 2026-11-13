import math
from types import SimpleNamespace as NS
import unittest
from kart_racing.behavior_inputs import Motion, Observations, fresh, transform


def header(stamp,frame):
    sec,ns=divmod(stamp,1_000_000_000)
    return NS(stamp=NS(sec=sec,nanosec=ns),frame_id=frame)


def map_message(stamp,objects=(),valid=True):
    return NS(header=header(stamp,'map'),valid=valid,reason='projected' if valid else 'target_tf_unavailable',objects=list(objects))


def obj(kind,x,y=0.):
    return NS(class_name=kind,score=.9,point=NS(x=x,y=y,z=0.))


class InputTests(unittest.TestCase):
    def test_motion_uses_distinct_source_stamps_and_world_position(self):
        motion=Motion()
        for index in range(9):
            motion.add(10_000_000_000+index*50_000_000,2+index*.05,1.)
        vx,vy=motion.estimate()
        self.assertAlmostEqual(vx,1.)
        self.assertAlmostEqual(vy,0.)
        self.assertFalse(motion.add(10_400_000_000,99,99))
        self.assertAlmostEqual(motion.estimate()[0],1.)
        motion.add(1_000_000_000,1,1)
        self.assertIsNone(motion.estimate())

    def test_class_label_alone_does_not_confirm_static_obstacle(self):
        inputs=Observations()
        for i in range(9):
            inputs.map_points(map_message(1_000_000_000+i*50_000_000,[obj('obstacle',2)]),i*.05)
        rows=inputs.objects(.4,.4,5.)
        self.assertEqual(len(rows),1)
        self.assertTrue(rows[0]['confirmed'])
        self.assertAlmostEqual(rows[0]['speed'],0.)
        inputs.map_points(map_message(1_450_000_000,valid=False),.45)
        self.assertFalse(inputs.map_valid)
        # Invalid same-stamp tombstones cannot masquerade as an actual clear frame.
        inputs.map_points(map_message(1_450_000_000,valid=True),.46)
        self.assertFalse(inputs.map_valid)

    def test_untracked_rc_is_unknown_and_association_does_not_merge_two_boxes(self):
        inputs=Observations()
        inputs.map_points(map_message(1_000_000_000,[obj('rc_car',2),obj('obstacle',3),obj('obstacle',3.1)]),0.)
        rows=inputs.objects(0.,.4,5.)
        self.assertEqual(sum(row['kind']=='obstacle' for row in rows),2)
        self.assertEqual(sum(row['id'].startswith('untracked_rc') for row in rows),1)
        self.assertTrue(all(not row['confirmed'] for row in rows))

    def test_transform_and_freshness_are_bounded(self):
        tf=NS(translation=NS(x=3.,y=2.,z=0.),rotation=NS(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5)))
        x,y,z=transform((1.,0.,0.),tf)
        self.assertAlmostEqual(x,3.)
        self.assertAlmostEqual(y,3.)
        self.assertTrue(fresh(10_000_000_000,20,10_100_000_000,20.1,.4))
        self.assertFalse(fresh(10_000_000_000,20,10_500_000_000,20.1,.4))
        self.assertFalse(fresh(10_000_000_000,20,10_100_000_000,20.5,.4))


if __name__=='__main__':
    unittest.main()
