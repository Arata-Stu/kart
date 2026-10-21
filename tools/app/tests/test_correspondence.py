"""Regression for unequal boundary lengths in concave closed courses."""
import tempfile
import time
import unittest
from pathlib import Path
from kart_studio.geometry import Corridor, PathValidationError, centerline
from kart_studio.lines import generate
from kart_studio.jobs import Jobs, Job
from kart_studio.job_worker import perform
from kart_studio.storage import read_json

left=[[.75,.3],[.8,2.7],[1,3.3],[1.4,3.35],[3.4,3.2],[5.5,3.2],[7,3.5],[8.6,3.55],[8.75,3.3],[8.77,3],[8.4,2.9],[2.1,3.05],[1.6,2.5],[1.55,1.7],[1.9,1],[2.6,.7],[7.9,.55],[8.4,.4],[8.6,.15],[8.55,0],[8.1,-.1],[3.6,-.1],[1.4,.05]]
right=[[0,0],[0,3],[.4,4],[1.3,4.2],[3.2,4],[5.3,4],[7,4.3],[9,4.4],[9.5,3.6],[9.6,2.5],[8.5,2.15],[2.5,2.3],[2.4,2.1],[2.4,1.85],[2.6,1.6],[8.2,1.45],[9,1.2],[9.5,.4],[9.2,-.5],[8.3,-.7],[3.5,-.7],[.2,-.3]]


class CorrespondenceTests(unittest.TestCase):
    def test_concave_course_keeps_full_vehicle_inside(self):
        for outer in (right, list(reversed(right[6:]+right[:6]))):
            with self.subTest(reversed=outer is not right):
                result=generate(dict(left=left,right=outer,closed=True),'centerline',{})
                Corridor(left,outer,True).validate(result['points'],.14)
                self.assertGreater(len(result['points']),200)

    def test_rejection_location_is_persisted_in_job(self):
        corridor=Corridor([[0,1],[5,1]],[[0,-1],[5,-1]],False)
        with self.assertRaises(PathValidationError) as caught:
            corridor.validate([[1,0],[3,1.2]])
        error=caught.exception
        self.assertEqual(error.diagnostic['segment'],1)
        self.assertEqual(error.diagnostic['reason'],'outside')
        self.assertIn('XY=',str(error))
        with tempfile.TemporaryDirectory() as directory:
            jobs=Jobs(Path(directory))
            job=Job(Path(directory),'centerline',[])
            job.persist_enabled=True
            def fail():raise error
            perform(job,fail)
            started={'id':job.id}
            deadline=time.monotonic()+3
            while read_json(Path(directory)/(started['id']+'.json'))['status']=='running' and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(jobs.list()[0]['status'],'failed')
            self.assertEqual(jobs.list()[0]['diagnostic'],error.diagnostic)
            self.assertEqual(read_json(Path(directory)/(started['id']+'.json'))['diagnostic'],error.diagnostic)

if __name__=='__main__':unittest.main()
