import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from kart_studio.jobs import Job, Jobs
from kart_studio.storage import atomic_json, read_json


class JobsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def wait_done(self, jobs, key):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            job = jobs.jobs[key]
            if (job.finished is not None and not jobs.claimed
                    and read_json(job.state_path).get('finished') is not None):
                return job
            time.sleep(0.01)
        self.fail('job did not finish')

    def test_monotonic_elapsed_and_duplicate_stop(self):
        job = Job(self.root, 'test', [])
        with patch('kart_studio.jobs.time.monotonic', return_value=job.started_monotonic + 5):
            self.assertEqual(job.info()['elapsed_s'], 5)
        with patch('kart_studio.jobs.threading.Thread') as worker:
            job.stop()
            job.stop()
            self.assertTrue(job.info()['cancel_requested'])
            self.assertEqual(worker.call_count, 1)
        job.status = 'succeeded'
        job.cancel.clear()
        job.stop()
        self.assertFalse(job.cancel.is_set())

    def test_real_subprocess_stop_releases_resources(self):
        jobs = Jobs(self.root)
        ready = self.root / 'ready'
        def work(job):
            job.run([sys.executable, '-c',
                     'import pathlib,sys,time; pathlib.Path(sys.argv[1]).touch(); time.sleep(60)', str(ready)])
        info = jobs.start('subprocess', ['gpu'], work)
        deadline = time.monotonic() + 5
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        job = jobs.jobs[info['id']]
        try:
            self.assertTrue(ready.exists())
            job.stop()
            self.wait_done(jobs, job.id)
            self.assertEqual(job.status, 'cancelled')
            self.assertIsNone(job.process)
            self.assertGreaterEqual(job.info()['elapsed_s'], 0)
        finally:
            jobs.shutdown()

    def test_completion_duration_persists_and_does_not_grow(self):
        jobs = Jobs(self.root)
        info = jobs.start('success', [], lambda job: {'ok': True})
        job = self.wait_done(jobs, info['id'])
        elapsed = job.info()['elapsed_s']
        with patch('kart_studio.jobs.time.monotonic', return_value=job.started_monotonic + 999):
            self.assertEqual(job.info()['elapsed_s'], elapsed)
        restored = Jobs(self.root).list()[0]
        self.assertEqual(restored['elapsed_s'], elapsed)
        self.assertIsNotNone(restored['finished'])

    def test_interrupted_history_does_not_invent_duration(self):
        atomic_json(self.root / 'old.json', {'id': 'old', 'started': 1,
                    'status': 'running', 'elapsed_s': 0})
        info = Jobs(self.root).list()[0]
        self.assertEqual(info['status'], 'interrupted')
        self.assertIsNone(info['elapsed_s'])
