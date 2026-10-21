import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from kart_studio.jobs import Job, Jobs, Task, JobRef, resource_conflict
from kart_studio.storage import atomic_json, read_json

REPO = Path(__file__).resolve().parents[3]


class JobsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.controllers = []
        self.addCleanup(self.cleanup_workers)

    def controller(self):
        jobs = Jobs(self.root, REPO, self.root/'data')
        self.controllers.append(jobs)
        return jobs

    def cleanup_workers(self):
        for manager in self.controllers:
            for state in manager.list():
                if state['status'] in ('running','orphaned'):
                    manager.jobs[state['id']].stop()
        end = time.monotonic()+14
        while time.monotonic()<end and any(j.claimed for j in self.controllers):
            time.sleep(.03)

    def wait(self, check, timeout=8):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            result=check()
            if result:return result
            time.sleep(.02)
        self.fail('condition did not complete')

    def done(self, manager, key):
        self.wait(lambda: manager.jobs[key].info()['status'] not in ('running','orphaned'))
        return manager.jobs[key].info()

    def command(self, seconds=1, ready=None):
        script='import time, pathlib, os; '
        if ready:script+=f'pathlib.Path({str(ready)!r}).write_text(str(os.getpid())); '
        script+=f'time.sleep({seconds})'
        return Task('command', [[sys.executable,'-c',script]])

    def test_monotonic_elapsed_and_duplicate_stop(self):
        job=Job(self.root,'test',[])
        with patch('kart_studio.jobs.time.monotonic',return_value=job.started_monotonic+5):
            self.assertEqual(job.info()['elapsed_s'],5)
        with patch('kart_studio.jobs.threading.Thread') as thread:
            job.stop(); job.stop()
            self.assertTrue(job.info()['cancel_requested']);self.assertEqual(thread.call_count,1)

    def test_real_subprocess_stop_releases_resources_after_reconnect(self):
        first=self.controller();ready=self.root/'ready'
        started=first.start('subprocess',['output:x'],self.command(60,ready))
        self.wait(ready.exists)
        self.wait(lambda:first.jobs[started['id']].info()['process_pid'])
        state=first.jobs[started['id']].info()
        self.assertEqual(state['process_pid'],int(ready.read_text()))
        self.assertIsNotNone(state['worker_pid'])
        first.shutdown()  # Detach, never cancel.
        again=self.controller()
        self.assertEqual(again.jobs[started['id']].info()['worker_pid'],state['worker_pid'])
        self.assertFalse(again.jobs[started['id']].info()['cancel_requested'])
        again.jobs[started['id']].stop();again.jobs[started['id']].stop()
        self.assertEqual(self.done(again,started['id'])['status'],'cancelled')
        self.assertFalse(again.claimed)

    def test_parallel_reads_and_exclusive_writes(self):
        jobs=self.controller()
        a=jobs.start('vgl',['read:map:track','vgl:one'],self.command(.6))
        b=jobs.start('train',['read:map:track','read:e2e:datasets:data','e2e:runs:one'],self.command(.6))
        with self.assertRaisesRegex(ValueError,'map:track'):
            jobs.start('edit',['map:track'],self.command(.01))
        self.assertEqual(self.done(jobs,a['id'])['status'],'succeeded')
        self.assertEqual(self.done(jobs,b['id'])['status'],'succeeded')
        self.assertFalse(resource_conflict('read:map:a','read:map:a'))
        self.assertTrue(resource_conflict('read:map:a','map:a'))

    def test_completion_duration_and_failure_are_persisted(self):
        jobs=self.controller()
        info=jobs.start('success',[],self.command(.01))
        result=self.done(jobs,info['id'])
        self.assertGreaterEqual(result['elapsed_s'],0)
        self.assertIsNotNone(result['finished'])
        self.assertIsNotNone(result['last_process_pid'])
        restored=self.controller().jobs[info['id']].info()
        self.assertEqual(restored['elapsed_s'],result['elapsed_s'])
        bad=jobs.start('failure',[],Task('command',[[sys.executable,'-c','raise RuntimeError("fixture")']]))
        self.assertEqual(self.done(jobs,bad['id'])['status'],'failed')

    def test_controller_process_exit_does_not_stop_worker(self):
        ready=self.root/'detached'
        script='''import sys,json\nfrom pathlib import Path\nfrom kart_studio.jobs import Jobs,Task\nr=Path(sys.argv[1]); j=Jobs(r,Path(sys.argv[2]),r/'data')\nprint(json.dumps(j.start('detached',['output:detached'],Task('command',[[sys.executable,'-c',"import pathlib,time; pathlib.Path("+repr(str(r/'detached'))+").touch(); time.sleep(1.5)"]]))),flush=True)\nj.shutdown()\n'''
        done=subprocess.run([sys.executable,'-c',script,str(self.root),str(REPO)],capture_output=True,text=True,check=True)
        info=__import__('json').loads(done.stdout)
        manager=self.controller();self.wait(ready.exists)
        self.assertEqual(manager.jobs[info['id']].info()['status'],'running')
        self.assertEqual(self.done(manager,info['id'])['status'],'succeeded')

    def test_pid_reuse_is_not_signalled(self):
        jobs=self.controller();ref=JobRef(jobs,'a'*12)
        state=dict(status='orphaned',process_pid=12345,process_identity='old')
        with patch.object(jobs,'read',return_value=state),patch('kart_studio.jobs.process_identity',return_value='new'),patch('kart_studio.jobs.os.killpg') as kill:
            with self.assertRaises(ValueError):ref.stop()
            kill.assert_not_called()

    def test_interrupted_history_does_not_invent_duration(self):
        atomic_json(self.root/'old.json',dict(id='old',started=1,status='running',elapsed_s=0))
        info=self.controller().list()[0]
        self.assertEqual(info['status'],'interrupted')
        self.assertIsNone(info['elapsed_s'])
