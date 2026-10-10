import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from kart_bag_manager.recorder import Recorder, Settings

# Own child process in a temporary directory; never invokes ros2 or records user data.
CHILD = '''
import pathlib, signal, sys, time
mode, output = sys.argv[1:]
if mode == 'fail': sys.exit(7)
if mode == 'ignore':
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
else:
    def stop(sig, frame):
        if pathlib.Path(output).is_dir():
            pathlib.Path(output, 'metadata.yaml').write_text('test-only finalized')
        sys.exit(0)
    signal.signal(signal.SIGINT, stop)
if mode != 'no_dir': pathlib.Path(output).mkdir()
while True: time.sleep(0.01)
'''

class RecorderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.settings = Settings(output_dir=self.tmp.name, topics=['/joy'],
                                 recording_start_timeout_s=0.2, stop_timeout_s=0.15, terminate_timeout_s=0.15)

    def make(self, mode='normal'):
        def popen(cmd, **kwargs):
            return subprocess.Popen([sys.executable, '-c', CHILD, mode, cmd[cmd.index('-o')+1]], **kwargs)
        recorder = Recorder(self.settings, popen=popen)
        self.addCleanup(recorder.close)
        return recorder

    def until(self, recorder, predicate, timeout=3):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            recorder.tick()
            if predicate(): return
            time.sleep(0.01)
        self.fail(f'timed out: {recorder.phase}: {recorder.last_event}')

    def test_lazy_session_directories_and_repeated_recording(self):
        root = Path(self.tmp.name).resolve() / "new_record"
        self.settings.output_dir = str(root)
        self.settings.session_layout = True
        self.settings.recording_name = "run_name"
        r = self.make()
        self.assertFalse(root.exists())
        r.close()
        self.assertFalse(root.exists())
        r.start()
        self.until(r, lambda: r.recording)
        first = Path(r.current_uri)
        self.assertEqual(first.relative_to(root).parts, (*r.session_stamp.split("/"), "run_name"))
        r.stop()
        self.until(r, lambda: r.process is None)
        r.start()
        self.until(r, lambda: r.recording)
        self.assertEqual(Path(r.current_uri), first.with_name("run_name_01"))

    def test_start_stop_restart_and_finalize(self):
        r=self.make()
        self.assertTrue(r.start('../../label'))
        self.assertEqual(r.phase,'starting')
        self.assertFalse(r.recording)
        self.assertFalse(r.start())
        self.until(r,lambda:r.recording)
        first=Path(r.current_uri)
        self.assertEqual(first.parent,Path(self.tmp.name).resolve())
        r.stop(); self.assertEqual(r.phase,'stopping')
        deadline=r._deadline; r.stop(); self.assertEqual(deadline,r._deadline)
        self.assertFalse(r.start())
        self.until(r,lambda:r.process is None)
        self.assertEqual(r.phase,'idle'); self.assertTrue((first/'metadata.yaml').exists())
        self.assertTrue(r.start('../../label')); self.assertNotEqual(r.current_uri,str(first))
        self.until(r,lambda:r.recording)

    def test_close_finalizes_child(self):
        r=self.make(); r.start(); self.until(r,lambda:r.recording)
        r.close()
        self.assertIsNone(r.process)
        self.assertEqual(r.phase,'idle')
        self.assertTrue((Path(r.current_uri)/'metadata.yaml').is_file())

    def test_unexpected_exit_without_raw_enabled(self):
        r=self.make('fail'); r.start()
        self.until(r,lambda:r.phase=='error')
        self.assertIn('7',r.last_event); self.assertFalse(r.recording)

    def test_start_timeout_and_early_stop(self):
        r=self.make('no_dir'); r.start()
        self.until(r,lambda:r.phase=='stopping')
        self.until(r,lambda:r.process is None)
        self.assertEqual(r.phase,'error')
        r=self.make(); r.start(); r.stop() # STOP during asynchronous startup.
        self.until(r,lambda:r.process is None)
        self.assertFalse(r.recording)

    def test_forced_shutdown_is_reported(self):
        r=self.make('ignore'); r.start(); self.until(r,lambda:r.recording)
        r.stop(); self.until(r,lambda:r.process is None)
        self.assertEqual(r.phase,'error'); self.assertIn('SIGKILL',r.last_event)

    def test_spawn_failure(self):
        with patch('subprocess.Popen', side_effect=FileNotFoundError('ros2 unavailable')):
            r=Recorder(self.settings,popen=subprocess.Popen)
            self.assertFalse(r.start()); self.assertEqual(r.phase,'error')
            self.assertIsNone(r.process)

    def test_no_topics_and_settings_validation(self):
        self.settings.topics=[]; r=Recorder(self.settings)
        self.assertFalse(r.start())
        for name in ('..','a/b','a\\b',' bad','bad\n'):
            with self.assertRaises(ValueError): Settings(recording_name=name).validate()
        for arg in ('--output=x','-d','--max-bag-duration=1','--max-bag-files=2'):
            with self.assertRaises(ValueError): Settings(extra_args=[arg]).validate()
        with self.assertRaises(ValueError): Settings(status_period_s=float('nan')).validate()

    def test_command_flags(self):
        s=Settings(topics=['/joy','/tf'],exclude_topics=['/joy/raw'],recording_split_duration_s=60,
                   compression_mode='file',compression_format='zstd',max_bag_size=123,max_cache_size=456,
                   qos_profile_overrides_path='/tmp/qos file.yaml',extra_args=['--log-level','warn'])
        cmd=s.command('/tmp/a b')
        self.assertEqual(cmd[cmd.index('-o')+1],'/tmp/a b')
        self.assertEqual(cmd[-3:],['--topics','/joy','/tf'])
        self.assertIn('--exclude-topics',cmd); self.assertNotIn('--exclude',cmd)
        self.assertEqual(cmd[cmd.index('--max-bag-duration')+1],'60')
        s.record_all=True; self.assertEqual(s.command('/tmp/b')[-1],'--all-topics')

    def test_named_collision_and_symlink(self):
        self.settings.recording_name='trial'
        r=Recorder(self.settings); first=r.next_path('ignored'); first.parent.mkdir(parents=True)
        first.symlink_to(first.parent/'missing')
        self.assertEqual(r.next_path('ignored').name,'trial_01')

if __name__=='__main__': unittest.main()
