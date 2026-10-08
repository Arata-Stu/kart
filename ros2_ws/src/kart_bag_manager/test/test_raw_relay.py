from concurrent.futures import Future
from dataclasses import fields
from pathlib import Path
import re
from kart_bag_manager.recorder import Settings
import unittest
from kart_bag_manager.raw_relay import RawRelay


class RawRelayTest(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.calls = []
        self.futures = []
        def submit(operation, argument):
            self.calls.append((operation, argument))
            future = Future()
            self.futures.append(future)
            return future
        self.relay = RawRelay(submit, timeout=1.0, clock=lambda: self.now)

    def ack(self, success=True, message='ok'):
        self.futures[-1].set_result((success, message))
        self.relay.tick()

    def start(self):
        self.relay.request('START', '/tmp/session_01')
        self.relay.tick()
        self.ack()
        self.ack()

    def test_directory_then_start_then_stop(self):
        self.start()
        self.assertEqual(self.calls, [('set_directory', '/tmp/session_01'), ('start', '')])
        self.assertEqual(self.relay.state, 'recording')
        self.relay.request('STOP')
        self.relay.tick()
        self.ack()
        self.assertFalse(self.relay.owns_raw)
        self.assertFalse(self.relay.pending)

    def test_stop_during_directory_setup_is_not_lost(self):
        self.relay.request('START', '/tmp/session_01')
        self.relay.tick()
        self.relay.request('STOP')
        self.ack()
        self.ack()
        self.assertEqual(self.calls[-1], ('stop', ''))
        self.ack()
        self.assertEqual(self.relay.state, 'idle')

    def test_rejected_directory_does_not_stop_manual_raw(self):
        self.relay.request('START', '/tmp/session_01')
        self.relay.tick()
        self.relay.request('STOP')
        self.ack(False, 'manual RAW already active')
        self.relay.tick()
        self.assertEqual(len(self.calls), 1)
        self.assertFalse(self.relay.owns_raw)
        self.assertTrue(self.relay.last_error)

    def test_unavailable_driver_reports_timeout(self):
        self.relay.submit = lambda *_: None
        self.relay.request('START', '/tmp/session_01')
        self.relay.tick()
        self.now = 1.1
        self.relay.tick()
        self.assertEqual(self.relay.state, 'error')
        self.assertIn('unavailable', self.relay.last_error)

    def test_start_timeout_requests_cleanup_stop(self):
        self.relay.request('START', '/tmp/session_01')
        self.relay.tick()
        self.ack()
        self.now = 1.1
        self.relay.tick()
        self.assertTrue(self.relay.owns_raw)
        self.relay.tick()
        self.assertEqual(self.calls[-1], ('stop', ''))
        self.ack()
        self.assertFalse(self.relay.owns_raw)
        self.assertIn('timeout', self.relay.last_error)

    def test_rejected_start_does_not_claim_recording(self):
        self.relay.request('START', '/tmp/session_01')
        self.relay.tick()
        self.ack()
        self.ack(False, 'camera failed')
        self.assertFalse(self.relay.owns_raw)
        self.assertEqual(self.relay.state, 'error')

    def test_split_and_next_session(self):
        self.start()
        self.relay.request('SPLIT')
        self.relay.tick()
        self.ack()
        self.relay.request('STOP')
        self.relay.request('START', '/tmp/session_02')
        self.relay.tick()
        self.ack()
        self.assertEqual(self.calls[-1], ('set_directory', '/tmp/session_02'))
        self.ack()
        self.ack()
        self.assertEqual(self.relay.state, 'recording')

    def test_stop_rejection_preserves_unverified_ownership(self):
        self.start()
        self.relay.request('STOP')
        self.relay.tick()
        self.ack(False, 'disk error')
        self.assertTrue(self.relay.owns_raw)
        self.assertEqual(self.relay.state, 'error')

    def test_relative_path_rejected(self):
        self.relay.request('START', 'relative/session')
        self.relay.tick()
        self.assertFalse(self.calls)
        self.assertEqual(self.relay.state, 'error')

    def test_driver_name_and_timeout_validation(self):
        for name in ('relative', '/', '/event_camera/'):
            with self.assertRaises(ValueError):
                Settings(raw_recording_driver_node=name).validate()
        for timeout in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                Settings(raw_recording_service_timeout_s=timeout).validate()
        Settings(raw_recording_driver_node='/event_camera/event_camera_driver').validate()

    def test_profiles_include_all_manager_parameters(self):
        expected = {field.name for field in fields(Settings())} | {'use_sim_time'}
        root = Path(__file__).resolve().parents[1]
        for name in ('bag_manager.yaml', 'bag_manager_openeb.yaml'):
            text = (root / 'config' / name).read_text()
            actual = set(re.findall(r'^    ([a-z_]+):', text, re.MULTILINE))
            self.assertEqual(actual, expected)

    def test_queue_limit_retains_stop(self):
        self.start()
        for _ in range(20):
            self.relay.request('SPLIT')
        self.relay.request('STOP')
        self.relay.tick()
        self.assertEqual(self.calls[-1], ('stop', ''))


if __name__ == '__main__':
    unittest.main()
