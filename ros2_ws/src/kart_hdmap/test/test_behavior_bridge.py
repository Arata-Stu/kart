"""Behavior-reset transaction boundary without ROS services or an executor."""
from concurrent.futures import Future
import queue
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from kart_hdmap.behavior_bridge import BehaviorBridge
from kart_hdmap.tuning_bridge import TuningBridge


class BehaviorBridgeTests(unittest.TestCase):
    def setUp(self):
        self.future, self.calls = Future(), []
        self.ready, self.fresh = True, True
        self.bridge = NS(identity='map', revision=lambda: 'r1', issue=lambda: '', gate=NS(issue=lambda: ''),
                         parameters=NS(pending=None),
                         node=NS(count_publishers=lambda topic: 1,
                                 get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))),
                         behavior=NS(view=lambda *args: dict(fresh=self.fresh, status={'mode': 'active', 'state': 'FAULT'})))
        self.manager = BehaviorBridge.__new__(BehaviorBridge)
        self.manager.bridge, self.manager.pending = self.bridge, None
        self.manager.type = NS(Request=lambda: object())
        self.manager.client = NS(service_is_ready=lambda: self.ready,
                                 call_async=lambda request: (self.calls.append(request), self.future)[1])

    def begin(self, **body):
        reply = queue.Queue()
        self.manager.begin(dict(map_id='map', expected_revision='r1', **body), reply, 20.)
        return reply

    def test_one_explicit_call_can_reset_fault_and_returns_the_service_result(self):
        with patch('kart_hdmap.behavior_bridge.time.monotonic', return_value=10.):
            reply = self.begin()
            self.assertEqual(len(self.calls), 1)
            other = self.begin()
            self.assertIn('応答待ち', other.get_nowait()['error'])
            self.assertEqual(len(self.calls), 1)
            self.future.set_result(NS(success=True, message='AUTOは変更しません'))
            self.manager.tick()
            self.assertEqual(reply.get_nowait(), dict(map_id='map', success=True, message='AUTOは変更しません'))
            self.manager.tick()
            self.assertEqual(len(self.calls), 1)

    def test_map_revision_stop_pending_and_source_guards_prevent_dispatch(self):
        cases = [('map', lambda: setattr(self.bridge, 'identity', 'other')),
                 ('revision', lambda: setattr(self.bridge, 'revision', lambda: 'r2')),
                 ('stop', lambda: setattr(self.bridge.gate, 'issue', lambda: 'STOP required')),
                 ('conflict', lambda: setattr(self.bridge, 'issue', lambda: 'publisher conflict')),
                 ('parameters', lambda: setattr(self.bridge.parameters, 'pending', object())),
                 ('freshness', lambda: setattr(self, 'fresh', False)),
                 ('service', lambda: setattr(self, 'ready', False))]
        for case, change in cases:
            self.setUp()
            change()
            with self.subTest(case=case), patch('kart_hdmap.behavior_bridge.time.monotonic', return_value=10.):
                self.assertIn('error', self.begin().get_nowait())
                self.assertEqual(self.calls, [])

    def test_timeout_is_unknown_and_never_retries(self):
        with patch('kart_hdmap.behavior_bridge.time.monotonic', return_value=10.):
            reply = self.begin()
        with patch('kart_hdmap.behavior_bridge.time.monotonic', return_value=21.):
            self.manager.tick()
            self.assertIn('結果が不明', reply.get_nowait()['error'])
            self.manager.tick()
            self.assertEqual(len(self.calls), 1)
            self.assertIsNone(self.manager.pending)

    def test_existing_executor_queue_routes_reset_without_blocking(self):
        queued = TuningBridge.__new__(TuningBridge)
        queued.parameters, queued.parking, queued.behavior_reset = NS(tick=Mock()), NS(tick=Mock()), NS(tick=Mock(), begin=Mock())
        queued.capture_pose, queued.requests = Mock(), queue.Queue()
        body, reply = dict(map_id='map', expected_revision='r1'), queue.Queue()
        queued.requests.put(('/behavior_reset', body, reply, 20.))
        queued.tick()
        queued.behavior_reset.begin.assert_called_once_with(body, reply, 20.)
        queued.behavior_reset.tick.assert_called_once()


if __name__ == '__main__':
    unittest.main()
