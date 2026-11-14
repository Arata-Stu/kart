import importlib.util
from pathlib import Path
import socket
import stat
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from kart_jetson_cache.client import SingleRequest, request_clear, validate_socket


spec = importlib.util.spec_from_file_location(
    'kart_cache_host', Path(__file__).parents[1] / 'host/cache_helper.py')
host = importlib.util.module_from_spec(spec)
spec.loader.exec_module(host)


class HostTests(unittest.TestCase):
    def test_arm_notebook_is_not_jetson(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertFalse(host.is_jetson(root, system='Linux', machine='aarch64'))
            (root / 'etc').mkdir()
            (root / 'etc/nv_tegra_release').write_text('# R36')
            self.assertTrue(host.is_jetson(root, system='Linux', machine='aarch64'))
            self.assertFalse(host.is_jetson(root, system='Darwin', machine='arm64'))
            self.assertFalse(host.is_jetson(root, system='Linux', machine='x86_64'))

    def test_only_fixed_clean_pagecache_operation(self):
        with patch.object(host.os, 'open', return_value=7) as open_file, \
                patch.object(host.os, 'write', return_value=2) as write, \
                patch.object(host.os, 'close') as close:
            self.assertEqual(host.drop_pagecache(), 'OK pagecache_drop_completed\n')
        self.assertEqual(open_file.call_args.args[0], '/proc/sys/vm/drop_caches')
        self.assertEqual(write.call_args.args, (7, b'1\n'))
        close.assert_called_once_with(7)

    def test_short_write_is_failure_and_closes_descriptor(self):
        with patch.object(host.os, 'open', return_value=7), \
                patch.object(host.os, 'write', return_value=1), \
                patch.object(host.os, 'close') as close:
            with self.assertRaises(OSError):
                host.drop_pagecache()
        close.assert_called_once_with(7)

    def test_timeout_stays_busy_until_operation_actually_ends(self):
        finish, started = threading.Event(), threading.Event()

        def operation():
            started.set()
            finish.wait(1)
            return 'OK pagecache_drop_completed\n'

        worker = host.ReleaseWorker(operation, clock=lambda: 20.)
        try:
            self.assertIn('completion_unknown', worker.request(timeout_s=0.01))
            self.assertTrue(started.is_set())
            self.assertIn('busy', worker.request(timeout_s=0.01))
        finally:
            finish.set()
            with worker.lock:
                pass
        self.assertIn('rate_limited', worker.request(timeout_s=0.01))

    def test_worker_rate_limit_and_failure_result(self):
        clock = Mock(side_effect=[0., .5, 1.1])
        operation = Mock(side_effect=PermissionError(13, 'not permitted'))
        worker = host.ReleaseWorker(operation, clock=clock)
        self.assertIn('errno_13', worker.request())
        self.assertIn('rate_limited', worker.request())
        self.assertIn('errno_13', worker.request())
        self.assertEqual(operation.call_count, 2)

    def test_invalid_protocol_never_releases_cache(self):
        client, server = socket.socketpair()
        worker = Mock()
        client.sendall(b'/etc/shadow\n')
        with server:
            host.handle_client(server, worker)
        self.assertEqual(client.recv(512), b'ERR invalid_request\n')
        client.close()
        worker.request.assert_not_called()

    def test_host_guard_denies_nonjetson_and_container(self):
        with patch.object(host.os, 'geteuid', return_value=1000):
            with self.assertRaisesRegex(RuntimeError, 'requires root'):
                host.validate_host()
        with patch.object(host.os, 'geteuid', return_value=0), \
                patch.object(host, 'is_jetson', return_value=True), \
                patch.object(host.Path, 'exists', return_value=True):
            with self.assertRaisesRegex(RuntimeError, 'outside Docker'):
                host.validate_host()


class ClientTests(unittest.TestCase):
    def test_socket_identity_requires_unwritable_root_parent_and_root_socket(self):
        parent = Mock(st_mode=stat.S_IFDIR | 0o750, st_uid=0)
        target = Mock(st_mode=stat.S_IFSOCK | 0o660, st_uid=0, st_gid=345)
        with patch.object(Path, 'lstat', side_effect=[parent, target]):
            validate_socket('/run/kart-cache/cache.sock')
        for invalid in (
            Mock(st_mode=stat.S_IFSOCK | 0o666, st_uid=0, st_gid=345),
            Mock(st_mode=stat.S_IFSOCK | 0o660, st_uid=1000, st_gid=345),
            Mock(st_mode=stat.S_IFLNK | 0o777, st_uid=0, st_gid=345),
            Mock(st_mode=stat.S_IFSOCK | 0o660, st_uid=0, st_gid=0),
        ):
            with patch.object(Path, 'lstat', side_effect=[parent, invalid]):
                with self.assertRaises(PermissionError):
                    validate_socket('/run/kart-cache/cache.sock')
        parent.st_mode |= 0o020
        with patch.object(Path, 'lstat', side_effect=[parent, target]):
            with self.assertRaises(PermissionError):
                validate_socket('/run/kart-cache/cache.sock')

    def protocol(self, response):
        # socketpair is allowed by the portable runner even when binding a Unix
        # pathname is sandboxed. Only connect/path identity are mocked here.
        client, server = socket.socketpair()
        requests = []

        class Connection:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                client.close()

            settimeout = client.settimeout
            connect = Mock()
            sendall = client.sendall
            recv = client.recv

        def respond():
            with server:
                requests.append(server.recv(64))
                server.sendall(response)

        thread = threading.Thread(target=respond, daemon=True)
        thread.start()
        try:
            with patch('kart_jetson_cache.client.validate_socket'), \
                    patch('kart_jetson_cache.client.socket.socket', return_value=Connection()):
                result = request_clear('/run/kart-cache/cache.sock', 1.)
        finally:
            thread.join(timeout=1.)
            client.close()
        self.assertEqual(requests, [b'DROP_PAGECACHE\n'])
        return result

    def test_success_means_kernel_completion_not_request_acceptance(self):
        self.assertEqual(self.protocol(b'OK pagecache_drop_completed\n'),
                         (True, 'OK pagecache_drop_completed'))
        self.assertEqual(self.protocol(b'ERR operation_timeout_completion_unknown_no_retry\n'),
                         (False, 'operation_timeout_completion_unknown_no_retry'))

    def test_malformed_and_oversized_responses_are_failures(self):
        for response in (b'OK accepted\n', b'x' * 513 + b'\n'):
            with self.assertRaises(RuntimeError):
                self.protocol(response)

    def test_adapter_timeout_never_starts_second_overlapping_worker(self):
        finish, calls = threading.Event(), []
        runner = SingleRequest()

        def operation():
            calls.append(True)
            finish.wait(1.)
            return True, 'complete'

        try:
            self.assertIn('completion unknown', runner.run(operation, .01)[1])
            self.assertIn('busy', runner.run(operation, .01)[1])
            self.assertEqual(len(calls), 1)
        finally:
            finish.set()
            with runner.lock:
                pass

    def test_adapter_reports_exceptions_as_failures(self):
        def failure():
            raise PermissionError('denied')
        success, message = SingleRequest().run(failure, 1.)
        self.assertFalse(success)
        self.assertIn('PermissionError', message)


if __name__ == '__main__':
    unittest.main()
