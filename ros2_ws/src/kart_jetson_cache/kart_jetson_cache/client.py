"""Bounded local protocol; no shell, sudo, filenames or operation values are sent."""

import math
from pathlib import Path
import socket
import stat
import threading
import time


REQUEST = b'DROP_PAGECACHE\n'
MAX_RESPONSE = 512


def validate_socket(path):
    path = Path(path)
    if not path.is_absolute():
        raise ValueError('cache helper socket_path must be absolute')
    parent = path.parent.lstat()
    target = path.lstat()
    if (not stat.S_ISDIR(parent.st_mode) or parent.st_uid != 0
            or parent.st_mode & 0o022):
        raise PermissionError('cache helper directory must be root-owned and not group/world writable')
    if (not stat.S_ISSOCK(target.st_mode) or target.st_uid != 0
            or stat.S_IMODE(target.st_mode) != 0o660 or target.st_gid == 0):
        raise PermissionError('cache helper socket must be root:non-root-group with mode 0660')


def request_clear(path, timeout_s):
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError('request_timeout_s must be finite and positive')
    validate_socket(path)
    deadline = time.monotonic() + timeout_s
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout_s)
        connection.connect(str(path))
        connection.settimeout(max(0.001, deadline - time.monotonic()))
        connection.sendall(REQUEST)
        response = bytearray()
        while not response.endswith(b'\n'):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('helper response timeout; completion unknown; do not retry this lap')
            connection.settimeout(remaining)
            chunk = connection.recv(MAX_RESPONSE + 1 - len(response))
            if not chunk:
                raise RuntimeError('cache helper closed without a complete response')
            response.extend(chunk)
            if len(response) > MAX_RESPONSE:
                raise RuntimeError('cache helper response exceeds limit')
        reply = response.decode('ascii', errors='strict').strip()
        if reply == 'OK pagecache_drop_completed':
            return True, reply
        if reply.startswith('ERR '):
            return False, reply[4:]
        raise RuntimeError('unrecognized cache helper response')


class SingleRequest:
    """At most one worker, including after its caller times out."""

    def __init__(self):
        self.lock = threading.Lock()

    def run(self, operation, timeout_s):
        if not self.lock.acquire(blocking=False):
            return False, 'cache request busy; no overlapping operation was started'
        completed = threading.Event()
        result = []

        def work():
            try:
                result.append(operation())
            except Exception as error:  # Middleware/socket errors are returned to the operator.
                result.append((False, f'{type(error).__name__}: {error}'))
            finally:
                self.lock.release()
                completed.set()

        try:
            threading.Thread(target=work, daemon=True, name='kart-cache-client').start()
        except RuntimeError as error:
            self.lock.release()
            return False, f'cache worker could not start: {error}'
        if not completed.wait(timeout_s):
            return False, 'cache request timeout; completion unknown; no retry for this lap'
        return result[0]
