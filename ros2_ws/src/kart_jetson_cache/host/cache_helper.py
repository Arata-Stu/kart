#!/usr/bin/env python3
"""Root-owned Jetson host helper. Its only operation writes 1 to drop_caches."""

import grp
import os
from pathlib import Path
import platform
import signal
import socket
import stat
import threading
import time


SOCKET_PATH = Path('/run/kart-cache/cache.sock')
DROP_CACHES = '/proc/sys/vm/drop_caches'
REQUEST = b'DROP_PAGECACHE\n'
OPERATION_TIMEOUT_S = 2.0
MIN_INTERVAL_S = 1.0


def is_jetson(root=Path('/'), system=None, machine=None):
    if (system or platform.system()) != 'Linux':
        return False
    if (machine or platform.machine()).lower() not in ('aarch64', 'arm64'):
        return False
    if (root / 'etc/nv_tegra_release').is_file():
        return True
    for relative in ('proc/device-tree/model', 'sys/firmware/devicetree/base/model'):
        try:
            if 'jetson' in (root / relative).read_text().lower():
                return True
        except (OSError, UnicodeError):
            pass
    return False


def validate_host():
    if os.geteuid() != 0 or not is_jetson():
        raise RuntimeError('cache helper requires root on a Jetson Linux host')
    if Path('/.dockerenv').exists() or Path('/run/.containerenv').exists():
        raise RuntimeError('cache helper must run on the host, outside Docker/Podman')


def drop_pagecache():
    # sync() is deliberately absent: only clean pages are candidates. Dirty bag
    # pages remain under normal kernel writeback instead of forcing disk IO.
    descriptor = os.open(DROP_CACHES, os.O_WRONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        if os.write(descriptor, b'1\n') != 2:
            raise OSError('incomplete drop_caches write')
    finally:
        os.close(descriptor)
    return 'OK pagecache_drop_completed\n'


class ReleaseWorker:
    """A timeout never releases the busy gate while the kernel call is running."""

    def __init__(self, operation=drop_pagecache, clock=time.monotonic):
        self.operation, self.clock = operation, clock
        self.lock = threading.Lock()
        self.last_started = float('-inf')

    def request(self, timeout_s=OPERATION_TIMEOUT_S):
        if not self.lock.acquire(blocking=False):
            return 'ERR busy_previous_operation_still_running\n'
        now = self.clock()
        if now - self.last_started < MIN_INTERVAL_S:
            self.lock.release()
            return 'ERR rate_limited_minimum_1_second\n'
        self.last_started = now
        event, result = threading.Event(), []

        def work():
            try:
                result.append(self.operation())
            except OSError as error:
                result.append(f'ERR kernel_write_failed_errno_{error.errno}\n')
            except Exception:
                result.append('ERR kernel_operation_failed\n')
            finally:
                self.lock.release()
                event.set()

        threading.Thread(target=work, daemon=True, name='kart-cache-release').start()
        if not event.wait(timeout_s):
            return 'ERR operation_timeout_completion_unknown_no_retry\n'
        return result[0]


def handle_client(connection, worker):
    # Bounded protocol: no paths, options, shell commands, or arbitrary values.
    deadline = time.monotonic() + 0.5
    request = bytearray()
    try:
        while not request.endswith(b'\n') and len(request) < 64:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            connection.settimeout(remaining)
            chunk = connection.recv(64 - len(request))
            if not chunk:
                break
            request.extend(chunk)
        response = worker.request() if request == REQUEST else 'ERR invalid_request\n'
        connection.settimeout(0.5)
        connection.sendall(response.encode('ascii'))
    except (OSError, TimeoutError):
        pass  # A disconnected client never causes an automatic replay.


def prepare_socket(path=SOCKET_PATH):
    directory = path.parent.lstat()
    group_id = grp.getgrnam('kart-cache').gr_gid
    if group_id == 0:
        raise RuntimeError('kart-cache group must not be root')
    if (not stat.S_ISDIR(directory.st_mode) or directory.st_uid != 0
            or directory.st_gid != group_id or stat.S_IMODE(directory.st_mode) != 0o755):
        raise PermissionError('socket directory must be root:kart-cache mode 0755')
    if path.exists() or path.is_symlink():
        existing = path.lstat()
        if not stat.S_ISSOCK(existing.st_mode) or existing.st_uid != 0:
            raise PermissionError('refusing to replace unexpected helper socket path')
        path.unlink()
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        # The parent directory is not writable by clients. Group access is
        # checked by the kernel, including Docker supplementary groups.
        server.bind(str(path))
        os.chown(path, 0, group_id)
        os.chmod(path, 0o660)
        server.listen(8)
        server.settimeout(0.25)
    except Exception:
        server.close()
        raise
    return server


def main():
    validate_host()
    worker = ReleaseWorker()
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_args: stopping.set())
    signal.signal(signal.SIGINT, lambda *_args: stopping.set())
    with prepare_socket() as server:
        while not stopping.is_set():
            try:
                connection, _ = server.accept()
            except socket.timeout:
                continue
            with connection:
                handle_client(connection, worker)
    # systemd preserves the directory inode for Docker's bind mount and removes
    # only the fixed socket in ExecStopPost.


if __name__ == '__main__':
    main()
