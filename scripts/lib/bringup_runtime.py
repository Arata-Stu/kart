"""Shared CLI/TUI/Studio ownership; flock lives for the whole ROS launch lifetime."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess

ROOT = Path(__file__).resolve().parents[2]
STATE = ROOT / '.kart-runtime'


def legacy_processes(proc=Path('/proc')):
    """Conservative detection of launches that predate the common lock."""
    result = []
    for path in proc.glob('[0-9]*/cmdline'):
        try:
            argv = path.read_bytes().decode(errors='replace').split('\0')
            if any(Path(v).name == 'ros2' for v in argv) and 'launch' in argv and 'kart_bringup' in argv:
                result.append({'pid': int(path.parent.name), 'command': ' '.join(argv).strip()})
        except (OSError, ValueError):
            continue
    return result


def lock_file():
    STATE.mkdir(parents=True, exist_ok=True)
    return (STATE / 'bringup.lock').open('a+')


def ownership():
    with lock_file() as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            stream.seek(0)
            try:
                owner = json.load(stream)
            except ValueError:
                owner = {}
            return {'locked': True, 'owner': owner}
    return {'locked': False, 'owner': None}


def run_command(command):
    with lock_file() as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError('bringupは起動済みです。既存のUI/tmux/端末から終了してください') from exc
        if legacy_processes():
            raise ValueError('既存のros2 launch kart_bringupを検出しました。先に終了してください')
        stream.seek(0)
        stream.truncate()
        json.dump({'pid': os.getpid(), 'command': command}, stream)
        stream.flush()
        child = None
        pending = []
        def forward(sig, _frame):
            if child is None:
                pending.append(sig)
                return
            try:
                os.killpg(child.pid, sig)
            except ProcessLookupError:
                pass
        previous = {sig: signal.signal(sig, forward) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
        try:
            child = subprocess.Popen(command, start_new_session=True)
            for sig in pending:
                forward(sig, None)
            code = child.wait()
            return code if code >= 0 else 128 - code
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
