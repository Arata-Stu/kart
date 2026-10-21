"""Detached PID-tracked jobs. Durable JSON state, process identity and data locks."""

import os
import signal
import subprocess
import threading
import time
import uuid
import fcntl
import hashlib
import json
from pathlib import Path
import re
import sys
from dataclasses import dataclass
from contextlib import contextmanager

from .storage import atomic_json


def process_identity(pid):
    """PID reuse guard. Linux boot/start ticks; macOS birth time (argv may change after exec)."""
    if type(pid) is not int or pid <= 1:
        return None
    try:
        stat = Path(f'/proc/{pid}/stat')
        if stat.exists():
            fields = stat.read_text().rsplit(')', 1)[1].split()
            if fields[0] in ('Z', 'X'):
                return None
            return Path('/proc/sys/kernel/random/boot_id').read_text().strip() + ':' + fields[19]
        result = subprocess.run(['ps', '-ww', '-p', str(pid), '-o', 'stat=', '-o', 'lstart='],
                                capture_output=True, text=True, timeout=2, env={**os.environ, 'LC_ALL':'C'})
        value = result.stdout.strip()
        if result.returncode or not value or value.split()[0].startswith('Z'):
            return None
        return hashlib.sha256(value.split(None, 1)[1].encode()).hexdigest()
    except (OSError, subprocess.TimeoutExpired):
        return None


def resource_conflict(left, right):
    a = left.removeprefix('read:')
    b = right.removeprefix('read:')
    return a == b and not (left.startswith('read:') and right.startswith('read:'))


def pid_exists(pid):
    if type(pid) is not int or pid <= 1:
        return False
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True  # Unknown identity is not evidence that an old task has exited.


def pack(value):
    if isinstance(value, Path):
        return {'__kart_path__': str(value)}
    if isinstance(value, (list, tuple)):
        return [pack(v) for v in value]
    if isinstance(value, dict):
        return {k: pack(v) for k, v in value.items()}
    return value


def unpack(value):
    if isinstance(value, dict):
        return Path(value['__kart_path__']) if set(value) == {'__kart_path__'} else {k:unpack(v) for k,v in value.items()}
    if isinstance(value, list):
        return [unpack(v) for v in value]
    return value


@dataclass
class Task:
    action: str
    args: list


def worker_locked(path):
    with path.open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        return False


class Cancelled(Exception):
    pass


class Job:
    def __init__(self, root, title, resources, key=None):
        self.id = key or uuid.uuid4().hex[:12]
        self.title, self.resources = title, resources
        self.status, self.message = "running", "開始しています"
        self.started = time.time()
        self.started_monotonic = time.monotonic()
        self.finished = None
        self.duration_s = None
        self.cancel = threading.Event()
        self.lock = threading.RLock()
        self.process = None
        self.persist_enabled = False
        self.worker_pid = self.worker_identity = None
        self.process_pid = self.process_identity = self.last_process_pid = None
        self.result = None
        self.diagnostic = None
        self.log_path = root / (self.id + ".log")
        self.state_path = root / (self.id + ".json")

    def info(self):
        with self.lock:
            return dict(
                id=self.id,
                title=self.title,
                status=self.status,
                message=self.message,
                started=self.started,
                started_monotonic=self.started_monotonic,
                finished=self.finished,
                elapsed_s=self.duration_s if self.duration_s is not None else max(0, time.monotonic() - self.started_monotonic),
                cancel_requested=self.cancel.is_set(),
                result=self.result,
                diagnostic=self.diagnostic,
                resources=self.resources, worker_pid=self.worker_pid, worker_identity=self.worker_identity,
                process_pid=self.process_pid, process_identity=self.process_identity, last_process_pid=self.last_process_pid,
                heartbeat=time.time(),
            )

    def persist(self):
        if self.persist_enabled:
            with self.lock:
                atomic_json(self.state_path, self.info())

    def check(self):
        if self.cancel.is_set():
            raise Cancelled("中止しました。一時ファイルは成果物として公開されません")

    def log(self, message):
        with self.lock:
            self.message = str(message)
            with self.log_path.open("a") as stream:
                stream.write(str(message) + "\n")
            self.persist()

    def run(self, args, *, env=None, timeout=None):
        self.check()
        self.log("実行: " + " ".join(map(str, args)))
        with self.log_path.open("a") as stream:
            with self.lock:
                self.check()
                self.process = subprocess.Popen(
                    args,
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    env=env,
                    start_new_session=True,
                )
                process = self.process
                self.process_pid = self.last_process_pid = process.pid
                self.process_identity = process_identity(process.pid)
                self.persist()
            try:
                code = process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self._terminate(signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._terminate(signal.SIGKILL)
                    process.wait()
                raise ValueError("処理がタイムアウトしました。ログを確認してください")
            except BaseException:
                self._terminate(signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._terminate(signal.SIGKILL)
                    process.wait()
                raise
            finally:
                with self.lock:
                    self.process = None
                    self.process_pid = self.process_identity = None
                    self.persist()
        self.check()
        if code:
            raise ValueError(
                f"処理が失敗しました（終了コード {code}）。ログを確認してください"
            )

    def _terminate(self, sig):
        with self.lock:
            if self.process and self.process.poll() is None:
                try:
                    os.killpg(self.process.pid, sig)
                except ProcessLookupError:
                    pass

    def stop(self):
        with self.lock:
            if self.status != "running" or self.cancel.is_set():
                return
            self.cancel.set()
        self.persist()
        self._terminate(signal.SIGINT)

        def escalate():
            time.sleep(8)
            self._terminate(signal.SIGTERM)
            time.sleep(3)
            self._terminate(signal.SIGKILL)

        threading.Thread(target=escalate, daemon=True).start()



ACTIVE = ('running', 'orphaned')


class JobRef:
    def __init__(self, owner, key):
        self.owner, self.id = owner, key
        self.state_path = owner.root / (key + '.json')

    def info(self):
        return self.owner.read(self.state_path)

    @property
    def status(self): return self.info()['status']
    @property
    def resources(self): return self.info().get('resources', [])
    @property
    def finished(self): return self.info().get('finished')
    @property
    def process(self): return None

    def stop(self):
        info = self.info()
        if info['status'] not in ACTIVE:
            return
        atomic_json(self.owner.root / (self.id + '.cancel'), {'requested': time.time()})
        if info['status'] == 'orphaned':
            pid = info.get('process_pid')
            expected = info.get('process_identity')
            if not expected or process_identity(pid) != expected:
                raise ValueError('実行プロセスの開始情報を検証できないため、他のPIDへ停止信号を送りません')
            if os.getpgid(pid) != pid:
                raise ValueError('実行プロセスグループを検証できません')
            os.killpg(pid, signal.SIGINT)


class Jobs:
    def __init__(self, root, repo=None, base=None):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.repo = Path(repo or Path(__file__).resolve().parents[3]).resolve()
        self.base = Path(base or self.root.parent.parent).resolve()
        self.lock = threading.RLock()
        self._guards = []
        self._spawned = {}
        self.jobs = {}
        self.list()

    @contextmanager
    def registry(self):
        with self.lock, (self.root / '.registry.lock').open('a') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            yield

    def read(self, path):
        from .storage import read_json
        state = read_json(path)
        if state.get('status') in ACTIVE:
            alive = worker_locked(self.root / (path.stem + '.worker.lock'))
            started = state.get('started_monotonic')
            startup = 0 <= time.monotonic() - started < 10 if isinstance(started, (float, int)) else 0 <= time.time() - state.get('started', 0) < 10
            if not alive and not startup:
                pid = state.get('process_pid')
                identity = process_identity(pid)
                if pid and identity and identity == state.get('process_identity'):
                    state.update(status='orphaned', message='管理プロセス喪失・子プロセスは実行中。結果は未確認')
                elif pid and identity is None and pid_exists(pid):
                    state.update(status='orphaned', message='管理プロセス喪失・子PIDの開始情報を検証できません。資源を保持します')
                elif state.get('worker_pid') and process_identity(state['worker_pid']) == state.get('worker_identity') and state.get('worker_identity'):
                    state.update(message='管理プロセス起動確認中')
                else:
                    state.update(status='interrupted', message='実行プロセスを確認できません。自動再実行は行いません',
                                 elapsed_s=None, finished=None)
                atomic_json(path, state)
            if state.get('status') in ACTIVE and state.get('started_monotonic') is not None:
                state['elapsed_s'] = max(0, time.monotonic() - state['started_monotonic'])
            state['cancel_requested'] = (self.root / (path.stem + '.cancel')).exists()
        return state

    def _states(self):
        states = []
        for path in self.root.glob('*.json'):
            if path.name.endswith('.request.json'):
                continue
            try:
                state = self.read(path)
                states.append(state)
                if re.fullmatch(r'[a-f0-9]{12}', path.stem):
                    self.jobs.setdefault(path.stem, JobRef(self, path.stem))
            except (OSError, ValueError, KeyError):
                continue
        for key, process in list(self._spawned.items()):
            if process.poll() is not None:
                self._spawned.pop(key)
        return states

    @property
    def claimed(self):
        return set(self._guards) | {r for s in self._states() if s['status'] in ACTIVE for r in s.get('resources', [])}

    def _require_available(self, resources):
        states = [s for s in self._states() if s['status'] in ACTIVE]
        for state in states:
            conflicts = [a for a in resources for b in state.get('resources', []) if resource_conflict(a, b)]
            if conflicts:
                raise ValueError(f'資源が使用中です: {conflicts[0]} / {state["title"]} (PID {state.get("process_pid") or state.get("worker_pid") or "起動中"})')
        if any(resource_conflict(a, b) for a in resources for b in self._guards):
            raise ValueError('対象データを変更中です。完了を待ってください')

    @contextmanager
    def guard(self, *resources):
        with self.registry():
            self._require_available(resources)
            self._guards.extend(resources)
        try:
            yield
        finally:
            with self.lock:
                for resource in resources:
                    self._guards.remove(resource)

    def start(self, title, resources, work):
        if not isinstance(work, Task):
            raise ValueError('ジョブには再接続可能なTask定義が必要です')
        from .job_worker import ACTIONS
        if work.action not in ACTIONS:
            raise ValueError('Unknown worker action')
        with self.registry():
            self._require_available(resources)
            job = Job(self.root, title, resources)
            request = self.root / (job.id + '.request.json')
            atomic_json(request, dict(schema=1, repo=str(self.repo), base=str(self.base),
                                     action=work.action, args=pack(work.args), state=job.info()))
            atomic_json(job.state_path, job.info())
            env = dict(os.environ)
            sources = [str(self.repo/'tools/app'), str(self.repo/'ros2_ws/src/kart_mapping'),
                       str(self.repo/'ros2_ws/src/kart_bringup')]
            env['PYTHONPATH'] = os.pathsep.join(sources + [env.get('PYTHONPATH', '')])
            try:
                with job.log_path.open('a') as stream:
                    process = subprocess.Popen([sys.executable, '-m', 'kart_studio.job_worker', str(request)],
                                               stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT,
                                               env=env, start_new_session=True, close_fds=True)
                self._spawned[job.id] = process
            except BaseException:
                job.status = 'failed'; job.message = '管理プロセスを起動できませんでした'
                atomic_json(job.state_path, job.info())
                raise
            ref = JobRef(self, job.id)
            self.jobs[job.id] = ref
            return ref.info()

    def list(self):
        with self.lock:
            states = sorted(self._states(), key=lambda s:s['started'], reverse=True)
            return [s for s in states if s['status'] in ACTIVE] + [s for s in states if s['status'] not in ACTIVE][:40]

    def shutdown(self):
        # UI shutdown detaches. Only explicit per-job STOP cancels the supervisor.
        for process in self._spawned.values():
            process.poll()
