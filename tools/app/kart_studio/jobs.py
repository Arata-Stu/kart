"""Cancellable jobs; workers own arguments, callers cannot submit commands."""

import os
import signal
import subprocess
import threading
import time
import uuid
from contextlib import contextmanager

from .storage import atomic_json


class Cancelled(Exception):
    pass


class Job:
    def __init__(self, root, title, resources):
        self.id = uuid.uuid4().hex[:12]
        self.title, self.resources = title, resources
        self.status, self.message = "running", "開始しています"
        self.started = time.time()
        self.cancel = threading.Event()
        self.lock = threading.RLock()
        self.process = None
        self.result = None
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
                result=self.result,
            )

    def check(self):
        if self.cancel.is_set():
            raise Cancelled("中止しました。一時ファイルは成果物として公開されません")

    def log(self, message):
        with self.lock:
            self.message = str(message)
            with self.log_path.open("a") as stream:
                stream.write(str(message) + "\n")

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
            finally:
                with self.lock:
                    self.process = None
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
        self.cancel.set()
        self._terminate(signal.SIGINT)

        def escalate():
            time.sleep(8)
            self._terminate(signal.SIGTERM)
            time.sleep(3)
            self._terminate(signal.SIGKILL)

        threading.Thread(target=escalate, daemon=True).start()


class Jobs:
    def __init__(self, root):
        self.root = root
        root.mkdir(parents=True, exist_ok=True)
        self.jobs, self.claimed = {}, set()
        self.lock = threading.RLock()
        self.history = []
        for path in sorted(
            root.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True
        )[:30]:
            from .storage import read_json

            try:
                state = read_json(path)
                if state["status"] == "running":
                    state.update(
                        status="interrupted", message="サーバー終了により結果未確認"
                    )
                self.history.append(state)
            except (ValueError, KeyError):
                pass

    @contextmanager
    def guard(self, *resources):
        with self.lock:
            if self.claimed.intersection(resources):
                raise ValueError(
                    "同じデータを処理中です。完了または中止を待ってください"
                )
            self.claimed.update(resources)
        try:
            yield
        finally:
            with self.lock:
                self.claimed.difference_update(resources)

    def start(self, title, resources, work):
        with self.lock:
            if self.claimed.intersection(resources):
                raise ValueError("同じデータを処理中です")
            self.claimed.update(resources)
            job = Job(self.root, title, resources)
            self.jobs[job.id] = job
            atomic_json(job.state_path, job.info())

        def execute():
            try:
                job.result = work(job)
                # Workers check cancellation immediately before publication.
                # A late stop cannot undo an already committed result.
                job.status = "succeeded"
                job.log("完了")
            except Cancelled as exc:
                job.status = "cancelled"
                job.log(exc)
            except Exception as exc:
                job.status = "failed"
                job.log(exc)
            finally:
                atomic_json(job.state_path, job.info())
                with self.lock:
                    self.claimed.difference_update(resources)

        threading.Thread(target=execute, daemon=True).start()
        return job.info()

    def list(self):
        with self.lock:
            return sorted(
                [j.info() for j in self.jobs.values()] + self.history,
                key=lambda j: j["started"],
                reverse=True,
            )[:40]

    def shutdown(self):
        active = [job for job in self.jobs.values() if job.status == "running"]
        for job in active:
            job.stop()
        deadline = time.monotonic() + 12
        while (
            any(job.process is not None for job in active)
            and time.monotonic() < deadline
        ):
            time.sleep(0.1)
        for job in active:
            job._terminate(signal.SIGKILL)
