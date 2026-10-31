"""One detached supervisor per job. UI may exit; this process owns work and cleanup."""
import fcntl
import os
from pathlib import Path
import signal
import sys
import threading
import time

from .jobs import Job, Jobs, ACTIVE, Cancelled, process_identity, unpack
from .storage import read_json

ACTIONS = {'coverage', 'command', 'audit', 'learning', 'model_push', 'record_pull', 'map_push',
           'map_copy', 'line_generate', 'vgl', 'mesh_color', 'dense_step', 'capture', 'map_build'}


def perform(job, work):
    """Late cancellation cannot undo an already published result."""
    try:
        job.result = work()
        job.status = 'succeeded'
        job.log('完了')
    except (Cancelled, KeyboardInterrupt) as exc:
        job.status = 'cancelled'; job.log(str(exc) or '中止しました')
    except BaseException as exc:
        job.status = 'failed'; job.diagnostic = getattr(exc, 'diagnostic', None); job.log(exc)


def execute(action, args, job, repo, base):
    if action == 'command':
        job.run(args[0])  # Internal/test API only; no HTTP endpoint accepts commands.
        return {'exit_code': 0}
    from .maps import Maps
    from .mapping import Mapping
    from .e2e import Learning
    from .transfers import Transfers
    from .audit import Audit
    from types import SimpleNamespace
    maps, records = Maps(base / 'map'), base / 'record'
    studio = SimpleNamespace(repo=repo, records=records, maps=maps,
                             mapping=Mapping(repo, records, maps))
    if action == 'coverage':
        from .coverage import Coverage
        return Coverage(repo, base, records).run(job, *args)
    if action == 'audit': return Audit(repo, base, records).run(job, *args)
    if action == 'learning': return Learning(repo, base, records, base / '.kart-studio').execute(job, *args)
    if action == 'model_push': return Learning(repo, base, records, base / '.kart-studio').push(job, *args)
    if action == 'record_pull': return Transfers(records, maps).pull(job, *args)
    if action == 'map_push': return Transfers(records, maps).push(job, *args)
    if action == 'map_copy':
        from .copies import copy_map
        return copy_map(maps, job, *args)
    if action == 'line_generate':
        key, kind, settings, revision, lane = args
        return maps.generate(key, kind, settings, revision, job.check, lane)
    if action == 'vgl':
        from .vgl import build
        return build(job, *args)
    if action == 'mesh_color':
        from .mesh_color import build
        return build(studio, job, *args)
    if action == 'dense_step':
        from .dense_steps import build
        return build(studio, job, *args)
    if action == 'capture':
        from .capture import capture
        return capture(studio, job, *args)
    if action == 'map_build': return studio.mapping.build(job, *args)
    raise ValueError('Unknown action')


def main():
    request = Path(sys.argv[1]).resolve()
    config = read_json(request)
    if config.get('schema') != 1 or config['action'] not in ACTIONS:
        raise ValueError('Invalid durable task request')
    saved = config['state']
    key = saved['id']
    if request.name != key + '.request.json':
        raise ValueError('Job identity does not match request file')
    root = request.parent
    with (root / (key + '.worker.lock')).open('a') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        job = Job(root, saved['title'], saved['resources'], key)
        job.queue = saved.get('queue')
        job.status = saved['status']
        job.started = saved['started']
        job.started_monotonic = saved['started_monotonic']
        job.worker_pid, job.worker_identity = os.getpid(), process_identity(os.getpid())
        job.persist_enabled = True
        done = threading.Event()
        def watch():
            while not done.wait(.2):
                if (root / (key + '.cancel')).exists():
                    job.stop()
                job.persist()
        def cancelled(_signum, _frame):
            job.stop()
            raise Cancelled('管理プロセスに停止要求が届きました')
        signal.signal(signal.SIGINT, cancelled)
        signal.signal(signal.SIGTERM, cancelled)
        monitor = threading.Thread(target=watch, daemon=True)
        job.persist(); monitor.start()
        def work():
            job.check()
            if (root / (key + '.cancel')).exists(): raise Cancelled('開始前に中止しました')
            if job.queue:
                manager = Jobs(root, Path(config['repo']), Path(config['base']))
                pending = config.get('predecessors', [])
                while pending:
                    job.check()
                    pending = [key for key in pending
                               if manager.read(root / (key + '.json'))['status'] in ACTIVE]
                    if pending:
                        job.cancel.wait(.2)
                job.check()
                job.status = 'running'
                job.log('転送キューから開始')
            return execute(config['action'], unpack(config['args']), job, Path(config['repo']), Path(config['base']))
        try:
            perform(job, work)
        finally:
            # Stop monitoring before the final atomic state; a heartbeat must not overwrite it.
            done.set(); monitor.join(timeout=2)
            job.finished = time.time()
            job.duration_s = max(0, time.monotonic() - job.started_monotonic)
            job.persist()


if __name__ == '__main__':
    main()
