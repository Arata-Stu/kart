"""Timing audit UI workers, publication and confined HTTP reports; synthetic bags only."""
import json
from pathlib import Path
import subprocess
import sys
import threading
from types import SimpleNamespace
from urllib.parse import urlencode
from urllib.request import urlopen, Request
from urllib.error import HTTPError
from http.server import ThreadingHTTPServer

import pytest

from kart_studio.audit import Audit, FILES
from kart_studio.jobs import Job, Cancelled
from kart_studio.server import Handler, Studio
from kart_studio.storage import atomic_json

REPO = Path(__file__).resolve().parents[3]


@pytest.fixture
def service(tmp_path):
    records = tmp_path / 'record'
    bag = records / '2026-10-21/11-00/run'
    bag.mkdir(parents=True)
    (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information:\n  relative_file_paths: [data.mcap]\n')
    (bag / 'data.mcap').write_bytes(b'fixture')
    return Audit(REPO, tmp_path, records)


def published(service):
    key = '2026-10-21/11-00-00/audit_12345678'
    folder = service.root / key
    folder.mkdir(parents=True)
    atomic_json(folder / 'artifact.json', dict(schema='kart.pipeline_audit.artifact.v1', created=1, bag='run'))
    for file in FILES:
        (folder / file).write_text('<script>console.log(1)</script><a href="events.csv">CSV</a>' if file == 'report.html' else '{}')
    return key, folder


def test_prepare_confined_and_explicit_arguments(service):
    plan = service.prepare(dict(bag='2026-10-21/11-00/run', source_topic='/event_camera/tensor_timing', age_spike_ms=60))
    assert plan['source_topic'] == '/event_camera/tensor_timing' and plan['age_spike_ms'] == 60
    assert plan['bag'].is_relative_to(service.records)
    for body in (dict(bag='../outside'), dict(bag='/tmp'), dict(bag='2026-10-21/11-00/run', command='sh'),
                 dict(bag='2026-10-21/11-00/run', source_topic='x; rm'),
                 dict(bag='2026-10-21/11-00/run', age_spike_ms=float('nan'))):
        with pytest.raises((ValueError, OSError)):
            service.prepare(body)
    bag = service.records / '2026-10-21/11-00/run'
    (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information:\n  relative_file_paths: [../outside.mcap]\n')
    with pytest.raises(ValueError):
        service.prepare(dict(bag='2026-10-21/11-00/run'))


def test_report_and_storage_symlinks_are_refused(service, tmp_path):
    key, folder = published(service)
    assert service.catalog()['results'][0]['id'] == key
    for file in ('../metadata.yaml', '/tmp/report.html', 'artifact.json'):
        with pytest.raises(ValueError):
            service.artifact(key, file)
    (folder / 'events.csv').unlink()
    (folder / 'events.csv').symlink_to(tmp_path / 'private.csv')
    with pytest.raises(ValueError):
        service.artifact(key, 'events.csv')
    bag = service.records / '2026-10-21/11-00/run'
    (bag / 'data.mcap').unlink()
    (bag / 'data.mcap').symlink_to(tmp_path / 'private.mcap')
    with pytest.raises(ValueError):
        service.prepare(dict(bag='2026-10-21/11-00/run'))


def make_bag(path):
    import numpy as np
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    store = get_typestore(Stores.LATEST)
    kind = 'kart_interfaces/msg/ControlCommand'
    store.register(get_types_from_msg((REPO / 'ros2_ws/src/kart_interfaces/msg/ControlCommand.msg').read_text(), kind))
    def make(typ, *args): return store.types[typ](*args)
    with Writer(path, version=9) as writer:
        image_kind = 'sensor_msgs/msg/Image'
        image_connection = writer.add_connection('/realsense/color/image_raw', image_kind, typestore=store)
        command_connection = writer.add_connection('/vehicle/control_cmd', kind, typestore=store)
        for index in range(40):
            stamp = 1790000000000000000 + index * 33000000
            header = make('std_msgs/msg/Header', make('builtin_interfaces/msg/Time', *divmod(stamp, 10**9)), 'camera')
            image = make(image_kind, header, 1, 1, 'rgb8', 0, 3, np.zeros(3, np.uint8))
            command = make(kind, header, .1 if index < 20 else -.1, 0. if index == 20 else .1, 1. if index == 20 else 0., 0.)
            writer.write(image_connection, stamp + 5000000, store.serialize_cdr(image, image_kind))
            writer.write(command_connection, stamp + 30000000, store.serialize_cdr(command, kind))



def test_real_cli_job_publishes_report_and_cancel_discards_stage(tmp_path):
    records = tmp_path / 'record'
    records.mkdir()
    make_bag(records / 'run')
    service = Audit(REPO, tmp_path, records)
    service.python = sys.executable
    logs = tmp_path / 'jobs'; logs.mkdir()
    result = service.run(Job(logs, 'audit', []), service.prepare(dict(bag='run')))
    assert service.artifact(result['audit'], 'report.html').stat().st_size > 100000
    assert service.catalog()['results'][0]['observed']['mean'] == 25
    assert service.catalog()['results'][0]['final']['count'] == 0
    assert not list(service.root.glob('.build-*'))
    class CancelAfterRun:
        def log(self, _): pass
        def run(self, args): subprocess.run(args, check=True, capture_output=True)
        def check(self): raise Cancelled()
    with pytest.raises(Cancelled):
        service.run(CancelAfterRun(), service.prepare(dict(bag='run')))
    assert len(service.catalog()['results']) == 1 and not list(service.root.glob('.build-*'))


def test_detached_audit_task_reconstructs_service_and_paths(tmp_path, monkeypatch):
    import time
    from kart_studio.jobs import Jobs, Task
    records = tmp_path / 'record'
    records.mkdir()
    make_bag(records / 'run')
    monkeypatch.setenv('KART_AUDIT_PYTHON', sys.executable)
    service = Audit(REPO, tmp_path, records)
    jobs = Jobs(tmp_path / '.kart-studio/jobs', REPO, tmp_path)
    info = jobs.start('audit', ['read:record:run', 'audit-output:test'],
                      Task('audit', [service.prepare(dict(bag='run'))]))
    jobs.shutdown()
    again = Jobs(jobs.root, REPO, tmp_path)
    end = time.monotonic() + 30
    try:
        while time.monotonic() < end:
            state = again.jobs[info['id']].info()
            if state['status'] not in ('running', 'orphaned'):
                break
            time.sleep(.05)
        assert state['status'] == 'succeeded', (state, (jobs.root / (info['id'] + '.log')).read_text())
        assert service.artifact(state['result']['audit'], 'report.html').stat().st_size > 100000
        assert state['worker_pid'] and state['last_process_pid']
    finally:
        if again.jobs[info['id']].info()['status'] in ('running', 'orphaned'):
            again.jobs[info['id']].stop()


def test_api_uses_fixed_worker_and_resource_lock(service):
    calls = []
    jobs = SimpleNamespace(start=lambda title, resources, work: calls.append((title, resources)) or {'id':'job'})
    studio = SimpleNamespace(audit=service, jobs=jobs)
    assert Studio.post(studio, '/api/audit/run', dict(bag='2026-10-21/11-00/run'))['id'] == 'job'
    assert calls[0][1][0] == 'read:record:2026-10-21/11-00/run'
    assert calls[0][1][1].startswith('audit-output:')


def test_http_report_scoped_csp_downloads_and_traversal(service):
    key, folder = published(service)
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    server.studio = SimpleNamespace(audit=service)
    worker = threading.Thread(target=server.serve_forever, daemon=True); worker.start()
    base = f'http://127.0.0.1:{server.server_port}/audit-file?'
    try:
        with urlopen(base + urlencode(dict(id=key, file='report.html'))) as response:
            csp = response.headers['Content-Security-Policy']
            body = response.read().decode()
            assert "script-src 'nonce-" in csp and "frame-ancestors 'self'" in csp
            assert 'sandbox allow-scripts allow-downloads' in csp and 'allow-same-origin' not in csp
            assert '<script nonce="' in body and '/audit-file?id=' in body and 'href="events.csv"' not in body
        with urlopen(base + urlencode(dict(id=key, file='events.csv'))) as response:
            assert response.read() == b'{}' and 'attachment' in response.headers['Content-Disposition']
        opaque_headers = {'Origin':'null', 'Sec-Fetch-Site':'cross-site'}
        with pytest.raises(HTTPError):
            urlopen(Request(base + urlencode(dict(id=key, file='events.csv')), headers=opaque_headers))
        with urlopen(Request(base + urlencode(dict(id=key, file='events.csv', token=service.download_token)), headers=opaque_headers)) as response:
            assert response.read() == b'{}'
        with pytest.raises(HTTPError):
            urlopen(base + urlencode(dict(id=key, file='../metadata.yaml')))
    finally:
        server.shutdown(); server.server_close(); worker.join()
