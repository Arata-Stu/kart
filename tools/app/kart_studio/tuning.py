"""Notebook tuning: reuse the saved SSH profile, compile locally, send snapshots."""
import importlib.util
import json
import os
import stat
import shlex
import subprocess
import sys
import threading
from pathlib import Path

from .transfers import profile, SSH_OPTIONS

# Import the ROS-free protocol directly from this checkout; no ROS install on Mac.
_source = Path(__file__).resolve().parents[3] / 'ros2_ws/src/kart_hdmap/kart_hdmap'
_spec = importlib.util.spec_from_file_location('_kart_tuning_protocol', _source / '__init__.py', submodule_search_locations=[str(_source)])
_package = importlib.util.module_from_spec(_spec)
sys.modules.setdefault(_spec.name, _package)
_protocol = importlib.import_module('_kart_tuning_protocol.tuning')


def remote(connection, action, payload):
    if action not in ('status', 'apply', 'rollback', 'parameters_get', 'parameters_set', 'parking_start', 'parking_reset'):
        raise ValueError('Unknown tuning action')
    p = profile(connection)
    script = """import sys,urllib.request,urllib.error
req=urllib.request.Request('http://127.0.0.1:8782/""" + action + """',data=sys.stdin.buffer.read(),headers={'Content-Type':'application/json'})
try:
    with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req,timeout=6) as r: print(r.read().decode())
except urllib.error.HTTPError as e: print(e.read().decode())
"""
    raw = _protocol.encoded(payload)
    if len(raw) > _protocol.MAX_BYTES + 4096:
        raise ValueError('調整データが大きすぎます')
    control_dir = Path('/tmp') / f'kart-tuning-{os.getuid()}'
    control_dir.mkdir(mode=0o700, exist_ok=True)
    info = control_dir.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise ValueError('SSH接続共有ディレクトリの権限が不正です')
    try:
        result = subprocess.run(['ssh', *SSH_OPTIONS, '-o', 'ControlMaster=auto',
                                 '-o', 'ControlPersist=30', '-o', f'ControlPath={control_dir}/%C',
                                 '-p', str(p['port']), f"{p['user']}@{p['host']}",
                                 'python3 -c ' + shlex.quote(script)],
                                input=raw, capture_output=True, timeout=10)
    except subprocess.TimeoutExpired as exc:
        raise ValueError('Jetsonの応答がありません。適用結果は未確認です。自動再送は行いません') from exc
    if result.returncode:
        raise ValueError('SSH接続またはJetsonの調整窓口(8782)を確認してください: ' + result.stderr.decode(errors='replace')[-400:])
    try:
        response = json.loads(result.stdout)
    except ValueError as exc:
        raise ValueError('Jetsonから正しいJSON応答がありません') from exc
    if not isinstance(response, dict) or response.get('error'):
        raise ValueError(response.get('error', 'Invalid tuning response') if isinstance(response, dict) else 'Invalid tuning response')
    return response


class Tuning:
    def __init__(self, maps):
        self.maps = maps
        self.identities = {}
        self.lock = threading.Lock()

    def identity(self, key):
        folder = self.maps.folder(key)
        # Localization assets are immutable for a connected session. Hash once,
        # never read a potentially large SLAM map on every telemetry poll.
        with self.lock:
            if key not in self.identities:
                self.identities[key] = _protocol.map_identity(folder)
            return self.identities[key]

    def prepare(self, body):
        key = body['id']
        document = self.maps.load(key)
        if document['revision'] != body['document_revision']:
            raise ValueError('地図が更新されています。再読み込みしてください')
        # Reuse footprint/corridor checks and generated-line freshness validation.
        self.maps.export(key)
        candidate = dict(map_id=self.identity(key),
                         document=document, lane_id=body['lane_id'],
                         line_type=body['line_type'], centerline_speed_mps=body['centerline_speed_mps'])
        candidate['revision'] = _protocol.digest(candidate)
        _protocol.validate(candidate, candidate['map_id'])
        return candidate

    def request(self, body, action):
        identity = self.identity(body['id'])
        payload = dict(map_id=identity, known_revision=body.get('known_revision'))
        if action == 'apply':
            candidate = body['snapshot']
            _protocol.validate(candidate, identity)
            if candidate['document'] != self.maps.load(body['id']):
                raise ValueError('プレビュー後に地図が変わりました。作り直してください')
            payload['snapshot'] = candidate
        if action in ('apply', 'rollback', 'parameters_set', 'parking_start', 'parking_reset'):
            payload['expected_revision'] = body['expected_revision']
        if action in ('parameters_get', 'parameters_set'):
            from _kart_tuning_protocol.parameter_policy import target
            target(body['target'])
            payload['target'] = body['target']
        if action == 'parameters_set':
            payload['changes'] = body['changes']
        response = remote(body['connection'], action, payload)
        if response.get('map_id') != identity:
            raise ValueError('NotebookとJetsonの地図が一致しません')
        return response
