"""Notebook adapter for explicit Jetson bringup requests and saved startup presets."""
import json
from pathlib import Path
import re
import shlex
import subprocess
import threading

from .storage import atomic_json, read_json
from .transfers import profile, SSH_OPTIONS

HOST_AGENT = Path(__file__).with_name('bringup_host.py').read_text()
ACTIONS = {'containers', 'container_start', 'catalog', 'status', 'assets', 'plan', 'start', 'stop'}


def remote(connection, body):
    p = profile(connection)
    if body.get('action') not in ACTIONS:
        raise ValueError('Unknown bringup action')
    try:
        response = subprocess.run(['ssh', *SSH_OPTIONS, '-p', str(p['port']),
                                   f"{p['user']}@{p['host']}", 'python3 -c ' + shlex.quote(HOST_AGENT)],
                                  input=json.dumps(body), text=True, capture_output=True, timeout=60)
    except subprocess.TimeoutExpired as exc:
        raise ValueError('接続がタイムアウトしました。起動・終了は自動再送しません。状態を再取得してください') from exc
    try:
        result = json.loads(response.stdout)
    except ValueError as exc:
        raise ValueError('Jetson接続を確認してください: ' + response.stderr[-1000:]) from exc
    if response.returncode or result.get('error'):
        raise ValueError(result.get('error') or response.stderr[-1000:])
    return result


class Bringup:
    def __init__(self, state):
        self.file = state / 'bringup-presets.json'
        self.lock = threading.Lock()

    def request(self, body, action):
        p = profile(body['connection'])
        key = f"{p['user']}@{p['host']}:{p['port']}"
        if action in ('preset_get', 'preset_save'):
            with self.lock:
                presets = read_json(self.file) if self.file.exists() else {}
                if action == 'preset_save':
                    value = body.get('preset')
                    if not isinstance(value, dict) or not isinstance(value.get('values'), dict):
                        raise ValueError('起動設定が不正です')
                    # Presets contain data only, never a command string.
                    presets[key] = value
                    atomic_json(self.file, presets)
                return dict(preset=presets.get(key))
        payload = {k: body[k] for k in ('container', 'values', 'revision', 'run_id', 'selections') if k in body}
        if action != 'containers' and not re.fullmatch('[a-f0-9]{64}', str(payload.get('container', ''))):
            raise ValueError('対象Dockerを選択してください')
        return remote(p, dict(payload, action=action))
