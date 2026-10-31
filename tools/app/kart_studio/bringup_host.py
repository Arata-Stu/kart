"""Fixed SSH host protocol, streamed as code; requests are JSON on stdin."""
import json
from pathlib import Path
import re
import subprocess
import sys


def command(argv, **kwargs):
    result = subprocess.run(argv, capture_output=True, text=True, timeout=50, **kwargs)
    if result.returncode:
        raise ValueError(result.stderr.strip()[-1500:] or result.stdout.strip()[-1500:])
    return result.stdout


def containers():
    ids = command(['docker', 'ps', '-aq', '--no-trunc']).split()
    if not ids:
        return []
    result = []
    for item in json.loads(command(['docker', 'inspect', *ids])):
        mounts = [m for m in item.get('Mounts', []) if m['Destination'] == '/workspaces']
        if not mounts:
            continue
        root = Path(mounts[0]['Source'])
        if not (root / 'scripts/bringup.sh').is_file():
            continue
        result.append(dict(id=item['Id'], name=item['Name'].lstrip('/'),
                           running=item['State']['Running'], status=item['State']['Status'],
                           root=str(root), image=item['Config']['Image']))
    return result


def launches():
    found = []
    for file in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = file.read_bytes().decode(errors='replace').split('\0')
            if any(Path(v).name == 'ros2' for v in argv) and 'launch' in argv and 'kart_bringup' in argv:
                found.append(dict(pid=int(file.parent.name), command=' '.join(argv).strip()))
        except (OSError, ValueError):
            pass
    return found


def dispatch(body):
    action = body.get('action')
    items = containers()
    if action == 'containers':
        return dict(containers=items)
    selected = next((c for c in items if c['id'] == body.get('container')), None)
    if selected is None:
        raise ValueError('対象のkartコンテナがありません。一覧を更新してください')
    if action == 'container_start':
        if not selected['running']:
            command(['docker', 'start', selected['id']])
        return dict(containers=containers())
    if action not in ('catalog', 'status', 'assets', 'plan', 'start', 'stop',
                      'recording_status', 'recording_start', 'recording_stop'):
        raise ValueError('Unknown action')
    if not selected['running']:
        raise ValueError('Dockerが停止しています。「既存Dockerを起動」を実行してください')
    if action == 'start' and launches():
        raise ValueError('Jetsonに既存のros2 launch kart_bringupがあります。二重起動を拒否しました')
    # Match run_dev.py: admin owns both terminal and Studio tmux sessions.
    script = ('if [ -f /etc/kart/ros.bash ]; then source /etc/kart/ros.bash >/dev/null; fi; '
              'exec python3 /workspaces/scripts/lib/bringup_remote.py')
    raw = command(['docker', 'exec', '-i', '-u', 'admin', '-w', '/workspaces',
                   '-e', 'KART_BRINGUP_TMUX_SOCKET=kart-bringup-' + selected['id'][:12], selected['id'],
                   'bash', '-c', script], input=json.dumps(body))
    return json.loads(raw)


if __name__ == '__main__':
    try:
        print(json.dumps(dispatch(json.load(sys.stdin)), ensure_ascii=False))
    except Exception as exc:
        print(json.dumps({'error': str(exc)}, ensure_ascii=False))
        sys.exit(1)
