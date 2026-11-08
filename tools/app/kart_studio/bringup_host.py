"""Fixed SSH host protocol, streamed as code; requests are JSON on stdin."""
import json
import os
from pathlib import Path
import re
import select
import subprocess
import sys
import time


def command(argv, **kwargs):
    result = subprocess.run(argv, capture_output=True, text=True, timeout=50, **kwargs)
    if result.returncode:
        raise ValueError('\n'.join(part.strip()[-1500:] for part in
                                   (result.stdout, result.stderr) if part.strip()))
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


def controller_rssi(address):
    # hcitool reads the active BR/EDR link. This is a relative RSSI, not dBm.
    if not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', address):
        return None
    try:
        result = subprocess.run(['hcitool', 'rssi', address], capture_output=True,
                                text=True, timeout=1)
    except (OSError, subprocess.TimeoutExpired):
        return None
    match = re.search(r'RSSI return value:\s*(-?\d+)', result.stdout)
    if result.returncode == 0 and match and int(match[1]) != 127:
        return int(match[1])
    return None


def sample_reports(fd, duration=0.5):
    previous = latest = maximum = None
    count = 0
    deadline = time.monotonic() + duration
    while (remaining := deadline - time.monotonic()) > 0:
        if not select.select([fd], [], [], remaining)[0]:
            break
        report = os.read(fd, 128)
        if not report:
            raise OSError('HID切断')
        if report[0] not in (0x01, 0x31):
            continue
        now = time.monotonic()
        latest = time.time_ns() // 1_000_000
        if previous is not None:
            maximum = max(maximum or 0, (now - previous) * 1000)
        previous = now
        count += 1
    age = (time.monotonic() - previous) * 1000 if previous is not None else None
    return dict(last_report_ms=latest, report_age_ms=age, max_gap_ms=maximum, reports=count)


def bluetooth_status():
    devices = []
    for entry in Path('/sys/class/hidraw').glob('hidraw*'):
        try:
            fields = dict(line.split('=', 1) for line in
                          (entry / 'device/uevent').read_text().splitlines() if '=' in line)
        except FileNotFoundError:
            continue
        if fields.get('HID_ID', '').upper() in ('0005:0000054C:00000CE6', '0005:0000054C:00000DF2'):
            devices.append((entry.name, fields.get('HID_UNIQ', '')))
    if len(devices) != 1:
        return dict(state='Bluetooth DualSense未検出' if not devices else '複数のBluetooth DualSenseを検出',
                    measured_ms=time.time_ns() // 1_000_000)
    device, address = devices[0]
    result = dict(state='Bluetooth HID検出', rssi=controller_rssi(address), address=address)
    try:
        fd = os.open('/dev/' + device, os.O_RDONLY | os.O_NONBLOCK)
        try:
            result.update(sample_reports(fd))
        finally:
            os.close(fd)
    except PermissionError:
        result['state'] = 'HID検出・読み取り権限なし'
    except OSError as exc:
        result['state'] = 'HID読み取り失敗: ' + str(exc)
    result['measured_ms'] = time.time_ns() // 1_000_000
    return result


def dispatch(body):
    action = body.get('action')
    if action == 'bluetooth_status':
        return bluetooth_status()
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
                      'link_timing', 'mode_status', 'mode_auto', 'mode_manual', 'mode_stop',
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
