#!/usr/bin/env python3
"""Generate runtime Docker arguments for host evdev, USB serial and plugdev access (Linux host only)."""
import grp
import os
from pathlib import Path
import platform
import shlex
import stat
import sys


def cache_helper_mount(directory):
    """Share only the explicitly installed, root-owned fixed-operation helper."""
    directory = Path(directory)
    try:
        info = directory.lstat()
    except FileNotFoundError:
        return None
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022:
        raise ValueError('Jetson cache helper directory must be root-owned and not writable by group/others')
    try:
        socket_info = (directory / 'cache.sock').lstat()
    except FileNotFoundError:
        return None
    if (not stat.S_ISSOCK(socket_info.st_mode) or socket_info.st_uid != 0
            or socket_info.st_gid == 0 or stat.S_IMODE(socket_info.st_mode) != 0o660):
        raise ValueError('Jetson cache helper socket must be root-owned, non-root group, mode 0660')
    if any(c in str(directory) for c in (',', '\n', '\r')):
        raise ValueError('Jetson cache helper path contains unsupported Docker mount characters')
    return socket_info.st_gid, f'type=bind,source={directory},target=/run/kart-cache,readonly'


def write_args(base, output, input_dir=Path('/dev/input'), dev_dir=Path('/dev'), ssh_dir=None,
               cache_dir=Path('/run/kart-cache')):
    text = Path(base).read_text()
    gids = set()
    # Include standard device groups before pairing/plugging in devices.
    for group in ('input', 'dialout', 'plugdev'):
        try:
            gids.add(grp.getgrnam(group).gr_gid)
        except KeyError:
            pass
    devices = [*input_dir.glob('event*'), *dev_dir.glob('ttyACM*'),
               *dev_dir.glob('ttyUSB*')]
    for device in devices:
        try:
            info = device.stat()
        except FileNotFoundError:
            continue
        if stat.S_ISCHR(info.st_mode):
            gids.add(info.st_gid)
    cache_helper = cache_helper_mount(cache_dir)
    if cache_helper:
        gids.add(cache_helper[0])
    additions = [f'--group-add {gid}' for gid in sorted(gids)]
    if cache_helper:
        additions.append('--mount ' + shlex.quote(cache_helper[1]))
        print('Jetson cache helper: sharing fixed-operation socket (cache clearing remains opt-in)', flush=True)
    # The CLI already mounts this directory on Jetson, but not on x86_64.
    if input_dir.is_dir() and platform.machine() != 'aarch64':
        additions.append(f'-v {input_dir}:/dev/input')
    # Match JetPilot: keep keys/config/known_hosts on the host across recreation.
    ssh_dir = Path.home() / '.ssh' if ssh_dir is None else Path(ssh_dir)
    if ssh_dir.is_dir():
        mount = f'type=bind,source={ssh_dir.resolve()},target=/home/admin/.ssh,readonly'
        if any(c in str(ssh_dir.resolve()) for c in (',', '\n', '\r')):
            raise ValueError('SSH directory contains unsupported Docker mount characters')
        additions.append('--mount ' + shlex.quote(mount))
        print('SSH: sharing host ~/.ssh read-only (keys/config/known_hosts)', flush=True)
    else:
        print('SSH: host ~/.ssh not found; configure SSH on the host first', flush=True)
    # The CLI already forwards the agent on aarch64; add the missing x86 path.
    agent = os.environ.get('SSH_AUTH_SOCK', '')
    if platform.machine() != 'aarch64' and agent and Path(agent).is_socket():
        if any(c in agent for c in (',', '\n', '\r')):
            raise ValueError('SSH agent path contains unsupported Docker mount characters')
        additions.extend([
            '--mount ' + shlex.quote(f'type=bind,source={agent},target=/kart-ssh-agent'),
            '-e SSH_AUTH_SOCK=/kart-ssh-agent',
        ])
    Path(output).write_text(text.rstrip() + '\n' + '\n'.join(additions) + '\n')
    if gids:
        print('Host device supplementary GIDs: ' + ', '.join(map(str, sorted(gids))), flush=True)
    return gids


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit('Usage: input-dockerargs.py BASE OUTPUT')
    write_args(sys.argv[1], sys.argv[2])
