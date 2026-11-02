"""Portable protocol and real tmux lifecycle tests; no SSH, Docker or ROS execution."""
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import tempfile
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import patch

import pytest
import bringup_remote as remote
import bringup_runtime as runtime


def test_plan_uses_shared_validation_and_shell_values_are_data():
    values = dict(mode='esc-trial', device='/dev/a $(touch /tmp/not-executed)')
    prepared = remote.plan(values)
    assert prepared['command'][-1] == 'device:=' + values['device']
    assert '--device=' + values['device'] in remote.arguments(values)
    assert len(prepared['revision']) == 64
    with pytest.raises(ValueError):
        remote.plan(dict(mode='esc-trial', evs=True))
    with pytest.raises(ValueError):
        remote.arguments(dict(mode='esc-trial', command='echo bad'))
    with pytest.raises(ValueError):
        remote.arguments(dict(mode='collect', evs='false'))


def test_config_change_invalidates_review(tmp_path):
    config = tmp_path/'settings.yaml'
    config.write_text('a: 1')
    with patch.object(remote, 'CONFIG', tmp_path):
        first = remote.plan({'mode':'esc-trial'})
        config.write_text('a: 2')
        second = remote.plan({'mode':'esc-trial'})
    assert first['revision'] != second['revision']


def test_asset_choices_reuse_editor_and_reject_changed_candidates():
    def candidates(editor):
        editor.args.e2e_model_dir = editor.choose('model', [Path('/a'),Path('/b')])
    with patch('bringup_tui.Editor.assets', candidates):
        first = remote.assets({'mode':'e2e'}, [])
        assert first['prompt']['values'] == ['/a','/b']
        selected = remote.assets({'mode':'e2e'}, ['/b'])
        assert selected['values']['e2e_model_dir'] == '/b'
        with pytest.raises(ValueError):
            remote.assets({'mode':'e2e'}, ['/gone'])


def test_flock_blocks_other_process_and_is_released_on_exit(tmp_path):
    with patch.object(runtime, 'STATE', tmp_path):
        with runtime.lock_file() as stream:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            command = [sys.executable, '-c',
                       'import bringup_runtime as r; from pathlib import Path; '
                       f'r.STATE=Path({str(tmp_path)!r}); r.run_command(["does-not-exist"])']
            p = subprocess.run(command, capture_output=True, text=True)
            assert p.returncode != 0 and '起動済み' in p.stderr
            assert runtime.ownership()['locked']
        assert not runtime.ownership()['locked']


def test_legacy_detection_is_argv_based(tmp_path):
    (tmp_path/'42').mkdir()
    (tmp_path/'42/cmdline').write_bytes(b'python3\0/usr/bin/ros2\0launch\0kart_bringup\0mission.launch.py\0')
    (tmp_path/'43').mkdir()
    (tmp_path/'43/cmdline').write_bytes(b'sh\0-c\0echo ros2 launch kart_bringup\0')
    assert [p['pid'] for p in runtime.legacy_processes(tmp_path)] == [42]


def test_start_and_stop_reject_stale_or_external_session():
    current = dict(session=False,ownership={'locked':False},legacy=[])
    with patch.object(remote,'status',return_value=current), patch.object(remote,'tmux') as tmux:
        with pytest.raises(ValueError, match='変更'):
            remote.start({'values':{'mode':'esc-trial'},'revision':'stale'})
        tmux.assert_not_called()
    with patch.object(remote,'status',return_value=dict(current,session=True,run_id='a',phase='running',managed=False)), patch.object(remote,'tmux') as tmux:
        with pytest.raises(ValueError):remote.stop({'run_id':'b'})
        with pytest.raises(ValueError):remote.stop({'run_id':'a'})
        tmux.assert_not_called()


@pytest.mark.skipif(not shutil.which('tmux'), reason='tmux unavailable')
def test_real_tmux_detach_reconnect_duplicate_and_sigint(tmp_path):
    root = tmp_path/'repo'
    lib = root/'scripts/lib'
    lib.mkdir(parents=True)
    for name in ('bringup.py','bringup_remote.py','bringup_runtime.py'):
        shutil.copy(Path(remote.__file__).parent/name, lib/name)
    # Exercise the actual common guard and logger with a harmless stand-in for ROS.
    (root/'scripts/bringup.sh').write_text(
        '#!/bin/bash\nexec '+sys.executable+' '+str(lib/'dummy.py')+'\n')
    dummy_command = [sys.executable, '-u', '-c', 'import time; print("READY",flush=True); time.sleep(90)']
    (lib/'dummy.py').write_text('import sys\nfrom bringup_runtime import run_command\nsys.exit(run_command(' + repr(dummy_command) + '))\n')
    socket_dir = tempfile.TemporaryDirectory(prefix='kart-tmux-', dir='/tmp')
    sock = Path(socket_dir.name)
    env = dict(os.environ, TMUX_TMPDIR=str(sock), PYTHONPATH=os.pathsep.join(sys.path), TMUX='')
    def call(action, **extra):
        p = subprocess.run([sys.executable,str(lib/'bringup_remote.py')],
                           input=json.dumps(dict(action=action,**extra)),text=True,capture_output=True,env=env,timeout=10)
        try: result=json.loads(p.stdout)
        except ValueError: pytest.fail(p.stdout+p.stderr)
        return result
    try:
        values={'mode':'esc-trial'}
        plan=call('plan',values=values)
        with ThreadPoolExecutor(max_workers=2) as pool:
            requests = [pool.submit(call, 'start', values=values, revision=plan['revision']) for _ in range(2)]
            results = [r.result() for r in requests]
        assert sum('error' not in r for r in results) == 1, results
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            running=call('status')
            if 'READY' in running['log']:break
            time.sleep(.05)
        assert 'READY' in running['log'], running
        assert running['session'] and running['ownership']['locked']
        # The request processes exited; tmux and its guarded child survive.
        assert 'error' in call('start',values=values,revision=plan['revision'])
        assert 'error' in call('stop',run_id='wrong')
        stopped=call('stop',run_id=running['run_id'])
        assert stopped['stop_requested']
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            final=call('status')
            if not final['ownership']['locked'] and not final['session']:break
            time.sleep(.05)
        assert not final['ownership']['locked'] and not final['session'], final
        assert final['phase']=='exited' and 'READY' in final['log']
    finally:
        subprocess.run(['tmux','-L','kart-bringup-local','kill-server'],env=env,capture_output=True)
        socket_dir.cleanup()


def test_catalog_exposes_mode_specific_stereo_defaults():
    with patch.object(remote, 'status', return_value={}):
        catalog = remote.dispatch({'action': 'catalog'})
    normal = catalog['stereo_defaults']['collect']
    environment = catalog['stereo_defaults']['environment']
    assert normal['depth_profile'] == '0'
    assert normal['infra_profile'] == normal['profile']
    assert environment['depth_profile'] == environment['infra_profile']
    assert environment['depth_profile'] != '0'


def test_preview_catalog_does_not_inspect_runtime():
    with patch.object(remote, 'status', side_effect=AssertionError('runtime access')):
        catalog = remote.dispatch({'action': 'preview_catalog'})
    assert catalog['status']['phase'] == 'preview'
    assert catalog['stereo_defaults']['collect']['depth_profile'] == '0'
    assert any(f['key'] == 'depth_profile' for f in catalog['fields'])
