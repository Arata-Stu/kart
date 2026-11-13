#!/usr/bin/env python3
"""JSON endpoint inside the Jetson container. No network listener and no shell input."""
import argparse
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import time
import uuid
import fcntl

import bringup
from bringup_runtime import STATE, ownership, legacy_processes

ROOT = bringup.ROOT
CONFIG = ROOT / 'ros2_ws/src/kart_bringup/config'
SESSION = 'kart-bringup'
# Jetson mounts host /tmp into Docker: never share the default tmux server.
SOCKET = os.environ.get('KART_BRINGUP_TMUX_SOCKET', 'kart-bringup-local')
if not re.fullmatch(r'kart-bringup-[a-z0-9-]{1,32}', SOCKET):
    raise ValueError('Invalid tmux socket name')
MODES = {'collect': 'データ記録', 'environment': 'データ記録',
         'drive': '地図走行', 'e2e': 'E2E走行', 'eval': 'オフライン評価',
         'e2e-latency': 'E2E遅延評価', 'esc-trial': 'ESC信号列試験'}


def atomic(path, data):
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False))
    temporary.replace(path)


def read(path):
    return json.loads(path.read_text()) if path.exists() else {}


def fields():
    from kart_bringup.mission import CAMERA_PROFILES
    result = []
    for action in bringup.argument_parser()._actions:
        if action.dest in ('help', 'mode', 'dry_run'):
            continue
        kind = 'boolean' if isinstance(action, (argparse._StoreTrueAction, argparse.BooleanOptionalAction)) else 'text'
        result.append(dict(key=action.dest, flag=action.option_strings[0], kind=kind,
                           choices=(['0', *CAMERA_PROFILES[{'rgb_profile':'RGB','infra_profile':'Infra','depth_profile':'Depth'}[action.dest]]]
                                    if action.dest in ('rgb_profile', 'infra_profile', 'depth_profile') else
                                    list(action.choices) if action.choices else None),
                           default=str(action.default) if isinstance(action.default, Path) else action.default,
                           help=action.help or ''))
    return result


def default_labels(mode):
    """Display configured defaults without turning them into runtime overrides."""
    import yaml
    from kart_bringup.mission import sensor_parameters, camera_config, composition
    from kart_bringup.recovery import configuration
    def params(path, node):
        return yaml.safe_load((CONFIG / path).read_text())[f'/**/{node}']['ros__parameters']
    labels = {f['key']: ('有効' if f['default'] is True else '無効' if f['default'] is False
                        else str(f['default'])) for f in fields() if f['default'] not in (None, '')}
    labels.update(run_name='センサー構成から自動生成', eval_localization='選択が必要',
                  e2e_fixed_throttle='モデル設定から決定', e2e_throttle_mode='fixed',
                  evs_tensor_config='処理方式・モデルに応じて決定', parking_site='選択した地図を使用',
                  parking_slot='指定なし', evs_serial='自動選択', latency_output='日時から自動生成')
    labels.update(multicam_mode=str(params('localization/cuvslam.yaml', 'visual_slam')['multicam_mode']))
    if mode in ('collect', 'environment', 'drive', 'e2e'):
        camera = sensor_parameters(camera_config(CONFIG, mode))
        labels['imu_profile'] = (f"加速度 {camera['accel_fps']} Hz / ジャイロ {camera['gyro_fps']} Hz"
                                 if camera['enable_accel'] and camera['enable_gyro'] else '個別YAML設定')
        labels['rgb_fps'] = camera['rgb_camera.color_profile'].split('x')[-1] if camera['enable_color'] else 'OFF'
        labels['foxglove'] = '有効' if composition(CONFIG / 'mission.yaml', mode)['enable_foxglove'] else '無効'
        perception = composition(CONFIG / 'mission.yaml', mode)
        labels['perception'] = perception['perception_engine'] or 'OFF（engine未選択）'
        labels['perception_mode'] = {'detect': '物体検出のみ', 'track': '物体検出＋車両追跡'}[perception['perception_mode']]
        labels['perception_debug_image'] = '有効' if perception['perception_debug_image'] else '無効'
        labels['overtake'] = {'off': '無効', 'shadow': '候補・判断の表示のみ',
                              'active': '追従・追越しを実行'}[perception['overtake_mode']]
        joy = params('input/joy_manager.yaml', 'kart_joy_manager')
        labels.update(joy_max_throttle=str(joy['speed_scale']),
                      auto_fixed_throttle='通常AUTO' if joy['auto_fixed_throttle'] < 0 else str(joy['auto_fixed_throttle']),
                      auto_fixed_throttle_r2_gate=('有効（R2で固定駆動／惰行）' if mode in ('collect', 'environment') and joy['auto_fixed_throttle_r2_gate'] else '無効'),
                      device=params('vehicle/bridge.yaml', 'kart_bridge')['device'])
        enabled, recovery = configuration(CONFIG / 'recovery')
        labels['recovery'] = recovery['sensor_profile'] if enabled else 'off'
        labels['tuning'] = '有効' if params('hdmap/hdmap.yaml', 'hdmap_server')['tuning_enabled'] else '無効'
    labels['evs_backend'] = params('sensors/openeb_tensor_pipeline.yaml', 'tensor_pipeline')['tensor_backend']
    labels['evs_aggregation'] = params(f"sensors/openeb_tensor_{labels['evs_backend']}.yaml", 'event_tensor').get('event_aggregation', 'events')
    labels['evs_image'] = '有効' if params('sensors/openeb_visualization.yaml', 'event_preprocessor')['event_image_enabled'] else '無効'
    labels['evs_bias_file'] = params('sensors/openeb.yaml', 'event_camera_driver')['bias_file'] or 'カメラ標準'
    return labels


def arguments(values):
    if not isinstance(values, dict) or values.get('mode') not in MODES:
        raise ValueError('用途を選択してください')
    schema = {f['key']: f for f in fields()}
    argv = ['--mode', values['mode']]
    for key, value in values.items():
        if key == 'mode':
            continue
        if key not in schema:
            raise ValueError('未知の起動設定: ' + key)
        f = schema[key]
        if value is None or value == '':
            continue
        if f['kind'] == 'boolean':
            if type(value) is not bool:
                raise ValueError(key + ': boolが必要です')
            if value:
                argv.append(f['flag'])
            elif f['default'] is None:
                argv.append('--no-' + key.replace('_', '-'))
        else:
            if not isinstance(value, (str, int, float)) or isinstance(value, bool):
                raise ValueError(key + ': 値が不正です')
            argv.append(f['flag'] + '=' + str(value))
    return argv


def parse(values):
    try:
        args = bringup.argument_parser().parse_args(arguments(values))
    except SystemExit as exc:
        raise ValueError('起動設定の値が不正です') from exc
    bringup.normalize_args(args)
    return args


def plan(values):
    args = parse(values)
    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        command = (bringup.esc_trial_entry(args, bringup.argument_parser()) if args.mode == 'esc-trial'
                   else bringup.build_command(args, CONFIG))
    # Include source configuration in the review identity. Revalidate again at start.
    digest = hashlib.sha256(json.dumps([values, command], sort_keys=True).encode())
    for path in sorted(CONFIG.rglob('*')):
        if path.is_file():
            digest.update(str(path.relative_to(CONFIG)).encode())
            digest.update(path.read_bytes())
    return dict(command=command, review=output.getvalue(), revision=digest.hexdigest())


class ChoiceNeeded(Exception):
    def __init__(self, title, values, labels):
        self.prompt = dict(title=title, values=values, labels=labels)


def assets(values, selections):
    from bringup_tui import Editor
    args = parse(values)
    if args.mode == 'esc-trial':
        return dict(values=values)
    if args.mode == 'eval' and args.eval_localization is None:
        args.eval_localization = 'vslam'
    cursor = 0
    def choose(title, candidates, label=str, default=0):
        nonlocal cursor
        candidates = list(candidates)
        if not candidates:
            raise ValueError(title + ': Jetson側に候補がありません')
        encoded = json.loads(json.dumps(candidates, default=str))
        if cursor >= len(selections):
            raise ChoiceNeeded(title, encoded, [str(label(v)) for v in candidates])
        selected = selections[cursor]
        cursor += 1
        if selected not in encoded:
            raise ValueError('候補が変わりました。構成の選択をやり直してください')
        return candidates[encoded.index(selected)]
    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        try:
            Editor(args, CONFIG, ROOT, choose).assets()
        except ChoiceNeeded as exc:
            return dict(prompt=exc.prompt, review=output.getvalue())
    defaults = vars(bringup.argument_parser().parse_args([]))
    result = {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()
              if k != 'dry_run' and (v != defaults[k] or k == 'mode')}
    return dict(values=result, review=output.getvalue())


def tmux(*args, check=True):
    p = subprocess.run(['tmux', '-u', '-L', SOCKET, *args], capture_output=True, text=True, timeout=5)
    if check and (p.returncode or 'error creating' in p.stderr):
        raise ValueError('tmux: ' + p.stderr.strip())
    return p


def status():
    state = read(STATE / 'session.json')
    sessions = tmux('list-sessions', '-F', '#{session_name}\t#{session_id}', check=False).stdout.splitlines()
    session_id = next((row.split('\t')[1] for row in sessions if row.split('\t')[0] == SESSION), None)
    has_session = session_id is not None
    marker = tmux('show-options', '-v', '-t', session_id, '@kart-run', check=False).stdout.strip() if has_session else ''
    owner = ownership()
    phase = state.get('phase', 'idle')
    if phase in ('starting', 'running', 'stopping') and not has_session:
        phase = 'lost'
    log = ''
    run = state.get('run_id', '')
    if re.fullmatch('[a-f0-9]{32}', run):
        path = STATE / (run + '.log')
        if path.exists():
            with path.open('rb') as stream:
                stream.seek(max(0, path.stat().st_size - 48000))
                log = stream.read().decode(errors='replace')
        bootstrap = STATE / (run + '.bootstrap.log')
        if bootstrap.exists():
            with bootstrap.open('rb') as stream:
                stream.seek(max(0, bootstrap.stat().st_size - 4000))
                log += stream.read().decode(errors='replace')
    return dict(**{**state, 'phase': phase}, session=has_session, session_id=session_id, ownership=owner,
                legacy=legacy_processes(), log=log, managed=has_session and marker == run,
                attach='tmux -u -L ' + SOCKET + ' attach -t ' + SESSION)


def start(body):
    current = status()
    if current['session'] or current['ownership']['locked'] or current['legacy']:
        raise ValueError('bringupまたは専用tmuxが起動済みです。状態を確認してください')
    prepared = plan(body['values'])
    if prepared['revision'] != body.get('revision'):
        raise ValueError('設定が変更されています。もう一度検証してください')
    run = uuid.uuid4().hex
    state = dict(run_id=run, phase='starting', values=body['values'], started_at=time.time())
    atomic(STATE / (run + '.json'), state)
    atomic(STATE / 'session.json', state)
    command = shlex.join([sys.executable, str(Path(__file__).resolve()), '--worker', run])
    command += ' 2>' + shlex.quote(str(STATE / (run + '.bootstrap.log')))
    try:
        tmux('new-session', '-d', '-s', SESSION, '-c', str(ROOT), command)
    except Exception:
        atomic(STATE / 'session.json', {**state, 'phase': 'failed'})
        raise
    return status()


def stop(body):
    current = status()
    if not body.get('run_id') or body['run_id'] != current.get('run_id'):
        raise ValueError('実行が切り替わっています。状態を再取得してください')
    if current['phase'] in ('starting', 'running', 'stopping') and current['session']:
        if not current['managed']:
            raise ValueError('専用セッションの所有を未確認です。状態を再取得してください')
        # The condition and send are evaluated together inside tmux, not in a shell.
        tmux('if-shell', '-F', '-t', current['session_id'], '#{==:#{@kart-run},' + body['run_id'] + '}',
             'send-keys -t ' + current['session_id'] + ' C-c')
        return {**current, 'stop_requested': True}
    return current


def worker(run):
    if not re.fullmatch('[a-f0-9]{32}', run):
        raise ValueError('Invalid run id')
    state = read(STATE / (run + '.json'))
    tmux('set-option', '@kart-run', run)
    tmux('set-window-option', 'remain-on-exit', 'off')
    # tmux Ctrl-C also reaches this logger. The common guard forwards it to ROS.
    process = None
    interrupted = False
    def forward(sig, _frame):
        nonlocal interrupted
        interrupted = True
        if process is not None:
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, forward)
    state.update(phase='running', pid=os.getpid())
    atomic(STATE / 'session.json', state)
    code = 1
    try:
        with (STATE / (run + '.log')).open('w') as log:
            command = ['bash', str(ROOT / 'scripts/bringup.sh'), *arguments(state['values'])]
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, bufsize=1, start_new_session=True, env={**os.environ, 'PYTHONUNBUFFERED': '1'})
            if interrupted:
                forward(signal.SIGINT, None)
            for line in process.stdout:
                log.write(line)
                log.flush()
                print(line, end='', flush=True)
            code = process.wait()
    finally:
        atomic(STATE / 'session.json', {**state, 'phase': 'exited' if code in (0, 130, -2) else 'failed',
                                      'exit_code': code, 'ended_at': time.time()})


def dispatch(body):
    action = body.get('action')
    if action == 'link_timing':
        from link_timing import measure
        return measure()
    if action in ('mode_status', 'mode_auto', 'mode_manual', 'mode_stop'):
        from mode_remote import request
        return request(action)
    if action in ('recording_status', 'recording_start', 'recording_stop'):
        from recording_remote import request
        return request(action, body.get('recording_service'))
    if action in ('catalog', 'preview_catalog'):
        from kart_bringup.mission import sensor_parameters, camera_config
        defaults = {}
        for mode in ('collect', 'environment', 'drive', 'e2e'):
            p = sensor_parameters(camera_config(CONFIG, mode))
            defaults[mode] = dict(
                rgb_profile=p['rgb_camera.color_profile'] if p['enable_color'] else '0',
                profile=p['depth_module.infra_profile'],
                infra_profile=p['depth_module.infra_profile'] if p['enable_infra1'] or p['enable_infra2'] else '0',
                depth_profile=p['depth_module.depth_profile'] if p['enable_depth'] else '0')
        return dict(modes={k: v for k, v in MODES.items() if k != 'environment'}, fields=fields(), status=(dict(phase='preview', session=False, log='プレビューです。Jetsonの実行ログではありません。') if action == 'preview_catalog' else status()), stereo_defaults=defaults, default_labels={mode: default_labels(mode) for mode in MODES})
    if action == 'status':
        return status()
    if action == 'assets':
        return assets(body['values'], body.get('selections', []))
    if action == 'plan':
        return plan(body['values'])
    if action in ('start', 'stop'):
        STATE.mkdir(parents=True, exist_ok=True)
        with (STATE / 'manager.lock').open('a') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            return start(body) if action == 'start' else stop(body)
    raise ValueError('Unknown bringup action')


if __name__ == '__main__':
    try:
        if len(sys.argv) == 3 and sys.argv[1] == '--worker':
            worker(sys.argv[2])
        else:
            result = dispatch(json.load(sys.stdin))
            print(json.dumps(result, ensure_ascii=False))
    except Exception as exc:
        print(json.dumps({'error': str(exc)}, ensure_ascii=False), file=sys.stderr if '--worker' in sys.argv else sys.stdout)
        sys.exit(1)
