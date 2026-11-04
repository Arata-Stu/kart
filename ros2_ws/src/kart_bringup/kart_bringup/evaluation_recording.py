"""Opt-in offline result recording; no writes until launch execution."""
from datetime import datetime, timezone
import json
from pathlib import Path
import uuid

import yaml


def recording_plan(config, enabled='', output=''):
    settings = yaml.safe_load((Path(config) / 'evaluation/recording.yaml').read_text())
    if enabled not in ('', 'true', 'false'):
        raise ValueError('eval_record must be true or false')
    active = settings['enabled'] if enabled == '' else enabled == 'true'
    if output and not active:
        raise ValueError('eval_output requires eval_record=true')
    if not active:
        return None
    destination = Path(output).expanduser() if output else Path(settings['output_root']) / (
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex[:8])
    destination = destination.resolve()
    if destination.exists():
        raise ValueError(f'評価保存先は新規ディレクトリを指定してください: {destination}')
    return dict(settings, output=str(destination))


def start_recording(config, plan, manifest):
    """Launch-owned process; SIGINT on launch shutdown finalizes rosbag metadata."""
    from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
    from launch.event_handlers import OnProcessExit, OnProcessIO
    from launch.events import Shutdown
    root = Path(plan['output'])
    root.mkdir(parents=True, exist_ok=False)
    snapshot = root / 'config'
    snapshot.mkdir()
    for folder in ('evaluation', 'localization', 'sim'):
        for source in (Path(config) / folder).glob('*'):
            if source.suffix in ('.yaml', '.json'):
                target = snapshot / folder / source.name
                target.parent.mkdir(exist_ok=True)
                target.write_bytes(source.read_bytes())
    (root / 'manifest.json').write_text(json.dumps(dict(
        schema_version=1, created_utc=datetime.now(timezone.utc).isoformat(),
        recording=plan, **manifest), indent=2, ensure_ascii=False))
    command = ['ros2', 'bag', 'record', '--node-name', plan['node_name'],
               '--storage', plan['storage_id'], '--output', str(root / 'bag'),
               '--disable-keyboard-controls', '--qos-profile-overrides-path',
               str(Path(config) / 'evaluation/recording_qos.yaml'), '--topics', *plan['topics']]
    recorder = ExecuteProcess(cmd=command, output='screen', sigterm_timeout='20', sigkill_timeout='10')
    def output(event):
        # Includes native cuVSLAM stderr, which is not necessarily sent to /rosout.
        text = event.text.decode('utf-8', errors='replace') if isinstance(event.text, bytes) else str(event.text)
        with (root / 'process_output.log').open('a') as stream:
            stream.write(text)
    def exited(event, context):
        (root / 'recorder_exit.json').write_text(json.dumps({'returncode': event.returncode}))
        if not context.is_shutdown:
            return [EmitEvent(event=Shutdown(reason='Offline recorder exited; evaluation stopped'))]
        return []
    return [RegisterEventHandler(OnProcessIO(on_stdout=output, on_stderr=output)),
            RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=exited)), recorder]
