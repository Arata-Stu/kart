"""JetPilot-derived rosbag command/path handling, with nonblocking lifecycle supervision."""
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

DEFAULT_TOPICS = [
    'joy', 'joy/connected', 'joy/ready', 'joy/status',
    'teleop/control_cmd', 'propo/control_cmd', 'auto/control_cmd', 'vehicle/control_cmd',
    'operation_mode/request', 'operation_mode/state', 'bag/request', 'bag/status', 'bag/raw_diagnostics',
    'vehicle/trim/request', 'vehicle/trim/state', 'vehicle/rc_channels',
    'vehicle/output_channels', 'vehicle/vbec', 'vehicle/active_path', 'diagnostics',
    '/system/jetson/diagnostics',
    '/tf', '/tf_static', 'robot_description', 'joint_states',
    # Candidate sensor names, to be matched to the future D455/localization launch.
    'realsense/color/camera_info', 'realsense/color/image_raw',
    'realsense/infra1/camera_info', 'realsense/infra1/image_rect_raw',
    'realsense/infra2/camera_info', 'realsense/infra2/image_rect_raw',
    'realsense/imu', 'realsense/accel/sample', 'realsense/gyro/sample',
    'visual_slam/tracking/odometry',
]

@dataclass
class Settings:
    output_dir: str = '/workspaces/record'
    recording_name: str = ''
    session_layout: bool = False
    record_all: bool = False
    topics: list = field(default_factory=lambda: list(DEFAULT_TOPICS))
    exclude_topics: list = field(default_factory=list)
    storage_id: str = 'mcap'
    serialization_format: str = ''
    max_bag_size: int = 0
    recording_split_duration_s: int = 0
    max_cache_size: int = 0
    compression_mode: str = ''
    compression_format: str = ''
    compression_queue_size: int = 0
    compression_threads: int = 0
    qos_profile_overrides_path: str = ''
    include_hidden_topics: bool = False
    no_discovery: bool = False
    snapshot_mode: bool = False
    start_paused: bool = False
    extra_args: list = field(default_factory=list)
    status_period_s: float = 1.0
    raw_recording_request_topic: str = ''
    raw_recording_driver_node: str = ''
    raw_recording_service_timeout_s: float = 5.0
    recording_start_timeout_s: float = 5.0
    stop_timeout_s: float = 10.0
    terminate_timeout_s: float = 5.0

    def validate(self):
        if not self.output_dir.strip():
            raise ValueError('output_dir must not be empty')
        name = self.recording_name
        if name and (name.strip() != name or name in ('.', '..') or len(name.encode()) > 120 or
                     any(c in '/\\' or ord(c) < 32 or ord(c) == 127 for c in name)):
            raise ValueError('recording_name must be a single folder name of at most 120 bytes')
        for key in ('max_bag_size', 'recording_split_duration_s', 'max_cache_size',
                    'compression_queue_size', 'compression_threads'):
            if getattr(self, key) < 0:
                raise ValueError(f'{key} must be non-negative')
        for key in ('status_period_s', 'recording_start_timeout_s', 'stop_timeout_s', 'terminate_timeout_s',
                    'raw_recording_service_timeout_s'):
            value = getattr(self, key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if self.raw_recording_driver_node and (not self.raw_recording_driver_node.startswith('/') or
                self.raw_recording_driver_node.endswith('/')):
            raise ValueError('raw_recording_driver_node must be an absolute ROS node name')
        if self.compression_mode not in ('', 'none', 'file', 'message'):
            raise ValueError('invalid compression_mode')
        # Retain an escape hatch, but do not let it defeat naming, stop control or topic selection.
        reserved = {'-o', '--output', '-d', '--max-bag-duration', '-a', '--all', '--all-topics',
                    '--topics', '--exclude-topics', '--node-name', '--start-paused', '--snapshot-mode',
                    '--max-bag-files', '--use-sim-time', '--ros-args'}
        for arg in self.extra_args:
            if arg.split('=')[0] in reserved or any(arg.startswith(p) and arg != p for p in ('-o', '-d')):
                raise ValueError(f'extra_args must not override managed option: {arg}')

    def command(self, uri):
        cmd = ['ros2', 'bag', 'record', '-o', str(uri), '--disable-keyboard-controls',
               '--node-name', 'kart_rosbag_recorder']
        values = {
            '--storage': self.storage_id, '--serialization-format': self.serialization_format,
            '--max-bag-size': self.max_bag_size, '--max-bag-duration': self.recording_split_duration_s,
            '--max-cache-size': self.max_cache_size, '--compression-mode': self.compression_mode,
            '--compression-format': self.compression_format, '--compression-queue-size': self.compression_queue_size,
            '--compression-threads': self.compression_threads, '--qos-profile-overrides-path': self.qos_profile_overrides_path,
        }
        for flag, value in values.items():
            if value:
                cmd.extend([flag, str(value)])
        for attr in ('include_hidden_topics', 'no_discovery', 'snapshot_mode', 'start_paused'):
            if getattr(self, attr):
                cmd.append('--' + attr.replace('_', '-'))
        cmd.extend(self.extra_args)
        if self.exclude_topics:
            cmd.extend(['--exclude-topics', *self.exclude_topics])
        cmd.extend(['--all-topics'] if self.record_all else ['--topics', *self.topics])
        return cmd


class Recorder:
    def __init__(self, settings, *, popen=subprocess.Popen, clock=time.monotonic):
        settings.validate()
        self.settings, self._popen, self._clock = settings, popen, clock
        self.output_dir = Path(settings.output_dir).expanduser().resolve()
        self.session_stamp = datetime.now().strftime('%Y-%m-%d/%H%M%S')
        self.process = None
        self.phase = 'idle'
        self.current_uri = ''
        self.last_event = 'idle'
        self._deadline = 0.0
        self._signal_stage = 0
        self._failed = False

    @property
    def recording(self):
        return self.phase == 'recording' and self.process is not None and self.process.poll() is None

    def next_path(self, label):
        now = datetime.now()
        if self.settings.session_layout:
            parent = self.output_dir / self.session_stamp
            name = self.settings.recording_name or 'run'
            path, suffix = parent / name, 1
            while path.exists() or path.is_symlink():
                path = parent / f'{name}_{suffix:02d}'
                suffix += 1
            return path
        if self.settings.recording_name:
            parent, name = self.output_dir / now.strftime('%Y-%m-%d'), self.settings.recording_name
        else:
            parent = self.output_dir
            label = ''.join(c if c.isalnum() or c in '-_' else '_' for c in label).strip('_')
            label = label.encode()[:100].decode(errors='ignore')
            name = now.strftime('%Y%m%d_%H%M%S') + ('_' + label if label else '')
        path, suffix = parent / name, 1
        while path.exists() or path.is_symlink():
            path = parent / f'{name}_{suffix:02d}'
            suffix += 1
        return path

    def start(self, label=''):
        self.tick()
        if self.process is not None:
            self.last_event = f'start ignored: {self.phase}'
            return False
        if not self.settings.record_all and not self.settings.topics:
            self.phase, self.last_event = 'error', 'start rejected: no topics configured'
            return False
        try:
            path = self.next_path(label)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.current_uri = str(path)
            # No shell and no preexec_fn; recorder has its own POSIX process group.
            self.process = self._popen(self.settings.command(path), start_new_session=True,
                                       stdin=subprocess.DEVNULL)
        except (OSError, ValueError) as exc:
            self.phase, self.last_event = 'error', f'start failed: {exc}'
            return False
        self.phase, self.last_event = 'starting', 'waiting for rosbag output directory'
        self._failed = False
        self._deadline = self._clock() + self.settings.recording_start_timeout_s
        return True

    def _send(self, sig):
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, sig)
            except ProcessLookupError:
                pass

    def stop(self, reason='stop requested', *, failed=False):
        if self.process is None:
            self.last_event = 'stop ignored: not recording'
            return
        if self.phase == 'stopping':
            return  # Do not restart the shutdown deadline on repeated STOP requests.
        self._failed = failed
        self.last_event, self.phase = reason, 'stopping'
        self._send(signal.SIGINT)
        self._signal_stage = 1
        self._deadline = self._clock() + self.settings.stop_timeout_s

    def tick(self):
        if self.process is None:
            return
        code = self.process.poll()
        if code is not None:
            if self.phase != 'stopping':
                self._failed = True
                self.last_event = f'rosbag process exited unexpectedly: {code}'
            elif code != 0:
                self._failed = True
                self.last_event += f'; recorder exit code {code}'
            if not self._failed and not (Path(self.current_uri) / 'metadata.yaml').is_file():
                self._failed = True
                self.last_event = 'recorder exited but metadata.yaml is missing; finalization unverified'
            self.phase = 'error' if self._failed else 'idle'
            self.process = None  # poll() reaps the child.
            return
        now = self._clock()
        if self.phase == 'starting':
            if Path(self.current_uri).is_dir():
                self.phase, self.last_event = 'recording', 'recording started'
            elif now >= self._deadline:
                self.stop('start failed: output directory timeout', failed=True)
        elif self.phase == 'stopping' and now >= self._deadline:
            sig = signal.SIGTERM if self._signal_stage == 1 else signal.SIGKILL
            self._send(sig)
            self._signal_stage += 1
            self._failed = True
            self.last_event = f'stop escalated to {sig.name}; bag finalization not guaranteed'
            self._deadline = now + self.settings.terminate_timeout_s

    def close(self):
        self.stop('node shutdown')
        deadline = self._clock() + self.settings.stop_timeout_s + self.settings.terminate_timeout_s + 1
        while self.process is not None and self._clock() < deadline:
            self.tick()
            time.sleep(0.01)
        if self.process is not None:
            self._send(signal.SIGKILL)
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.last_event = 'recorder did not exit after SIGKILL'
            else:
                self.process = None
