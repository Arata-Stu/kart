"""Standalone ROS adapter; camera processing stays outside the control mux executor."""
from dataclasses import asdict, fields
import json
import math
import time

import cv2
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from kart_interfaces.msg import ControlCommand, OperationModeRequest, OperationModeState
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, String

from .core import Command, Observation, Recovery, Settings
from .vision import FrameMotion, event_activity, grayscale, image_array


# Every default is also listed in config/recovery.yaml and the README.
NODE_DEFAULTS = {
    'sensor_profile': 'infra_imu',
    'image_topic': '/realsense/infra1/image_rect_raw',
    'imu_topic': '/realsense/imu',
    'evs_image_topic': '/event_camera/event_image',
    'control_rate_hz': 50.0,
    'image_rate_hz': 15.0,
    'input_timeout_s': 0.25,
    'sensor_timeout_s': 0.3,
    'mode_timeout_s': 0.3,
    'future_tolerance_s': 0.05,
    'wall_timeout_s': 0.0,
    'processing_width': 212,
    'min_features': 20,
    'max_features': 120,
    'min_feature_cells': 4,
    'flow_fb_error_px': 1.0,
    'evs_min_neighbors': 2,
    'evs_activity_threshold': 0.02,
}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RecoveryNode(Node):
    def __init__(self):
        super().__init__('recovery')
        descriptor = ParameterDescriptor(read_only=True)
        self.cfg = {key: self.declare_parameter(key, value, descriptor).value
                    for key, value in NODE_DEFAULTS.items()}
        settings = Settings(**{f.name: self.declare_parameter(f.name, getattr(Settings(), f.name), descriptor).value
                               for f in fields(Settings)})
        self.machine = Recovery(settings)
        c = self.cfg
        if c['sensor_profile'] not in ('rgb_imu', 'infra_imu', 'evs_assisted'):
            raise ValueError('Unknown sensor_profile')
        for key in ('control_rate_hz', 'image_rate_hz', 'input_timeout_s', 'sensor_timeout_s',
                    'mode_timeout_s', 'flow_fb_error_px'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('future_tolerance_s', 'wall_timeout_s', 'evs_activity_threshold'):
            if not math.isfinite(c[key]) or c[key] < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        for key in ('processing_width', 'min_features', 'max_features', 'min_feature_cells', 'evs_min_neighbors'):
            if type(c[key]) is not int or c[key] < 1:
                raise ValueError(f'{key} must be a positive integer')
        if (c['processing_width'] < 32 or c['max_features'] < c['min_features']
                or c['min_feature_cells'] > 12 or c['evs_min_neighbors'] > 9
                or c['evs_activity_threshold'] > 1):
            raise ValueError('Invalid vision settings')
        self.motion = FrameMotion(c['processing_width'], c['min_features'], c['max_features'],
                                  c['min_feature_cells'], c['flow_fb_error_px'], c['sensor_timeout_s'])
        self.samples = {}
        self.command_msg = None
        self.observation = Observation()
        self.mode = 0
        self.gyro = 0.0
        self.flow = 0.0
        self.features = 0
        self.evs_activity = 0.0
        self.frame_valid = False
        self.last_image_wall = -math.inf
        self.last_sensor_stamp = {}
        self.last_status_wall = -math.inf
        self.last_reported = None
        self.release_wait_until = -math.inf
        self.last_ros_time = None
        self.auto_pub = self.create_publisher(ControlCommand, 'auto/control_cmd', 1)
        self.stop_pub = self.create_publisher(OperationModeRequest, 'operation_mode/request', 10)
        self.active_pub = self.create_publisher(Bool, 'recovery/active', 1)
        self.status_pub = self.create_publisher(String, 'recovery/status', 1)
        self.create_subscription(ControlCommand, 'recovery/input_cmd', self.on_command, 1)
        self.create_subscription(OperationModeState, 'operation_mode/state', self.on_mode, 1)
        self.create_subscription(Image, c['image_topic'], self.on_image, qos_profile_sensor_data)
        self.create_subscription(Imu, c['imu_topic'], self.on_imu, qos_profile_sensor_data)
        if c['sensor_profile'] == 'evs_assisted':
            self.create_subscription(Image, c['evs_image_topic'], self.on_events, qos_profile_sensor_data)
        self.timer = self.create_timer(1 / c['control_rate_hz'], self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Recovery settings (unvalidated on vehicle): ' +
                               json.dumps({**c, **asdict(settings)}))

    def remember(self, key, stamp):
        self.samples[key] = (stamp_seconds(stamp), time.monotonic())

    def fresh(self, key, limit):
        stamp, received = self.samples.get(key, (-math.inf, -math.inf))
        now = self.get_clock().now().nanoseconds * 1e-9
        wall_limit = self.cfg['wall_timeout_s'] or limit
        return (stamp > 0 and -self.cfg['future_tolerance_s'] <= now - stamp <= limit
                and time.monotonic() - received <= wall_limit)

    def accept_sensor(self, key, stamp):
        value = stamp_seconds(stamp)
        last = self.last_sensor_stamp.get(key, -math.inf)
        # Duplicate/out-of-order messages cannot refresh sensor health.
        if value <= last:
            return False
        self.last_sensor_stamp[key] = value
        self.remember(key, stamp)
        return True

    def on_command(self, msg):
        self.command_msg = msg
        self.remember('command', msg.header.stamp)

    def on_mode(self, msg):
        self.mode = msg.mode
        self.remember('mode', msg.header.stamp)
        # Cancel on receipt, even if mode changes twice between control ticks.
        if self.fresh('mode', self.cfg['mode_timeout_s']) and msg.mode in (2, 3, 4):
            now = self.get_clock().now().nanoseconds * 1e-9
            self.machine.step(now, msg.mode, Command(), Observation())
            self.auto_pub.publish(self.to_message(Command()))
            self.active_pub.publish(Bool(data=False))
            self.command_msg = None

    def on_imu(self, msg):
        v = msg.angular_velocity
        norm = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        if not math.isfinite(norm) or msg.angular_velocity_covariance[0] < 0:
            self.samples.pop('imu', None)
            return
        if self.accept_sensor('imu', msg.header.stamp):
            self.gyro = norm

    def on_image(self, msg):
        wall = time.monotonic()
        if wall - self.last_image_wall < 1 / self.cfg['image_rate_hz']:
            return
        self.last_image_wall = wall
        if not self.accept_sensor('image', msg.header.stamp):
            return
        try:
            gray = grayscale(image_array(msg), msg.encoding, self.cfg['processing_width'])
            self.frame_valid, self.flow, self.features = self.motion.update(gray, stamp_seconds(msg.header.stamp))
        except (ValueError, RuntimeError, cv2.error) as error:
            self.frame_valid = False
            self.get_logger().warning(str(error), throttle_duration_sec=2)

    def on_events(self, msg):
        if not self.accept_sensor('events', msg.header.stamp):
            return
        try:
            self.evs_activity = event_activity(image_array(msg), msg.encoding, self.cfg['evs_min_neighbors'])
        except (ValueError, RuntimeError, cv2.error):
            self.samples.pop('events', None)

    def to_message(self, command):
        msg = ControlCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        for key, value in asdict(command).items():
            setattr(msg, key, float(value))
        return msg

    def tick(self):
        c = self.cfg
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_ros_time is not None and now < self.last_ros_time:
            self.samples.clear()
            self.last_sensor_stamp.clear()
            self.motion.previous = self.motion.stamp = None
            self.frame_valid = False
            self.machine.fault('clock_rewind')
        self.last_ros_time = now
        command = Command() if self.command_msg is None else Command(
            self.command_msg.steering, self.command_msg.throttle,
            self.command_msg.brake, self.command_msg.reverse)
        mode = self.mode if self.fresh('mode', c['mode_timeout_s']) else 0
        sensors = self.fresh('image', c['sensor_timeout_s']) and self.fresh('imu', c['sensor_timeout_s'])
        assisted = c['sensor_profile'] == 'evs_assisted'
        if assisted:
            sensors = sensors and self.fresh('events', c['sensor_timeout_s'])
        obs = Observation(sensors and self.frame_valid, self.flow, self.gyro,
                          not assisted or self.evs_activity < c['evs_activity_threshold'])
        waiting_for_producer = self.command_msg is None and time.monotonic() <= self.release_wait_until
        healthy = ((waiting_for_producer or (self.command_msg is not None and self.fresh('command', c['input_timeout_s'])))
                   and self.count_publishers('recovery/input_cmd') == 1
                   and self.count_publishers('auto/control_cmd') == 1)
        was_active = self.machine.active
        if mode == 0 and (self.machine.active or self.machine.state in ('driving', 'suspect')):
            result = self.machine.fault('mode_unavailable')
        else:
            stamp = self.samples.get('command', (0, 0))[0]
            result = self.machine.step(now, mode, command, obs, healthy=healthy, history_stamp=stamp)
        # A pass-through preserves the producer's original stamp; no rejuvenation.
        passthrough = (self.machine.state in ('driving', 'suspect') and not was_active
                       and self.command_msg is not None)
        self.auto_pub.publish(self.command_msg if passthrough else self.to_message(result))
        self.active_pub.publish(Bool(data=self.machine.active))
        if was_active and not self.machine.active:
            self.command_msg = None  # Require a post-recovery producer sample.
            self.release_wait_until = time.monotonic() + c['input_timeout_s']
        if self.machine.state == 'fault':
            request = OperationModeRequest()
            request.header.stamp = self.get_clock().now().to_msg()
            request.mode = OperationModeRequest.STOP
            request.source = 'recovery:' + self.machine.reason
            self.stop_pub.publish(request)
        wall = time.monotonic()
        state_key = (self.machine.state, self.machine.reason)
        if state_key != self.last_reported or wall - self.last_status_wall >= 0.1:
            self.last_status_wall, self.last_reported = wall, state_key
            status = {'schema': 'kart.recovery.v1', 'stamp': now,
                      'state': self.machine.state, 'reason': self.machine.reason,
                      'active': self.machine.active, 'attempts': self.machine.attempts,
                      'fixed_steering': self.machine.fixed_steering,
                      'selected_interval': self.machine.selected_interval,
                      'suspect_at': self.machine.suspect_at, 'phase_at': self.machine.phase_at,
                      'flow_px_s': self.flow, 'features': self.features, 'gyro_rad_s': self.gyro,
                      'evs_activity': self.evs_activity, 'observation_valid': bool(obs.valid),
                      'rotation_rad': self.machine.rotation,
                      'settings': {**c, **asdict(self.machine.p)}}
            self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
