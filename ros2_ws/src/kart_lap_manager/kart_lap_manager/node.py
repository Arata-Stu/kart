"""ROS adapter for lap events; never publishes driving commands."""

from dataclasses import fields
import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from kart_interfaces.msg import ReferenceLine
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .core import LapDetector, RequestState, Settings


DEFAULTS = {
    'vgl_enabled': False,
    'cache_clear_enabled': False,
    'reference_topic': '/planning/reference_line',
    'odometry_topic': '/visual_slam/tracking/odometry',
    'diagnostics_topic': '/localization/vslam/diagnostics',
    'pose_hint_topic': '/localization/pose_hint',
    'lap_events_topic': '/localization/lap_events',
    'lap_status_topic': '/localization/lap_status',
    'vgl_trigger_service': '/visual_localization/trigger_localization',
    'cache_clear_service': '/kart/cache/clear',
    'map_frame': 'map',
    'odom_frame': 'odom',
    'base_frame': 'base_link',
    'pose_timeout_s': 0.5,
    'diagnostics_timeout_s': 1.0,
    'map_correction_translation_m': 0.15,
    'map_correction_yaw_rad': 0.15,
    'service_timeout_s': 3.0,
    'vgl_result_timeout_s': 30.0,
    'cache_result_timeout_s': 15.0,
    'status_period_s': 1.0,
    'own_hint_grace_s': 3.0,
}
DEFAULTS.update({field.name: getattr(Settings(), field.name) for field in fields(Settings)})


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def fresh(stamp, received, now, wall, timeout):
    return (stamp is not None and received is not None and
            0 <= now - stamp <= timeout and 0 <= wall - received <= timeout)


def transform_pose(transform):
    p, q = transform.translation, transform.rotation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if any(not math.isfinite(v) for v in values):
        raise ValueError('nonfinite_tf')
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if abs(norm - 1.) > .01:
        raise ValueError('invalid_tf_quaternion')
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    return p.x, p.y, yaw


class LapManager(Node):
    def __init__(self, **node_options):
        super().__init__('lap_manager', **node_options)
        fixed = ParameterDescriptor(read_only=True)
        self.settings = {key: self.declare_parameter(key, value, fixed).value
                         for key, value in DEFAULTS.items()}
        for key, default in DEFAULTS.items():
            value = self.settings[key]
            if isinstance(default, bool):
                if type(value) is not bool:
                    raise ValueError(f'{key} must be bool')
            elif isinstance(default, str):
                if not isinstance(value, str) or not value.strip():
                    raise ValueError(f'{key} must be nonempty')
            elif not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be positive finite')
        self.detector = LapDetector(Settings(**{field.name: self.settings[field.name]
                                               for field in fields(Settings)}))
        self.diag_stamp = self.diag_at = None
        self.diag_good = False
        self.pose_stamp = self.pose_at = None
        self.pending = None
        self.last_odom_stamp = None
        self.last_clock = None
        self.last_map_to_odom = None
        self.last_status = -math.inf
        self.hint_stamp = None
        self.request_ros_stamp = None
        self.request_lap_anchor = None
        self.vgl = RequestState(needs_hint=True)
        self.cache = RequestState()
        self.futures = {'vgl': None, 'cache': None}
        # Use the same ROS clock as the measurements so bag time jumps clear TF.
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ReferenceLine, self.settings['reference_topic'], self.on_reference, latched)
        self.create_subscription(Odometry, self.settings['odometry_topic'], self.on_odometry,
                                 qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, self.settings['diagnostics_topic'], self.on_diagnostic, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.settings['pose_hint_topic'], self.on_hint, 10)
        self.event_pub = self.create_publisher(String, self.settings['lap_events_topic'], 10)
        self.status_pub = self.create_publisher(String, self.settings['lap_status_topic'], 1)
        # Node.clients is rclpy's read-only registry used by its executor.
        self.maintenance_clients = {
            'vgl': self.create_client(Trigger, self.settings['vgl_trigger_service'])
                   if self.settings['vgl_enabled'] else None,
            'cache': self.create_client(Trigger, self.settings['cache_clear_service'])
                     if self.settings['cache_clear_enabled'] else None,
        }
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.tick, clock=self.steady_clock)
        self.get_logger().info(f'Lap manager config: {self.settings}; recording/AUTO are independent')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def invalidate(self, reason):
        self.detector.invalidate(reason)
        self.last_map_to_odom = None

    def on_reference(self, msg):
        try:
            if msg.header.frame_id != self.settings['map_frame']:
                raise ValueError('reference_frame_mismatch')
            if any(not math.isfinite(p.z) or abs(p.z) > 1e-6 for p in msg.points):
                raise ValueError('nonplanar_reference')
            self.detector.set_reference([(p.x, p.y) for p in msg.points], msg.closed)
        except (ValueError, TypeError) as error:
            self.detector.points, self.detector.segments, self.detector.length = (), (), 0.
            self.invalidate('invalid_reference')
            self.get_logger().warning(f'Lap reference rejected: {error}', throttle_duration_sec=5.)

    def on_diagnostic(self, msg):
        stamp, now, wall = stamp_s(msg.header), self.now_s(), time.monotonic()
        if self.diag_stamp is not None and stamp <= self.diag_stamp:
            if stamp < self.diag_stamp:
                self.invalidate('diagnostic_time_reversed')
                self.diag_good = False
            return
        found, good = False, False
        for status in msg.status:
            if status.hardware_id == 'visual_slam':
                values = {item.key: item.value for item in status.values}
                current = (status.level == 0 and values.get('localized_in_exist_map') == 'Yes'
                           and values.get('vo_status') == 'OK')
                good = current if not found else good and current
                found = True
        if not found:
            return
        self.diag_stamp, self.diag_at = stamp, wall
        self.diag_good = good and fresh(stamp, wall, now, wall, self.settings['diagnostics_timeout_s'])
        if not self.diag_good:
            self.invalidate('vslam_unhealthy')
        elif self.hint_stamp is not None and stamp >= self.hint_stamp:
            self.vgl.healthy_diagnostic(wall)

    def on_hint(self, msg):
        self.pending = None
        stamp, now, wall = stamp_s(msg.header), self.now_s(), time.monotonic()
        pose = msg.pose.pose
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        valid = (msg.header.frame_id == self.settings['map_frame'] and all(math.isfinite(v) for v in values)
                # Upstream stamps the input image, before VGL inference (~0.8 s
                # in the reported log). This is not a live odometry sample.
                and fresh(stamp, wall, now, wall, self.settings['vgl_result_timeout_s'])
                and self.request_ros_stamp is not None
                and stamp >= self.request_ros_stamp - self.settings['pose_timeout_s'])
        # A hint caused by this lap's request commonly arrives ~0.8 s after the
        # finish. Preserve only the exact next-lap anchor captured at dispatch;
        # a prior quality/jump reset or another finish changes that anchor. Do
        # not constrain progress: 0.8 s is already >15% of a short 5 s lap.
        # Exact-time map->odom and position jump checks still reject large moves.
        recently_completed = (self.detector.armed_at is not None and
                              0 <= now - self.detector.armed_at <= self.settings['own_hint_grace_s'])
        same_anchor = (self.request_lap_anchor is not None and
                       self.detector.armed_at == self.request_lap_anchor)
        if not (valid and self.vgl.busy and recently_completed and same_anchor):
            self.invalidate('localization_hint_received')
        if valid:
            self.hint_stamp = stamp
            self.vgl.hint(wall)

    def on_odometry(self, msg):
        stamp = stamp_s(msg.header)
        if self.last_odom_stamp is not None and stamp <= self.last_odom_stamp:
            if stamp < self.last_odom_stamp:
                self.pending = None
                self.invalidate('odometry_time_reversed')
            return
        self.last_odom_stamp = stamp
        self.retry_pose()
        if self.pending is None:
            self.pending = (msg, time.monotonic())
        self.retry_pose()

    def quality_ready(self, now, wall):
        return self.diag_good and fresh(self.diag_stamp, self.diag_at, now, wall,
                                       self.settings['diagnostics_timeout_s'])

    def retry_pose(self):
        if self.pending is None:
            return
        msg, received = self.pending
        stamp, now, wall = stamp_s(msg.header), self.now_s(), time.monotonic()
        if not self.quality_ready(now, wall):
            self.pending = None
            self.invalidate('vslam_diagnostic_unavailable')
            return
        if not fresh(stamp, received, now, wall, self.settings['pose_timeout_s']):
            self.pending = None
            self.invalidate('pose_stale')
            return
        if msg.header.frame_id != self.settings['odom_frame']:
            self.pending = None
            self.invalidate('odometry_frame_mismatch')
            return
        try:
            when = Time.from_msg(msg.header.stamp)
            pose = transform_pose(self.buffer.lookup_transform(self.settings['map_frame'],
                                                               self.settings['base_frame'], when).transform)
            correction = transform_pose(self.buffer.lookup_transform(self.settings['map_frame'],
                                                                     self.settings['odom_frame'], when).transform)
        except TransformException:
            return  # Keep original capture/receipt times while waiting for exact-time TF.
        except ValueError as error:
            self.pending = None
            self.invalidate(str(error))
            return
        self.pending = None
        old = self.last_map_to_odom
        if old is not None:
            dyaw = abs(math.atan2(math.sin(correction[2]-old[2]), math.cos(correction[2]-old[2])))
            if (math.hypot(correction[0]-old[0], correction[1]-old[1]) > self.settings['map_correction_translation_m']
                    or dyaw > self.settings['map_correction_yaw_rad']):
                self.invalidate('map_to_odom_correction')
        self.last_map_to_odom = correction
        self.pose_stamp, self.pose_at = stamp, received
        event = self.detector.sample(pose[0], pose[1], stamp)
        if event is not None:
            self.request('vgl', wall)
            self.request('cache', wall)
            event.update(schema='kart.lap_event.v1', vgl=self.vgl.summary(), cache=self.cache.summary())
            self.event_pub.publish(String(data=json.dumps(event, allow_nan=False)))
            self.get_logger().info(f'Lap {event["lap"]}: {event["lap_time_s"]:.2f} s; '
                                   f'VGL={self.vgl.state}; cache={self.cache.state}')

    def request(self, key, wall):
        client = self.maintenance_clients[key]
        if client is None:
            return
        state = self.vgl if key == 'vgl' else self.cache
        if state.busy:
            state.skipped += 1
            return
        if not client.service_is_ready():
            state.skipped += 1
            state.message = 'service unavailable at lap completion; no retry for this lap'
            self.get_logger().warning(f'{key}: {state.message}')
            return
        state.start(wall)
        if key == 'vgl':
            self.hint_stamp, self.request_ros_stamp = None, self.now_s()
            self.request_lap_anchor = self.detector.armed_at
        try:
            self.futures[key] = client.call_async(Trigger.Request())
        except Exception as error:  # Middleware errors do not authorize resending.
            state.fail_uncertain(error)

    def poll_requests(self, wall):
        for key, state, result_timeout in (
            ('vgl', self.vgl, self.settings['vgl_result_timeout_s']),
            ('cache', self.cache, self.settings['cache_result_timeout_s']),
        ):
            future = self.futures[key]
            if future is not None and future.done():
                self.futures[key] = None
                try:
                    result = future.result()
                    state.reply(result.success, result.message)
                    if not result.success:
                        self.get_logger().warning(f'{key} request rejected: {result.message}')
                except Exception as error:
                    state.fail_uncertain(error)
            service_timeout = (self.settings['service_timeout_s'] if key == 'vgl'
                               else self.settings['cache_result_timeout_s'])
            state.check_timeout(wall, service_timeout, result_timeout)

    def tick(self):
        now, wall = self.now_s(), time.monotonic()
        if self.last_clock is not None and now < self.last_clock:
            # Also clear on our explicit rewind path, including sub-second
            # jumps below tf2's own jump-callback threshold.
            self.buffer.clear()
            self.pending = None
            self.diag_stamp = self.diag_at = None
            self.diag_good = False
            self.pose_stamp = self.pose_at = self.last_odom_stamp = None
            self.invalidate('ros_clock_reversed')
            if self.vgl.busy:
                self.vgl.fail_uncertain('ROS clock reversed during request')
        self.last_clock = now
        if not self.quality_ready(now, wall):
            self.invalidate('vslam_diagnostic_unavailable')
        elif self.pose_stamp is not None and not fresh(self.pose_stamp, self.pose_at, now, wall,
                                                      self.settings['pose_timeout_s']):
            self.invalidate('pose_stale')
        self.retry_pose()
        self.poll_requests(wall)
        if wall - self.last_status >= self.settings['status_period_s']:
            self.last_status = wall
            status = {'schema': 'kart.lap_status.v1', 'lap': self.detector.count,
                      'reason': self.detector.reason, 'armed': self.detector.armed_at is not None,
                      'course_length_m': self.detector.length, 'progress_m': self.detector.progress,
                      'vgl_enabled': self.settings['vgl_enabled'],
                      'cache_clear_enabled': self.settings['cache_clear_enabled'],
                      'vgl': self.vgl.summary(), 'cache': self.cache.summary()}
            self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))


def main():
    rclpy.init()
    node = None
    try:
        node = LapManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
