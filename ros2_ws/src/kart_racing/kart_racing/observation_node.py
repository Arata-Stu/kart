import hashlib
import math
from collections import deque

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from kart_interfaces.msg import HeadToHeadObservation, OpponentGroundPoints, PassingCorridor, ReferenceLine
from .observation import Course, velocity


class ObservationNode(Node):
    def __init__(self):
        super().__init__("racing_observer")
        fixed = ParameterDescriptor(read_only=True)
        self.sigma = self.declare_parameter("position_sigma_m", .03, fixed).value
        if not 0 < self.sigma <= .5:
            raise ValueError("Invalid position_sigma_m")
        self.pub = self.create_publisher(HeadToHeadObservation, "/racing/observation", qos_profile_sensor_data)
        reference_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ReferenceLine, "/planning/reference_line", self.reference, reference_qos)
        self.create_subscription(OpponentGroundPoints, "/racing/ground_points", self.observe, qos_profile_sensor_data)
        self.create_subscription(PassingCorridor, "/racing/passing_corridor", self.clearance, qos_profile_sensor_data)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.course = self.zone = None
        self.revision = self.track = self.frame = ""
        self.samples = deque(maxlen=256)

    def clearance(self, msg):
        self.zone = msg

    def reference(self, msg):
        xy = [(p.x, p.y) for p in msg.points]
        digest = hashlib.sha256(np.asarray(xy, dtype="<f8").tobytes() +
                                np.asarray(msg.speeds, dtype="<f8").tobytes()).hexdigest()
        revision = f"{msg.header.frame_id}/{msg.lane_id}/{msg.line_type}/{digest}"
        if self.course and msg.closed and revision == self.revision:
            return
        self.course = self.zone = None
        self.samples.clear()
        self.frame, self.revision = msg.header.frame_id, revision
        if msg.closed and self.frame:
            try:
                self.course = Course(xy, msg.speeds)
            except ValueError as exc:
                self.get_logger().warning(str(exc), throttle_duration_sec=2.)

    def observe(self, msg):
        out = HeadToHeadObservation()
        out.header.stamp = msg.header.stamp
        out.measurement_stamp = msg.header.stamp
        out.header.frame_id, out.reference_id = self.frame, self.revision
        stamp = Time.from_msg(msg.header.stamp)
        t = stamp.nanoseconds * 1e-9
        if self.course:
            out.course_length = self.course.length
        if (not self.course or len(msg.tracks) != 1 or msg.header.frame_id != "base_link" or
                not 0 <= (self.get_clock().now().nanoseconds - stamp.nanoseconds) * 1e-9 <= .2):
            self.samples.clear()
            self.pub.publish(out)
            return
        target = msg.tracks[0]
        try:
            tf = self.buffer.lookup_transform(self.frame, "base_link", stamp).transform
            q = np.array([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
            origin = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
            point = np.array([target.point.x, target.point.y, target.point.z])
            rotated = point + 2*np.cross(q[:3], np.cross(q[:3], point) + q[3]*point)
            ego, out.ego_d, out.speed_cap = self.course.project(origin[:2])
            other, out.opponent_d, _ = self.course.project((origin+rotated)[:2])
            if self.track != target.track_id or (self.samples and not 0 < t-self.samples[-1][0] <= .2):
                self.samples.clear()
            self.track = target.track_id
            if self.samples:
                ego = self.course.unwrap(ego, self.samples[-1][1])
            out.ego_s = ego
            out.opponent_s = self.course.unwrap(other, ego)
            if self.samples:
                dt = t-self.samples[-1][0]
                if max(abs(ego-self.samples[-1][1]), abs(out.opponent_s-self.samples[-1][2])) > 20*dt+.15:
                    raise ValueError("discontinuous Frenet projection")
            self.samples.append((t, ego, out.opponent_s))
            while t-self.samples[0][0] > .8:
                self.samples.popleft()
            out.track_id = int.from_bytes(hashlib.sha256(self.track.encode()).digest()[:8], "big")
            if len(self.samples) >= 3 and t-self.samples[0][0] >= .35:
                ego_v, _ = velocity(self.samples, 1, self.sigma)
                v, var = velocity(self.samples, 2, self.sigma)
                if min(ego_v, v) < -.12 or max(ego_v, v) > 20:
                    raise ValueError("unsupported reverse or implausible motion")
                out.ego_speed, out.opponent_speed = max(0., ego_v), max(0., v)
                out.opponent_speed_variance, out.opponent_d_variance = var, self.sigma**2
                out.observed = True
                self.apply_zone(out, stamp)
        except (TransformException, ValueError) as exc:
            self.samples.clear()
            self.get_logger().warning(str(exc), throttle_duration_sec=2.)
        self.pub.publish(out)

    def apply_zone(self, out, stamp):
        z = self.zone
        if not z or not z.clear or z.header.frame_id != self.frame or z.reference_id != self.revision or z.track_id != out.track_id:
            return
        age = (stamp.nanoseconds-Time.from_msg(z.header.stamp).nanoseconds)*1e-9
        if (not 0 <= age < .2 or not all(math.isfinite(x) for x in
                (z.start_s, z.end_s, z.left_width, z.right_width, z.speed_cap)) or
                not z.start_s <= out.ego_s < z.end_s or min(z.left_width,z.right_width,z.speed_cap) <= 0):
            return
        out.passing_allowed = True
        out.header.stamp = z.header.stamp
        out.zone_end_s, out.left_width, out.right_width = z.end_s, z.left_width, z.right_width
        out.speed_cap = min(out.speed_cap, z.speed_cap)


def main():
    rclpy.init()
    node = ObservationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
