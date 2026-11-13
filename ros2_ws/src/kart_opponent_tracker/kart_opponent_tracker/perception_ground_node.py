"""Display-only map projection, independent of opponent tracking/control."""
import time

import message_filters
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from kart_interfaces.msg import GroundDetection, GroundDetections
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

from .perception_ground import (COLOURS, MARKER_LIFETIME_S, Outcome, ProjectionStatus, Settings,
                                project_points, projection_issue, selected_hypothesis, stamp_ns)


class PerceptionGroundNode(Node):
    def __init__(self):
        super().__init__('perception_ground')
        fixed, defaults = ParameterDescriptor(read_only=True), Settings.__dataclass_fields__
        values = {name: self.declare_parameter(name, field.default, fixed).value for name, field in defaults.items()}
        self.settings = Settings(**values)
        self.status = ProjectionStatus()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.output = self.create_publisher(GroundDetections, 'map_points', qos)
        self.markers = self.create_publisher(MarkerArray, 'map_markers', qos)
        self.diagnostics = self.create_publisher(DiagnosticArray, 'map_diagnostics', 1)
        self.inputs = [message_filters.Subscriber(self, cls, topic, qos_profile=qos_profile_sensor_data)
                       for cls, topic in ((Detection2DArray, 'detections'),
                                          (CameraInfo, '/realsense/color/camera_info'))]
        self.inputs[0].registerCallback(lambda msg: self.status.receive('detections', msg))
        self.inputs[1].registerCallback(lambda msg: self.status.receive('camera_info', msg))
        self.sync = message_filters.TimeSynchronizer(self.inputs, 60)
        self.sync.registerCallback(self.project)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.watchdog_timer = self.create_timer(.1, self.watchdog, clock=self.wall_clock)
        self.diagnostic_timer = self.create_timer(1., self.publish_diagnostics, clock=self.wall_clock)

    def publish_objects(self, objects, source_stamp=None, *, valid=True, reason='projected'):
        result = GroundDetections()
        result.valid, result.reason = bool(valid), str(reason)
        result.header.frame_id = self.settings.target_frame
        result.header.stamp = (source_stamp if source_stamp is not None and stamp_ns(source_stamp) > 0
                               else self.get_clock().now().to_msg())
        for obj in objects:
            entry = GroundDetection(detection_id=obj.detection_id, class_id=obj.class_id,
                                    class_name=obj.class_name, score=float(obj.score))
            entry.point = Point(x=float(obj.point[0]), y=float(obj.point[1]), z=float(obj.point[2]))
            result.objects.append(entry)
        self.output.publish(result)
        markers = []
        for kind, colour in COLOURS.items():
            marker = Marker(header=result.header, ns=kind, id=0, type=Marker.SPHERE_LIST)
            marker.points = [obj.point for obj in result.objects if obj.class_name == kind]
            marker.action = Marker.ADD if marker.points else Marker.DELETE
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .08
            marker.color.r, marker.color.g, marker.color.b = colour
            marker.color.a = 1.
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = int(MARKER_LIFETIME_S * 1_000_000_000)
            markers.append(marker)
        self.markers.publish(MarkerArray(markers=markers))

    def project(self, detections, info):
        # Ignore duplicate or delayed callbacks for a previous input image.
        if not self.status.admit(detections):
            return
        now = self.get_clock().now().nanoseconds
        # A paused ROS clock must not make a delayed queued image fresh again.
        issue = self.status.input_issue(time.monotonic()) or projection_issue(detections, info, now)
        if issue:
            outcome = Outcome(issue)
        elif not detections.detections:
            outcome = Outcome('zero_detections')
        elif not any(selected_hypothesis(det, self.settings)
                     for det in detections.detections[:self.settings.max_objects]):
            outcome = Outcome('zero_selected', rejected=dict(class_or_score=min(
                len(detections.detections), self.settings.max_objects),
                object_limit=max(0, len(detections.detections) - self.settings.max_objects)))
        else:
            capture = Time.from_msg(detections.header.stamp)
            try:
                camera_tf = self.tf.lookup_transform(self.settings.projection_frame,
                                                     detections.header.frame_id, capture).transform
            except TransformException as error:
                outcome = Outcome('camera_tf_unavailable', detail=str(error))
            else:
                try:
                    map_tf = self.tf.lookup_transform(self.settings.target_frame,
                                                      self.settings.projection_frame, capture).transform
                except TransformException as error:
                    outcome = Outcome('target_tf_unavailable', detail=str(error))
                else:
                    outcome = project_points(detections, info, camera_tf, map_tf, self.settings, now)
        if outcome.objects:
            expired = (self.status.input_issue(time.monotonic()) or
                       projection_issue(detections, info, self.get_clock().now().nanoseconds))
            if expired:
                outcome = Outcome(expired)
        self.status.synchronized(detections, outcome)
        self.publish_objects(outcome.objects, detections.header.stamp,
                             valid=ProjectionStatus.REASONS[outcome.reason][0] == 0, reason=outcome.reason)
        if outcome.detail:
            self.get_logger().warning(outcome.detail, throttle_duration_sec=5.)

    def watchdog(self):
        snapshot = self.status.view(time.monotonic())
        if snapshot['level'] > 0:
            # These are fresh EMPTY messages, never a replay of old valid points.
            self.publish_objects([], valid=False, reason=snapshot['values']['reason'])

    def publish_diagnostics(self):
        snapshot = self.status.view(time.monotonic())
        status = DiagnosticStatus(level=snapshot['level'], name='perception_ground/projection',
                                  message=snapshot['message'], hardware_id='perception_ground')
        values = dict(snapshot['values'], projection_frame=self.settings.projection_frame,
                      target_frame=self.settings.target_frame, ground_z_m=str(self.settings.ground_z_m))
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        result = DiagnosticArray(status=[status])
        result.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(result)


def main():
    rclpy.init()
    node = PerceptionGroundNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
