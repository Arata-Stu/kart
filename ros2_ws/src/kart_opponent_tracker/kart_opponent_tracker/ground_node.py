import math
import message_filters
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2DArray
from kart_interfaces.msg import OpponentGroundPoint, OpponentGroundPoints
from .ground import ground_point


class GroundNode(Node):
    def __init__(self):
        super().__init__("opponent_ground")
        fixed = ParameterDescriptor(read_only=True)
        self.base = self.declare_parameter("target_frame", "base_link", fixed).value
        self.z = self.declare_parameter("ground_z_m", float("nan"), fixed).value
        self.limit = self.declare_parameter("max_range_m", 10.0, fixed).value
        if not self.base or not math.isfinite(self.z) or not 0 < self.limit < math.inf:
            raise ValueError("Set measured ground_z_m, valid target_frame and positive max_range_m")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.output = self.create_publisher(OpponentGroundPoints, "ground_points", qos_profile_sensor_data)
        self.markers = self.create_publisher(Marker, "ground_markers", qos_profile_sensor_data)
        inputs = [message_filters.Subscriber(self, cls, topic, qos_profile=qos_profile_sensor_data)
                  for cls, topic in [(Detection2DArray, "tracks"), (CameraInfo, "camera_info")]]
        self.sync = message_filters.TimeSynchronizer(inputs, 60)
        self.sync.registerCallback(self.project)

    def project(self, msg, info):
        result = OpponentGroundPoints()
        result.header.stamp = msg.header.stamp
        result.header.frame_id = self.base
        if (msg.header.frame_id == info.header.frame_id and msg.header.frame_id and
                (info.width, info.height) == (640, 480) and max(info.binning_x, info.binning_y) <= 1 and
                not (info.roi.x_offset or info.roi.y_offset) and
                (info.roi.width, info.roi.height) in ((0, 0), (640, 480)) and Time.from_msg(msg.header.stamp).nanoseconds):
            try:
                tf = self.tf.lookup_transform(self.base, msg.header.frame_id,
                                              Time.from_msg(msg.header.stamp)).transform
            except TransformException as exc:
                self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            else:
                t, q = tf.translation, tf.rotation
                for det in msg.detections:
                    box = det.bbox
                    u = box.center.position.x
                    v = box.center.position.y + box.size_y / 2
                    if not (box.size_x > 0 and box.size_y > 0 and 0 <= u < info.width and
                            0 < v < info.height - 1):
                        continue
                    point = ground_point((u, v), info.k, info.d, info.distortion_model,
                                         (t.x, t.y, t.z), (q.x, q.y, q.z, q.w), self.z, self.limit)
                    if point is not None:
                        entry = OpponentGroundPoint(track_id=det.id)
                        entry.point.x, entry.point.y, entry.point.z = map(float, point)
                        result.tracks.append(entry)
        self.output.publish(result)
        marker = Marker(header=result.header, ns="opponent_contact", id=0, type=Marker.SPHERE_LIST)
        marker.action = Marker.ADD if result.tracks else Marker.DELETE
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color.g = marker.color.a = 1.0
        marker.lifetime.nanosec = 300000000
        marker.points = [track.point for track in result.tracks]
        self.markers.publish(marker)


def main():
    rclpy.init()
    node = GroundNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
