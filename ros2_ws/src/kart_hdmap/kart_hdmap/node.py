"""Read-only HDMap server. No TF broadcaster or online planner."""

import json
import math

import rclpy
from kart_interfaces.msg import ReferenceLine
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from .document import KINDS, load, select
from .messages import markers, path_message, reference_message


class HDMapNode(Node):
    def __init__(self):
        super().__init__("hdmap_server")
        defaults = {
            "map_file": "",
            "frame_id": "map",
            "lane_id": "",
            "line_type": "centerline",
            "centerline_speed_mps": 0.0,
            "publish_rate_hz": 1.0,
            "line_width_m": 0.04,
            "label_height_m": 0.25,
        }
        for key, value in defaults.items():
            self.declare_parameter(
                key, value, ParameterDescriptor(read_only=key != "line_type")
            )
        self.settings = {k: self.get_parameter(k).value for k in defaults}
        for key in ("publish_rate_hz", "line_width_m", "label_height_m"):
            if not math.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(key + " must be finite and positive")
        if self.settings["publish_rate_hz"] > 30:
            raise ValueError("publish_rate_hz must be <= 30")
        if self.settings["line_type"] not in KINDS:
            raise ValueError("Unknown line_type")
        speed = self.settings["centerline_speed_mps"]
        if not math.isfinite(speed) or speed < 0:
            raise ValueError("centerline_speed_mps must be finite and nonnegative")
        self.line_type = self.settings["line_type"]
        self.document = load(self.settings["map_file"], self.settings["frame_id"])
        self.selected = self.settings["lane_id"]
        select(self.document, self.selected)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.reference_pub = self.create_publisher(
            ReferenceLine, "planning/reference_line", qos
        )
        self.create_subscription(String, "hdmap/select_line", self.choose_line, 1)
        self.marker_pub = self.create_publisher(MarkerArray, "hdmap/markers", qos)
        self.document_pub = self.create_publisher(String, "hdmap/document", qos)
        self.selected_pub = self.create_publisher(String, "hdmap/selected_lane", qos)
        self.selected_line_pub = self.create_publisher(
            String, "hdmap/selected_line", qos
        )
        self.paths = {k: self.create_publisher(Path, "hdmap/" + k, qos) for k in KINDS}
        self.create_subscription(String, "hdmap/select_lane", self.choose, 1)
        self.create_service(Trigger, "hdmap/reload", self.reload)
        self.create_timer(1.0 / self.settings["publish_rate_hz"], self.publish)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.add_post_set_parameters_callback(self.apply_parameters)
        self.get_logger().info("HDMap settings: " + json.dumps(self.settings))
        self.publish()

    def validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == "line_type" and (
                parameter.type_ != Parameter.Type.STRING or parameter.value not in KINDS
            ):
                return SetParametersResult(
                    successful=False,
                    reason="line_type must be centerline, raceline or customline",
                )
        return SetParametersResult(successful=True)

    def apply_parameters(self, parameters):
        # Apply only after the entire parameter transaction has committed.
        for parameter in parameters:
            if parameter.name == "line_type":
                self.line_type = parameter.value
                self.settings["line_type"] = parameter.value
                self.publish()
                break

    def choose_line(self, msg):
        result = self.set_parameters_atomically(
            [Parameter("line_type", value=msg.data)]
        )
        if not result.successful:
            self.get_logger().error(result.reason)

    def choose(self, msg):
        try:
            select(self.document, msg.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self.selected = msg.data
        self.publish()

    def reload(self, request, response):
        try:
            candidate = load(self.settings["map_file"], self.settings["frame_id"])
            select(candidate, self.selected)
            self.document = candidate
            self.publish()
            response.success, response.message = True, "HDMap reloaded"
        except (ValueError, KeyError, TypeError, OSError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        lane = select(self.document, self.selected)
        self.reference_pub.publish(
            reference_message(
                lane,
                self.line_type,
                self.settings["centerline_speed_mps"],
                self.document["frame_id"],
                stamp,
            )
        )
        self.marker_pub.publish(
            markers(
                self.document,
                self.selected,
                stamp,
                self.settings["line_width_m"],
                self.settings["label_height_m"],
            )
        )
        for kind, publisher in self.paths.items():
            publisher.publish(
                path_message(
                    lane["lines"].get(kind) if lane else None,
                    lane["closed"] if lane else False,
                    self.document["frame_id"],
                    stamp,
                )
            )
        self.document_pub.publish(
            String(data=json.dumps(self.document, allow_nan=False))
        )
        self.selected_pub.publish(String(data=self.selected))
        self.selected_line_pub.publish(String(data=self.line_type))


def main():
    rclpy.init()
    node = None
    try:
        node = HDMapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
