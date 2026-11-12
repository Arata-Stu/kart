"""Explicitly launched, bounded-rate JPEG view of image-space YOLO results."""
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray

from .debug_image import (Settings, OverlayBuffer, decode_image, draw_frame, encode_jpeg,
                          header_key, placeholder, result_from_message)


class DebugImageNode(Node):
    def __init__(self):
        super().__init__('opponent_debug_image')
        fixed = ParameterDescriptor(read_only=True)
        defaults = Settings()
        values = {name: self.declare_parameter(name, getattr(defaults, name), fixed).value
                  for name in defaults.__dataclass_fields__}
        self.settings = Settings(**values)
        self.class_names = self.declare_parameter('class_names', [''], fixed).value
        if not isinstance(self.class_names, (tuple, list)) or not all(isinstance(n, str) for n in self.class_names):
            raise ValueError('class_names must be a string array')
        self.buffer = OverlayBuffer(self.settings)
        self.last_image = None
        self.last_admitted = float('-inf')
        self.last_admitted_key = None
        self.last_published = float('-inf')
        self.source_issue = 'RGB: waiting for camera input'
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.output = self.create_publisher(CompressedImage, 'debug_image/compressed', qos)
        self.inputs = [self.create_subscription(Image, 'rgb', self.image, qos)]
        for channel in ('detections', 'tracks'):
            self.inputs.append(self.create_subscription(Detection2DArray, channel,
                lambda message, selected=channel: self.result(selected, message), qos))
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(min(.02, 1 / self.settings.max_fps), self.publish, clock=self.wall_clock)

    def image(self, message):
        if self.output.get_subscription_count() == 0:
            return
        now = time.monotonic()
        key = header_key(message.header)
        if key == self.last_admitted_key:
            return
        if now - self.last_admitted < 1 / self.settings.max_fps:
            return
        # Admit at the preview rate before allocating/caching full image arrays.
        self.last_admitted = now
        try:
            image = decode_image(message.width, message.height, message.encoding, message.step, message.data)
        except ValueError as error:
            self.source_issue = 'RGB: invalid input - ' + str(error)
            self.last_image = None
            self.buffer.clear()
            self.get_logger().warning(str(error), throttle_duration_sec=5.)
            return
        self.last_image = now
        self.last_admitted_key = key
        self.source_issue = ''
        self.buffer.add_image(key, image, now)

    def result(self, channel, message):
        if self.output.get_subscription_count() == 0:
            return
        self.buffer.add_result(channel, header_key(message.header),
                               result_from_message(message, self.settings.max_boxes), time.monotonic())

    def publish(self):
        if self.output.get_subscription_count() == 0:
            self.buffer.clear()
            self.last_image = None
            self.last_admitted = float('-inf')
            self.last_admitted_key = None
            self.source_issue = 'RGB: waiting for camera input'
            return
        now = time.monotonic()
        if now - self.last_published < 1 / self.settings.max_fps:
            return
        if self.last_image is None or now - self.last_image > self.settings.image_timeout_s:
            self.buffer.clear()
            image = placeholder(self.source_issue or 'RGB: MISSING - camera input stopped')
            stamp, frame_id = self.get_clock().now().nanoseconds, ''
        else:
            frame = self.buffer.take(now)
            if frame is None:
                return
            image = draw_frame(frame, self.class_names)
            stamp, frame_id = frame.key
        message = CompressedImage()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.header.frame_id = frame_id
        message.format = 'rgb8; jpeg compressed bgr8'
        message.data = encode_jpeg(image, self.settings.jpeg_quality)
        self.output.publish(message)
        self.last_published = now


def main():
    rclpy.init()
    node = DebugImageNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
