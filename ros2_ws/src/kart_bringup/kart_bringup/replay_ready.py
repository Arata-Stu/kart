"""Resume paused playback once both localization components subscribe to stereo."""

import math
import time

import rclpy
from rclpy.node import Node
from rosbag2_interfaces.srv import Resume


class ReplayReady(Node):
    def __init__(self):
        super().__init__("replay_ready")
        self.topics = self.declare_parameter(
            "image_topics",
            ["/realsense/infra1/image_rect_raw", "/realsense/infra2/image_rect_raw"],
        ).value
        self.nodes = set(
            self.declare_parameter(
                "subscriber_nodes", ["visual_slam", "visual_global_localization"]
            ).value
        )
        service = self.declare_parameter(
            "resume_service", "/rosbag2_player/resume"
        ).value
        timeout = self.declare_parameter("timeout_s", 120.0).value
        if (
            not self.topics
            or not self.nodes
            or not math.isfinite(timeout)
            or timeout <= 0
        ):
            raise ValueError(
                "Require nonempty readiness topics/nodes and positive finite timeout_s"
            )
        self.deadline = time.monotonic() + timeout
        self.client = self.create_client(Resume, service)
        self.future = None
        self.finished = False
        self.failed = False
        self.timer = self.create_timer(0.25, self.check)

    def check(self):
        if self.future is not None and self.future.done():
            try:
                reply = self.future.result()
                if getattr(reply, "return_code", 0) != 0:
                    raise RuntimeError(str(reply))
                self.get_logger().info(
                    "Localization input subscribers ready; bag playback resumed"
                )
            except Exception as error:  # noqa: BLE001 - ROS future may hold middleware errors.
                self.failed = True
                self.get_logger().error(str(error))
            self.finished = True
        elif time.monotonic() > self.deadline:
            self.failed = self.finished = True
            self.get_logger().error(
                "Startup timeout: bag remains paused. Check component/engine errors. Resume: ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'"
            )
        elif self.future is None and self.client.service_is_ready():
            if all(
                self.nodes
                <= {s.node_name for s in self.get_subscriptions_info_by_topic(topic)}
                for topic in self.topics
            ):
                self.future = self.client.call_async(Resume.Request())


def main():
    rclpy.init()
    node = ReplayReady()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        failed = node.failed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(1)
