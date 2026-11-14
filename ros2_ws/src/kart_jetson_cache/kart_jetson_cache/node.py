"""Dedicated process translating ROS Trigger to the restricted host helper."""

import math

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger

from .client import SingleRequest, request_clear, validate_socket


class JetsonCacheService(Node):
    def __init__(self):
        super().__init__('jetson_cache_service')
        self.settings = {
            key: self.declare_parameter(key, value, ParameterDescriptor(read_only=True)).value
            for key, value in {
                'service_name': '/kart/cache/clear',
                'socket_path': '/run/kart-cache/cache.sock',
                'request_timeout_s': 3.0,
            }.items()
        }
        timeout = self.settings['request_timeout_s']
        if not math.isfinite(timeout) or not 0.1 <= timeout <= 30.0:
            raise ValueError('request_timeout_s must be 0.1..30 seconds')
        if not self.settings['service_name'].startswith('/'):
            raise ValueError('service_name must be absolute')
        self.requests = SingleRequest()
        self.service = self.create_service(
            Trigger, self.settings['service_name'], self.clear,
            callback_group=ReentrantCallbackGroup())
        try:
            validate_socket(self.settings['socket_path'])
        except (OSError, ValueError) as error:
            self.get_logger().error(f'Jetson cache helper unavailable: {error}; requests will fail explicitly')
        self.get_logger().info(
            f'Jetson page cache release service ready: {self.settings}; '
            'only clean file pagecache is released; no GPU/application cache or files are deleted')

    def clear(self, _request, response):
        timeout = self.settings['request_timeout_s']
        response.success, response.message = self.requests.run(
            lambda: request_clear(self.settings['socket_path'], timeout), timeout + 0.1)
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warning(response.message)
        return response


def main():
    rclpy.init()
    node = JetsonCacheService()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
