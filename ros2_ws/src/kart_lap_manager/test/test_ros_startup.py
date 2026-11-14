"""Construct real ROS nodes without spinning or requesting any maintenance."""

import unittest

try:
    import rclpy
except ImportError as error:
    raise unittest.SkipTest(f'ROS startup check requires installed ROS dependencies: {error}') from error

from rclpy.context import Context
from rclpy.parameter import Parameter
from kart_lap_manager.node import LapManager


class RosStartupTests(unittest.TestCase):
    def test_each_feature_selection_constructs_with_real_ros_client_registry(self):
        for vgl, cache in ((False, False), (True, False), (False, True), (True, True)):
            with self.subTest(vgl=vgl, cache=cache):
                context = Context()
                node = None
                rclpy.init(args=[], context=context)
                try:
                    node = LapManager(context=context, parameter_overrides=[
                        Parameter('vgl_enabled', value=vgl),
                        Parameter('cache_clear_enabled', value=cache),
                    ])
                    self.assertEqual(node.get_name(), 'lap_manager')
                    self.assertEqual(len(list(node.clients)), int(vgl) + int(cache))
                    for key, enabled in (('vgl', vgl), ('cache', cache)):
                        client = node.maintenance_clients[key]
                        if enabled:
                            self.assertIn(client, list(node.clients))
                        else:
                            self.assertIsNone(client)
                finally:
                    if node is not None:
                        node.destroy_node()
                    rclpy.shutdown(context=context)


if __name__ == '__main__':
    unittest.main()
