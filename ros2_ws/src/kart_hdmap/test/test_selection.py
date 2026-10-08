"""Actual ROS parameter transaction tests; no emulated ROS implementation."""

import unittest
from unittest.mock import patch

try:
    import rclpy
    from kart_hdmap.node import HDMapNode
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.parameter import Parameter
except ImportError as exc:
    raise unittest.SkipTest(f"ROS/generated messages unavailable: {exc}") from exc


class SelectionTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        with patch(
            "kart_hdmap.node.load",
            return_value={"frame_id": "map", "lanes": [], "obstacles": []},
        ):
            self.node = HDMapNode()
        self.addCleanup(self.node.destroy_node)

    def test_parameter_and_topic_share_state(self):
        from std_msgs.msg import String

        with patch.object(self.node, "publish") as publish:
            result = self.node.set_parameters_atomically(
                [Parameter("line_type", value="raceline")]
            )
            self.assertTrue(result.successful)
            self.assertEqual(self.node.line_type, "raceline")
            self.node.choose_line(String(data="customline"))
            self.assertEqual(self.node.get_parameter("line_type").value, "customline")
            self.assertEqual(publish.call_count, 2)

    def test_invalid_value_keeps_previous_selection(self):
        with patch.object(self.node, "publish") as publish:
            result = self.node.set_parameters_atomically(
                [Parameter("line_type", value="typo")]
            )
            self.assertFalse(result.successful)
            self.assertEqual(self.node.line_type, "centerline")
            publish.assert_not_called()

    def test_other_validator_rejection_does_not_apply_selection(self):
        self.node.add_on_set_parameters_callback(
            lambda _: SetParametersResult(successful=False, reason="rejected")
        )
        with patch.object(self.node, "publish") as publish:
            result = self.node.set_parameters_atomically(
                [Parameter("line_type", value="raceline")]
            )
            self.assertFalse(result.successful)
            self.assertEqual(self.node.line_type, "centerline")
            publish.assert_not_called()
