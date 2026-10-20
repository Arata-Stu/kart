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

    def test_speed_override_opt_out_and_transaction_rollback(self):
        with patch.object(self.node, 'publish') as publish:
            result=self.node.set_parameters_atomically([Parameter('speed_override_mps', value=.8)])
            self.assertFalse(result.successful)
            self.assertEqual(self.node.settings['speed_override_mps'], -1.)
            publish.assert_not_called()
            # Opt-in setting would normally be supplied at node startup.
            self.node.settings['allow_speed_override']=True
            result=self.node.set_parameters_atomically([Parameter('speed_override_mps', value=.8)])
            self.assertTrue(result.successful)
            self.assertEqual(self.node.settings['speed_override_mps'], .8)
            publish.assert_called_once()
            self.node.add_on_set_parameters_callback(
                lambda _: SetParametersResult(successful=False, reason='other validator'))
            result=self.node.set_parameters_atomically([Parameter('speed_override_mps', value=.9)])
            self.assertFalse(result.successful)
            self.assertEqual(self.node.settings['speed_override_mps'], .8)
            self.assertEqual(publish.call_count, 1)

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

    def test_tuning_replace_publishes_only_complete_transaction(self):
        from kart_hdmap.tuning_bridge import TuningBridge
        bridge = TuningBridge.__new__(TuningBridge)
        bridge.node = self.node
        candidate = dict(document={"frame_id":"map", "lanes":[], "obstacles":[]},
                         lane_id='next', line_type='customline', centerline_speed_mps=.4,
                         speed_override_mps=-1.)
        seen = []
        with patch.object(self.node, 'publish', side_effect=lambda: seen.append(
                (self.node.document, self.node.selected, self.node.line_type,
                 self.node.runtime_centerline_speed))):
            bridge.replace(candidate)
        self.assertEqual(seen, [(candidate['document'], 'next', 'customline', .4)])
        self.assertEqual(self.node.get_parameter('line_type').value, 'customline')
        self.assertFalse(self.node.tuning_transaction)

    def test_tuning_parameter_rejection_keeps_previous_map(self):
        from kart_hdmap.tuning_bridge import TuningBridge
        bridge = TuningBridge.__new__(TuningBridge)
        bridge.node = self.node
        old = self.node.document
        self.node.add_on_set_parameters_callback(
            lambda _: SetParametersResult(successful=False, reason='reject tuning'))
        with patch.object(self.node, 'publish') as publish:
            with self.assertRaises(ValueError):
                bridge.replace(dict(document={'changed':True}, lane_id='next',
                                    line_type='customline', centerline_speed_mps=.4,
                                    speed_override_mps=-1.))
            publish.assert_not_called()
        self.assertIs(self.node.document, old)
        self.assertEqual(self.node.line_type, 'centerline')
        self.assertFalse(self.node.tuning_transaction)
