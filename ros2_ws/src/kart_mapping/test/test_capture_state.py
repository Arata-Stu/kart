import unittest

from kart_mapping.capture_state import LocalizationGate


class GateTests(unittest.TestCase):
    def test_requires_localized_and_tracking(self):
        gate = LocalizationGate()
        self.assertFalse(gate.accepts(100))
        gate.update(100, {"localized_in_exist_map": "No", "vo_status": "OK"}, 1)
        with self.assertRaises(ValueError):
            gate.require_final(100, 0)
        gate.update(110, {"localized_in_exist_map": "Yes", "vo_status": "OK"}, 2)
        self.assertFalse(gate.accepts(109))
        self.assertTrue(gate.accepts(110))
        gate.require_final(110, 0)

    def test_old_diagnostics_do_not_reset_current_epoch(self):
        gate = LocalizationGate()
        gate.update(200, {"localized_in_exist_map": "Yes", "vo_status": "OK"}, 1)
        gate.update(100, {"localized_in_exist_map": "No"}, 2)
        self.assertTrue(gate.localized)
        self.assertEqual(gate.since, 200)

    def test_loss_requires_new_epoch_and_tail(self):
        gate = LocalizationGate()
        gate.update(100, {"localized_in_exist_map": "Yes", "vo_status": "OK"}, 1)
        gate.update(110, {"localized_in_exist_map": "Yes", "vo_status": "Lost"}, 2)
        self.assertFalse(gate.accepts(120))
        gate.update(130, {"localized_in_exist_map": "Yes", "vo_status": "OK"}, 3)
        self.assertEqual(gate.since, 130)
        with self.assertRaises(ValueError):
            gate.require_final(200, 10)
        gate.require_final(140, 10)
