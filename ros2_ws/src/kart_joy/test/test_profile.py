import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import contextlib
import io
import threading
import time

from kart_joy_tools import profile
from kart_joy_tools.app import Bridge, cui


class ProfileTests(unittest.TestCase):
    def test_bridge_rejects_stale_or_previous_device_samples(self):
        bridge = Bridge.__new__(Bridge)
        bridge.lock = threading.Lock()
        bridge.data = dict(connected=True, generation=2)
        bridge.raw = dict(generation=1, raw_axes=[1.0], raw_buttons=[1])
        bridge.status_time = bridge.raw_time = time.monotonic()
        self.assertFalse(bridge.state()["connected"])
        bridge.raw["generation"] = 2
        self.assertTrue(bridge.state()["connected"])
        bridge.raw_time -= 1
        self.assertFalse(bridge.state()["connected"])
        bridge.status_time -= 4
        self.assertFalse(bridge.state()["online"])

    def test_cui_edit_save_resume(self):
        class Bridge:
            def __init__(self):
                self.value = profile.default_profile()
                self.calls = []
            def get_profile(self):
                return copy.deepcopy(self.value)
            def call(self, operation, enabled):
                self.calls.append((operation, enabled))
                return "ok"
            def save(self, value):
                self.value = copy.deepcopy(profile.validate(value))
                self.calls.append(("save", None))
                return "saved"
        bridge = Bridge()
        # The first resume must be refused because the edit is unsaved.
        answers = ["button", "cross", "disable", "resume", "save", "resume"]
        with patch("builtins.input", side_effect=answers), contextlib.redirect_stdout(io.StringIO()):
            cui(bridge)
        self.assertEqual(bridge.value["buttons"]["cross"]["code"], -1)
        self.assertEqual(bridge.calls, [("configure", True), ("save", None), ("configure", False)])

    def test_shipped_defaults_match_schema(self):
        path = Path(__file__).parents[1] / "config/dualsense.yaml"
        self.assertEqual(profile.load(path), profile.default_profile())

    def test_bad_profiles_rejected(self):
        for key, value in (("code", 64), ("deadzone", 1), ("deadzone", float("nan")),
                           ("center", 1), ("mode", "unknown")):
            data = profile.default_profile()
            data["axes"]["left_x"][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                profile.validate(data)
        data = profile.default_profile()
        data["buttons"]["cross"]["source"] = "axes"
        with self.assertRaises(ValueError):
            profile.validate(data)

    def test_atomic_save_preserves_previous_on_invalid_input(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "profile.yaml"
            original = profile.default_profile()
            profile.save(path, original)
            invalid = copy.deepcopy(original)
            invalid["axes"]["l2"]["positive"] = -1
            with self.assertRaises(ValueError):
                profile.save(path, invalid)
            self.assertEqual(profile.load(path), original)

    def test_button_capture_ignores_already_held_button_and_noise(self):
        baseline = dict(generation=1, connected=True, raw_axes=[0.0], raw_buttons=[1, 0],
                        axes=[dict(code=0)], buttons=[dict(code=0), dict(code=1)])
        current = copy.deepcopy(baseline)
        current["raw_axes"][0] = 0.03
        self.assertIsNone(profile.detect(baseline, current, "buttons"))
        current["raw_buttons"][1] = 1
        self.assertEqual(profile.detect(baseline, current, "buttons")["code"], 1)

    def test_axis_dpad_capture_and_reconnect_abort(self):
        baseline = dict(generation=1, connected=True, raw_axes=[0.0], raw_buttons=[],
                        axes=[dict(code=0)], buttons=[])
        current = copy.deepcopy(baseline)
        current["raw_axes"][0] = -1.0
        self.assertEqual(profile.detect(baseline, current, "buttons"),
                         dict(source="axis", code=0, direction=-1, threshold=0.5))
        current["generation"] = 2
        with self.assertRaises(ValueError):
            profile.detect(baseline, current, "axes")


if __name__ == "__main__":
    unittest.main()
