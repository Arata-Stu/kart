import json
import tempfile
import unittest
from pathlib import Path

import yaml
from kart_bringup.e2e import boolean, container_config, validate_encoder

ROOT = Path(__file__).resolve().parents[1] / "config/e2e"


class E2EConfigTests(unittest.TestCase):
    def test_throttle_selection_rejects_missing_output_and_conflicts(self):
        from kart_bringup.e2e import throttle_selection
        self.assertTrue(throttle_selection('', 'steer_only'))
        self.assertTrue(throttle_selection('fixed', 'steer_throttle', 0.0))
        self.assertFalse(throttle_selection('model', 'steer_throttle'))
        for selection, mode, fixed in [('model', 'steer_only', None),
                                        ('model', 'steer_throttle', 0.0),
                                        ('invalid', 'steer_throttle', None)]:
            with self.subTest(selection=selection, mode=mode, fixed=fixed), self.assertRaises(ValueError):
                throttle_selection(selection, mode, fixed)

    def test_container_ownership_and_false_override(self):
        config = container_config(ROOT / "launch.json", {})
        self.assertTrue(config["create_container"])
        config = container_config(
            ROOT / "launch.json",
            {"create_container": False, "container_name": "/shared/c"},
        )
        self.assertFalse(config["create_container"])
        self.assertEqual(config["container_name"], "/shared/c")
        with self.assertRaises(ValueError):
            container_config(ROOT / "launch.json", {"container_name": "/shared/c"})
        with self.assertRaises(ValueError):
            boolean("yes")

    def test_encoder_contract_and_mismatch(self):
        result = validate_encoder(ROOT / "image_encoder.yaml")
        self.assertEqual(len(result), 6)
        with tempfile.TemporaryDirectory() as directory:
            config = yaml.safe_load((ROOT / "image_encoder.yaml").read_text())
            config["/**/e2e_resize"]["ros__parameters"]["keep_aspect_ratio"] = True
            path = Path(directory) / "bad.yaml"
            path.write_text(yaml.safe_dump(config))
            with self.assertRaises(ValueError):
                validate_encoder(path)

    def test_unknown_launch_key(self):
        with tempfile.TemporaryDirectory() as directory:
            config = json.loads((ROOT / "launch.json").read_text())
            config["typo"] = True
            path = Path(directory) / "launch.json"
            path.write_text(json.dumps(config))
            with self.assertRaises(ValueError):
                container_config(path, {})
