import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from kart_e2e.contract import DINOV3_COMMIT, SPEC, model_contract, rgb_spec


class ContractTests(unittest.TestCase):
    def test_mode_shape_and_checksum(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "model.onnx").write_bytes(b"contract fixture, not a model")
            metadata = dict(
                schema=1,
                model_spec=SPEC,
                output_mode="steer_only",
                input_name="image",
                input_shape=[1, 3, 120, 212],
                output_name="control",
                output_shape=[1, 1],
                onnx_sha256=hashlib.sha256(
                    (root / "model.onnx").read_bytes()
                ).hexdigest(),
            )
            path = root / "metadata.json"
            path.write_text(json.dumps(metadata))
            self.assertEqual(model_contract(root)[1], "steer_only")
            metadata["model_spec"] = rgb_spec("dinov3_vitt16")
            path.write_text(json.dumps(metadata))
            self.assertEqual(model_contract(root)[1], "steer_only")
            metadata["model_spec"] = dict(SPEC, encoder_commit="6876159a11b4df116f30f667f8c9888617df0751")
            path.write_text(json.dumps(metadata))
            with self.assertRaises(ValueError):
                model_contract(root)
            metadata["model_spec"] = SPEC
            metadata["output_shape"] = [1, 2]
            path.write_text(json.dumps(metadata))
            with self.assertRaises(ValueError):
                model_contract(root)
            metadata["output_mode"] = "steer_throttle"
            path.write_text(json.dumps(metadata))
            self.assertEqual(model_contract(root)[1], "steer_throttle")
            (root / "model.onnx").write_bytes(b"modified")
            with self.assertRaises(ValueError):
                model_contract(root)

    def test_source_pin_matches_contract(self):
        root = next(parent for parent in Path(__file__).resolve().parents
                    if (parent / "e2e.repos").is_file())
        self.assertIn("version: " + DINOV3_COMMIT, (root / "e2e.repos").read_text())

    def test_missing_directory(self):
        with self.assertRaises(ValueError):
            model_contract("")
