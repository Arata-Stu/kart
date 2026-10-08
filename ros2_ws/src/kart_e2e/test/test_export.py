import importlib.util
import os
import tempfile
import unittest
from pathlib import Path


@unittest.skipUnless(
    os.environ.get("KART_DINOV3_REPO")
    and all(importlib.util.find_spec(x) for x in ("torch", "onnx", "onnxruntime")),
    "Optional ONNX dependencies and KART_DINOV3_REPO required",
)
class ExportTests(unittest.TestCase):
    def test_both_modes_onnx_parity(self):
        import torch
        from kart_e2e.contract import model_contract
        from kart_e2e.export_onnx import export
        from kart_e2e.model import Policy, backbone

        torch.set_num_threads(2)
        encoder = backbone(os.environ["KART_DINOV3_REPO"])
        with tempfile.TemporaryDirectory() as directory:
            for mode in ("steer_only", "steer_throttle"):
                output = Path(directory) / mode
                metadata = export(Policy(encoder, mode), output)
                self.assertEqual(model_contract(output)[1], mode)
                self.assertLess(metadata["ort_max_abs_error"], 1e-5)
