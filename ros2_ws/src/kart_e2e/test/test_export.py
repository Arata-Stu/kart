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
        from kart_e2e.contract import model_contract, runtime_settings
        from kart_e2e.export_onnx import export
        from kart_e2e.model import Policy, backbone

        torch.set_num_threads(2)
        encoder = backbone(os.environ["KART_DINOV3_REPO"])
        with tempfile.TemporaryDirectory() as directory:
            for mode in ("steer_only", "steer_throttle"):
                output = Path(directory) / mode
                policy = Policy(encoder, mode)
                policy.runtime = {"fixed_throttle": 0.12, "max_throttle": 0.2}
                metadata = export(policy, output)
                self.assertEqual(runtime_settings(output), policy.runtime)
                self.assertEqual(model_contract(output)[1], mode)
                self.assertLess(metadata["ort_max_abs_error"], 1e-5)
                # Extend export parity to raw RGB pixels through training preprocessing.
                import numpy as np
                import onnxruntime as ort
                from PIL import Image
                from kart_e2e.model import preprocess
                rgb = np.random.default_rng(42).integers(0, 256, (240, 424, 3), dtype=np.uint8)
                resized = np.rint(rgb.reshape(120, 2, 212, 2, 3).astype(np.float32).mean((1, 3)))
                normalized = (resized / 255 - np.array([.485,.456,.406], np.float32)) / np.array([.229,.224,.225], np.float32)
                tensor = normalized.transpose(2,0,1)[None].copy()
                with torch.no_grad():
                    expected = policy(preprocess(Image.fromarray(rgb)).unsqueeze(0)).numpy()
                session = ort.InferenceSession(str(output/'model.onnx'), providers=['CPUExecutionProvider'])
                actual = session.run(['control'], {'image': tensor})[0]
                np.testing.assert_allclose(actual, expected, rtol=1e-4, atol=1e-5)
