"""Optional real DINOv3 smoke test; no pretrained quality claim or network access."""

import importlib.util
import os
import tempfile
import unittest
from pathlib import Path

REPO = os.environ.get("KART_DINOV3_REPO")


@unittest.skipUnless(
    REPO and importlib.util.find_spec("torch"), "Set KART_DINOV3_REPO and install torch"
)
class ModelTests(unittest.TestCase):
    def test_both_heads_gradient_and_checkpoint(self):
        import torch
        from kart_e2e.model import SPEC, Policy, backbone, load_checkpoint, preprocess
        from PIL import Image

        torch.set_num_threads(2)
        image = preprocess(Image.new("RGB", (424, 240), (200, 100, 20))).unsqueeze(0)
        self.assertEqual(tuple(image.shape), (1, 3, 128, 224))
        self.assertTrue(torch.equal(image[:, :, :4], torch.zeros_like(image[:, :, :4])))
        encoder = backbone(REPO)
        for mode, width in [("steer_only", 1), ("steer_throttle", 2)]:
            model = Policy(encoder, mode)
            output = model(image)
            self.assertEqual(tuple(output.shape), (1, width))
            output.sum().backward()
            self.assertIsNotNone(model.head[0].weight.grad)
            self.assertTrue(all(p.grad is None for p in encoder.parameters()))
            self.assertFalse(encoder.training)
            model.eval()
            expected = model(image).detach()
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "policy.pt"
                state = {"spec": SPEC, "mode": mode, "state_dict": model.state_dict()}
                torch.save(state, path)
                restored = load_checkpoint(path, REPO)
                torch.testing.assert_close(expected, restored(image))
                state["spec"] = {**SPEC, "width": 999}
                torch.save(state, path)
                with self.assertRaises(ValueError):
                    load_checkpoint(path, REPO)
        model = Policy(encoder, "steer_only", frozen=False)
        model(image).sum().backward()
        self.assertTrue(any(p.grad is not None for p in encoder.parameters()))
