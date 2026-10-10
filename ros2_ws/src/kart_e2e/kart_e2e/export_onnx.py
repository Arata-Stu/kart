"""Offline export only; there is no PyTorch ROS inference node."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch
from torch import nn
from torch.nn import functional as F

from .model import SPEC, load_checkpoint


class ExportPolicy(nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, image):
        # Encoder emits normalized NCHW 212x120. Patch padding must follow normalization.
        return self.policy(F.pad(image, (6, 6, 4, 4)))


def export(policy, output):
    from .contract import validate_runtime

    runtime = validate_runtime(policy.runtime)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    model = ExportPolicy(policy.cpu().eval()).eval()
    generator = torch.Generator().manual_seed(42)
    example = torch.randn((1, 3, 120, 212), generator=generator)
    path = output / "model.onnx"
    with torch.inference_mode():
        torch.onnx.export(
            model,
            example,
            str(path),
            input_names=["image"],
            output_names=["control"],
            opset_version=17,
            dynamo=False,
            do_constant_folding=True,
        )
    graph = onnx.load(path)
    onnx.checker.check_model(graph)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    session = ort.InferenceSession(
        str(path), sess_options=options, providers=["CPUExecutionProvider"]
    )
    # Multiple probes, including neutral normalized input. Failure leaves no valid metadata.
    maximum_error = 0.0
    for probe in (
        example,
        torch.zeros_like(example),
        torch.randn(example.shape, generator=generator),
    ):
        with torch.inference_mode():
            reference = model(probe).numpy()
        actual = session.run(["control"], {"image": probe.numpy()})[0]
        np.testing.assert_allclose(actual, reference, rtol=1e-4, atol=1e-5)
        maximum_error = max(maximum_error, float(np.abs(actual - reference).max()))
    metadata = {
        "schema": 1,
        "model_spec": SPEC,
        "output_mode": policy.mode,
        "runtime": runtime,
        "input_name": "image",
        "input_shape": [1, 3, 120, 212],
        "output_name": "control",
        "output_shape": [1, 1 if policy.mode == "steer_only" else 2],
        "onnx_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "ort_max_abs_error": maximum_error,
    }
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2))
    return metadata


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("checkpoint", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--repo", required=True)
    p.add_argument("--fixed-throttle", type=float, default=None)
    p.add_argument("--max-throttle", type=float, default=None)
    a = p.parse_args()
    policy = load_checkpoint(a.checkpoint, a.repo)
    if a.fixed_throttle is not None or a.max_throttle is not None:
        runtime = policy.runtime or {"fixed_throttle": 0.0, "max_throttle": 0.2}
        if (
            policy.runtime is None
            and policy.mode == "steer_only"
            and a.fixed_throttle is None
        ):
            p.error("Legacy steer-only checkpoint needs explicit --fixed-throttle")
        if a.fixed_throttle is not None:
            runtime["fixed_throttle"] = a.fixed_throttle
        if a.max_throttle is not None:
            runtime["max_throttle"] = a.max_throttle
        policy.runtime = runtime
    print(json.dumps(export(policy, a.output), indent=2))


if __name__ == "__main__":
    main()
