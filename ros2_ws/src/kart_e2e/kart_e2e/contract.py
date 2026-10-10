"""Lightweight export contract: safe to import in launch without torch/onnx/CUDA."""

import hashlib
import json
from pathlib import Path

DINOV3_COMMIT = "6876159a11b4df116f30f667f8c9888617df0751"
SPEC = {
    "schema": 2,
    "encoder": "dinov3_vits16",
    "width": 212,
    "height": 120,
    "preprocess": "rgb_linear_halfpixel_noaa_uint8_imagenet_center_zero_pad16_v2",
    "encoder_commit": DINOV3_COMMIT,
}


def model_contract(directory):
    if not directory:
        raise ValueError(
            "model_dir is required: export a kart checkpoint to ONNX first"
        )
    directory = Path(directory).expanduser().resolve(strict=True)
    path = directory / "model.onnx"
    metadata = json.loads((directory / "metadata.json").read_text())
    mode = metadata.get("output_mode")
    if (
        metadata.get("schema") != 1
        or metadata.get("model_spec") != SPEC
        or mode not in ("steer_only", "steer_throttle")
        or metadata.get("input_name") != "image"
        or metadata.get("input_shape") != [1, 3, 120, 212]
        or metadata.get("output_name") != "control"
        or metadata.get("output_shape") != [1, 1 if mode == "steer_only" else 2]
    ):
        raise ValueError("ONNX metadata does not match the kart E2E contract")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != metadata.get("onnx_sha256"):
        raise ValueError("ONNX checksum mismatch: re-export the model")
    return {
        "model_file_path": str(path),
        "engine_file_path": str(directory / f"model_{digest[:12]}.plan"),
    }, mode


def validate_runtime(values):
    import math

    if not isinstance(values, dict) or set(values) != {
        "fixed_throttle",
        "max_throttle",
    }:
        raise ValueError(
            "Model runtime metadata missing: re-export with fixed/max throttle settings"
        )
    if any(
        type(v) not in (int, float) or not math.isfinite(v) for v in values.values()
    ):
        raise ValueError("Throttle settings must be finite numbers")
    if not 0 <= values["fixed_throttle"] <= values["max_throttle"] <= 1:
        raise ValueError("Require 0 <= fixed_throttle <= max_throttle <= 1")
    return dict(values)


def runtime_settings(directory):
    metadata = json.loads((Path(directory) / "metadata.json").read_text())
    return validate_runtime(metadata.get("runtime"))
