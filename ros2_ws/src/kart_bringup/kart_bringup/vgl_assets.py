"""Explicit ALIKED assets and paired cuVSLAM/cuVGL bundles; no model fallback."""

import hashlib
import json
import re
from pathlib import Path

CONFIG_FILES = (
    "keypoint_creation_config.pb.txt",
    "localizer_config.pb.txt",
    "matching_config.pb.txt",
    "query_map_config.pb.txt",
)


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def require_file(path):
    path = Path(path)
    if not path.is_file() or not path.stat().st_size:
        raise ValueError(f"Required nonempty file missing: {path}")
    return path


def block(text, name):
    masked = re.sub(
        r'"(?:\\.|[^"\\])*"|#[^\n]*|//[^\n]*', lambda m: " " * len(m.group()), text
    )
    matches = list(re.finditer(r"\b" + re.escape(name) + r"\s*\{", masked))
    if len(matches) != 1:
        raise ValueError(f"Expected one {name} block")
    start, depth = matches[0].end(), 1
    for end in range(start, len(masked)):
        depth += (masked[end] == "{") - (masked[end] == "}")
        if depth == 0:
            return start, end
    raise ValueError(f"Unclosed {name}")


def extractor_shape(text, shape=None):
    start, end = block(text, "aliked_detector")
    a, b = block(text[start:end], "tensorrt_config")
    c, d = block(text[start + a : start + b], "input_dimension")
    lo, hi = start + a + c, start + a + d
    values = {
        key: [int(v) for v in re.findall(r"\b" + key + r"\s*:\s*(\d+)", text[lo:hi])]
        for key in ("min", "opt", "max")
    }
    if any(len(v) != 4 or v[:2] != [1, 3] for v in values.values()):
        raise ValueError("Unsupported ALIKED NCHW config")
    if shape is None:
        if values["min"] != values["opt"] or values["max"] != values["opt"]:
            raise ValueError("ALIKED profile must have a fixed input shape")
        return values["opt"]
    replacement = "\n" + "".join(f"      {key}: {v}\n" for key in values for v in shape)
    return text[:lo] + replacement + "    " + text[hi:]


def model_files(model_dir):
    root = Path(model_dir).expanduser().resolve(strict=True)
    engines = {}
    for kind, pattern in (
        ("aliked", "aliked_*.engine"),
        ("lightglue", "lightglue_aliked_*.engine"),
    ):
        candidates = list((root / "aliked_lightglue").glob(pattern))
        if len(candidates) != 1:
            raise ValueError(f"Expected exactly one {kind} engine in {root}")
        engines[kind] = require_file(candidates[0])
    onnx = root / "aliked_lightglue/aliked.onnx"
    if not onnx.is_file() and root.name == "runtime_models":
        onnx = root.parent / "aliked.onnx"
    return root, require_file(onnx), engines


def inspect_engines(engines, shape):
    # Must run on the actual GPU. Loading tests compatibility; it is not an inference test.
    import tensorrt as trt

    logger = trt.Logger(trt.Logger.WARNING)
    trt.init_libnvinfer_plugins(logger, "")
    with trt.Runtime(logger) as runtime:
        for kind, path in engines.items():
            engine = runtime.deserialize_cuda_engine(path.read_bytes())
            if engine is None:
                raise ValueError(
                    f"Cannot load {kind} engine on this GPU; rebuild it here"
                )
            try:
                if kind == "aliked" and list(engine.get_tensor_shape("image")) != shape:
                    raise ValueError(f"ALIKED engine input differs from {shape}")
            finally:
                del engine


def map_files(root):
    files = sorted((root / "cuvslam_map").glob("*.mdb"))
    if not files:
        raise ValueError("cuVSLAM .mdb map missing")
    for file in files:
        require_file(file)
    return {str(p.relative_to(root)): digest(p) for p in files}


def validate_bundle(map_dir, model_dir, inspect=None):
    root = Path(map_dir).expanduser().resolve(strict=True)
    profile = json.loads(require_file(root / "vgl_profile.json").read_text())
    if profile.get("schema") != "kart.vgl.v1" or profile.get("status") != "complete":
        raise ValueError("Run prepare_vgl_map to create a complete kart.vgl.v1 bundle")
    shape = profile["input_shape"]
    if (
        len(shape) != 4
        or shape[:2] != [1, 3]
        or any(type(v) is not int or v <= 0 for v in shape)
    ):
        raise ValueError("Invalid model shape")
    models, onnx, engines = model_files(model_dir)
    if digest(onnx) != profile["onnx_sha256"]:
        raise ValueError("ALIKED ONNX does not match the model used to build this map")
    config = root / "vgl_runtime_config"
    for name in CONFIG_FILES:
        file = require_file(config / name)
        if digest(file) != profile["config_sha256"][name]:
            raise ValueError(f"VGL config changed since map preparation: {name}")
    if extractor_shape((config / CONFIG_FILES[0]).read_text()) != shape:
        raise ValueError("Map config and model shape disagree")
    if map_files(root) != profile["cuvslam_sha256"]:
        raise ValueError("VSLAM map differs from the paired VGL map")
    for name in ("keyframes/frames_meta.json", "bow_index.pb"):
        require_file(root / "cuvgl_map" / name)
    if not any(
        p.is_file() and p.stat().st_size
        for p in (root / "cuvgl_map/vocabulary").rglob("*")
    ):
        raise ValueError("VGL vocabulary missing")
    if inspect:
        inspect(engines, shape)
    return root, models, profile
