#!/usr/bin/env python3
"""Build and validate YOLOv8s inside the target kart container."""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import tempfile

from kart_e2e.engine_cache import digest, fingerprint
from kart_bringup.racing_models import get_profile
from prepare_racing_onnx import prepare


def check_bindings(path, num_classes=80):
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.WARNING)
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(path.read_bytes())
    if engine is None or engine.num_io_tensors != 2:
        raise ValueError("Invalid TensorRT engine")
    for name, mode, shape in (("images", trt.TensorIOMode.INPUT, (1, 3, 480, 640)),
                              ("output0", trt.TensorIOMode.OUTPUT, (1, 4 + num_classes, 6300))):
        if (engine.get_tensor_mode(name) != mode or tuple(engine.get_tensor_shape(name)) != shape
                or engine.get_tensor_dtype(name) != trt.float32):
            raise ValueError(f"Invalid engine binding: {name}")


def build(root, profile, force=False):
    root = Path(root)
    source = root / profile["source_name"]
    if not source.is_file():
        raise ValueError(f"Place your fixed-shape Detect ONNX at {source}")
    hardware = fingerprint()
    major = int(hardware["tensorrt"].split(".")[0])
    if major < 10:
        raise ValueError("TensorRT 10+ required")
    target = root / "engines" / profile["name"] / (hardware["gpu_uuid"] + "-" + hardware["tensorrt"])
    target.mkdir(parents=True, exist_ok=True)
    with (target / ".build.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        identity = {"model_profile": profile["name"], "num_classes": profile["num_classes"],
                    "hardware": hardware, "source_sha256": digest(source),
                    "prepare_sha256": digest(Path(__file__).with_name("prepare_racing_onnx.py")),
                    "builder_sha256": digest(Path(__file__))}
        plan = target / profile["engine_name"]
        manifest = target / "build.json"
        if not force and plan.is_file() and manifest.is_file():
            try:
                cached = json.loads(manifest.read_text())
            except (OSError, ValueError):
                cached = None
            if cached == dict(identity, engine_sha256=digest(plan)):
                check_bindings(plan, profile["num_classes"])
                return plan
        with tempfile.TemporaryDirectory(dir=target) as temp:
            tmp = Path(temp)
            prepared = tmp / "ros.onnx"
            prepare(source, prepared, profile["num_classes"])
            cmd = ["/usr/src/tensorrt/bin/trtexec", f"--onnx={prepared}",
                   f"--saveEngine={tmp / plan.name}", "--warmUp=500", "--duration=3"]
            if major == 10:
                cmd.append("--stronglyTyped")
            # Preserve ONNX graph precision on typed TensorRT 10/11 builds.
            # FP32 exports are supported without claiming FP16 execution.
            with (target / "last-build.log").open("w") as log:
                result = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
            if result.returncode:
                raise RuntimeError(f"trtexec failed: {target / 'last-build.log'}")
            built = tmp / plan.name
            check_bindings(built, profile["num_classes"])
            if digest(source) != identity["source_sha256"]:
                raise RuntimeError("ONNX changed during build")
            (tmp / manifest.name).write_text(json.dumps(dict(identity, engine_sha256=digest(built)), indent=2))
            built.replace(plan)
            (tmp / manifest.name).replace(manifest)
        return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=("coco", "opponent", "racing"), default="coco",
                        help="Official COCO80 (default), racing2class, or legacy opponent1class")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--model-root", type=Path,
                        default=Path(__file__).resolve().parents[1] / "weights/racing/yolov8s",
                        help="Directory containing the profile's ONNX; engines are cached below this directory")
    args = parser.parse_args()
    root = args.model_root.expanduser().resolve()
    print(build(root, get_profile(args.model), args.force))


if __name__ == "__main__":
    main()
