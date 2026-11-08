#!/usr/bin/env python3
"""Build and validate YOLOv8s inside the target kart container."""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import tempfile

from kart_e2e.engine_cache import digest, fingerprint
from prepare_racing_onnx import prepare


def check_bindings(path):
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.WARNING)
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(path.read_bytes())
    if engine is None or engine.num_io_tensors != 2:
        raise ValueError("Invalid TensorRT engine")
    for name, mode, shape in (("images", trt.TensorIOMode.INPUT, (1, 3, 480, 640)),
                              ("output0", trt.TensorIOMode.OUTPUT, (1, 5, 6300))):
        if (engine.get_tensor_mode(name) != mode or tuple(engine.get_tensor_shape(name)) != shape
                or engine.get_tensor_dtype(name) != trt.float32):
            raise ValueError(f"Invalid engine binding: {name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1] / "weights/racing/yolov8s"
    source = root / "opponent.onnx"
    if not source.is_file():
        parser.error(f"Place your FP16 Detect ONNX at {source}")
    hardware = fingerprint()
    major = int(hardware["tensorrt"].split(".")[0])
    target = root / "engines" / (hardware["gpu_uuid"] + "-" + hardware["tensorrt"])
    target.mkdir(parents=True, exist_ok=True)
    with (target / ".build.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        identity = {"hardware": hardware, "source_sha256": digest(source),
                    "prepare_sha256": digest(Path(__file__).with_name("prepare_racing_onnx.py")),
                    "builder_sha256": digest(Path(__file__))}
        plan = target / "opponent.plan"
        manifest = target / "build.json"
        if not args.force and plan.is_file() and manifest.is_file():
            if json.loads(manifest.read_text()) == dict(identity, engine_sha256=digest(plan)):
                check_bindings(plan)
                print(plan)
                return
        with tempfile.TemporaryDirectory(dir=target) as temp:
            tmp = Path(temp)
            prepared = tmp / "ros.onnx"
            prepare(source, prepared)
            cmd = ["/usr/src/tensorrt/bin/trtexec", f"--onnx={prepared}",
                   f"--saveEngine={tmp / plan.name}", "--warmUp=500", "--duration=3"]
            if major == 10:
                cmd.append("--stronglyTyped")
            elif major < 10:
                raise ValueError("TensorRT 10+ required")
            with (target / "last-build.log").open("w") as log:
                result = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
            if result.returncode:
                raise RuntimeError(f"trtexec failed: {target / 'last-build.log'}")
            built = tmp / plan.name
            check_bindings(built)
            if digest(source) != identity["source_sha256"]:
                raise RuntimeError("ONNX changed during build")
            (tmp / manifest.name).write_text(json.dumps(dict(identity, engine_sha256=digest(built)), indent=2))
            built.replace(plan)
            (tmp / manifest.name).replace(manifest)
        print(plan)


if __name__ == "__main__":
    main()
