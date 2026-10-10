"""Verify a locally built engine before allowing ROS TensorRT cache reuse."""

import ctypes
import hashlib
import json
import os
import platform
from pathlib import Path

from .contract import model_contract


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fingerprint():
    import tensorrt as trt

    cuda = ctypes.CDLL("libcuda.so.1")

    def call(name, *args):
        status = getattr(cuda, name)(*args)
        if status:
            raise RuntimeError(f"{name} failed: {status}")

    call("cuInit", 0)
    device, driver = ctypes.c_int(), ctypes.c_int()
    call("cuDeviceGet", ctypes.byref(device), 0)
    uuid = ctypes.create_string_buffer(16)
    name = ctypes.create_string_buffer(256)
    call("cuDeviceGetUuid", uuid, device)
    call("cuDeviceGetName", name, 256, device)
    call("cuDriverGetVersion", ctypes.byref(driver))
    return dict(
        architecture=platform.machine(),
        tensorrt=trt.__version__,
        driver=driver.value,
        gpu_uuid=uuid.raw.hex(),
        gpu_name=name.value.decode(),
        cuda_visible_devices=os.environ.get("CUDA_VISIBLE_DEVICES", ""),
    )


def validate_engine(path, mode):
    import tensorrt as trt

    logger = trt.Logger(trt.Logger.WARNING)
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(Path(path).read_bytes())
    if engine is None:
        raise ValueError("TensorRT engine could not be loaded on this GPU")
    expected = {
        "image": (trt.TensorIOMode.INPUT, (1, 3, 120, 212)),
        "control": (trt.TensorIOMode.OUTPUT, (1, 1 if mode == "steer_only" else 2)),
    }
    if engine.num_io_tensors != 2:
        raise ValueError("Unexpected TensorRT bindings")
    for name, (io, shape) in expected.items():
        if (
            engine.get_tensor_mode(name) != io
            or tuple(engine.get_tensor_shape(name)) != shape
            or engine.get_tensor_dtype(name) != trt.float32
        ):
            raise ValueError(f"Incompatible TensorRT binding: {name}")


def reusable(model):
    paths, mode = model_contract(model)
    engine = Path(paths["engine_file_path"])
    try:
        report = json.loads(engine.with_suffix(".json").read_text())
        if report.get("schema") != 1 or report["hardware"] != fingerprint():
            return False
        if report["onnx_sha256"] != digest(paths["model_file_path"]) or report[
            "engine_sha256"
        ] != digest(engine):
            return False
        validate_engine(engine, mode)
        return True
    except (OSError, ValueError, KeyError, RuntimeError, ImportError):
        return False


def require_engine(model):
    """Fail before launch actions are returned; building is an explicit operation."""
    if not reusable(model):
        raise ValueError(
            "警告: TensorRT engineが未buildまたは不適合です。"
            "bash /workspaces/scripts/e2e_trt.shで事前buildしてから起動してください"
        )
