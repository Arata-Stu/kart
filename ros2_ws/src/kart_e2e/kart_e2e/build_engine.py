"""Notebook-local TensorRT build and synthetic-input benchmark. No ROS outputs."""

import argparse
import hashlib
import json
import platform
import shutil
import subprocess
import time
from pathlib import Path

from .contract import model_contract, input_contract
from .engine_cache import validate_engine


def build(model, output, precision="fp32"):
    if precision not in ("fp16", "fp32"):
        raise ValueError("TensorRT precision must be fp16 or fp32")
    if platform.machine().lower() not in ("x86_64", "amd64"):
        raise ValueError("Notebook engine build is restricted to x86_64")
    contract, mode = model_contract(model)
    executable = shutil.which("trtexec") or "/usr/src/tensorrt/bin/trtexec"
    if not Path(executable).is_file():
        raise ValueError(
            "trtexecがありません。Notebookのkart Docker環境で実行してください"
        )
    output.mkdir(parents=True, exist_ok=False)
    engine = output / "model.plan"
    timings = output / "timings.json"
    command = [
        executable,
        f"--onnx={contract['model_file_path']}",
        f"--saveEngine={engine}",
        "--warmUp=500",
        "--duration=3",
        f"--exportTimes={timings}",
    ]
    command.append("--fp16" if precision == "fp16" else "--noTF32")
    print("TensorRT build + synthetic benchmark: " + " ".join(command), flush=True)
    started = time.perf_counter()
    subprocess.run(command, check=True)
    elapsed = time.perf_counter() - started
    if not engine.is_file() or not engine.stat().st_size:
        raise ValueError("TensorRT did not produce a nonempty engine")
    _, input_name, input_shape = input_contract(model)
    validate_engine(engine, mode, input_name, input_shape)
    samples = json.loads(timings.read_text())
    if not isinstance(samples, list) or not samples:
        raise ValueError("TensorRT benchmark timings missing")
    import numpy as np

    stats = {}
    for key in ("computeMs", "latencyMs", "h2dMs", "d2hMs"):
        values = [r[key] for r in samples if key in r]
        if values and np.isfinite(values).all():
            stats[key] = dict(
                mean=float(np.mean(values)),
                median=float(np.median(values)),
                p95=float(np.percentile(values, 95)),
            )
    gpu = (
        subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=name,uuid,driver_version",
                "--format=csv,noheader",
            ],
            capture_output=True,
            text=True,
        )
        if shutil.which("nvidia-smi")
        else None
    )
    source = Path(contract["model_file_path"])
    report = dict(
        schema=1,
        mode=mode,
        precision=precision,
        io_dtype="fp32",
        architecture=platform.machine(),
        scope="notebook_only_synthetic_inputs_not_rosbag_accuracy_or_jetson_performance",
        onnx_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        onnx_size_bytes=source.stat().st_size,
        engine_size_bytes=engine.stat().st_size,
        total_build_and_benchmark_s=elapsed,
        command=command,
        gpu=gpu.stdout.strip() if gpu and gpu.returncode == 0 else "unavailable",
        timing_samples=len(samples),
        timing_ms=stats,
    )
    (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    print(f"Notebook engine ready: {engine.stat().st_size} bytes", flush=True)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--precision", choices=["fp16", "fp32"], default="fp32")
    args = p.parse_args()
    build(args.model, args.output, args.precision)


if __name__ == "__main__":
    main()
