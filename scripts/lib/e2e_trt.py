#!/usr/bin/env python3
"""Select a deployed model and build a reusable TensorRT engine locally."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

from bringup import ROOT, choose
from kart_e2e.contract import model_contract, runtime_settings
from kart_e2e.engine_cache import digest, fingerprint, reusable, validate_engine


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", nargs="?", type=Path)
    parser.add_argument("--models-root", type=Path, default=ROOT / "models")
    precision = parser.add_mutually_exclusive_group()
    precision.add_argument("--fp16", action="store_true")
    precision.add_argument("--fp32", action="store_true")
    parser.add_argument(
        "--force", action="store_true", help="Rebuild a verified existing engine"
    )
    args = parser.parse_args()
    model = args.model
    if model is None:
        candidates = sorted(
            p.parent
            for p in args.models_root.rglob("metadata.json")
            if (p.parent / "model.onnx").is_file()
            and not any(
                part.startswith(".") for part in p.relative_to(args.models_root).parts
            )
        )
        model = choose(
            "TensorRTを作るモデル",
            candidates,
            label=lambda p: str(p.relative_to(args.models_root)),
        )
    if model.name == "model.onnx":
        model = model.parent
    paths, mode = model_contract(model)
    runtime_settings(model)
    engine = Path(paths["engine_file_path"])
    precision = "fp16" if args.fp16 else "fp32"
    hardware = fingerprint()
    if args.fp16 and int(hardware["tensorrt"].split(".")[0]) >= 11:
        raise ValueError(
            "TensorRT 11以降は--fp16フラグ非対応です。既定のFP32で実行してください"
        )
    executable = shutil.which("trtexec") or "/usr/src/tensorrt/bin/trtexec"
    if not Path(executable).is_file():
        raise ValueError("trtexecがありません。kart Docker内で実行してください")
    with engine.with_suffix(".lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        report_path = engine.with_suffix(".json")
        if (
            not args.force
            and reusable(model)
            and json.loads(report_path.read_text()).get("precision") == precision
        ):
            print(f"検証済みengineを再利用: {engine}\n再構築する場合は --force")
            return
        log_path = engine.with_suffix(".build.log")
        with tempfile.TemporaryDirectory(
            prefix=".trt-build-", dir=engine.parent
        ) as tmp:
            built = Path(tmp) / "model.plan"
            timings = Path(tmp) / "timings.json"
            command = [
                executable,
                f"--onnx={paths['model_file_path']}",
                f"--saveEngine={built}",
                "--warmUp=500",
                "--duration=3",
                f"--exportTimes={timings}",
            ]
            if args.fp16:
                command.append("--fp16")
            elif int(hardware["tensorrt"].split(".")[0]) < 11:
                command.append("--noTF32")
            print("実行: " + " ".join(command), flush=True)
            started = time.monotonic()
            with log_path.open("w") as log:
                with subprocess.Popen(
                    command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
                ) as process:
                    try:
                        for line in process.stdout:
                            print(line, end="", flush=True)
                            log.write(line)
                            log.flush()
                        code = process.wait()
                    except BaseException:
                        process.terminate()
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                        raise
            if code:
                raise RuntimeError(f"TensorRT build失敗（{code}）。ログ: {log_path}")
            validate_engine(built, mode)
            timing_rows = json.loads(timings.read_text())
            import numpy as np

            stats = {}
            for key in ("computeMs", "latencyMs", "h2dMs", "d2hMs"):
                values = [r[key] for r in timing_rows if key in r]
                if values and np.isfinite(values).all():
                    stats[key] = dict(
                        mean=float(np.mean(values)),
                        median=float(np.median(values)),
                        p95=float(np.percentile(values, 95)),
                    )
            report = dict(
                schema=1,
                hardware=hardware,
                precision=precision,
                onnx_sha256=digest(paths["model_file_path"]),
                engine_sha256=digest(built),
                engine_size_bytes=built.stat().st_size,
                total_seconds=time.monotonic() - started,
                timing_ms=stats,
                scope="synthetic_input_benchmark_not_driving_accuracy",
                command=command,
            )
            manifest = Path(tmp) / "report.json"
            manifest.write_text(json.dumps(report, indent=2, allow_nan=False))
            os.replace(built, engine)
            os.replace(timings, engine.with_suffix(".timings.json"))
            os.replace(manifest, report_path)
            print(
                f"保存: {engine}\nサイズ: {report['engine_size_bytes'] / 1048576:.1f} MiB\n"
                + json.dumps(stats, indent=2)
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (ValueError, RuntimeError, OSError) as error:
        raise SystemExit(str(error))
