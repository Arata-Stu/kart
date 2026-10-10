"""Offline ONNX evaluation on labeled MANUAL frames from a rosbag; no ROS outputs."""

import argparse
import csv
import hashlib
import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
from PIL import Image

from .contract import model_contract, runtime_settings


def metrics(predicted, labels):
    error = np.asarray(predicted, dtype=float) - np.asarray(labels, dtype=float)
    if not error.size or not np.isfinite(error).all():
        raise ValueError("Empty or non-finite evaluation")
    return {
        "mae": float(np.abs(error).mean()),
        "rmse": float(np.sqrt((error**2).mean())),
        "p95_abs": float(np.percentile(np.abs(error), 95)),
    }


def evaluate(model_dir, dataset, output):
    import onnxruntime as ort
    from .model import preprocess

    contract, mode = model_contract(model_dir)
    runtime = runtime_settings(model_dir)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    session = ort.InferenceSession(
        contract["model_file_path"],
        sess_options=options,
        providers=["CPUExecutionProvider"],
    )
    warmup_runs = 10
    rows = []
    first = None
    with (dataset / "samples.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            image = (dataset / row["image"]).resolve(strict=True)
            if not image.is_relative_to(dataset.resolve()):
                raise ValueError("Image outside evaluation dataset")
            with Image.open(image) as rgb:
                # Training adds patch padding; ONNX already embeds that padding.
                tensor = preprocess(rgb)[:, 4:-4, 6:-6].unsqueeze(0).numpy()
            if not rows:
                for _ in range(warmup_runs):
                    session.run(["control"], {"image": tensor})
            started = time.perf_counter()
            prediction = session.run(["control"], {"image": tensor})[0]
            elapsed = (time.perf_counter() - started) * 1000
            if (
                prediction.shape != (1, 1 if mode == "steer_only" else 2)
                or not np.isfinite(prediction).all()
            ):
                raise ValueError("Invalid ONNX prediction")
            steer = float(prediction[0, 0])
            throttle = (
                runtime["fixed_throttle"]
                if mode == "steer_only"
                else float(prediction[0, 1])
            )
            if not -1 <= steer <= 1 or not 0 <= throttle <= 1:
                raise ValueError("Prediction outside ROS decoder control range")
            stamp = int(row["stamp_ns"])
            if first is None:
                first = stamp
            rows.append(
                dict(
                    stamp_ns=stamp,
                    time_s=(stamp - first) / 1e9,
                    steering_label=float(row["steering"]),
                    steering_raw=steer,
                    steering_command=float(np.clip(steer, -1, 1)),
                    throttle_label=float(row["throttle"]),
                    throttle_raw=throttle,
                    throttle_command=float(
                        np.clip(throttle, 0, runtime["max_throttle"])
                    ),
                    inference_ms=elapsed,
                )
            )
            if len(rows) % 100 == 0:
                print(f"Evaluated {len(rows)} frames", flush=True)
    if not rows:
        raise ValueError("No usable labeled MANUAL frames")
    summary = dict(
        schema=1,
        mode=mode,
        runtime=runtime,
        samples=len(rows),
        provider="CPUExecutionProvider",
        onnx_size_bytes=Path(contract["model_file_path"]).stat().st_size,
        warmup_runs=warmup_runs,
        timing_scope="session.run_only_excludes_preprocessing_and_warmup",
        inference_ms={
            "mean": float(np.mean([r["inference_ms"] for r in rows])),
            "median": float(np.median([r["inference_ms"] for r in rows])),
            "p95": float(np.percentile([r["inference_ms"] for r in rows], 95)),
            "min": float(min(r["inference_ms"] for r in rows)),
            "max": float(max(r["inference_ms"] for r in rows)),
        },
        scope="open_loop_labeled_manual_frames_not_driving_or_tensorrt_validation",
        onnx_sha256=hashlib.sha256(
            Path(contract["model_file_path"]).read_bytes()
        ).hexdigest(),
        extraction=json.loads((dataset / "metadata.json").read_text()),
        metrics={
            key: metrics([r[key] for r in rows], [r[label] for r in rows])
            for key, label in (
                ("steering_raw", "steering_label"),
                ("steering_command", "steering_label"),
                ("throttle_raw", "throttle_label"),
                ("throttle_command", "throttle_label"),
            )
        },
        inference_ms_median=float(np.median([r["inference_ms"] for r in rows])),
        preview=[
            rows[i]
            for i in np.unique(
                np.linspace(0, len(rows) - 1, min(500, len(rows)), dtype=int)
            )
        ],
    )
    output.mkdir(parents=True, exist_ok=False)
    with (output / "predictions.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output / "report.json").write_text(json.dumps(summary, indent=2, allow_nan=False))
    print(
        f"Evaluation complete: {len(rows)} frames; steering MAE={summary['metrics']['steering_command']['mae']:.4f}",
        flush=True,
    )
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag", type=Path, required=True)
    p.add_argument("--model", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--image-topic", default="/realsense/color/image_raw")
    p.add_argument("--command-topic", default="/teleop/control_cmd")
    p.add_argument("--mode-topic", default="/operation_mode/state")
    p.add_argument("--clock", choices=["bag", "header"], default="bag")
    p.add_argument("--max-skew-ms", type=float, default=100)
    a = p.parse_args()
    model_contract(a.model)
    runtime_settings(a.model)
    with tempfile.TemporaryDirectory(prefix=".eval-data-", dir=a.output.parent) as tmp:
        dataset = Path(tmp) / "dataset"
        args = [
            sys.executable,
            "-m",
            "kart_e2e.preprocess_bag",
            str(a.bag),
            str(dataset),
        ]
        for key in (
            "image_topic",
            "command_topic",
            "mode_topic",
            "clock",
            "max_skew_ms",
        ):
            args.extend(["--" + key.replace("_", "-"), str(getattr(a, key))])
        subprocess.run(args, check=True)
        evaluate(a.model, dataset, a.output)


if __name__ == "__main__":
    main()
