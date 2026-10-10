#!/usr/bin/env python3
"""Isolated ALIKED export lab; heavy libraries are imported only by explicit stages."""

import argparse
import hashlib
import importlib
import inspect
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import urllib.request

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
ARTIFACTS = REPO / "models/vgl"
sys.path.insert(0, str(REPO / "ros2_ws/src/kart_bringup"))
from kart_bringup.vgl_assets import extractor_shape, inspect_engines, model_files

COMMIT = "96f1f9e7a9932a5da4f0d1aa62017c0fd48141ce"
WEIGHT_SHA = "5be8704840ed662d9d8c561bf7279c222092674e7eb05fd0feab94899e9d82f2"
REFERENCE_SHA = "bd4bd09d2f2dda23fd0bbe98abf47c71fbf93e3a4b5d5e7a99a4930592120193"
SOURCE = ROOT / "vendor/aliked-tensorrt"
REFERENCE = ROOT / "reference/aliked.onnx"
FILES = [
    "LICENSE",
    "nets/aliked.py",
    "nets/blocks.py",
    "nets/padder.py",
    "nets/soft_detect.py",
    "deform_conv2d_onnx_exporter.py",
    "models/aliked-n16.pth",
]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked_hash(path, expected):
    actual = digest(path)
    if actual != expected:
        raise ValueError(f"SHA-256 mismatch: {path}: {actual}")


def save(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")


def positive(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def run_dir(name):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name):
        raise ValueError("run name must contain only letters, digits, _, . or -")
    return ARTIFACTS / name


def manifest(name):
    directory = run_dir(name)
    data = json.loads((directory / "manifest.json").read_text())
    checked_hash(directory / "aliked.onnx", data["onnx_sha256"])
    return directory, data


def stage_reference(args):
    """Stage the exact NVIDIA ONNX for the same build path as a source export."""
    directory = run_dir(args.name)
    if directory.exists():
        raise ValueError(f"Run already exists; choose a new --name: {directory}")
    checked_hash(args.reference, REFERENCE_SHA)
    directory.mkdir(parents=True)
    output = directory / "aliked.onnx"
    shutil.copyfile(args.reference, output)
    checked_hash(output, REFERENCE_SHA)
    save(
        directory / "manifest.json",
        {
            "origin": "NVIDIA distributed ONNX, copied without re-export",
            "reference_path": str(args.reference.resolve()),
            "width": 1920,
            "height": 1200,
            "input_shape": [1, 3, 1200, 1920],
            "onnx_sha256": REFERENCE_SHA,
        },
    )
    print("Verified NVIDIA reference staged:", output)
    print(f"Next: bash scripts/vgl_model.sh build --name {args.name}")


def prepare(args):
    SOURCE.mkdir(parents=True, exist_ok=True)
    lock = SOURCE / "source-lock.json"
    previous = json.loads(lock.read_text()) if lock.exists() else None
    if previous and previous["commit"] != COMMIT:
        raise ValueError("Source commit differs from this workspace")
    hashes = {}
    for relative in FILES:
        destination = SOURCE / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if previous and destination.exists():
            checked_hash(destination, previous["files"][relative])
        else:
            url = f"https://raw.githubusercontent.com/ajuric/aliked-tensorrt/{COMMIT}/{relative}"
            print(f"Download: {relative}", flush=True)
            request = urllib.request.Request(url, headers={"User-Agent": "kart-VGL"})
            with urllib.request.urlopen(request, timeout=60) as response:
                content = response.read()
            if destination.exists() and destination.read_bytes() != content:
                raise ValueError(f"Refusing to replace different source: {destination}")
            destination.write_bytes(content)
        hashes[relative] = digest(destination)
    checked_hash(SOURCE / "models/aliked-n16.pth", WEIGHT_SHA)
    save(lock, {"commit": COMMIT, "files": hashes})
    if not args.source_only:
        checked_hash(args.reference, REFERENCE_SHA)
        REFERENCE.parent.mkdir(parents=True, exist_ok=True)
        if REFERENCE.exists():
            checked_hash(REFERENCE, REFERENCE_SHA)
        else:
            shutil.copy2(args.reference, REFERENCE)
    (ROOT / "inputs").mkdir(exist_ok=True)
    print("Prepared:", SOURCE)


def doctor(args):
    failures = []
    packages = {
        "export": ["numpy", "torch", "torchvision", "onnx", "PIL"],
        "compare": ["numpy", "PIL", "onnxruntime"],
        "build": ["tensorrt"],
        "all": [
            "numpy",
            "torch",
            "torchvision",
            "onnx",
            "PIL",
            "onnxruntime",
            "tensorrt",
        ],
    }[args.stage]
    for name in packages:
        try:
            module = importlib.import_module(name)
            print(name, getattr(module, "__version__", "available"))
            if name == "torch":
                print(
                    "CUDA:",
                    module.version.cuda,
                    "available:",
                    module.cuda.is_available(),
                )
                if not module.cuda.is_available():
                    failures.append(
                        "CUDA-enabled PyTorch is required for the upstream exporter"
                    )
            if name == "onnxruntime":
                print("ORT providers:", module.get_available_providers())
        except (ImportError, OSError, RuntimeError) as error:
            failures.append(f"{name}: {error}")
    print("Source ready:", (SOURCE / "source-lock.json").exists())
    print("Reference ready:", REFERENCE.exists())
    if failures:
        raise ValueError("\n".join(failures))


def source_check():
    lock = json.loads((SOURCE / "source-lock.json").read_text())
    if lock["commit"] != COMMIT:
        raise ValueError("Unexpected source revision")
    for relative in FILES:
        checked_hash(SOURCE / relative, lock["files"][relative])
    checked_hash(SOURCE / "models/aliked-n16.pth", WEIGHT_SHA)


def load_image(path, width, height):
    import numpy as np
    from PIL import Image

    if path is None:
        return (
            __import__("numpy")
            .random.default_rng(0)
            .random((1, 3, height, width), dtype=np.float32)
        )
    with Image.open(path) as image:
        image = image.convert("RGB").resize((width, height), Image.Resampling.BILINEAR)
        return (
            np.asarray(image, dtype=np.float32).transpose(2, 0, 1)[None].copy() / 255.0
        )


def configure_exporter_types(exporter):
    """Bridge the PyTorch 2.11 move without changing pinned vendor sources."""
    if getattr(exporter, "JitScalarType", None) is not None:
        return "vendor"
    module_name = "torch.onnx._internal.torchscript_exporter._type_utils"
    try:
        scalar_type = importlib.import_module(module_name).JitScalarType
    except (ImportError, AttributeError) as error:
        raise ValueError(
            "Cannot find JitScalarType for the ALIKED ONNX exporter"
        ) from error
    for method in ("from_value", "from_dtype", "onnx_type", "dtype"):
        if not callable(getattr(scalar_type, method, None)):
            raise ValueError(f"Unsupported JitScalarType API: missing {method}")
    exporter.JitScalarType = scalar_type
    return module_name


def export(args):
    # Validate the destination before allocating any GPU memory.
    directory = run_dir(args.name or f"{args.width}x{args.height}")
    if directory.exists():
        raise ValueError(f"Run already exists; choose a new --name: {directory}")
    if args.width * args.height < 2048:
        raise ValueError("Input has fewer pixels than TopK=2048")
    source_check()
    import onnx
    import torch

    if not torch.cuda.is_available():
        raise ValueError("This upstream implementation requires CUDA-enabled PyTorch")
    sys.path.insert(0, str(SOURCE))
    from nets.aliked import ALIKED
    import deform_conv2d_onnx_exporter

    exporter_types = configure_exporter_types(deform_conv2d_onnx_exporter)
    print("Exporter scalar types:", exporter_types, flush=True)
    deform_conv2d_onnx_exporter.register_deform_conv2d_onnx_op()
    torch.manual_seed(0)
    model = ALIKED(model_name="aliked-n16", device="cuda", top_k=2048).eval()
    tensor = torch.from_numpy(load_image(args.image, args.width, args.height)).cuda()
    directory.mkdir(parents=True)
    output = directory / "aliked.onnx"
    with torch.inference_mode():
        options = dict(
            opset_version=17,
            input_names=["image"],
            output_names=["keypoints", "descriptors", "scores"],
            export_params=True,
            do_constant_folding=True,
        )
        if "dynamo" in inspect.signature(torch.onnx.export).parameters:
            options["dynamo"] = False
        torch.onnx.export(model, tensor, str(output), **options)
    proto = onnx.load(str(output))
    onnx.checker.check_model(proto)
    shape = [d.dim_value for d in proto.graph.input[0].type.tensor_type.shape.dim]
    if shape != [1, 3, args.height, args.width]:
        raise ValueError(f"Exported input does not match requested size: {shape}")
    outputs = [v.name for v in proto.graph.output]
    if outputs != ["keypoints", "descriptors", "scores"]:
        raise ValueError(f"Unexpected outputs: {outputs}")
    save(
        directory / "manifest.json",
        {
            "source_commit": COMMIT,
            "weight_sha256": WEIGHT_SHA,
            "model": "aliked-n16",
            "top_k": 2048,
            "width": args.width,
            "height": args.height,
            "opset": 17,
            "torch_version": torch.__version__,
            "onnx_version": onnx.__version__,
            "exporter_scalar_types": exporter_types,
            "input_image": str(args.image.resolve())
            if args.image
            else "synthetic_seed_0_export_only",
            "image_sha256": digest(args.image) if args.image else None,
            "onnx_sha256": digest(output),
            "input_shape": shape,
            "onnx_outputs": outputs,
            "preprocessing": "Pillow RGB bilinear resize, float32 /255, NCHW",
        },
    )
    print("ONNX exported and shape checked:", output)
    print(
        "Next: compare at 1920x1200, or build the test engine; runtime compatibility is not yet verified."
    )


def infer_ort(path, image, provider):
    import onnxruntime as ort

    if provider not in ort.get_available_providers():
        raise ValueError(
            f"{provider} unavailable; installed: {ort.get_available_providers()}"
        )
    if provider == "CUDAExecutionProvider" and hasattr(ort, "preload_dlls"):
        ort.preload_dlls()
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    session = ort.InferenceSession(
        str(path), sess_options=options, providers=[provider]
    )
    if provider not in session.get_providers():
        raise ValueError(f"{provider} failed to initialize; inspect ONNX Runtime logs")
    session.disable_fallback()
    inp = session.get_inputs()
    if (
        len(inp) != 1
        or inp[0].name != "image"
        or list(inp[0].shape) != list(image.shape)
    ):
        raise ValueError(f"Unexpected input in {path}")
    result = session.run(["keypoints", "descriptors", "scores"], {"image": image})
    del session
    return result


def compare(args):
    import numpy as np

    directory, data = manifest(args.name)
    if (data["width"], data["height"]) != (1920, 1200):
        raise ValueError("NVIDIA reference comparison requires a 1920x1200 export")
    checked_hash(REFERENCE, REFERENCE_SHA)
    image = load_image(args.image, 1920, 1200)
    # Sequential sessions reduce peak memory. Do not compare keypoints by row order.
    ref = infer_ort(REFERENCE, image, args.provider)
    test = infer_ort(directory / "aliked.onnx", image, args.provider)
    for label, result in [("reference", ref), ("candidate", test)]:
        expected = [(2048, 2), (2048, 128), (2048,)]
        if [tuple(x.shape) for x in result] != expected or not all(
            np.isfinite(x).all() for x in result
        ):
            raise ValueError(f"{label}: unexpected output shape or non-finite values")
    scale = np.array([1919, 1199], dtype=np.float32) / 2
    a, b = (ref[0] + 1) * scale, (test[0] + 1) * scale

    def nearest(x, y):
        indices, distances = [], []
        for start in range(0, len(x), 128):
            squared = ((x[start : start + 128, None] - y[None]) ** 2).sum(axis=2)
            idx = squared.argmin(axis=1)
            indices.extend(idx.tolist())
            distances.extend(np.sqrt(squared[np.arange(len(idx)), idx]).tolist())
        return np.asarray(indices), np.asarray(distances)

    ab, distance = nearest(a, b)
    ba, _ = nearest(b, a)
    matched = (ba[ab] == np.arange(len(a))) & (distance <= args.pixel_tolerance)
    idx = np.flatnonzero(matched)
    ratio = len(idx) / len(a)
    cosines = (ref[1][idx] * test[1][ab[idx]]).sum(axis=1)
    cosine = float(cosines.mean()) if len(idx) else None
    score_error = (
        float(np.abs(ref[2][idx] - test[2][ab[idx]]).mean()) if len(idx) else None
    )
    passed = (
        ratio >= args.min_match_ratio
        and cosine is not None
        and cosine >= args.min_cosine
        and score_error <= args.max_score_mae
    )
    report = dict(
        reference_sha256=REFERENCE_SHA,
        candidate_sha256=data["onnx_sha256"],
        image_sha256=digest(args.image),
        provider=args.provider,
        match_ratio=ratio,
        descriptor_cosine_mean=cosine,
        score_mae=score_error,
        pixel_error_max=float(distance[idx].max()) if len(idx) else None,
        criteria=dict(
            pixel_tolerance=args.pixel_tolerance,
            min_match_ratio=args.min_match_ratio,
            min_cosine=args.min_cosine,
            max_score_mae=args.max_score_mae,
        ),
        passed=passed,
        scope="single-image regression check, not localization validation",
    )
    target = directory / f"comparison-{digest(args.image)[:12]}.json"
    save(target, report)
    print(json.dumps(report, indent=2))
    if not passed:
        raise ValueError(f"Comparison did not meet criteria: {target}")


def engine_info(path, width, height):
    import tensorrt as trt

    logger = trt.Logger(trt.Logger.WARNING)
    trt.init_libnvinfer_plugins(logger, "")
    with trt.Runtime(logger) as runtime:
        engine = runtime.deserialize_cuda_engine(path.read_bytes())
        if engine is None:
            raise ValueError(f"Cannot deserialize {path}")
        tensors = []
        for i in range(engine.num_io_tensors):
            name = engine.get_tensor_name(i)
            tensors.append(
                dict(
                    name=name,
                    shape=list(engine.get_tensor_shape(name)),
                    mode=str(engine.get_tensor_mode(name)),
                )
            )
        actual = list(engine.get_tensor_shape("image"))
        if actual != [1, 3, height, width]:
            raise ValueError(
                f"Engine input size mismatch: {actual}; expected {[1, 3, height, width]}"
            )
        info = dict(
            file=str(path),
            sha256=digest(path),
            tensors=tensors,
            context_memory_bytes=engine.device_memory_size_v2,
            context_memory_gib=engine.device_memory_size_v2 / 1024**3,
            tensorrt_version=trt.__version__,
        )
        del engine
    return info


def inspect_engine(args):
    directory, data = manifest(args.name)
    paths = list(
        (directory / "runtime_models/aliked_lightglue").glob("aliked_*.engine")
    )
    if len(paths) != 1:
        raise ValueError(f"Expected exactly one ALIKED engine; found {len(paths)}")
    info = engine_info(paths[0], data["width"], data["height"])
    save(directory / "engine-inspection.json", info)
    print(json.dumps(info, indent=2))


def build(args):
    directory, data = manifest(args.name)
    target = directory / "runtime_models"
    if target.exists():
        raise ValueError(
            "runtime_models already exists; inspect it or create a new named export"
        )
    prefix = Path(
        subprocess.check_output(
            ["ros2", "pkg", "prefix", "isaac_ros_visual_mapping"], text=True
        ).strip()
    )
    share = Path(
        subprocess.check_output(
            ["ros2", "pkg", "prefix", "--share", "isaac_ros_visual_mapping"], text=True
        ).strip()
    )
    exporters = [
        prefix / "bin/visual_mapping" / name
        for name in ["export_extractor_engine", "export_lightglue_engine"]
    ]
    if not all(p.is_file() for p in exporters):
        raise ValueError("Isaac ROS exporter binaries not found")
    import tensorrt  # Fail before building if the inspection dependency is unavailable.

    source_models = directory / "source_models"
    source_models.mkdir(exist_ok=True)
    for child in (share / "models").iterdir():
        destination = source_models / child.name
        if child.name == "aliked_lightglue":
            destination.mkdir(exist_ok=True)
            for model in child.glob("*.onnx"):
                shutil.copy2(model, destination / model.name)
            shutil.copy2(directory / "aliked.onnx", destination / "aliked.onnx")
        elif not destination.exists():
            destination.symlink_to(child, target_is_directory=child.is_dir())
    if not (source_models / "aliked_lightglue/aliked.onnx").exists():
        raise ValueError("Unexpected installed model layout")
    config = directory / "keypoint_creation_config.pb.txt"
    config.write_text(
        extractor_shape(
            (share / "configs/isaac/keypoint_creation_config.pb.txt").read_text(),
            [1, 3, data["height"], data["width"]],
        )
    )
    target.mkdir()
    commands = [
        [
            str(exporters[0]),
            "--configure_file",
            str(config),
            "--model_dir",
            str(source_models),
            "--output_model_dir",
            str(target),
        ],
        [
            str(exporters[1]),
            "--worker_config_file",
            str(share / "configs/isaac/matching_task_worker_config.pb.txt"),
            "--model_dir",
            str(source_models),
            "--output_model_dir",
            str(target),
        ],
    ]
    save(directory / "build-commands.json", commands)
    # Build and inspect ALIKED first, before spending time on LightGlue.
    subprocess.run(commands[0], check=True)
    inspect_engine(args)
    subprocess.run(commands[1], check=True)
    if not list((target / "aliked_lightglue").glob("lightglue_aliked_*.engine")):
        raise ValueError("LightGlue engine was not generated")
    shutil.copy2(directory / "aliked.onnx", target / "aliked_lightglue/aliked.onnx")
    _, _, engines = model_files(target)
    inspect_engines(engines, [1, 3, data["height"], data["width"]])
    print("VGL model directory (UI model path):", target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser(
        "stage-reference", help="copy exact NVIDIA ONNX into an isolated build run"
    )
    p.add_argument("--name", default="official-1920x1200")
    p.add_argument("--reference", type=Path, default=REFERENCE)
    p.set_defaults(func=stage_reference)
    p = sub.add_parser(
        "prepare", help="download pinned source and copy verified NVIDIA reference"
    )
    p.add_argument("--source-only", action="store_true")
    p.add_argument(
        "--reference",
        type=Path,
        default=Path(
            "/opt/ros/lyrical/share/isaac_ros_visual_mapping/models/aliked_lightglue/aliked.onnx"
        ),
    )
    p.set_defaults(func=prepare)
    p = sub.add_parser(
        "doctor", help="check existing runtime dependencies; install nothing"
    )
    p.add_argument(
        "--stage", choices=["export", "compare", "build", "all"], default="all"
    )
    p.set_defaults(func=doctor)
    p = sub.add_parser("export", help="export FP32 ONNX from pinned n16 weights")
    p.add_argument(
        "--image",
        type=Path,
        help="Real image optional; otherwise deterministic synthetic export input",
    )
    p.add_argument("--width", type=positive, default=424)
    p.add_argument("--height", type=positive, default=240)
    p.add_argument("--name")
    p.set_defaults(func=export)
    p = sub.add_parser(
        "compare", help="compare baseline ONNX with NVIDIA using one real image"
    )
    p.add_argument("--name", default="baseline")
    p.add_argument("--image", required=True, type=Path)
    p.add_argument(
        "--provider",
        choices=["CPUExecutionProvider", "CUDAExecutionProvider"],
        default="CUDAExecutionProvider",
    )
    p.add_argument("--pixel-tolerance", type=float, default=0.5)
    p.add_argument("--min-match-ratio", type=float, default=0.99)
    p.add_argument("--min-cosine", type=float, default=0.999)
    p.add_argument("--max-score-mae", type=float, default=0.001)
    p.set_defaults(func=compare)
    for name, function in [("build", build), ("inspect", inspect_engine)]:
        p = sub.add_parser(name)
        p.add_argument("--name", default="424x240")
        p.set_defaults(func=function)
    args = parser.parse_args()
    try:
        args.func(args)
    except (ValueError, OSError, ImportError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"ERROR: {error}\n")


if __name__ == "__main__":
    main()
