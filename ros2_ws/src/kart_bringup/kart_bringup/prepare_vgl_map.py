"""Create a new paired runtime bundle from existing official poses/images."""

import argparse
import json
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

from .vgl_assets import (
    CONFIG_FILES,
    digest,
    extractor_shape,
    inspect_engines,
    map_files,
    model_files,
    require_file,
    validate_bundle,
)


def source_frames(source):
    result = source / "mapping_result.json"
    if result.is_file():
        metadata = json.loads(result.read_text())
        official = (source / metadata["official_output"]).resolve(strict=True)
        if not official.is_relative_to(source):
            raise ValueError("Official output must stay inside its map folder")
    else:
        official = source
    frames = official / "map_frames/rectified"
    require_file(frames / "frames_meta.json")
    map_files(source)
    return frames


def build(
    source,
    model_dir,
    output,
    settings,
    config_source,
    binary_dir,
    inspect=inspect_engines,
    run=subprocess.run,
):
    source = Path(source).expanduser().resolve(strict=True)
    output = Path(output).expanduser().resolve()
    if output.exists() or output.is_relative_to(source):
        raise ValueError("Choose a new output directory outside the original map")
    frames = source_frames(source)
    models, onnx, engines = model_files(model_dir)
    width, height = settings["width"], settings["height"]
    if (
        any(type(v) is not int or not 16 <= v <= 8192 for v in (width, height))
        or width * height < 2048
    ):
        raise ValueError("Invalid ALIKED image size")
    if settings["feature_type"] != "aliked":
        raise ValueError("Only ALIKED is supported")
    shape = [1, 3, height, width]
    inspect(engines, shape)
    help_result = run(
        ["ros2", "run", "isaac_ros_visual_mapping", "create_cuvgl_map.py", "--help"],
        check=True,
        capture_output=True,
        text=True,
    )
    required = {
        "--map_folder",
        "--raw_image_folder",
        "--config_folder_path",
        "--model_dir",
        "--binary_folder_path",
        "--feature_type",
        "--extract_feature",
        "--build_bow_index",
    }
    if required - set(re.findall(r"--[A-Za-z_][A-Za-z_0-9-]*", help_result.stdout)):
        raise ValueError(
            "Installed create_cuvgl_map.py lacks required options; inspect its --help"
        )
    config_source = Path(config_source)
    for name in CONFIG_FILES:
        require_file(config_source / name)
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".vgl-", dir=output.parent))
    try:
        shutil.copytree(source / "cuvslam_map", stage / "cuvslam_map")
        config = stage / "vgl_runtime_config"
        shutil.copytree(config_source, config)
        extractor = config / CONFIG_FILES[0]
        extractor.write_text(extractor_shape(extractor.read_text(), shape))
        # Keep the installed full algorithm config; only disable debug image saving.
        localizer = config / "localizer_config.pb.txt"
        text = localizer.read_text()
        text, count = re.subn(
            r"\bsave_debug_images\s*:\s*(?:true|false)",
            "save_debug_images: false",
            text,
        )
        if count != 1:
            raise ValueError(
                "Unexpected localizer config; cannot verify debug-image saving is disabled"
            )
        localizer.write_text(text)
        vgl = stage / "cuvgl_map"
        vgl.mkdir()
        command = [
            "ros2",
            "run",
            "isaac_ros_visual_mapping",
            "create_cuvgl_map.py",
            f"--map_folder={vgl}",
            f"--raw_image_folder={frames}",
            f"--config_folder_path={config}",
            f"--model_dir={models}",
            f"--binary_folder_path={binary_dir}",
            "--feature_type=aliked",
            "--extract_feature",
            "--build_bow_index",
        ]
        print("VGL preparation: " + " ".join(command), flush=True)
        run(command, check=True)
        profile = {
            "schema": "kart.vgl.v1",
            "status": "complete",
            "input_shape": shape,
            "onnx_sha256": digest(onnx),
            "cuvslam_sha256": map_files(stage),
            "config_sha256": {name: digest(config / name) for name in CONFIG_FILES},
            "source_map": str(source),
            "source_frames_sha256": digest(frames / "frames_meta.json"),
            "mapping_engine_sha256": {key: digest(p) for key, p in engines.items()},
            "feature_type": "aliked",
            "command": command,
        }
        (stage / "vgl_profile.json").write_text(json.dumps(profile, indent=2) + "\n")
        validate_bundle(stage, models)
        if output.exists():
            raise ValueError("Output appeared while preparing the map")
        os.rename(stage, output)
    finally:
        if stage.exists():
            shutil.rmtree(stage)
    return output


def main():
    from ament_index_python.packages import (
        get_package_prefix,
        get_package_share_directory,
    )

    own = Path(get_package_share_directory("kart_bringup")) / "config/localization"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-map", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    args = parser.parse_args()
    settings = json.loads((own / "preparation.json").read_text())
    for name in ("width", "height"):
        if getattr(args, name) is not None:
            settings[name] = getattr(args, name)
    share = Path(get_package_share_directory("isaac_ros_visual_mapping"))
    prefix = Path(get_package_prefix("isaac_ros_visual_mapping"))
    try:
        result = build(
            args.source_map,
            args.model_dir,
            args.output,
            settings,
            share / "configs/isaac",
            prefix / "bin/visual_mapping",
        )
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        parser.exit(1, str(exc) + "\n")
    print(f"Prepared localization bundle: {result}", flush=True)
