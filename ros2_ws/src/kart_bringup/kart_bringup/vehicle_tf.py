"""Validate vehicle mounts without inventing missing calibration values."""

import math
from pathlib import Path

import yaml


def load_transforms(path, require_camera=False):
    path = Path(path)
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or set(data) != {"transforms"}:
        raise ValueError(f"{path}: expected transforms mapping")
    entries = data["transforms"]
    frames = {
        "rear_axle": "rear_axle",
        "camera_mount": "camera_link",
        "evs_mount": "event_camera",
    }
    if not isinstance(entries, dict) or set(entries) != set(frames):
        raise ValueError(f"{path}: expected rear_axle, camera_mount, evs_mount")
    ready, missing = {}, []
    for name, child in frames.items():
        entry = entries[name]
        if not isinstance(entry, dict) or set(entry) != {
            "parent",
            "child",
            "xyz",
            "rpy",
        }:
            raise ValueError(f"{path}: invalid transform {name}")
        if entry["parent"] != "base_link" or entry["child"] != child:
            raise ValueError(f"{path}: {name} must connect base_link -> {child}")
        if entry["xyz"] is None or entry["rpy"] is None:
            missing.append(name)
            continue
        for field in ("xyz", "rpy"):
            values = entry[field]
            if (
                not isinstance(values, list)
                or len(values) != 3
                or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in values
                )
            ):
                raise ValueError(
                    f"{path}: {name}.{field} requires three finite numbers"
                )
        if name == "rear_axle" and any(entry["xyz"] + entry["rpy"]):
            raise ValueError("base_link and rear_axle share the rear axle midpoint")
        ready[name] = entry
    if "rear_axle" in missing:
        raise ValueError("rear_axle definition must not be null")
    if require_camera and "camera_mount" in missing:
        raise ValueError(
            f"車体取付TF base_link -> camera_link が未設定です。{path} の "
            "camera_mount.xyz（m）とrpy（rad）を設定してください。"
        )
    return ready, missing


def publisher_arguments(entry):
    return [
        "--frame-id",
        entry["parent"],
        "--child-frame-id",
        entry["child"],
        *[
            value
            for key, number in zip(
                ("x", "y", "z", "roll", "pitch", "yaw"), entry["xyz"] + entry["rpy"]
            )
            for value in ("--" + key, str(number))
        ],
    ]
