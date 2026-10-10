"""Pure configuration helpers shared by the mission launch and terminal selector."""

import json
import os
import re
from pathlib import Path

import yaml

KINDS = ("centerline", "raceline", "customline")


def boolean(value):
    if str(value).lower() not in ("true", "false"):
        raise ValueError(f"Expected true/false: {value}")
    return str(value).lower() == "true"


def sensor_parameters(path, rgb_fps="", infra_fps=""):
    params = yaml.safe_load(Path(path).read_text())["/**/realsense"]["ros__parameters"]
    params = dict(params)
    for stream, value, key, flags in (
        ("RGB", rgb_fps, "rgb_camera.color_profile", ("enable_color",)),
        (
            "Infra",
            infra_fps,
            "depth_module.infra_profile",
            ("enable_infra1", "enable_infra2"),
        ),
    ):
        if value == "":
            continue
        if str(value) not in ("0", "30", "60", "90"):
            raise ValueError(f"{stream} FPS must be 0, 30, 60, or 90")
        width, height, _old = re.split(r"[x,]", params[key])
        if int(value):
            params[key] = f"{width}x{height}x{int(value)}"
        for flag in flags:
            params[flag] = int(value) != 0
    return params


def composition(path, mode, overrides=None):
    cfg = yaml.safe_load(Path(path).read_text())
    allowed = {
        "sensor_container",
        "create_sensor_container",
        "sensor_container_type",
        "enable_evs",
        "enable_foxglove",
        "enable_bridge",
        "modes",
        "e2e_drive_enabled",
    }
    if set(cfg) != allowed or mode not in cfg["modes"]:
        raise ValueError("Invalid mission config or mode")
    for key, value in (overrides or {}).items():
        if key not in allowed - {"modes", "sensor_container_type"}:
            raise ValueError(f"Unknown mission override: {key}")
        if value != "":
            cfg[key] = value
    for key in (
        "create_sensor_container",
        "enable_evs",
        "enable_foxglove",
        "enable_bridge",
        "e2e_drive_enabled",
    ):
        cfg[key] = boolean(cfg[key])
    if not re.fullmatch(
        r"/?[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*", cfg["sensor_container"]
    ):
        raise ValueError("Invalid sensor container name")
    if cfg["create_sensor_container"] and "/" in cfg["sensor_container"].lstrip("/"):
        raise ValueError("Namespaced sensor container must be externally owned")
    if cfg["sensor_container_type"] not in ("multithreaded", "isolated_multithreaded"):
        raise ValueError("Sensor container requires multithread executor")
    selected = cfg.pop("modes")[mode]
    if set(selected) != {"localization", "tracking", "e2e"} or any(
        type(v) is not bool for v in selected.values()
    ):
        raise ValueError("Invalid mode flags")
    if mode == "collect" and any(selected.values()):
        raise ValueError("Collection must not start localization/tracking")
    cfg.update(selected)
    return cfg


def discover(root, kind):
    """Bounded tree search. Never follow symlink dirs or include trash/staging."""
    root = Path(root).expanduser().resolve()
    result = []
    if not root.is_dir():
        return result
    for current, dirs, files in os.walk(root):
        relative = Path(current).relative_to(root)
        dirs[:] = (
            sorted(
                d
                for d in dirs
                if not d.startswith(".")
                and d not in ("record", "ros2_ws", "node_modules")
                and not (Path(current) / d).is_symlink()
            )
            if len(relative.parts) < 6
            else []
        )
        p = Path(current)
        if kind == "hdmap":
            for name in ("map.json", "hd_map.yaml"):
                if name not in files:
                    continue
                try:
                    doc = yaml.safe_load((p / name).read_text())
                    if (
                        doc.get("schema") == "kart.hdmap.v2"
                        and doc.get("snapshot_status") != "pending"
                    ) and hdmap_choices(p / name):
                        result.append(p / name)
                except (ValueError, OSError, AttributeError, yaml.YAMLError):
                    pass
        elif kind == "vslam" and (p / "cuvslam_map").is_dir():
            if any(
                f.is_file() and f.stat().st_size
                for f in (p / "cuvslam_map").glob("*.mdb")
            ):
                result.append(p)
        elif kind == "bundle" and "vgl_profile.json" in files:
            try:
                profile = json.loads((p / "vgl_profile.json").read_text())
                if (
                    profile.get("schema") == "kart.vgl.v1"
                    and profile.get("status") == "complete"
                ):
                    result.append(p)
            except (ValueError, OSError, AttributeError):
                pass
        elif kind == "bag" and "metadata.yaml" in files:
            try:
                info = yaml.safe_load((p / "metadata.yaml").read_text())
                if "rosbag2_bagfile_information" in info:
                    result.append(p)
                    dirs[:] = []
            except (ValueError, TypeError, OSError, yaml.YAMLError):
                pass
        elif kind == "e2e" and "metadata.json" in files and "model.onnx" in files:
            from kart_e2e.contract import model_contract, runtime_settings

            try:
                model_contract(p)
                runtime_settings(p)
                result.append(p)
            except (ValueError, OSError, KeyError, TypeError):
                pass
        elif kind == "models" and (p / "aliked_lightglue").is_dir():
            from .vgl_assets import model_files

            try:
                model_files(p)
                result.append(p)
            except (ValueError, OSError):
                pass
    return sorted(set(result))


def hdmap_choices(path):
    from kart_hdmap.document import load

    try:
        doc = load(path)
    except (KeyError, TypeError, AttributeError) as error:
        raise ValueError(f"Invalid HDMap: {path}: {error}") from error
    return [
        (lane["id"], kind)
        for lane in doc["lanes"]
        for kind in KINDS
        if lane["lines"].get(kind)
    ]
