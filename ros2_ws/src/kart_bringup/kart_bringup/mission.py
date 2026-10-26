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


# USB3 D455, S/N 239222300906, FW 5.16.0.1.
# Source: user-supplied rs-enumerate-devices -o -c (2026-10-23).
# Candidates are a subset of reported Color RGB8 / Infrared 1+2 Y8 profiles.
# Enumeration confirms individual profiles, not simultaneous stream stability.
INFRA_PROFILES = (
    "424x240x30", "424x240x60", "424x240x90",
    "480x270x30", "480x270x60", "480x270x90",
    "640x360x30", "640x360x60", "640x360x90",
    "640x480x30", "640x480x60", "848x480x30", "848x480x60",
    "848x480x90", "1280x720x15", "1280x720x30", "1280x800x30",
)
RGB_PROFILES = tuple(p for p in INFRA_PROFILES if p != "848x480x90")
CAMERA_PROFILES = {"RGB": RGB_PROFILES, "Infra": INFRA_PROFILES}


def validate_camera_profile(stream, profile):
    # Full enumerated support for existing YAML and legacy FPS overrides.
    rates = {
        "RGB": {"424x240": (5, 15, 30, 60, 90), "480x270": (5, 15, 30, 60, 90),
                "640x360": (5, 15, 30, 60, 90), "640x480": (5, 15, 30, 60),
                "848x480": (5, 15, 30, 60), "1280x720": (5, 10, 15, 30),
                "1280x800": (5, 10, 15, 30)},
        "Infra": {"424x240": (5, 15, 30, 60, 90), "480x270": (5, 15, 30, 60, 90),
                  "640x360": (5, 15, 30, 60, 90), "640x480": (5, 15, 30, 60, 90),
                  "848x480": (5, 15, 30, 60, 90), "848x100": (100,),
                  "1280x720": (5, 15, 30), "1280x800": (15, 30)},
    }
    parts = re.split(r"[x,]", str(profile))
    if len(parts) != 3 or not all(p.isdigit() for p in parts) or int(parts[2]) not in rates[stream].get("x".join(parts[:2]), ()):
        raise ValueError(f"{stream}: unsupported D455 profile: {profile}")


def camera_config(root, mode):
    """Select the full static sensor config, shared by CLI and launch."""
    return Path(root) / "sensors" / (
        "realsense_environment.yaml" if mode == "environment" else "realsense.yaml"
    )


def recording_config(root, mode, evs=False):
    suffix = "_environment" if mode == "environment" else ""
    suffix += "_openeb" if evs else ""
    return Path(root) / "recording" / f"bag_manager{suffix}.yaml"


def sensor_parameters(path, rgb_fps="", infra_fps="", rgb_profile="", infra_profile="", imu_profile=""):
    # ArgumentContainer evaluates numeric launch arguments ("0" becomes int 0).
    # Normalize before testing emptiness so OFF never falls back to YAML.
    rgb_fps, infra_fps, rgb_profile, infra_profile = map(
        str, (rgb_fps, infra_fps, rgb_profile, infra_profile)
    )
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
    for stream, value, fps, key, flags in (
        ("RGB", rgb_profile, rgb_fps, "rgb_camera.color_profile", ("enable_color",)),
        ("Infra", infra_profile, infra_fps, "depth_module.infra_profile", ("enable_infra1", "enable_infra2")),
    ):
        if not value:
            continue
        if str(fps) != "":
            raise ValueError(f"{stream}: profile and FPS cannot both be specified")
        if value != "0" and value not in CAMERA_PROFILES[stream]:
            raise ValueError(f"{stream}: unsupported profile selection: {value}")
        if value != "0":
            params[key] = value
        for flag in flags:
            params[flag] = value != "0"
    for stream, key, enabled in (
        ("RGB", "rgb_camera.color_profile", params["enable_color"]),
        ("Infra", "depth_module.infra_profile", params["enable_infra1"] or params["enable_infra2"]),
    ):
        if enabled:
            validate_camera_profile(stream, params[key])
    if imu_profile:
        if imu_profile not in ("low", "high", "off"):
            raise ValueError("IMU profile must be low, high, or off")
        params["enable_accel"] = params["enable_gyro"] = imu_profile != "off"
        if imu_profile != "off":
            params["accel_fps"], params["gyro_fps"] = (100, 200) if imu_profile == "low" else (200, 400)
            params["unite_imu_method"] = 2
    # Depth and Infra share the D455 stereo sensor. Keep one capture profile.
    if params["enable_depth"] and (params["enable_infra1"] or params["enable_infra2"]):
        params["depth_module.depth_profile"] = params["depth_module.infra_profile"]
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
    if mode in ("collect", "environment") and any(selected.values()):
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


def validate_localization_imu(parameters, sensors):
    """Reject VIO without a unified IMU before starting any nodes."""
    if parameters["cuvslam"].get("tracking_mode", 0) == 1 and not (
        sensors["enable_accel"] and sensors["enable_gyro"] and sensors["unite_imu_method"] == 2
    ):
        raise ValueError("VIO requires IMU: choose low or high (imu_profile)")
