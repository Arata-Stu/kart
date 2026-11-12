"""Shared recording-profile defaults, separate from ROS node parameters."""
import math
from pathlib import Path

import yaml


RECORD_PROFILES = ("", "standard", "vslam-map", "vehicle-model")
VEHICLE_KEYS = {"auto_fixed_throttle", "auto_fixed_throttle_r2_gate"}


def validate_profile_name(profile, mode):
    if profile not in RECORD_PROFILES:
        raise ValueError(f"Unknown record profile: {profile}")
    if profile in ("vslam-map", "vehicle-model") and mode != "collect":
        raise ValueError(f"{profile} profileはcollect専用です")


def vehicle_model_defaults(root):
    """Read the profile's explicit opt-in defaults, never change module YAML."""
    path = Path(root) / "recording/vehicle_model.yaml"
    values = yaml.safe_load(path.read_text())
    if not isinstance(values, dict) or set(values) != VEHICLE_KEYS:
        raise ValueError(f"{path}: expected only {sorted(VEHICLE_KEYS)}")
    throttle = values["auto_fixed_throttle"]
    gate = values["auto_fixed_throttle_r2_gate"]
    if (type(throttle) not in (int, float) or not math.isfinite(throttle)
            or not 0 <= throttle <= 1):
        raise ValueError(f"{path}: auto_fixed_throttle must be finite and in 0..1")
    if type(gate) is not bool or not gate:
        raise ValueError(f"{path}: auto_fixed_throttle_r2_gate must be true")
    return {"auto_fixed_throttle": float(throttle), "auto_fixed_throttle_r2_gate": gate}


def profile_vehicle_overrides(root, profile, mode, overrides):
    """Fill empty launch-string selections for the explicit vehicle-model profile."""
    validate_profile_name(profile, mode)
    values = dict(overrides)
    if profile != "vehicle-model":
        return values
    defaults = vehicle_model_defaults(root)
    throttle = values.get("auto_fixed_throttle", "")
    if throttle == "":
        values["auto_fixed_throttle"] = str(defaults["auto_fixed_throttle"])
    else:
        if not isinstance(throttle, str):
            raise ValueError("vehicle-model: auto_fixed_throttle must be a launch string")
        try:
            number = float(throttle)
        except ValueError as exc:
            raise ValueError("vehicle-model: auto_fixed_throttle must be finite and in 0..1") from exc
        if not math.isfinite(number) or not 0 <= number <= 1:
            raise ValueError("vehicle-model: auto_fixed_throttle must be finite and in 0..1")
    gate = values.get("auto_fixed_throttle_r2_gate", "")
    if gate == "":
        values["auto_fixed_throttle_r2_gate"] = "true"
    elif gate != "true":
        raise ValueError("vehicle-model requires auto_fixed_throttle_r2_gate=true")
    return values
