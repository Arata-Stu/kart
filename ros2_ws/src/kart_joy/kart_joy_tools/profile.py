"""ROS-independent schema, atomic persistence and input capture."""
import math
import os
from pathlib import Path
import tempfile

import yaml

AXES = ("left_x", "left_y", "right_x", "right_y", "l2", "r2", "dpad_x", "dpad_y")
BUTTONS = ("cross", "circle", "triangle", "square", "l1", "r1", "l2", "r2",
           "create", "options", "ps", "l3", "r3", "dpad_up", "dpad_down", "dpad_left", "dpad_right")


def default_profile():
    axes = {}
    for name, code in zip(AXES, (0, 1, 3, 4, 2, 5, 16, 17)):
        trigger = name in ("l2", "r2")
        axes[name] = dict(code=code, mode="unipolar" if trigger else "bipolar",
                          negative=-1.0, center=-1.0 if trigger else 0.0,
                          positive=1.0, deadzone=0.02 if trigger else 0.05)
    buttons = {}
    for name, code in zip(BUTTONS[:13], (304, 305, 307, 308, 310, 311, 312, 313, 314, 315, 316, 317, 318)):
        buttons[name] = dict(source="button", code=code, direction=1, threshold=0.5)
    for name, code, direction in (("dpad_up", 17, -1), ("dpad_down", 17, 1),
                                  ("dpad_left", 16, -1), ("dpad_right", 16, 1)):
        buttons[name] = dict(source="axis", code=code, direction=direction, threshold=0.5)
    return dict(version=1, device=dict(vendor=1356, product=3302, name="", unique=""),
                axes=axes, buttons=buttons)


def _keys(value, allowed):
    if not isinstance(value, dict) or set(value) != set(allowed):
        raise ValueError(f"Required keys: {', '.join(allowed)}")


def _integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"Expected integer in {low}..{high}: {value!r}")


def _number(value, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"Expected finite number in {low}..{high}: {value!r}")


def validate(profile):
    _keys(profile, ("version", "device", "axes", "buttons"))
    _integer(profile["version"], 1, 1)
    _keys(profile["device"], ("vendor", "product", "name", "unique"))
    for key in ("vendor", "product"):
        _integer(profile["device"][key], 0, 65535)
    for key in ("name", "unique"):
        if not isinstance(profile["device"][key], str) or len(profile["device"][key]) > 256:
            raise ValueError(f"Invalid device {key}")
    _keys(profile["axes"], AXES)
    _keys(profile["buttons"], BUTTONS)
    for name, item in profile["axes"].items():
        _keys(item, ("code", "mode", "negative", "center", "positive", "deadzone"))
        _integer(item["code"], -1, 63)
        if item["mode"] not in ("bipolar", "unipolar"):
            raise ValueError(f"Invalid mode: {name}")
        for key in ("negative", "center", "positive"):
            _number(item[key], -1, 1)
        _number(item["deadzone"], 0, 1)
        positive = item["positive"] - item["center"]
        negative = item["negative"] - item["center"]
        if item["deadzone"] >= 1 or abs(positive) < 1e-6 or (
                item["mode"] == "bipolar" and positive * negative >= -1e-6):
            raise ValueError(f"Invalid axis endpoints/deadzone: {name}")
    for name, item in profile["buttons"].items():
        _keys(item, ("source", "code", "direction", "threshold"))
        if item["source"] not in ("axis", "button"):
            raise ValueError(f"Invalid source: {name}")
        _integer(item["code"], -1, 63 if item["source"] == "axis" else 767)
        _integer(item["direction"], -1, 1)
        _number(item["threshold"], 0, 1)
        if not item["direction"] or item["threshold"] <= 0:
            raise ValueError(f"Invalid direction/threshold: {name}")
    return profile


def load(path):
    path = Path(path)
    if path.stat().st_size > 65536:
        raise ValueError("Profile exceeds 64 KiB")
    return validate(yaml.safe_load(path.read_text()))


def save(path, profile):
    validate(profile)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as stream:
            temporary = stream.name
            yaml.safe_dump(profile, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


def detect(baseline, current, kind):
    """Detect a new press/movement, excluding held buttons and small stick noise."""
    if baseline["generation"] != current["generation"] or not current["connected"]:
        raise ValueError("Controller changed or disconnected; restart capture")
    if kind == "buttons":
        for item in current["buttons"]:
            code = item["code"]
            if current["raw_buttons"][code] and not baseline["raw_buttons"][code]:
                return dict(source="button", code=code, direction=1, threshold=0.5)
    moved = [(abs(current["raw_axes"][item["code"]] - baseline["raw_axes"][item["code"]]),
              item["code"]) for item in current["axes"]]
    if moved:
        distance, code = max(moved)
        if distance >= 0.5:
            if kind == "axes":
                return dict(code=code)
            value = current["raw_axes"][code]
            if abs(value) >= 0.5:
                return dict(source="axis", code=code, direction=1 if value > 0 else -1, threshold=0.5)
    return None
