"""Portable Jetson selection. Do not mistake every ARM notebook for a Jetson."""

import platform
from pathlib import Path


def is_jetson(machine=None, root=Path("/")):
    machine = machine or platform.machine()
    if machine.lower() not in ("aarch64", "arm64"):
        return False
    if (root / "etc/nv_tegra_release").is_file() or (
        root / "run/jtop.sock"
    ).is_socket():
        return True
    for name in ("proc/device-tree/model", "sys/firmware/devicetree/base/model"):
        try:
            if "jetson" in (root / name).read_text().lower():
                return True
        except (OSError, UnicodeError):
            pass
    return False


def should_start(mode, detected):
    if mode not in ("auto", "on", "off"):
        raise ValueError("Monitoring mode must be auto, on or off")
    return mode == "on" or (mode == "auto" and detected)
