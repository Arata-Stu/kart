"""Container ownership and routing, independent of ROS launch imports."""

import json
import re
from pathlib import Path


def plan(path, overrides):
    config = json.loads(Path(path).read_text())
    if set(config) != {"vslam", "vgl"}:
        raise ValueError("Expected vslam and vgl container settings")
    allowed = {f"{kind}_container" for kind in config} | {
        f"create_{kind}_container" for kind in config
    }
    if set(overrides) - allowed:
        raise ValueError("Unknown container override")
    targets, owners = {}, {}
    for kind, settings in config.items():
        if set(settings) != {"name", "create", "type"}:
            raise ValueError("Invalid container settings")
        settings = settings.copy()
        for key, field in (
            (f"{kind}_container", "name"),
            (f"create_{kind}_container", "create"),
        ):
            if overrides.get(key, "") != "":
                settings[field] = overrides[key]
        if str(settings["create"]).lower() not in {"true", "false"}:
            raise ValueError("Container create must be true or false")
        settings["create"] = str(settings["create"]).lower() == "true"
        name = settings["name"]
        if not isinstance(name, str) or not re.fullmatch(
            r"/?[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*", name
        ):
            raise ValueError("Invalid container name")
        if settings["type"] not in {"multithreaded", "isolated_multithreaded"}:
            raise ValueError("Localization requires a multithreaded executor")
        name = name.lstrip("/")
        if settings["create"] and "/" in name:
            raise ValueError(
                "Owned containers must use a root node name; use create=false for external namespaced containers"
            )
        settings["name"] = name
        target = "/" + name
        if target in owners and settings != owners[target]:
            raise ValueError(
                "Shared container must have the same ownership and executor settings"
            )
        owners[target] = settings
        targets[kind] = target
    return targets, [s for s in owners.values() if s["create"]]
