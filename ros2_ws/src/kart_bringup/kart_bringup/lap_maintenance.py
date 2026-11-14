"""Validate optional lap maintenance before mission creates any processes."""
import json
from pathlib import Path

import yaml


def maintenance_parameters(root, composition, map_file, lane_id, line_type, *, parking=False):
    from kart_hdmap.document import load

    if not composition.get("localization") or not composition.get("tracking"):
        raise ValueError("Lap maintenance requires saved-map localization and line tracking")
    if parking:
        raise ValueError("Lap maintenance cannot be combined with parking")
    document = load(map_file)
    lane = next((item for item in document["lanes"] if item["id"] == lane_id), None)
    if lane is None or line_type not in lane["lines"] or not lane["closed"]:
        raise ValueError("Lap maintenance requires a selected closed reference line")
    root = Path(root)
    path = root / "localization/lap_manager.yaml"
    parameters = yaml.safe_load(path.read_text())["/**/lap_manager"]["ros__parameters"]
    parameters = dict(parameters)
    parameters.update(vgl_enabled=composition["lap_vgl"],
                      cache_clear_enabled=composition["lap_cache_clear"])
    workflow = json.loads((root / "localization/workflow.json").read_text())
    for parameter, route in (("vgl_trigger_service", "request_hint"),
                             ("diagnostics_topic", "vslam_diagnostics"),
                             ("pose_hint_topic", "pose_hint")):
        if parameters[parameter] != workflow[route]:
            raise ValueError(f"Lap {parameter} differs from localization workflow")
    if parameters["map_frame"] != document["frame_id"]:
        raise ValueError("Lap map_frame differs from the HDMap frame")
    if composition["lap_cache_clear"]:
        cache = yaml.safe_load((root / "system/jetson_cache.yaml").read_text())[
            "/**/jetson_cache_service"]["ros__parameters"]
        if cache["service_name"] != parameters["cache_clear_service"]:
            raise ValueError("Lap cache service differs from the cache adapter service")
        if not Path(cache["socket_path"]).is_socket():
            raise ValueError("Jetson cache helper socket is unavailable. On the Jetson host run "
                             "sudo bash ros2_ws/src/kart_jetson_cache/host/install.sh, then recreate "
                             "the container with scripts/dev.sh. Keep lap_cache_clear=false until configured.")
    return parameters
