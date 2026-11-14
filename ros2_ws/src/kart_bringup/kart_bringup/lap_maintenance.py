"""Validate lap maintenance before live or offline launch creates processes."""
import json
from pathlib import Path

import yaml


def reference_parameters(root, map_file, lane_id, line_type):
    """Common saved-map, closed-reference and localization-route validation."""
    from kart_hdmap.document import load

    document = load(map_file)
    lane = next((item for item in document["lanes"] if item["id"] == lane_id), None)
    if lane is None or line_type not in lane["lines"] or not lane["closed"]:
        raise ValueError("Lap maintenance requires a selected closed reference line")
    root = Path(root)
    path = root / "localization/lap_manager.yaml"
    parameters = yaml.safe_load(path.read_text())["/**/lap_manager"]["ros__parameters"]
    parameters = dict(parameters)
    workflow = json.loads((root / "localization/workflow.json").read_text())
    for parameter, route in (("vgl_trigger_service", "request_hint"),
                             ("diagnostics_topic", "vslam_diagnostics"),
                             ("pose_hint_topic", "pose_hint")):
        if parameters[parameter] != workflow[route]:
            raise ValueError(f"Lap {parameter} differs from localization workflow")
    if parameters["map_frame"] != document["frame_id"]:
        raise ValueError("Lap map_frame differs from the HDMap frame")
    return parameters


def maintenance_parameters(root, composition, map_file, lane_id, line_type, *, parking=False):
    if not composition.get("localization") or not composition.get("tracking"):
        raise ValueError("Lap maintenance requires saved-map localization and line tracking")
    if parking:
        raise ValueError("Lap maintenance cannot be combined with parking")
    parameters = reference_parameters(root, map_file, lane_id, line_type)
    parameters.update(vgl_enabled=composition["lap_vgl"],
                      cache_clear_enabled=composition["lap_cache_clear"])
    if composition["lap_cache_clear"]:
        cache = yaml.safe_load((Path(root) / "system/jetson_cache.yaml").read_text())[
            "/**/jetson_cache_service"]["ros__parameters"]
        if cache["service_name"] != parameters["cache_clear_service"]:
            raise ValueError("Lap cache service differs from the cache adapter service")
        if not Path(cache["socket_path"]).is_socket():
            raise ValueError("Jetson cache helper socket is unavailable. On the Jetson host run "
                             "sudo bash ros2_ws/src/kart_jetson_cache/host/install.sh, then recreate "
                             "the container with scripts/dev.sh. Keep lap_cache_clear=false until configured.")
    return parameters


def evaluation_lap_parameters(root, map_file, lane_id, line_type, *, enabled,
                              enable_vgl, base_frame, odom_frame, map_frame):
    """Offline lap detection needs localization and a line, without control nodes."""
    from .mission import boolean

    if not boolean(enabled):
        return None
    if not boolean(enable_vgl):
        raise ValueError("Offline lap VGL requires enable_vgl=true")
    parameters = reference_parameters(root, map_file, lane_id, line_type)
    frames = (base_frame, odom_frame, map_frame)
    if any(not isinstance(frame, str) or not frame or frame.startswith('/') or
           any(part in ('', '.', '..') for part in frame.split('/')) or
           any(char.isspace() for char in frame) for frame in frames) or len(set(frames)) != 3:
        raise ValueError("Offline lap VGL requires distinct valid localization frames")
    if map_frame != parameters['map_frame']:
        raise ValueError("Lap map_frame differs from offline localization map_frame")
    parameters.update(vgl_enabled=True, cache_clear_enabled=False, use_sim_time=True,
                      base_frame=base_frame, odom_frame=odom_frame)
    return parameters
