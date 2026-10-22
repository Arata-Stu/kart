"""Resolve snapshot capture and optional VGL bootstrap from explicit settings."""

from pathlib import Path

import yaml

from .localization import resolve


def resolve_capture(config, job, inspect=None):
    config = Path(config)
    workflow = job["workflow"]
    overrides = dict(workflow.get("overrides", {}))
    allowed = {"base_frame", "camera_optical_frames", "imu_frame", "tracking_mode"}
    if set(overrides) - allowed:
        raise ValueError("Unknown VSLAM override")
    slam = yaml.safe_load((config / "mapping/capture_cuvslam.yaml").read_text())[
        "/**/visual_slam"
    ]["ros__parameters"]
    slam.update(overrides, load_map_folder_path=job["map_dir"])
    vgl = job["capture"].get("vgl")
    if vgl:
        params, remaps, topics, profile = resolve(
            config / "localization", Path(job["map_dir"]).parent, vgl["model_dir"],
            {"base_frame": slam["base_frame"], "use_sim_time": True,
             "tracking_mode": slam["tracking_mode"], "imu_frame": slam["imu_frame"]},
            inspect,
        )
        params["cuvslam"] = slam
        params["vgl"]["camera_optical_frames"] = slam["camera_optical_frames"]
        # The capture collector uses /diagnostics. Keep VGL diagnostics separate.
        replacements = {topics[k]: workflow[k] for k in
                        ("left_image", "right_image", "left_info", "right_info")}
        if "imu" in workflow:
            replacements[topics["imu"]] = workflow["imu"]
        replacements[topics["vslam_diagnostics"]] = "/diagnostics"
        remaps = {kind: [(a, replacements.get(b, b)) for a, b in pairs]
                  for kind, pairs in remaps.items()}
        return params, remaps, profile["input_shape"]
    # Compatibility for previously saved capture jobs; the UI always requires VGL.
    slam["enable_request_hint"] = False
    remaps = [("visual_slam/" + target, workflow[key]) for target, key in (
        ("image_0", "left_image"), ("image_1", "right_image"),
        ("camera_info_0", "left_info"), ("camera_info_1", "right_info"))]
    if slam["tracking_mode"] == 1:
        remaps.append(("visual_slam/imu", workflow["imu"]))
    return {"cuvslam": slam}, {"cuvslam": remaps}, None
