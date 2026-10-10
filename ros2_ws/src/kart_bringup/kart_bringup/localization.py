"""Resolve localization assets/settings without launching sensors or the vehicle."""

import json
from pathlib import Path

from .vgl_assets import validate_bundle


def resolve(
    config_dir, map_dir, model_dir, overrides=None, inspect=None, enable_vgl=True
):
    import yaml

    config = Path(config_dir)
    workflow = json.loads((config / "workflow.json").read_text())
    required = {
        "left_image",
        "right_image",
        "left_info",
        "right_info",
        "pose_hint",
        "request_hint",
        "vslam_diagnostics",
        "vgl_diagnostics",
    }
    if set(workflow) != required or any(
        not isinstance(v, str) or not v.strip() for v in workflow.values()
    ):
        raise ValueError("Invalid localization workflow keys or values")
    if len(set(workflow.values())) != len(workflow):
        raise ValueError("Localization topics must have distinct names")
    params = {}
    for name, node in (
        ("cuvslam", "visual_slam"),
        ("vgl", "visual_global_localization"),
    ):
        params[name] = yaml.safe_load((config / f"{name}.yaml").read_text())[
            f"/**/{node}"
        ]["ros__parameters"]
    overrides = overrides or {}
    if set(overrides) - {"use_sim_time", "base_frame"}:
        raise ValueError("Unknown localization override")
    for key, value in overrides.items():
        if key == "use_sim_time":
            if str(value).lower() not in ("true", "false"):
                raise ValueError("use_sim_time must be true or false")
            value = str(value).lower() == "true"
        elif not isinstance(value, str) or not value.strip() or value.startswith("/"):
            raise ValueError(
                "base_frame must be a nonempty TF frame without leading slash"
            )
        for values in params.values():
            values[key] = value
    slam, vgl = params["cuvslam"], params["vgl"]
    for key in (
        "base_frame",
        "map_frame",
        "odom_frame",
        "camera_optical_frames",
        "use_sim_time",
    ):
        if slam[key] != vgl[key]:
            raise ValueError(f"VSLAM/VGL disagree on {key}")
    if vgl["publish_map_to_base_tf"] or vgl["publish_map_to_odom_tf"]:
        raise ValueError("VSLAM alone must publish dynamic TF")
    if not slam["rectified_images"] or vgl["enable_rectify_images"]:
        raise ValueError("This workflow requires pre-rectified stereo images")
    if (
        slam["num_cameras"] != 2
        or vgl["num_cameras"] != 2
        or vgl["stereo_localizer_cam_ids"] != "0,1"
    ):
        raise ValueError("This workflow supports one stereo pair")
    if slam["tracking_mode"] != 0:
        raise ValueError("This launch currently supports stereo VO, not IMU/RGBD")
    if slam["save_map_folder_path"]:
        raise ValueError("Runtime localization must not overwrite the reference map")
    if enable_vgl:
        root, models, profile = validate_bundle(map_dir, model_dir, inspect)
        slam["load_map_folder_path"] = str(root / "cuvslam_map")
        vgl.update(
            map_dir=str(root / "cuvgl_map"),
            model_dir=str(models),
            config_dir=str(root / "vgl_runtime_config"),
        )
    else:
        root = Path(map_dir).expanduser().resolve(strict=True)
        if (root / "cuvslam_map").is_dir():
            root = root / "cuvslam_map"
        if not root.is_dir() or not any(
            p.is_file() and p.stat().st_size for p in root.glob("*.mdb")
        ):
            raise ValueError("VSLAM地図が空です")
        slam["load_map_folder_path"] = str(root)
        slam["enable_request_hint"] = False
        slam["localize_on_startup"] = True
        profile = {"input_shape": None}
    mappings = {}
    for kind, prefix in (("cuvslam", "visual_slam"), ("vgl", "visual_localization")):
        mappings[kind] = [
            (f"{prefix}/{topic}_{i}", workflow[key])
            for topic, i, key in (
                ("image", 0, "left_image"),
                ("image", 1, "right_image"),
                ("camera_info", 0, "left_info"),
                ("camera_info", 1, "right_info"),
            )
        ]
    mappings["cuvslam"] += [
        ("visual_slam/initial_pose", workflow["pose_hint"]),
        ("visual_slam/trigger_hint", workflow["request_hint"]),
        ("/diagnostics", workflow["vslam_diagnostics"]),
    ]
    mappings["vgl"] += [
        ("visual_localization/pose", workflow["pose_hint"]),
        ("visual_localization/trigger_localization", workflow["request_hint"]),
        ("/diagnostics", workflow["vgl_diagnostics"]),
    ]
    return params, mappings, workflow, profile
