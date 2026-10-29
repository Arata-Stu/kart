"""Resolve localization assets/settings without launching sensors or the vehicle."""

import json
import re
from pathlib import Path

from .vgl_assets import validate_bundle
from .multicam import multicam_value


def vslam_package(backend):
    """Select one implementation; never load both into the same container."""
    packages = {"apt": "isaac_ros_cuvslam", "fixed": "kart_visual_localization"}
    if backend not in packages:
        raise ValueError("vslam_backend must be apt or fixed")
    return packages[backend]


def resolve(
    config_dir, map_dir, model_dir, overrides=None, inspect=None, enable_vgl=True, vslam_backend="apt"
):
    import yaml

    vslam_package(vslam_backend)
    config = Path(config_dir)
    workflow = json.loads((config / "workflow.json").read_text())
    required = {
        "imu",
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
        filename = "fixed_localization" if name == "cuvslam" and vslam_backend == "fixed" else name
        params[name] = yaml.safe_load((config / f"{filename}.yaml").read_text())[
            f"/**/{node}"
        ]["ros__parameters"]
    overrides = overrides or {}
    if set(overrides) - {"use_sim_time", "base_frame", "tracking_mode", "imu_frame", "multicam_mode"}:
        raise ValueError("Unknown localization override")
    for key, value in overrides.items():
        if key == "use_sim_time":
            if str(value).lower() not in ("true", "false"):
                raise ValueError("use_sim_time must be true or false")
            value = str(value).lower() == "true"
        elif key == "multicam_mode":
            value=multicam_value(value)
        elif key == "tracking_mode":
            if str(value) not in ("0","1"):
                raise ValueError("tracking_mode must be 0 (VO) or 1 (VIO)")
            value=int(value)
        elif not isinstance(value,str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_/]*',value) or value.endswith('/') or '//' in value:
            raise ValueError(f"{key} must be a nonempty TF frame without leading slash")
        targets=[params["cuvslam"]] if key in ("tracking_mode","imu_frame","multicam_mode") else params.values()
        for values in targets:
            values[key] = value
    slam, vgl = params["cuvslam"], params["vgl"]
    slam['multicam_mode']=multicam_value(slam['multicam_mode'])
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
    if str(slam["tracking_mode"]) not in ("0","1"):
        raise ValueError("This launch supports stereo VO/VIO, not RGBD")
    slam["tracking_mode"]=int(slam["tracking_mode"])
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
    if slam["tracking_mode"] == 1:
        mappings["cuvslam"].append(("visual_slam/imu", workflow["imu"]))
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
