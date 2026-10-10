"""Collect, drive or E2E composition. Never starts recording or requests AUTO."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from ament_index_python.packages import get_package_share_directory
from kart_bringup.configuration import load
from kart_bringup.localization import resolve
from kart_bringup.mission import composition, hdmap_choices, sensor_parameters
from kart_bringup.vehicle_tf import load_transforms

OVERRIDES = (
    "sensor_container",
    "create_sensor_container",
    "enable_evs",
    "enable_foxglove",
    "enable_bridge",
)


def add_mission(args):
    root = lu.get_path("kart_bringup", "config")
    cfg = composition(
        root / "mission.yaml", args.mode, {k: getattr(args, k) for k in OVERRIDES}
    )
    params = sensor_parameters(
        root / "sensors/realsense.yaml", args.rgb_fps, args.infra_fps
    )
    vehicle = {
        "enable_bridge": str(cfg["enable_bridge"]).lower(),
        "device": args.device,
        "record_dir": args.record_dir,
        "run_name": args.run_name,
    }
    evs_values = {key: getattr(args, key, "") for key in ("evs_backend", "evs_serial", "evs_bias_file")}
    if any(evs_values.values()) and not cfg["enable_evs"]:
        raise ValueError("EVS startup overrides require enable_evs:=true")
    if cfg["enable_evs"]:
        from kart_bringup.evs import pipeline_configuration
        pipeline_configuration(root / "sensors", *evs_values.values())
        get_package_share_directory("openeb_ros2")
        vehicle["bag_config"] = str(root / "recording/bag_manager_openeb.yaml")
        vehicle["bag_shutdown_timeout"] = "26"
    load(
        root / "bringup.yaml", vehicle
    )  # Validate device/config before any node starts.
    get_package_share_directory("realsense2_camera")
    if cfg["localization"]:
        load_transforms(root / "vehicle/transforms.yaml", require_camera=True)
        if not params["enable_infra1"] or not params["enable_infra2"]:
            raise ValueError("Drive localization requires stereo infra (infra_fps > 0)")
        _, _, _, profile = resolve(root / "localization", args.map_dir, args.model_dir)
        width, height, _ = params["depth_module.infra_profile"].split("x")
        if profile["input_shape"][2:] != [int(height), int(width)]:
            raise ValueError("RealSense resolution differs from VGL map/model shape")
    if cfg["tracking"] and (args.lane_id, args.line_type) not in hdmap_choices(
        args.map_file
    ):
        raise ValueError("Choose a generated line from the selected HDMap/lane")
    if cfg["e2e"]:
        from kart_e2e.contract import model_contract, runtime_settings

        if not params["enable_color"]:
            raise ValueError("E2E requires RGB (rgb_fps > 0)")
        model_contract(args.e2e_model_dir)
        runtime_settings(args.e2e_model_dir)
        from kart_e2e.engine_cache import require_engine
        require_engine(args.e2e_model_dir)
    actions = [
        lu.log_info(
            f"Mission: {args.mode}; config: {cfg}; recording/AUTO require Joy operation"
        )
    ]
    if cfg["create_sensor_container"]:
        actions.append(
            lu.component_container(
                cfg["sensor_container"].lstrip("/"),
                container_type=cfg["sensor_container_type"],
            )
        )

    def include(file, values):
        return lu.include(
            "kart_bringup",
            "launch/" + file,
            launch_arguments=values,
            scoped=True,
            forwarding=False,
        )

    actions.append(
        include(
            "modules/sensors/realsense.launch.py",
            {
                "container_name": "/" + cfg["sensor_container"].lstrip("/"),
                "rgb_fps": args.rgb_fps,
                "infra_fps": args.infra_fps,
            },
        )
    )
    if cfg["enable_evs"]:
        actions.append(
            include(
                "modules/sensors/openeb.launch.py",
                evs_values,
            )
        )
    actions.append(include("vehicle.launch.py", vehicle))
    if cfg["localization"]:
        actions.append(
            include(
                "localization.launch.py",
                {
                    "map_dir": args.map_dir,
                    "model_dir": args.model_dir,
                    "vslam_container": "/" + cfg["sensor_container"].lstrip("/"),
                    "create_vslam_container": "false",
                },
            )
        )
    if cfg["tracking"]:
        actions.append(
            include(
                "tracking.launch.py",
                {
                    "map_file": args.map_file,
                    "lane_id": args.lane_id,
                    "line_type": args.line_type,
                },
            )
        )
    if cfg["e2e"]:
        actions.append(
            include(
                "e2e.launch.py",
                {
                    "container_name": "/" + cfg["sensor_container"].lstrip("/"),
                    "create_container": "false",
                    "model_dir": args.e2e_model_dir,
                    "drive_enabled": str(cfg["e2e_drive_enabled"]).lower(),
                },
            )
        )
    if cfg["enable_foxglove"]:
        actions.append(include("foxglove.launch.py", {}))
    return actions


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg("mode", "collect", choices=["collect", "drive", "e2e"], cli=True)
    for key in (
        *OVERRIDES,
        "device",
        "record_dir",
        "run_name",
        "e2e_model_dir",
        "evs_backend",
        "evs_serial",
        "evs_bias_file",
        "rgb_fps",
        "infra_fps",
        "map_dir",
        "model_dir",
        "map_file",
        "lane_id",
        "line_type",
    ):
        args.add_arg(
            key,
            "",
            description="Explicit runtime selection; empty preserves module YAML",
            cli=True,
        )
    args.add_opaque_function(add_mission)
    return lut.LaunchDescription(args.get_launch_actions())
