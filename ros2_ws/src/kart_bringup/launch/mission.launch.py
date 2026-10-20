"""Collect, drive or E2E composition. Never starts recording or requests AUTO."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from ament_index_python.packages import get_package_share_directory
from kart_bringup.configuration import load
from kart_bringup.localization import resolve
from kart_bringup.mission import composition, hdmap_choices, sensor_parameters, camera_config, recording_config
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
        camera_config(root, args.mode), args.rgb_fps, args.infra_fps,
        getattr(args, "rgb_profile", ""), getattr(args, "infra_profile", "")
    )
    section_routing = None
    if cfg['e2e']:
        from kart_e2e.contract import routing_settings, input_contract
        section_routing = routing_settings(args.e2e_model_dir)
        if section_routing:
            from kart_e2e.sections import validate_maps
            if input_contract(args.e2e_model_dir)[0] != 'rgb':
                raise ValueError('Image-stamped section routing currently requires RGB E2E')
            validate_maps(section_routing, args.map_file, args.map_dir)
            cfg['localization'] = True
    vehicle = {
        "enable_bridge": str(cfg["enable_bridge"]).lower(),
        "device": args.device,
        "record_dir": args.record_dir,
        "run_name": args.run_name,
        "auto_fixed_throttle": getattr(args, "auto_fixed_throttle", ""),
    }
    if args.mode == "environment" or cfg["enable_evs"]:
        vehicle["bag_config"] = str(recording_config(root, args.mode, cfg["enable_evs"]))
    if cfg['e2e']:
        # Collection's Joy throttle must never take ownership of E2E steering.
        vehicle['auto_fixed_throttle'] = '-1'
    evs_values = {key: getattr(args, key, "") for key in ("evs_backend", "evs_serial", "evs_bias_file", "evs_tensor_config")}
    if any(evs_values.values()) and not cfg["enable_evs"]:
        raise ValueError("EVS startup overrides require enable_evs:=true")
    if cfg["enable_evs"]:
        from kart_bringup.evs import pipeline_configuration
        pipeline_configuration(root / "sensors", *evs_values.values())
        get_package_share_directory("openeb_ros2")
        vehicle["bag_shutdown_timeout"] = "26"
    _, vehicle_modules = load(
        root / "bringup.yaml", vehicle
    )  # Validate device/config before any node starts.
    from kart_bringup.recovery import configuration, validate_inputs
    recovery = getattr(args, 'recovery', '')
    if recovery and args.mode not in ('drive', 'e2e'):
        raise ValueError('recovery is only supported for drive/e2e')
    if cfg['tracking'] or (cfg['e2e'] and cfg['e2e_drive_enabled']):
        recovery_enabled, recovery_params = configuration(root / 'recovery', recovery)
        if recovery_enabled:
            validate_inputs(recovery_params, params, cfg['enable_evs'],
                            vehicle_modules['joy_manager']['effective']['auto_fixed_throttle'])
    if cfg["localization"]:
        load_transforms(root / "vehicle/transforms.yaml", require_camera=True)
        if not params["enable_infra1"] or not params["enable_infra2"]:
            raise ValueError("Drive localization requires stereo infra (infra_fps > 0)")
        overrides={'multicam_mode':args.multicam_mode} if args.multicam_mode!="" else {}
        if section_routing: overrides['base_frame'] = section_routing['base_frame']
        _, _, _, profile = resolve(root / "localization", args.map_dir, args.model_dir,overrides, enable_vgl=not bool(section_routing))
        width, height, _ = params["depth_module.infra_profile"].split("x")
        if not section_routing and profile["input_shape"][2:] != [int(height), int(width)]:
            raise ValueError("RealSense resolution differs from VGL map/model shape")
    if cfg["tracking"] and (args.lane_id, args.line_type) not in hdmap_choices(
        args.map_file
    ):
        raise ValueError("Choose a generated line from the selected HDMap/lane")
    if cfg["e2e"]:
        from kart_e2e.contract import model_contract, runtime_settings, input_contract, validate_runtime

        modality, _, _ = input_contract(args.e2e_model_dir)
        if modality == 'evs':
            from kart_bringup.e2e import validate_evs_tensor
            if not cfg['enable_evs']:
                raise ValueError('EVS policy requires enable_evs:=true')
            validate_evs_tensor(args.evs_tensor_config, args.e2e_model_dir)
        elif not params["enable_color"]:
            raise ValueError("RGB E2E requires RGB (rgb_fps > 0)")
        model_contract(args.e2e_model_dir)
        runtime = runtime_settings(args.e2e_model_dir)
        fixed_override = getattr(args, 'e2e_fixed_throttle', '')
        if fixed_override != '':
            validate_runtime(dict(runtime, fixed_throttle=float(fixed_override)))
        from kart_e2e.engine_cache import require_engine
        require_engine(args.e2e_model_dir)
    realsense_enabled = not (cfg['e2e'] and modality == 'evs'
                             and not any(params[k] for k in ('enable_color', 'enable_infra1', 'enable_infra2')))
    if realsense_enabled:
        get_package_share_directory('realsense2_camera')
    actions = [
        lu.log_info(
            f"Mission: {args.mode}; config: {cfg}; recording/AUTO require Joy operation"
        )
    ]
    if cfg["create_sensor_container"] and realsense_enabled:
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

    if realsense_enabled:
        actions.append(
            include(
                "modules/sensors/realsense.launch.py",
                {
                    "container_name": "/" + cfg["sensor_container"].lstrip("/"),
                    "camera_config": str(camera_config(root, args.mode)),
                    "rgb_profile": getattr(args, "rgb_profile", ""),
                    "infra_profile": getattr(args, "infra_profile", ""),
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
                    "enable_vgl": "false" if section_routing else "true",
                    "base_frame": section_routing["base_frame"] if section_routing else "",
                    "map_dir": args.map_dir,
                    "model_dir": args.model_dir,
                    "multicam_mode": args.multicam_mode,
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
                    "tuning_enabled": getattr(args, "tuning_enabled", ""),
                    "tuning_map_dir": getattr(args, "tuning_map_dir", ""),
                    "tuning_tracking_frame": getattr(args, "tuning_tracking_frame", ""),
                    "map_file": args.map_file,
                    "lane_id": args.lane_id,
                    "line_type": args.line_type,
                    "recovery": recovery,
                },
            )
        )
    if cfg["e2e"]:
        actions.append(
            include(
                'evs_e2e.launch.py' if modality == 'evs' else 'e2e.launch.py',
                {
                    **({'evs_tensor_config': args.evs_tensor_config} if modality == 'evs' else {
                        'container_name': '/' + cfg['sensor_container'].lstrip('/'),
                        'create_container': 'false',
                    }),
                    'model_dir': args.e2e_model_dir,
                    'recovery': recovery,
                    'drive_enabled': str(cfg['e2e_drive_enabled']).lower(),
                    'use_fixed_throttle': 'true',
                    'fixed_throttle': getattr(args, 'e2e_fixed_throttle', ''),
                },
            )
        )
    if cfg["enable_foxglove"]:
        actions.append(include("foxglove.launch.py", {}))
    return actions


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg("mode", "collect", choices=["collect", "environment", "drive", "e2e"], cli=True)
    for key in (
        *OVERRIDES,
        "device",
        "record_dir",
        "run_name",
        "auto_fixed_throttle",
        "recovery",
        "e2e_model_dir",
        "e2e_fixed_throttle",
        "evs_backend",
        "evs_serial",
        "evs_bias_file",
        "evs_tensor_config",
        "rgb_profile",
        "infra_profile",
        "rgb_fps",
        "infra_fps",
        "map_dir",
        "model_dir",
        "tuning_enabled",
        "tuning_map_dir",
        "tuning_tracking_frame",
        "map_file",
        "lane_id",
        "line_type",
        "multicam_mode",
    ):
        args.add_arg(
            key,
            "",
            description="Explicit runtime selection; empty preserves module YAML",
            cli=True,
        )
    args.add_opaque_function(add_mission)
    return lut.LaunchDescription(args.get_launch_actions())
