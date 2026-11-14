"""Collect, drive or E2E composition. Never starts recording or requests AUTO."""

import json
import isaac_ros_launch_utils as lu
from kart_bringup.launch_arguments import LaunchArguments
import isaac_ros_launch_utils.all_types as lut
from ament_index_python.packages import get_package_share_directory
from kart_bringup.configuration import load, collection_r2_gate
from kart_bringup.localization import resolve
from kart_bringup.mission import composition, hdmap_choices, sensor_parameters, camera_config, recording_config, validate_localization_imu, validate_record_profile, validate_overtake
from kart_bringup.vehicle_tf import load_transforms
from kart_bringup.recording_profiles import profile_vehicle_overrides

OVERRIDES = (
    "sensor_container",
    "create_sensor_container",
    "enable_evs",
    "perception_engine",
    "perception_mode",
    "perception_debug_image",
    "overtake_mode",
    "lap_vgl",
    "lap_cache_clear",
    "enable_foxglove",
    "enable_bridge",
)


def add_mission(args):
    root = lu.get_path("kart_bringup", "config")
    cfg = composition(
        root / "mission.yaml", args.mode, {k: getattr(args, k, "") for k in OVERRIDES}
    )
    record_profile = getattr(args, "record_profile", "")
    params = sensor_parameters(
        camera_config(root, args.mode, record_profile), args.rgb_fps, args.infra_fps,
        getattr(args, "rgb_profile", ""), getattr(args, "infra_profile", ""), getattr(args, "imu_profile", ""), getattr(args, "depth_profile", "")
    )
    from kart_bringup.mission import perception_engine
    engine = perception_engine(cfg["perception_engine"], params)
    if cfg["perception_debug_image"] and not engine:
        raise ValueError("認識デバッグ画像にはperception_engineを指定してください")
    validate_record_profile(record_profile, args.mode, params, cfg["enable_evs"])
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
        "joy_max_throttle": getattr(args, "joy_max_throttle", ""),
        "auto_fixed_throttle": getattr(args, "auto_fixed_throttle", ""),
        "auto_fixed_throttle_r2_gate": collection_r2_gate(
            args.mode, getattr(args, "auto_fixed_throttle_r2_gate", "")),
    }
    vehicle = profile_vehicle_overrides(root, record_profile, args.mode, vehicle)
    if args.mode == "environment" or cfg["enable_evs"] or record_profile:
        vehicle["bag_config"] = str(recording_config(root, args.mode, cfg["enable_evs"], record_profile))
    if cfg['e2e']:
        # Collection's Joy throttle must never take ownership of E2E steering.
        vehicle['auto_fixed_throttle'] = '-1'
    evs_values = {key: getattr(args, key, "") for key in ("evs_backend", "evs_serial", "evs_bias_file", "evs_tensor_config", "evs_aggregation", "evs_image")}
    if any(evs_values.values()) and not cfg["enable_evs"]:
        raise ValueError("EVS startup overrides require enable_evs:=true")
    if cfg["enable_evs"]:
        from kart_bringup.evs import pipeline_configuration
        pipeline_configuration(root / "sensors", *evs_values.values())
        get_package_share_directory("openeb_ros2")
        vehicle["bag_shutdown_timeout"] = "26"
    parking_site = getattr(args, 'parking_site', '') or (args.map_file if getattr(args, 'parking_slot', '') else '')
    if parking_site:
        if args.mode != 'drive':
            raise ValueError('Parking is drive-only')
        from kart_bringup.parking import validate
        validate(root, parking_site, getattr(args, 'parking_slot', ''),
                 getattr(args, 'recovery', ''), getattr(args, 'tuning_enabled', ''))
    elif getattr(args, 'parking_slot', ''):
        raise ValueError('parking_slot requires map_file or parking_site')
    _, vehicle_modules = load(
        root / "bringup.yaml", vehicle
    )  # Validate device/config before any node starts.
    if vehicle_modules['joy_manager']['effective']['auto_fixed_throttle_r2_gate'] and record_profile == 'vslam-map':
        raise ValueError('auto_fixed_throttle_r2_gate requires the standard record profile; vslam-map omits vehicle commands and modes')
    if vehicle_modules['joy_manager']['effective']['auto_fixed_throttle_r2_gate'] and not (
            params['enable_infra1'] and params['enable_infra2']):
        raise ValueError('auto_fixed_throttle_r2_gate requires stereo Infra for offline vehicle speed estimation')
    mapless = None
    if record_profile == 'vehicle-model':
        if not cfg['create_sensor_container']:
            raise ValueError('vehicle-model must own its sensor container; stop other mission launches first')
        if engine or cfg['localization'] or cfg['tracking'] or cfg['e2e']:
            raise ValueError('vehicle-model cannot start perception, saved-map localization, tracking or E2E')
        if any(getattr(args, key, '') for key in ('map_dir', 'map_file', 'model_dir', 'lane_id', 'line_type', 'e2e_model_dir')):
            raise ValueError('vehicle-model does not use saved maps, lanes or models')
        from kart_bringup.tuning import mapless_configuration
        mapless = mapless_configuration(root, params, multicam_mode=args.multicam_mode)
    if parking_site and vehicle_modules["joy_manager"]["effective"]["auto_fixed_throttle"] >= 0:
        raise ValueError("Parking cannot use fixed AUTO throttle")
    from kart_bringup.recovery import configuration, validate_inputs
    recovery = getattr(args, 'recovery', '')
    recovery_enabled = False
    if recovery and args.mode not in ('drive', 'e2e'):
        raise ValueError('recovery is only supported for drive/e2e')
    if cfg['tracking'] or (cfg['e2e'] and cfg['e2e_drive_enabled']):
        recovery_enabled, recovery_params = configuration(root / 'recovery', recovery)
        if recovery_enabled:
            validate_inputs(recovery_params, params, cfg['enable_evs'],
                            vehicle_modules['joy_manager']['effective']['auto_fixed_throttle'])
    validate_overtake(cfg['overtake_mode'], args.mode, engine, cfg['perception_mode'],
                      parking=bool(parking_site),
                      fixed_throttle=vehicle_modules['joy_manager']['effective'].get('auto_fixed_throttle', -1.),
                      recovery_enabled=recovery_enabled)
    if cfg["localization"]:
        load_transforms(root / "vehicle/transforms.yaml", require_camera=True)
        if not params["enable_infra1"] or not params["enable_infra2"]:
            raise ValueError("Drive localization requires stereo infra (infra_fps > 0)")
        overrides={'multicam_mode':args.multicam_mode} if args.multicam_mode!="" else {}
        if section_routing: overrides['base_frame'] = section_routing['base_frame']
        localization_params, _, _, profile = resolve(root / "localization", args.map_dir, args.model_dir,overrides, enable_vgl=not bool(section_routing))
        validate_localization_imu(localization_params, params)
        width, height, _ = params["depth_module.infra_profile"].split("x")
        if not section_routing and profile["input_shape"][2:] != [int(height), int(width)]:
            raise ValueError("RealSense resolution differs from VGL map/model shape")
    if cfg["tracking"] and (args.lane_id, args.line_type) not in hdmap_choices(
        args.map_file
    ):
        raise ValueError("Choose a generated line from the selected HDMap/lane")
    lap_actions = []
    if cfg["lap_vgl"] or cfg["lap_cache_clear"]:
        from launch_ros.actions import Node
        from kart_bringup.lap_maintenance import maintenance_parameters
        lap_parameters = maintenance_parameters(root, cfg, args.map_file, args.lane_id,
                                                args.line_type, parking=bool(parking_site))
        get_package_share_directory("kart_lap_manager")
        lap_actions.append(Node(package="kart_lap_manager", executable="lap_manager",
                                name="lap_manager", parameters=[lap_parameters], output="screen"))
        if cfg["lap_cache_clear"]:
            get_package_share_directory("kart_jetson_cache")
            lap_actions.append(Node(package="kart_jetson_cache", executable="jetson_cache_service",
                                    name="jetson_cache_service", parameters=[str(root / "system/jetson_cache.yaml")],
                                    output="screen"))
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
        _, output_mode = model_contract(args.e2e_model_dir)
        runtime = runtime_settings(args.e2e_model_dir)
        fixed_override = getattr(args, 'e2e_fixed_throttle', '')
        from kart_bringup.e2e import throttle_selection
        use_fixed = throttle_selection(getattr(args, 'e2e_throttle_mode', ''), output_mode, fixed_override)
        if fixed_override != '':
            validate_runtime(dict(runtime, fixed_throttle=float(fixed_override)))
        from kart_e2e.engine_cache import require_engine
        require_engine(args.e2e_model_dir)
    realsense_enabled = not (cfg['e2e'] and modality == 'evs'
                             and not any(params[k] for k in ('enable_color', 'enable_infra1', 'enable_infra2', 'enable_depth')))
    if realsense_enabled:
        get_package_share_directory('realsense2_camera')
    actions = [
        lu.log_info(
            f"Mission: {args.mode}; config: {cfg}; recording/AUTO require Joy operation"
        )
    ]
    if lap_actions:
        actions.append(lu.log_info(f"Lap maintenance config: {root / 'localization/lap_manager.yaml'}; "
                                   f"effective parameters: {lap_parameters}; cache config: {root / 'system/jetson_cache.yaml'}"))
        actions.extend(lap_actions)
    if mapless is not None:
        mapless_params, mapless_remaps = mapless
        actions.append(lu.log_info(
            f'Vehicle-model mapless odometry: config={root / "tuning/cuvslam.yaml"}; '
            f'effective VSLAM={mapless_params}; remaps={mapless_remaps}. Recording and AUTO remain explicit.'))
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
                    "camera_config": str(camera_config(root, args.mode, record_profile)),
                    "rgb_profile": getattr(args, "rgb_profile", ""),
                    "depth_profile": getattr(args, "depth_profile", ""),
                    "infra_profile": getattr(args, "infra_profile", ""),
                    "imu_profile": getattr(args, "imu_profile", ""),
                    "rgb_fps": args.rgb_fps,
                    "infra_fps": args.infra_fps,
                },
            )
        )
    if engine:
        actions.append(include("racing_perception.launch.py", {
            "engine": engine,
            "tracking_enabled": str(cfg["perception_mode"] == "track").lower(),
            "debug_image": str(cfg["perception_debug_image"]).lower(),
            "target_container": "/" + cfg["sensor_container"].lstrip("/"),
        }))
    if getattr(args, "validate_infra", "false") == "true":
        if not params["enable_infra1"] or not params["enable_infra2"]:
            raise ValueError("validate_infra requires both Infra streams")
        actions.append(lu.load_composable_nodes(
            "/" + cfg["sensor_container"].lstrip("/"),
            [lut.ComposableNode(
                package="kart_bag_manager", plugin="kart_bag_manager::InfraStampValidator",
                name="infra_stamp_validator",
                parameters=[str(root / "recording/infra_validator.yaml")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )],
        ))
    if cfg["enable_evs"]:
        actions.append(
            include(
                "modules/sensors/openeb.launch.py",
                evs_values,
            )
        )
    actions.append(include("vehicle.launch.py", vehicle))
    if mapless is not None:
        mapless_params, mapless_remaps = mapless
        actions.append(include('modules/localization/vslam.launch.py', {
            'container_name': '/' + cfg['sensor_container'].lstrip('/'),
            'parameters_json': json.dumps(mapless_params),
            'remappings_json': json.dumps(mapless_remaps),
        }))
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
                    "vgl_container": "/" + cfg["sensor_container"].lstrip("/"),
                    "create_vgl_container": "false",
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
                    "parking_site": parking_site,
                    "parking_slot": getattr(args, "parking_slot", ""),
                    "map_file": args.map_file,
                    "lane_id": args.lane_id,
                    "line_type": args.line_type,
                    "recovery": recovery,
                    "overtake_mode": cfg['overtake_mode'],
                    "perception_engine": engine,
                    "perception_mode": cfg['perception_mode'],
                    "auto_fixed_throttle": str(vehicle_modules['joy_manager']['effective'].get('auto_fixed_throttle', -1.)),
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
                    'use_fixed_throttle': str(use_fixed).lower(),
                    'fixed_throttle': getattr(args, 'e2e_fixed_throttle', ''),
                },
            )
        )
    if cfg["enable_foxglove"]:
        actions.append(include("foxglove.launch.py", {}))
    return actions


def generate_launch_description():
    args = LaunchArguments()
    args.add_arg("validate_infra", "false", choices=["true", "false"], cli=True)
    args.add_arg("mode", "collect", choices=["collect", "environment", "drive", "e2e"], cli=True)
    for key in (
        *OVERRIDES,
        "device",
        "record_dir",
        "run_name",
        "record_profile",
        "auto_fixed_throttle",
        "auto_fixed_throttle_r2_gate",
        "joy_max_throttle",
        "recovery",
        "parking_site",
        "parking_slot",
        "e2e_model_dir",
        "e2e_fixed_throttle",
        "e2e_throttle_mode",
        "evs_backend",
        "evs_serial",
        "evs_bias_file",
        "evs_tensor_config",
        "evs_aggregation",
        "evs_image",
        "rgb_profile",
        "infra_profile",
        "depth_profile",
        "imu_profile",
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
