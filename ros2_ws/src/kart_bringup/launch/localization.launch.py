"""Paired localization following Isaac ROS mapping launch conventions."""

import json

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from kart_bringup.container_plan import plan
from kart_bringup.localization import resolve
from kart_bringup.vgl_assets import inspect_engines


def add_localization(args: lu.ArgumentContainer) -> list[lut.Action]:
    """Validate the paired assets before starting either component container."""
    config = lu.get_path("kart_bringup", "config/localization")
    # ArgumentContainer evaluates 'False' to bool: preserve explicit false overrides.
    overrides = {
        key: getattr(args, key)
        for key in ("use_sim_time", "base_frame")
        if getattr(args, key) != ""
    }
    parameters, remaps, _workflow, profile = resolve(
        config, args.map_dir, args.model_dir, overrides, inspect_engines
    )

    targets, containers = plan(
        config / "containers.json",
        {
            key: getattr(args, key)
            for key in (
                "vslam_container",
                "vgl_container",
                "create_vslam_container",
                "create_vgl_container",
            )
        },
    )
    actions = [
        lu.log_info(f"Localization config: {config}; explicit overrides: {overrides}"),
        lu.log_info(f"Map: {args.map_dir}; VGL input shape: {profile['input_shape']}"),
        lu.log_info(f"Effective parameters: {parameters}; remappings: {remaps}"),
        lu.log_info(f"Container targets: {targets}; owned containers: {containers}"),
    ]
    actions.extend(
        lu.component_container(item["name"], container_type=item["type"])
        for item in containers
    )
    for kind, key in (("vslam", "cuvslam"), ("vgl", "vgl")):
        actions.append(
            lu.include(
                "kart_bringup",
                f"launch/modules/localization/{kind}.launch.py",
                launch_arguments={
                    "container_name": targets[kind],
                    "parameters_json": json.dumps(parameters[key]),
                    "remappings_json": json.dumps(remaps[key]),
                },
                scoped=True,
                forwarding=False,
            )
        )
    return actions


def generate_launch_description() -> lut.LaunchDescription:
    """Expose runtime inputs; node tuning remains in bringup YAML."""
    args = lu.ArgumentContainer()
    args.add_arg(
        "map_dir",
        description="Paired localization bundle from prepare_vgl_map",
        cli=True,
    )
    args.add_arg(
        "model_dir",
        description="ALIKED/LightGlue model directory for this GPU",
        cli=True,
    )
    args.add_arg(
        "base_frame",
        "",
        description="Override both base frames; empty preserves YAML",
        cli=True,
    )
    args.add_arg(
        "use_sim_time",
        "",
        description="Override both clocks; empty preserves YAML",
        choices=["", "true", "false", "True", "False"],
        cli=True,
    )
    for kind in ("vslam", "vgl"):
        args.add_arg(
            f"{kind}_container",
            "",
            description="Target container; empty preserves containers.json",
            cli=True,
        )
        args.add_arg(
            f"create_{kind}_container",
            "",
            description="Create container or load into an externally owned one",
            choices=["", "true", "false", "True", "False"],
            cli=True,
        )
    args.add_opaque_function(add_localization)
    return lut.LaunchDescription(args.get_launch_actions())
