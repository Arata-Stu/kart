"""Offline line server plus tracking component; no vehicle or localization startup."""

import json
import re

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def add_tracking(args):
    config = json.loads(
        lu.get_path("kart_bringup", "config/control/launch.json").read_text()
    )
    for key in ("container_name", "create_container"):
        if getattr(args, key) != "":
            config[key] = getattr(args, key)
    if str(config["create_container"]).lower() not in ("true", "false"):
        raise ValueError("create_container must be true or false")
    create = str(config["create_container"]).lower() == "true"
    name = config["container_name"]
    if not isinstance(name, str) or not re.fullmatch(
        r"/?[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*", name
    ):
        raise ValueError("Invalid container name")
    name = name.lstrip("/")
    if create and "/" in name:
        raise ValueError("Namespaced container must be externally owned")
    if config["container_type"] not in ("multithreaded", "isolated_multithreaded"):
        raise ValueError("Unsupported executor")
    actions = [lu.log_info(f"Tracking container config: {config}")]
    if create:
        actions.append(
            lu.component_container(name, container_type=config["container_type"])
        )
    actions += [
        lu.include(
            "kart_bringup",
            "launch/hdmap.launch.py",
            launch_arguments={
                "map_file": args.map_file,
                "lane_id": args.lane_id,
                "line_type": args.line_type,
                "use_sim_time": args.use_sim_time,
            },
            scoped=True,
            forwarding=False,
        ),
        lu.include(
            "kart_bringup",
            "launch/modules/control/pure_pursuit.launch.py",
            launch_arguments={
                "container_name": "/" + name,
                "use_sim_time": args.use_sim_time,
            },
            scoped=True,
            forwarding=False,
        ),
    ]
    actions.append(
        lu.include(
            "kart_bringup",
            "launch/modules/control/speed_controller.launch.py",
            launch_arguments={
                "container_name": "/" + name,
                "use_sim_time": args.use_sim_time,
            },
            scoped=True,
            forwarding=False,
        )
    )
    return actions


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg("map_file", description="HDMap map.json or hd_map.yaml", cli=True)
    args.add_arg(
        "lane_id", "", description="Lane to track; empty disables selection", cli=True
    )
    args.add_arg(
        "line_type",
        "",
        choices=["", "centerline", "raceline", "customline"],
        description="Initial reference line; empty preserves YAML",
        cli=True,
    )
    args.add_arg("container_name", "", cli=True)
    args.add_arg(
        "create_container", "", choices=["", "true", "false", "True", "False"], cli=True
    )
    args.add_arg(
        "use_sim_time", "", choices=["", "true", "false", "True", "False"], cli=True
    )
    args.add_opaque_function(add_tracking)
    return lut.LaunchDescription(args.get_launch_actions())
