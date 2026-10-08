"""One host-level jtop process, normally owned by vehicle.launch.py."""

import json
from pathlib import Path

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from kart_bringup.monitoring import is_jetson, should_start


def add_monitor(args):
    settings = json.loads(
        lu.get_path("kart_bringup", "config/monitoring/launch.json").read_text()
    )
    if set(settings) != {"mode"}:
        raise ValueError("Unknown monitoring setting")
    mode = args.mode if args.mode != "" else settings["mode"]
    if not should_start(mode, is_jetson()):
        return [
            lu.log_info(
                f"Jetson monitoring skipped: mode={mode}, non-Jetson or disabled"
            )
        ]
    try:
        get_package_prefix("isaac_ros_jetson_stats")
        if not Path("/run/jtop.sock").is_socket():
            raise RuntimeError(
                "Missing /run/jtop.sock: start host jtop.service and mount its socket"
            )
    except (PackageNotFoundError, RuntimeError) as exc:
        if mode == "on":
            raise RuntimeError(f"Jetson monitoring unavailable: {exc}") from exc
        return [
            lut.LogInfo(
                msg=f"WARNING: Jetson monitoring unavailable; no hardware metrics will be recorded: {exc}"
            )
        ]
    config = lu.get_path("kart_bringup", "config/monitoring/jetson_stats.yaml")
    return [
        lu.log_info(f"Jetson stats config: {config}; mode={mode}"),
        lut.Node(
            package="isaac_ros_jetson_stats",
            executable="jtop",
            name="jtop",
            namespace="",
            output="screen",
            parameters=[str(config)],
            remappings=[("diagnostics", "/system/jetson/diagnostics")],
        ),
    ]


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg(
        "mode",
        "",
        choices=["", "auto", "on", "off"],
        description="Empty uses monitoring/launch.json",
        cli=True,
    )
    args.add_opaque_function(add_monitor)
    return lut.LaunchDescription(args.get_launch_actions())
