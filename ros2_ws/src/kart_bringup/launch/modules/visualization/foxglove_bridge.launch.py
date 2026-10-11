"""Own one standalone Bridge process; do not create/load a component container."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
import yaml


def add_bridge(args):
    config = lu.get_path("kart_bringup", "config/visualization/foxglove.yaml")
    params = yaml.safe_load(config.read_text())["/**/foxglove_bridge"][
        "ros__parameters"
    ]
    return [
        lu.log_info(f"Foxglove config: {config}; effective parameters={params}"),
        lut.Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            namespace="",
            output="screen",
            parameters=[str(config)],
        ),
    ]


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_opaque_function(add_bridge)
    return lut.LaunchDescription(args.get_launch_actions())
