"""Load Pure Pursuit into an existing multithreaded container."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def add_node(args):
    overrides = {}
    if args.use_sim_time != "":
        if str(args.use_sim_time).lower() not in ("true", "false"):
            raise ValueError("use_sim_time must be true or false")
        overrides["use_sim_time"] = str(args.use_sim_time).lower() == "true"
    config = lu.get_path("kart_bringup", "config/control/pure_pursuit.yaml")
    node = lut.ComposableNode(
        name="pure_pursuit",
        package="kart_control",
        plugin="kart_control::PurePursuitNode",
        parameters=[str(config), overrides],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return [
        lu.log_info(f"Pure Pursuit config: {config}; overrides: {overrides}"),
        lu.load_composable_nodes(args.container_name, [node]),
    ]


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg("container_name", description="Existing component container", cli=True)
    args.add_arg("use_sim_time", "", cli=True)
    args.add_opaque_function(add_node)
    return lut.LaunchDescription(args.get_launch_actions())
