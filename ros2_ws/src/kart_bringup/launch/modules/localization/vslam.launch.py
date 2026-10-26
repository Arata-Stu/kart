"""Load visual_slam into a caller-owned container; never create a process."""

import json
from types import SimpleNamespace

import isaac_ros_launch_utils as lu
from kart_bringup.launch_arguments import LaunchArguments
import isaac_ros_launch_utils.all_types as lut


def load_component(args: SimpleNamespace) -> list[lut.Action]:
    # Launch keeps JSON text; direct Python callers may supply decoded values.
    parameters = (
        json.loads(args.parameters_json)
        if isinstance(args.parameters_json, str)
        else args.parameters_json
    )
    remappings = (
        json.loads(args.remappings_json)
        if isinstance(args.remappings_json, str)
        else args.remappings_json
    )
    node = lut.ComposableNode(
        name="visual_slam",
        package="isaac_ros_cuvslam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        parameters=[parameters],
        remappings=[tuple(pair) for pair in remappings],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return [lu.load_composable_nodes(args.container_name, [node])]


def generate_launch_description() -> lut.LaunchDescription:
    args = LaunchArguments()
    args.add_arg(
        "container_name",
        description="Existing multithreaded component container",
        cli=True,
    )
    args.add_arg(
        "parameters_json",
        description="Validated parameters supplied by parent launch",
        cli=True,
    )
    args.add_arg(
        "remappings_json",
        description="Validated remappings supplied by parent launch",
        cli=True,
    )
    args.add_opaque_function(load_component)
    return lut.LaunchDescription(args.get_launch_actions())
