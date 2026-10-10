"""Filtered bag player and startup readiness monitor; no physical drivers."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from kart_bringup.replay import playback
from kart_bringup.mission import boolean
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def add_replay(args):
    root = lu.get_path("kart_bringup", "config")
    command = playback(root, args.bag, args.rate)
    overrides = (
        {} if boolean(args.enable_vgl) else {"subscriber_nodes": ["visual_slam"]}
    )
    return [
        lu.log_info(f"Replay (filtered, initially paused): {command}"),
        ExecuteProcess(cmd=command, output="screen"),
        Node(
            package="kart_bringup",
            executable="replay_ready",
            name="replay_ready",
            parameters=[str(root / "evaluation/replay_ready.yaml"), overrides],
            output="screen",
        ),
    ]


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg("bag", cli=True)
    args.add_arg("enable_vgl", "true", cli=True)
    args.add_arg("rate", "", cli=True)
    args.add_opaque_function(add_replay)
    return lut.LaunchDescription(args.get_launch_actions())
