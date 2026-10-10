"""Bag-only localization review: no vehicle, sensor drivers, recording, or control."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
import yaml
from kart_bringup.localization import resolve
from kart_bringup.mission import boolean, hdmap_choices
from kart_bringup.replay import playback


def add_evaluation(args):
    root = lu.get_path("kart_bringup", "config")
    settings = yaml.safe_load((root / "evaluation/replay.yaml").read_text())
    base_frame = args.base_frame if args.base_frame != "" else settings["base_frame"]
    playback(root, args.bag, args.rate)
    resolve(
        root / "localization",
        args.map_dir,
        args.model_dir,
        {"use_sim_time": "true", "base_frame": base_frame},
        enable_vgl=boolean(args.enable_vgl),
    )
    if (args.lane_id, args.line_type) not in hdmap_choices(args.map_file):
        raise ValueError("Select a generated HDMap line")

    def include(file, values):
        return lu.include(
            "kart_bringup",
            "launch/" + file,
            launch_arguments=values,
            scoped=True,
            forwarding=False,
        )

    actions = [
        include(
            "localization.launch.py",
            {
                "map_dir": args.map_dir,
                "model_dir": args.model_dir,
                "use_sim_time": "true",
                "visualize": "true",
                "base_frame": base_frame,
                "enable_vgl": args.enable_vgl,
            },
        ),
        include(
            "hdmap.launch.py",
            {
                "map_file": args.map_file,
                "lane_id": args.lane_id,
                "line_type": args.line_type,
                "use_sim_time": "true",
            },
        ),
    ]
    rviz = (
        yaml.safe_load((root / "evaluation/replay.yaml").read_text())["rviz"]
        if args.rviz == ""
        else boolean(args.rviz)
    )
    if rviz:
        actions.append(include("modules/evaluation/rviz.launch.py", {}))
    actions.append(
        include(
            "modules/evaluation/replay.launch.py",
            {"bag": args.bag, "rate": args.rate, "enable_vgl": args.enable_vgl},
        )
    )
    return actions


def generate_launch_description():
    args = lu.ArgumentContainer()
    for key in ("bag", "map_dir", "map_file", "lane_id", "line_type"):
        args.add_arg(key, cli=True)
    args.add_arg("model_dir", "", cli=True)
    args.add_arg("enable_vgl", "false", cli=True)
    args.add_arg("rate", "", cli=True)
    args.add_arg("base_frame", "", cli=True)
    args.add_arg("rviz", "", cli=True)
    args.add_opaque_function(add_evaluation)
    return lut.LaunchDescription(args.get_launch_actions())
