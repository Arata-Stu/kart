"""Image encoder + TensorRT + control decoder in one owned or external container."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from kart_bringup.e2e import container_config, validate_encoder
from kart_e2e.contract import model_contract


def add_pipeline(args):
    keys = (
        "container_name",
        "create_container",
        "model_dir",
        "image_topic",
        "camera_info_topic",
    )
    config = container_config(
        lu.get_path("kart_bringup", "config/e2e/launch.json"),
        {key: getattr(args, key) for key in keys},
    )
    model_contract(config["model_dir"])
    validate_encoder(lu.get_path("kart_bringup", "config/e2e/image_encoder.yaml"))
    actions = [lu.log_info(f"E2E pipeline: {config}")]
    if config["create_container"]:
        actions.append(
            lu.component_container(
                config["container_name"].lstrip("/"),
                container_type=config["container_type"],
            )
        )
    actions.append(
        lu.include(
            "kart_bringup",
            "launch/modules/e2e/image_encoder.launch.py",
            launch_arguments={
                "container_name": config["container_name"],
                "image_topic": config["image_topic"],
                "camera_info_topic": config["camera_info_topic"],
                "use_sim_time": args.use_sim_time,
            },
            scoped=True,
            forwarding=False,
        )
    )
    actions.append(
        lu.include(
            "kart_bringup",
            "launch/modules/e2e/inference.launch.py",
            launch_arguments={
                "container_name": config["container_name"],
                "model_dir": config["model_dir"],
                **{
                    key: getattr(args, key)
                    for key in (
                        "fixed_throttle",
                        "drive_enabled",
                        "force_engine_update",
                        "use_sim_time",
                    )
                },
            },
            scoped=True,
            forwarding=False,
        )
    )
    return actions


def generate_launch_description():
    args = lu.ArgumentContainer()
    for key in (
        "container_name",
        "create_container",
        "model_dir",
        "image_topic",
        "camera_info_topic",
        "fixed_throttle",
        "drive_enabled",
        "force_engine_update",
        "use_sim_time",
    ):
        args.add_arg(key, "", cli=True)
    args.add_opaque_function(add_pipeline)
    return lut.LaunchDescription(args.get_launch_actions())
