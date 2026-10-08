"""Load the official release-5.0 image encoder components into an existing container."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from kart_bringup.e2e import boolean, validate_encoder


def add_nodes(args):
    path = lu.get_path("kart_bringup", "config/e2e/image_encoder.yaml")
    configs = validate_encoder(path)
    definitions = [
        (
            "e2e_resize",
            "isaac_ros_image_proc",
            "nvidia::isaac_ros::image_proc::ResizeNode",
            [
                ("image", args.image_topic),
                ("camera_info", args.camera_info_topic),
                ("resize/image", "/e2e/resize/image"),
                ("resize/camera_info", "/e2e/resize/camera_info"),
            ],
        ),
        (
            "e2e_format",
            "isaac_ros_image_proc",
            "nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
            [("image_raw", "/e2e/resize/image"), ("image", "/e2e/rgb/image")],
        ),
        (
            "e2e_to_tensor",
            "isaac_ros_tensor_proc",
            "nvidia::isaac_ros::dnn_inference::ImageToTensorNode",
            [("image", "/e2e/rgb/image"), ("tensor", "/e2e/image_tensor")],
        ),
        (
            "e2e_normalize",
            "isaac_ros_tensor_proc",
            "nvidia::isaac_ros::dnn_inference::ImageTensorNormalizeNode",
            [
                ("tensor", "/e2e/image_tensor"),
                ("normalized_tensor", "/e2e/normalized_tensor"),
            ],
        ),
        (
            "e2e_planar",
            "isaac_ros_tensor_proc",
            "nvidia::isaac_ros::dnn_inference::InterleavedToPlanarNode",
            [
                ("interleaved_tensor", "/e2e/normalized_tensor"),
                ("planar_tensor", "/e2e/planar_tensor"),
            ],
        ),
        (
            "e2e_reshape",
            "isaac_ros_tensor_proc",
            "nvidia::isaac_ros::dnn_inference::ReshapeNode",
            [
                ("tensor", "/e2e/planar_tensor"),
                ("reshaped_tensor", "/e2e/tensor_input"),
            ],
        ),
    ]
    nodes = []
    for name, package, plugin, remappings in definitions:
        params = configs[name]
        if args.use_sim_time != "":
            params["use_sim_time"] = boolean(args.use_sim_time)
        nodes.append(
            lut.ComposableNode(
                name=name,
                package=package,
                plugin=plugin,
                parameters=[params],
                remappings=remappings,
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
    return [
        lu.log_info(
            f"E2E encoder config: {path}; use_sim_time override: {args.use_sim_time}"
        ),
        lu.load_composable_nodes(args.container_name, nodes),
    ]


def generate_launch_description():
    args = lu.ArgumentContainer()
    for key in ("container_name", "image_topic", "camera_info_topic"):
        args.add_arg(key, cli=True)
    args.add_arg("use_sim_time", "", cli=True)
    args.add_opaque_function(add_nodes)
    return lut.LaunchDescription(args.get_launch_actions())
