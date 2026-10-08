"""Load official TensorRT and kart's C++ control decoder; no container creation."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from kart_bringup.e2e import boolean, parameters
from kart_e2e.contract import model_contract


def add_nodes(args):
    root = lu.get_path("kart_bringup", "config/e2e")
    paths, mode = model_contract(args.model_dir)
    trt = parameters(root / "tensor_rt.yaml")
    decoder = parameters(root / "control_decoder.yaml")
    if (
        trt["input_tensor_names"] != ["image"]
        or trt["input_binding_names"] != ["image"]
        or trt["output_tensor_names"] != ["control"]
        or trt["output_binding_names"] != ["control"]
        or decoder["output_tensor_name"] != "control"
    ):
        raise ValueError("Tensor names must match exported ONNX bindings")
    trt.update(paths)
    decoder["output_mode"] = mode
    if args.use_sim_time != "":
        trt["use_sim_time"] = decoder["use_sim_time"] = boolean(args.use_sim_time)
    if args.fixed_throttle != "":
        decoder["fixed_throttle"] = float(args.fixed_throttle)
    if args.drive_enabled != "":
        decoder["drive_enabled"] = boolean(args.drive_enabled)
    if args.force_engine_update != "":
        trt["force_engine_update"] = boolean(args.force_engine_update)
    nodes = [
        lut.ComposableNode(
            name="e2e_tensor_rt",
            package="isaac_ros_tensor_rt",
            plugin="nvidia::isaac_ros::dnn_inference::TensorRTNode",
            parameters=[trt],
            remappings=[
                ("tensor_pub", "/e2e/tensor_input"),
                ("tensor_sub", "/e2e/tensor_output"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        lut.ComposableNode(
            name="e2e_control_decoder",
            package="kart_e2e",
            plugin="kart_e2e::ControlDecoderNode",
            parameters=[decoder],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
    ]
    return [
        lu.log_info(f"E2E config: {root}; TensorRT: {trt}; decoder: {decoder}"),
        lu.load_composable_nodes(args.container_name, nodes),
    ]


def generate_launch_description():
    args = lu.ArgumentContainer()
    for key in ("container_name", "model_dir"):
        args.add_arg(key, cli=True)
    for key in (
        "fixed_throttle",
        "drive_enabled",
        "force_engine_update",
        "use_sim_time",
    ):
        args.add_arg(key, "", cli=True)
    args.add_opaque_function(add_nodes)
    return lut.LaunchDescription(args.get_launch_actions())
