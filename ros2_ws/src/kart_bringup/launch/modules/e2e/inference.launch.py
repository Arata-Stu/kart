"""Load official TensorRT and kart's C++ control decoder; no container creation."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from kart_bringup.e2e import boolean, parameters
from kart_e2e.contract import model_contract, runtime_settings, input_contract, validate_runtime, routing_settings
from kart_e2e.engine_cache import require_engine


def add_nodes(args):
    root = lu.get_path("kart_bringup", "config/e2e")
    paths, mode = model_contract(args.model_dir)
    modality, input_name, _ = input_contract(args.model_dir)
    trt = parameters(root / ('evs_tensor_rt.yaml' if modality == 'evs' else 'tensor_rt.yaml'))
    decoder = parameters(root / "control_decoder.yaml")
    if (
        trt["input_tensor_names"] != [input_name]
        or trt["input_binding_names"] != [input_name]
        or trt["output_tensor_names"] != ["control"]
        or trt["output_binding_names"] != ["control"]
        or decoder["output_tensor_name"] != "control"
    ):
        raise ValueError("Tensor names must match exported ONNX bindings")
    trt.update(paths)
    decoder.update(runtime_settings(args.model_dir))
    decoder["output_mode"] = mode
    routing = routing_settings(args.model_dir)
    selector_actions = []
    if routing:
        if modality != 'rgb':
            raise ValueError('Section runtime routing requires RGB image timestamps')
        from kart_e2e.sections import Router
        from launch_ros.actions import Node
        decoder.update(head_count=len(routing['heads']), routing_sha256=Router(routing).sha,
                       head_timeout_s=routing['max_pose_age_s'])
        selector_actions.append(Node(package='kart_e2e', executable='e2e_section_selector',
            name='e2e_section_selector', output='screen', parameters=[str(root/'section_selector.yaml'),
            dict(model_dir=str(args.model_dir),use_sim_time=boolean(args.use_sim_time) if args.use_sim_time!='' else False)]))
    if args.use_sim_time != "":
        trt["use_sim_time"] = decoder["use_sim_time"] = boolean(args.use_sim_time)
    if args.fixed_throttle != "":
        decoder["fixed_throttle"] = float(args.fixed_throttle)
    validate_runtime({key: decoder[key] for key in ("fixed_throttle", "max_throttle")})
    if getattr(args, "use_fixed_throttle", "") != "":
        decoder["use_fixed_throttle"] = boolean(args.use_fixed_throttle)
    if args.drive_enabled != "":
        decoder["drive_enabled"] = boolean(args.drive_enabled)
    if args.force_engine_update != "":
        trt["force_engine_update"] = boolean(args.force_engine_update)
    if trt["force_engine_update"]:
        raise ValueError(
            "bringupではTensorRT buildを行いません。scripts/e2e_trt.sh --forceを実行してください"
        )
    require_engine(args.model_dir)
    from kart_bringup.recovery import route
    # Observation-only E2E must never create a vehicle command producer.
    remappings, recovery_actions = (route(lu.get_path("kart_bringup", "config/recovery"), args.use_sim_time,
                                         selection=getattr(args, "recovery", ""))
                                     if decoder["drive_enabled"] else ([], []))
    nodes = [
        lut.ComposableNode(
            name="e2e_tensor_rt",
            package="isaac_ros_tensor_rt",
            plugin="nvidia::isaac_ros::dnn_inference::TensorRTNode",
            parameters=[trt],
            remappings=[
                ("tensor_pub", args.input_topic or "/e2e/tensor_input"),
                ("tensor_sub", "/e2e/tensor_output"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        lut.ComposableNode(
            name="e2e_control_decoder",
            package="kart_e2e",
            plugin="kart_e2e::ControlDecoderNode",
            parameters=[decoder],
            remappings=remappings,
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
    ]
    return [
        lu.log_info(f"E2E config: {root}; TensorRT: {trt}; decoder: {decoder}"),
        lu.load_composable_nodes(args.container_name, nodes),
        *selector_actions,
        *recovery_actions,
    ]


def generate_launch_description():
    args = lu.ArgumentContainer()
    for key in ("container_name", "model_dir"):
        args.add_arg(key, cli=True)
    for key in (
        "use_fixed_throttle",
        "fixed_throttle",
        "drive_enabled",
        "force_engine_update",
        "use_sim_time",
        "input_topic",
        "recovery",
    ):
        args.add_arg(key, "", cli=True)
    args.add_opaque_function(add_nodes)
    return lut.LaunchDescription(args.get_launch_actions())
