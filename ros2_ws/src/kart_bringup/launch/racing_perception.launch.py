"""640x480 RGB to image-space opponent tracks."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def pipeline(context):
    engine = Path(LaunchConfiguration("engine").perform(context)).expanduser().resolve()
    if not engine.is_file() or engine.stat().st_size == 0:
        raise ValueError(f"Missing TensorRT engine: {engine}")
    image = LaunchConfiguration("image_topic").perform(context)
    root = Path(get_package_share_directory("kart_bringup")) / "config/racing"
    settings = yaml.safe_load((root / "perception.yaml").read_text())
    settings["trt"]["engine_file_path"] = str(engine)
    dnn = "nvidia::isaac_ros::dnn_inference::"
    definitions = [
        ("format", "isaac_ros_image_proc", "nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
         [("image_raw", image), ("image", "rgb")]),
        ("to_tensor", "isaac_ros_tensor_proc", dnn + "ImageToTensorNode",
         [("image", "rgb"), ("tensor", "interleaved")]),
        ("planar", "isaac_ros_tensor_proc", dnn + "InterleavedToPlanarNode",
         [("interleaved_tensor", "interleaved"), ("planar_tensor", "planar_output")]),
        ("reshape", "isaac_ros_tensor_proc", dnn + "ReshapeNode",
         [("tensor", "planar_output"), ("reshaped_tensor", "network_input")]),
        ("trt", "isaac_ros_tensor_rt", dnn + "TensorRTNode",
         [("tensor_pub", "network_input"), ("tensor_sub", "network_output")]),
        ("decoder", "isaac_ros_yolov8", "nvidia::isaac_ros::yolov8::YoloV8DecoderNode",
         [("tensor_sub", "network_output"), ("detections_output", "detections")]),
        ("opponent_tracker", "kart_opponent_tracker", "kart_opponent_tracker::OpponentTracker", []),
    ]
    tracker_config = root / "tracker.yaml"
    settings["opponent_tracker"] = yaml.safe_load(tracker_config.read_text())[
        "/**/opponent_tracker"]["ros__parameters"]
    nodes = [
        ComposableNode(package=pkg, plugin=plugin, namespace="/racing", name=name,
                       parameters=[settings[name]], remappings=topics,
                       extra_arguments=[{"use_intra_process_comms": True}])
        for name, pkg, plugin, topics in definitions
    ]
    target = LaunchConfiguration("target_container").perform(context)
    if target:
        action = LoadComposableNodes(target_container=target, composable_node_descriptions=nodes)
    else:
        action = ComposableNodeContainer(name="racing_perception", namespace="", output="screen",
                                         package="rclcpp_components", executable="component_container_mt",
                                         composable_node_descriptions=nodes)
    return [LogInfo(msg=f"Racing perception: {root}, container={target or '/racing_perception'}, "
                        f"image={image}, {settings}"), action]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("target_container", default_value="", description="Existing component container"),
        DeclareLaunchArgument("engine", description="Prebuilt FP16 engine with FP32 bindings"),
        DeclareLaunchArgument("image_topic", default_value="/realsense/color/image_raw"),
        OpaqueFunction(function=pipeline),
    ])
