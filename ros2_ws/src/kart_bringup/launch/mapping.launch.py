"""Local offline cuVSLAM and VGL bootstrap. No sensors, vehicle or Jetson bringup."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def setup(context):
    config = Path(get_package_share_directory("kart_bringup")) / "config" / "mapping"
    job = json.loads(Path(LaunchConfiguration("job_file").perform(context)).read_text())
    from kart_bringup.capture_localization import resolve_capture
    from kart_bringup.vgl_assets import inspect_engines

    if job.get("operation") != "capture":
        raise ValueError("This launch is only for saved-map snapshot capture")
    map_dir = Path(job["map_dir"]).resolve(strict=True)
    if not any(map_dir.glob("*.mdb")):
        raise ValueError("Saved VSLAM map is missing")
    parameters, remaps, shape = resolve_capture(config.parent, job, inspect_engines)
    print(f"Capture parameters: {parameters}; remappings: {remaps}; VGL shape: {shape}", flush=True)
    nodes = [ComposableNode(
        package="isaac_ros_cuvslam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        name="visual_slam", parameters=[parameters["cuvslam"]],
        remappings=remaps["cuvslam"],
        extra_arguments=[{"use_intra_process_comms": True}],
    )]
    if "vgl" in parameters:
        nodes.append(ComposableNode(
            package="isaac_ros_visual_global_localization",
            plugin="nvidia::isaac_ros::visual_global_localization::VisualGlobalLocalizationNode",
            name="visual_global_localization", parameters=[parameters["vgl"]],
            remappings=remaps["vgl"],
        ))
    return [
        ComposableNodeContainer(
            name="kart_mapping_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=nodes,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument("job_file"), OpaqueFunction(function=setup)]
    )
