"""Local offline cuVSLAM only. No sensors, vehicle or Jetson bringup."""

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
    workflow = job["workflow"]
    parameters = str(config / "capture_cuvslam.yaml")
    if job.get("operation") != "capture":
        raise ValueError("This launch is only for saved-map snapshot capture")
    allowed = {"base_frame", "camera_optical_frames", "imu_frame", "tracking_mode"}
    overrides = dict(workflow.get("overrides", {}))
    if set(overrides) - allowed:
        raise ValueError("Unknown VSLAM override")
    map_dir = Path(job["map_dir"]).resolve(strict=True)
    if not any(map_dir.glob("*.mdb")):
        raise ValueError("Saved VSLAM map is missing")
    overrides["load_map_folder_path"] = str(map_dir)
    print(
        f"Kart mapping config: {parameters}; explicit overrides: {overrides}",
        flush=True,
    )
    remappings = [
        ("visual_slam/" + target, workflow[key])
        for target, key in [
            ("image_0", "left_image"),
            ("image_1", "right_image"),
            ("camera_info_0", "left_info"),
            ("camera_info_1", "right_info"),
        ]
    ]
    return [
        ComposableNodeContainer(
            name="kart_mapping_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="isaac_ros_cuvslam",
                    plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
                    name="visual_slam",
                    parameters=[parameters, overrides],
                    remappings=remappings,
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument("job_file"), OpaqueFunction(function=setup)]
    )
