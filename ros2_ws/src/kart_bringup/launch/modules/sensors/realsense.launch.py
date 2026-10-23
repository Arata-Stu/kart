"""Load RealSense into a caller-owned multithreaded container."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from kart_bringup.mission import sensor_parameters


def add_camera(args):
    config = getattr(args, "camera_config", "") or lu.get_path("kart_bringup", "config/sensors/realsense.yaml")
    params = sensor_parameters(config, args.rgb_fps, args.infra_fps,
                               getattr(args, "rgb_profile", ""), getattr(args, "infra_profile", ""), getattr(args, "imu_profile", ""))
    return [
        lu.log_info(f"RealSense config: {config}; effective parameters: {params}"),
        lu.load_composable_nodes(
            args.container_name,
            [
                lut.ComposableNode(
                    package="realsense2_camera",
                    plugin="realsense2_camera::RealSenseNodeFactory",
                    name="realsense",
                    namespace="",
                    parameters=[params],
                    remappings=[("/diagnostics", "/realsense/diagnostics")],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            ],
        ),
    ]


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg("container_name", description="Existing sensor container", cli=True)
    args.add_arg(
        "rgb_fps", "", description="Empty preserves YAML; 0 disables RGB", cli=True
    )
    args.add_arg(
        "infra_fps",
        "",
        description="Empty preserves YAML; 0 disables both infra",
        cli=True,
    )
    args.add_arg("camera_config", "", description="Full RealSense YAML; empty uses standard config", cli=True)
    args.add_arg("rgb_profile", "", description="WIDTHxHEIGHTxFPS; 0 disables RGB", cli=True)
    args.add_arg("infra_profile", "", description="WIDTHxHEIGHTxFPS; 0 disables Infra", cli=True)
    args.add_arg("imu_profile", "", description="low: accel100/gyro200Hz; high: accel200/gyro400Hz; off; empty preserves YAML", cli=True)
    args.add_opaque_function(add_camera)
    return lut.LaunchDescription(args.get_launch_actions())
