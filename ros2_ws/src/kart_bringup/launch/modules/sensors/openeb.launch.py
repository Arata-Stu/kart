"""Optional SilkyEvCam driver; native RAW starts only on bag-manager request."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def add_driver(args):
    config = lu.get_path("kart_bringup", "config/sensors/openeb.yaml")
    return [
        lu.log_info(f"OpenEB config: {config}; RAW auto-start disabled"),
        lu.load_composable_nodes(
            args.container_name,
            [
                lut.ComposableNode(
                    package="openeb_ros2",
                    plugin="openeb_ros2::DriverComponent",
                    name="event_camera_driver",
                    namespace="event_camera",
                    parameters=[str(config)],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            ],
        ),
    ]


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg("container_name", description="Existing sensor container", cli=True)
    args.add_opaque_function(add_driver)
    return lut.LaunchDescription(args.get_launch_actions())
