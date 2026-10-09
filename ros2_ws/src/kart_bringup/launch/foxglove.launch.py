"""Start the host-wide Foxglove Bridge once, independently of vehicle/localization."""

import isaac_ros_launch_utils as lu
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            lu.include(
                "kart_bringup",
                "launch/modules/visualization/foxglove.launch.py",
                scoped=True,
                forwarding=False,
            ),
        ]
    )
