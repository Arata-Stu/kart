"""Notebook offline map/localization visualization."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from launch_ros.actions import Node


def generate_launch_description():
    return lut.LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    str(
                        lu.get_path(
                            "kart_bringup", "config/evaluation/localization.rviz"
                        )
                    ),
                ],
                parameters=[
                    str(lu.get_path("kart_bringup", "config/evaluation/rviz.yaml"))
                ],
                output="screen",
            )
        ]
    )
