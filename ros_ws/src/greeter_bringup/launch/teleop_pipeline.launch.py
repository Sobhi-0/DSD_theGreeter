from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy_linux",
                executable="joy_linux_node",
                name="joy_linux_node",
                output="screen",
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_node",
                output="screen",
                parameters=[
                    {"enable_button": 0},
                    {"axis_linear.x": 1},
                    {"axis_angular.yaw": 0},
                    {"scale_linear.x": 1.0},
                    {"scale_angular.yaw": 1.0},
                    {"enable_turbo_button": 1}
                ],
            ),
            Node(
                package="sphero_rvr",
                executable="sphero_rvr_node",
                name="sphero_rvr_node",
                output="screen",
                parameters=[
                    {"wheel_base_in_m": 0.2}
                ]
            )
        ]
    )
