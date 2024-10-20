from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    apriltag_config = os.path.join(
        get_package_share_directory("greeter_bringup"), "config", "apriltag_config.yaml"
    )

    ld = LaunchDescription()
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("greeter_description"), "launch"
                    ),
                    "/robot_description.launch.py",
                ]
            )
        )
    )
    ld.add_action(
        Node(
            package="image_proc",
            executable="image_proc",
            name="apriltag_node",
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag_node",
            output="screen",
            parameters=[apriltag_config],
        )
    )
    return ld
