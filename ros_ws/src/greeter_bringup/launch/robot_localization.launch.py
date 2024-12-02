from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    robot_localization_config_path = os.path.join(get_package_share_directory("greeter_bringup"), "config", "robot_localization_config.yaml")
    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_localization_node",
                name="ekf_localization_node",
                output="screen",
                parameters=[robot_localization_config_path],
            ),
        ]
    )

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    robot_localization_config_path = os.path.join(get_package_share_directory("greeter_bringup"), "config", "robot_localization_config.yaml")
    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_localization_node",
                name="ekf_localization_node",
                output="screen",
                parameters=[robot_localization_config_path],
            ),
        ]
    )
