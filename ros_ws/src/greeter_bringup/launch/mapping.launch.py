from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    camera_frame = "camera_frame"
    apriltag_config = os.path.join(
        get_package_share_directory("greeter_bringup"), "config", "apriltag_config.yaml"
    )

    ld = LaunchDescription()
    ld.add_action(
        Node(
            package="nav2_map_server",
            executable="map_saver_cli",
            output="screen",
            remappings=[("/image", "/image_raw")]
        )
    )
    return ld
