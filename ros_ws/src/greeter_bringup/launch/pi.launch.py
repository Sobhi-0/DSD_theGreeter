from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("greeter_bringup"), "launch"
                        ),
                        "/camera_pipeline.launch.py",
                    ]
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("greeter_description"), 
                            "launch",
                        ),
                        "/robot_description.launch.py",
                    ]
                )
            ),
            Node(
                package="sphero_rvr",
                executable="sphero_rvr_node",
                name="sphero_rvr_node",
                output="screen",
                parameters=[
                    {"wheel_base_in_m": 0.18}
                ]
            ),
            Node(
                package="odom_publisher",
                executable="odom_publisher_node",
                name="odom_publisher_node",
                output="screen",
                parameters=[
                    {"encoder_topic": "/encoder_ticks"},
                    {"odom_topic": "/odom"},
                    {"wheel_base": 0.18},
                    {"wheel_radius": 0.065},
                    {"ticks_per_rev": 890},
                ]
            ),
        ]
    )
