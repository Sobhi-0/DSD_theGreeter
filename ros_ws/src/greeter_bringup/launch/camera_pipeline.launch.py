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
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam_node_exe",
            output="screen",
            parameters=[
                {"video_device": "/dev/video0"},
                {
                    "camera_info_url": "file://"
                    + os.path.join(
                        get_package_share_directory("greeter_bringup"),
                        "config",
                        "camera_info.yaml",
                    )
                },
            ],
        )
    )
    ld.add_action(
        Node(
            package="image_proc",
            executable="rectify_node",
            output="screen",
            remappings=[("/image", "/image_raw")]
        )
    )
    ld.add_action(
        Node(
            package="image_proc",
            executable="debayer_node",
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
            remappings=[("detections", "/apriltag_detections"),("image_rect", "/image_raw")],
        )
    )
    # ld.add_action(
    #     Node(
    #         package="apriltag_merge",
    #         executable="apriltag_merge_node",
    #         name="apriltag_merge_node",
    #         output="screen",
    #         parameters=[{"allowed_ids": [0]}, {"camera_frame": camera_frame}],
    #     )
    # )
    return ld
