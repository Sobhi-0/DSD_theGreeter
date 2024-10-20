from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory
import xacro


def generate_launch_description():
    # urdf_file_name = "sphero_rvr.urdf.xacro"
    urdf_file_name = "robot.urdf"
    urdf_path = os.path.join(
        get_package_share_directory("greeter_description"), "urdf", urdf_file_name
    )

    with open(urdf_path, "r") as infp:
        # robot_desc = xacro.process_file(
        #     urdf_path, mappings={"input": "new_input"}
        # ).toxml()
        robot_desc = infp.read()

    return LaunchDescription(
        [
            # robot state publisher for 3d model with argument for sensors.urdf
            # Node(
            #     package="joint_state_publisher_gui",
            #     executable="joint_state_publisher_gui",
            #     name="joint_state_publisher_gui",
            #     output="screen",
            #     parameters=[robot_desc],
            # ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_desc},
                ],
                arguments=[urdf_path],
            ),
        ]
    )
