from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sphero_rvr_driver',
            executable='sphero_rvr_node',
            name='sphero_rvr_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['/home/sobhi/DSD_theGreeter/ros_ws/src/greeter_description/urdf/robot.urdf']
        ),
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2_bringup',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])