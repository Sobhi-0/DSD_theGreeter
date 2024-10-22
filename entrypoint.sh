cd /ros_ws
colcon build
source install/setup.bash
ros2 launch greeter_bringup sim.launch.py
echo "ROS 2 Greeter has started!"
