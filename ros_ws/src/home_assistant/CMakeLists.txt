cmake_minimum_required(VERSION 3.22)
project(home_assistant)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TextToSpeech.srv"
  DEPENDENCIES std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_srvs)
ament_export_dependencies(std_msgs)

# Install Python modules
install(
  DIRECTORY home_assistant/
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
