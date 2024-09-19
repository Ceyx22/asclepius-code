cmake_minimum_required(VERSION 3.5)
project(endeffector_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# Include directories
include_directories(
  include
  ${dynamixel_sdk_INCLUDE_DIRS}
)

# Add library
add_library(${PROJECT_NAME} SHARED
  src/endeffector_hardware.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  std_msgs
  pluginlib
  rclcpp
  dynamixel_sdk
)

ament_export_include_directories(
  include
)

# Export the plugin
pluginlib_export_plugin_description_file(hardware_interface endeffector_hardware.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)


install(
  FILES endeffector_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
