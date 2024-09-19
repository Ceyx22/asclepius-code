cmake_minimum_required(VERSION 3.5)
project(uart_hardware_interface)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Include directories
include_directories(
  include
)

# Library
add_library(${PROJECT_NAME}
  src/uart_hardware_interface.cpp
  src/uart_comm.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Pluginlib export
pluginlib_export_plugin_description_file(hardware_interface uart_hardware_interface_plugin.xml)

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
  FILES uart_hardware_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
