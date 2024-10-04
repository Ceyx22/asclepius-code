cmake_minimum_required(VERSION 3.8)
project(endeffector_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)


find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
# find_package(dynamixel_sdk REQUIRED)

# find dependencies
# find_package(backward_ros REQUIRED)
# find_package(ament_cmake REQUIRED)
# foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()
# foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()
# foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()

# Add library
add_library(${PROJECT_NAME} SHARED
  src/endeffector_controller.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
## COMPILE
# add_executable(send_trajectory reference_generator/send_trajectory.cpp)
# target_compile_features(dynamixel_hw PUBLIC cxx_std_17)
ament_export_include_directories(
  include
)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  control_msgs
  pluginlib
  rclcpp
  realtime_tools
)
pluginlib_export_plugin_description_file(controller_interface endeffector_controller.xml)
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
  FILES endeffector_controller.xml
  DESTINATION share/${PROJECT_NAME}
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
