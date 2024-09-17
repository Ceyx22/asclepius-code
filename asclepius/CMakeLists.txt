cmake_minimum_required(VERSION 3.8)
project(asclepius)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rviz2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


include_directories(include)
# add_executable(ik src/KDLInverseKinematicsNode.cpp)
add_executable(kinPub src/KinematicPublisher.cpp)
# ament_target_dependencies(ik rclcpp geometry_msgs urdf kdl_parser std_msgs trajectory_msgs control_msgs rclcpp_action tf2_kdl tf2_geometry_msgs)
ament_target_dependencies(kinPub rclcpp geometry_msgs urdf kdl_parser std_msgs sensor_msgs trajectory_msgs control_msgs rclcpp_action)


install(DIRECTORY
  config
  meshes
  description
  launch
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(TARGETS
  kinPub
  DESTINATION lib/${PROJECT_NAME}
  )
  
install(PROGRAMS    
  scripts/KinematicSol.py
  scripts/TransformationHelp.py
  DESTINATION lib/${PROJECT_NAME})
# Install things for simulation


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
