#!/usr/bin/env python3


import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Launch arguments
    # use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    project_path = os.path.join(
        get_package_share_directory('asclepius'))

    xacro_file = os.path.join(project_path,
                              'description',
                              'asclepius.urdf.xacro')
    rvizcfg = os.path.join(get_package_share_directory('asclepius'), 'rviz/viewrobot.rviz')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    controller_config = os.path.join(
        project_path, "config", "robot_controller.yaml"
    )

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                params, controller_config],
            # prefix=['xterm -e gdb -ex run --args'],
            # prefix=['gdbserver localhost:3000'],
            arguments=['--ros-args', '--log-level', 'DEBUG'],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                params],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rvizcfg],
            output="screen",
        ),

    ])