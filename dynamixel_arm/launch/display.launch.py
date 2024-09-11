
import os
from ament_index_python.packages import get_package_share_directory as pkgdir
from launch import LaunchDescription

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_name = 'dynamixel_arm'
    file_subpath = 'urdf/acps_bot.urdf.xacro'
    
    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir(pkg_name), 'rviz/viewrobot.rviz')

    # Locate xacro file
    xacro_file = os.path.join(pkgdir(pkg_name),file_subpath)
    robot_description_config = xacro.process_file(xacro_file).toxml()


    joint_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen")
    
    robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config}],
            output="screen")
    
    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rvizcfg],
            output="screen")
    
#     transform_node = Node(
#             name       = 'transform_node', 
#             package    = 'dynamixel_arm',
#             executable = 'transform_node',
#             output     = 'screen') 
    
    return LaunchDescription([
        joint_node,
        robot_state_publisher_node,
        rviz_node,
        # transform_node,
    ])
