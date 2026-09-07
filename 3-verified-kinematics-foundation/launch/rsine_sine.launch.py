#!/usr/bin/env python3
"""
Bring up the whole R_sine demo:
  * robot_state_publisher  — publishes TF from our /joint_states + the arm URDF
  * sine_ik_node           — streams the sine trajectory to /joint_states
  * path_tracer_node       — accumulates the real drawn line from TF
  * rviz2                  — visualizes robot + target sine + drawn line

Usage:
  ros2 launch Rsine rsine_sine.launch.py
  ros2 launch Rsine rsine_sine.launch.py board_x:=0.13 amplitude:=0.04 rviz:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('Rsine')
    urdf_path = os.path.join(pkg, 'urdf', 'mycobot_280_arm.urdf')
    rviz_path = os.path.join(pkg, 'rviz', 'rsine.rviz')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    args = [
        DeclareLaunchArgument('board_x', default_value='0.14'),
        DeclareLaunchArgument('z_center', default_value='0.21'),
        DeclareLaunchArgument('amplitude', default_value='0.05'),
        DeclareLaunchArgument('length', default_value='0.20'),
        DeclareLaunchArgument('cycles', default_value='2.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]
    lc = LaunchConfiguration

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    sine_ik = Node(
        package='Rsine',
        executable='sine_ik_node',
        output='screen',
        parameters=[{
            'board_x': lc('board_x'),
            'z_center': lc('z_center'),
            'amplitude': lc('amplitude'),
            'length': lc('length'),
            'cycles': lc('cycles'),
        }],
    )

    path_tracer = Node(
        package='Rsine',
        executable='path_tracer_node',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        condition=IfCondition(lc('rviz')),
        output='screen',
    )

    return LaunchDescription(args + [robot_state_publisher, sine_ik, path_tracer, rviz])
