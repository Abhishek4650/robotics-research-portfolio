#!/usr/bin/env python3
"""
Bring up the thesis drawing demo in RViz:
  * robot_state_publisher  — TF from /joint_states + the arm URDF
  * draw_node              — streams the chosen surface's sine to /joint_states
  * path_tracer_node       — accumulates the real drawn line
  * rviz2                  — robot + target sine (green) + drawn line (red)

Usage:
  ros2 launch mycobot_thesis thesis_draw.launch.py                 # horizontal table
  ros2 launch mycobot_thesis thesis_draw.launch.py surface:=vertical
  ros2 launch mycobot_thesis thesis_draw.launch.py rviz:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mycobot_thesis')
    urdf = os.path.join(pkg, 'urdf', 'mycobot_280_arm.urdf')
    rviz_cfg = os.path.join(pkg, 'rviz', 'thesis.rviz')
    with open(urdf, 'r') as f:
        robot_description = f.read()

    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('surface', default_value='horizontal',
                              description="'horizontal' or 'vertical'"),
        DeclareLaunchArgument('amplitude', default_value='0.03'),
        DeclareLaunchArgument('cycles', default_value='2.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen', parameters=[{'robot_description': robot_description}])
    draw = Node(package='mycobot_thesis', executable='draw_node', output='screen',
                parameters=[{'surface': lc('surface'),
                             'amplitude': lc('amplitude'),
                             'cycles': lc('cycles')}])
    tracer = Node(package='mycobot_thesis', executable='path_tracer_node', output='screen')
    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
                condition=IfCondition(lc('rviz')), output='screen')

    return LaunchDescription(args + [rsp, draw, tracer, rviz])
