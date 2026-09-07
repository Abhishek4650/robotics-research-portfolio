"""RViz display for ARM-450. Run:  ros2 launch arm450_description display.launch.py"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('arm450_description')
    urdf = os.path.join(pkg, 'urdf', 'arm450_meshes.urdf')
    rviz = os.path.join(pkg, 'rviz', 'arm450.rviz')
    with open(urdf) as f:
        robot_desc = f.read()

    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='joint_state_publisher_gui sliders'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_desc}]),
        # sliders for manual posing; the sine node publishes to the same topic
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(gui)),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', rviz]),
    ])
