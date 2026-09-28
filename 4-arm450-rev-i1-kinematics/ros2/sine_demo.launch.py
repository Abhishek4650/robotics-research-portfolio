"""ARM-450 rev I.1 sine demo in RViz (the Rsine pattern, no build needed):
    ros2 launch sine_demo.launch.py which:=vertical   (or which:=table, rviz:=false)
robot_state_publisher (the release URDF + a fixed 'tool_face' frame on the
flange face), the sine player (/joint_states), the path tracer (/drawn_path)."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
URDF_DIR = os.path.join(HERE, "..", "..", "EDITABLE_CAD", "urdf")
TOOL_FACE = 0.0235          # flange link frame (J6 servo point) -> tool-flange face, m (CAD: 518.458 - 494.958)


def generate_launch_description():
    urdf = open(os.path.join(URDF_DIR, "arm450.urdf")).read().replace(
        "package://arm450_rev_i_view/meshes/", "file://" + os.path.realpath(os.path.join(URDF_DIR, "meshes")) + "/")
    urdf = urdf.replace("</robot>", '  <link name="tool_face"/>\n  <joint name="tool_face_fixed" type="fixed">\n'
                        '    <parent link="flange"/><child link="tool_face"/>\n'
                        '    <origin xyz="0 0 %.4f" rpy="0 0 0"/>\n  </joint>\n</robot>' % TOOL_FACE)
    which = LaunchConfiguration("which")
    return LaunchDescription([
        DeclareLaunchArgument("which", default_value="vertical"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        ExecuteProcess(cmd=["python3", os.path.join(HERE, "arm450_sine_player.py"), "--ros-args", "-p",
                            ["which:=", which]], output="screen"),
        ExecuteProcess(cmd=["python3", os.path.join(HERE, "arm450_path_tracer.py")], output="screen"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(HERE, "arm450_sine.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
