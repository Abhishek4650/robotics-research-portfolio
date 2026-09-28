#!/usr/bin/env python3
"""
arm450_sine_player -- plays a precomputed sine trajectory on the ARM-450 URDF
(ROS 2, Python; the Rsine pattern: solve once, stream /joint_states).

The trajectory is the one analysis/s7_sine_demo.py solved with the closed-form
IK (servo angles in degrees, sine_<which>_trajectory.csv).

Publishes:
  /joint_states       sensor_msgs/JointState  (J1_base_yaw .. J6_tool_roll, rad), looped
  /target_sine_path   nav_msgs/Path           (the planned tool-face path, frame 'ground', m)
Parameters: which (vertical | table), rate_hz (30.0)
"""
import csv
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

KIN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAMES = ["J1_base_yaw", "J2_shoulder", "J3_elbow", "J4_forearm_roll", "J5_wrist_pitch", "J6_tool_roll"]


class SinePlayer(Node):
    def __init__(self):
        super().__init__("arm450_sine_player")
        self.declare_parameter("which", "vertical")
        self.declare_parameter("rate_hz", 30.0)
        which = self.get_parameter("which").value
        rows = list(csv.DictReader(open(os.path.join(KIN, "sine_%s_trajectory.csv" % which))))
        self.q = [[math.radians(float(r["q%d_deg" % i])) for i in range(1, 7)] for r in rows]
        self.p = [[float(r[k]) / 1000.0 for k in ("x_mm", "y_mm", "z_mm")] for r in rows]
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.path_pub = self.create_publisher(Path, "/target_sine_path", 10)
        self.k = 0
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)
        self.create_timer(1.0, self.publish_target)
        self.get_logger().info("playing %s sine: %d points" % (which, len(self.q)))

    def tick(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = NAMES
        m.position = self.q[self.k]
        self.js_pub.publish(m)
        self.k = (self.k + 1) % len(self.q)

    def publish_target(self):
        path = Path(); path.header.frame_id = "ground"; path.header.stamp = self.get_clock().now().to_msg()
        for x, y, z in self.p:
            ps = PoseStamped(); ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)


def main():
    rclpy.init()
    rclpy.spin(SinePlayer())


if __name__ == "__main__":
    main()
