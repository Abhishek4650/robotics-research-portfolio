#!/usr/bin/env python3
"""Headless check of the ROS 2 demo: listen to /drawn_path (from TF of the
real URDF chain) for a few seconds and measure each drawn point's distance to
the planned sine (/target_sine_path). Run while sine_demo.launch.py runs."""
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path


class Check(Node):
    def __init__(self):
        super().__init__("arm450_demo_check")
        self.target, self.drawn = None, None
        self.create_subscription(Path, "/target_sine_path", lambda m: setattr(self, "target", m), 10)
        self.create_subscription(Path, "/drawn_path", lambda m: setattr(self, "drawn", m), 10)


def seg_dist(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]; ap = [p[i] - a[i] for i in range(3)]
    L = sum(v * v for v in ab)
    t = 0.0 if L == 0 else max(0.0, min(1.0, sum(ap[i] * ab[i] for i in range(3)) / L))
    return math.dist(p, [a[i] + t * ab[i] for i in range(3)])


def main():
    rclpy.init(); n = Check()
    end = n.get_clock().now().nanoseconds + 8e9
    while rclpy.ok() and n.get_clock().now().nanoseconds < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.target or not n.drawn:
        print("no paths received"); sys.exit(1)
    T = [(q.pose.position.x, q.pose.position.y, q.pose.position.z) for q in n.target.poses]
    D = [(q.pose.position.x, q.pose.position.y, q.pose.position.z) for q in n.drawn.poses]
    worst = max(min(seg_dist(p, T[i], T[i + 1]) for i in range(len(T) - 1)) for p in D)
    print("ROS 2 demo: %d drawn points (TF ground -> tool_face), farthest from the planned sine: %.4f mm" % (len(D), 1000 * worst))


if __name__ == "__main__":
    main()
