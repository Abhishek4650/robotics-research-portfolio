#!/usr/bin/env python3
"""
arm450_path_tracer -- the path the tool face REALLY draws (Rsine's tracer):
TF ground -> tool_face from robot_state_publisher, accumulated into
/drawn_path (nav_msgs/Path) to compare with /target_sine_path in RViz.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class PathTracer(Node):
    def __init__(self):
        super().__init__("arm450_path_tracer")
        self.buf = Buffer(); self.tl = TransformListener(self.buf, self)
        self.pub = self.create_publisher(Path, "/drawn_path", 10)
        self.path = Path(); self.path.header.frame_id = "ground"
        self.create_timer(0.02, self.tick)

    def tick(self):
        try:
            tf = self.buf.lookup_transform("ground", "tool_face", rclpy.time.Time())
        except TransformException:
            return
        ps = PoseStamped(); ps.header = tf.header
        t = tf.transform.translation
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = t.x, t.y, t.z
        ps.pose.orientation = tf.transform.rotation
        self.path.poses.append(ps)
        self.path.poses = self.path.poses[-600:]
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path)


def main():
    rclpy.init()
    rclpy.spin(PathTracer())


if __name__ == "__main__":
    main()
