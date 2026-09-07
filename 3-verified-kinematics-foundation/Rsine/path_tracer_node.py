#!/usr/bin/env python3
"""
path_tracer_node — draws the ACTUAL end-effector trail (ROS 2, Python).

Listens to TF (base_link -> link6_flange, published by robot_state_publisher from
our /joint_states) and accumulates the tool position into a Path, so RViz shows the
real drawn line and you can compare it to /target_sine_path.

Publishes:
  /drawn_path   nav_msgs/Path   (accumulated end-effector positions)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class PathTracerNode(Node):
    def __init__(self):
        super().__init__('path_tracer_node')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tool_frame', 'link6_flange')
        self.declare_parameter('max_points', 600)
        self.base_frame = self.get_parameter('base_frame').value
        self.tool_frame = self.get_parameter('tool_frame').value
        self.max_points = int(self.get_parameter('max_points').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path_pub = self.create_publisher(Path, '/drawn_path', 10)
        self.path = Path()
        self.path.header.frame_id = self.base_frame

        self.timer = self.create_timer(0.02, self.on_timer)   # 50 Hz
        self.get_logger().info(
            f'Path tracer up: {self.base_frame} -> {self.tool_frame}.')

    def on_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.tool_frame, rclpy.time.Time())
        except TransformException:
            return
        ps = PoseStamped()
        ps.header = tf.header
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.position.z = tf.transform.translation.z
        ps.pose.orientation = tf.transform.rotation
        self.path.poses.append(ps)
        if len(self.path.poses) > self.max_points:
            self.path.poses.pop(0)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathTracerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
