#!/usr/bin/env python3
"""
sine_ik_node — the main R_sine driver (ROS 2, Python).

Precomputes the sine trajectory once with the analytical DLS IK, then streams it
to /joint_states in a seamless forward-and-back loop. Also publishes the *intended*
sine curve as a Path so RViz shows the goal the arm is tracing.

The drawing plane is fully configurable via ROS parameters, so the same node draws
on a vertical board or any other plane without code changes.

Publishes:
  /joint_states        sensor_msgs/JointState   (robot_state_publisher -> TF)
  /target_sine_path    nav_msgs/Path            (the intended sine, base_link frame)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from Rsine import kinematics as K
from Rsine.ik import DLSIKSolver
from Rsine.sine_path import DrawingPlane, sine_waypoints


class SineIKNode(Node):
    def __init__(self):
        super().__init__('sine_ik_node')

        # ---- configurable drawing plane / sine (ROS params) ----
        self.declare_parameter('board_x', 0.14)
        self.declare_parameter('z_center', 0.21)
        self.declare_parameter('amplitude', 0.05)
        self.declare_parameter('length', 0.20)
        self.declare_parameter('cycles', 2.0)
        self.declare_parameter('n_points', 120)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('frame_id', 'base_link')

        g = self.get_parameter
        board_x = g('board_x').value
        z_center = g('z_center').value
        amp = g('amplitude').value
        length = g('length').value
        cycles = g('cycles').value
        n_points = int(g('n_points').value)
        rate = g('rate_hz').value
        self.frame_id = g('frame_id').value

        # ---- precompute trajectory (analytical IK, reachable pen direction) ----
        plane = DrawingPlane.vertical_board(x=board_x, z_center=z_center)
        positions, _ = sine_waypoints(plane, amp, length, cycles, n_points)
        solver = DLSIKSolver(lam=0.04, max_iters=200, tol=1e-6)
        best = None
        for R_t in plane.candidate_orientations():
            Q, infos = solver.solve_trajectory(positions, R_t)
            worst = max(i['pos_err'] for i in infos)
            if best is None or worst < best[0]:
                best = (worst, R_t, Q)
        worst, _, Q = best
        # seamless loop: forward then back (drop shared endpoints)
        self.traj = np.vstack([Q, Q[-2:0:-1]])
        self.positions = positions

        # ---- publishers ----
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/target_sine_path', latched)
        self._publish_target_path()

        self.idx = 0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'R_sine driver up: {len(self.traj)} frames, max IK pos error '
            f'{worst*1000:.3f} mm, streaming at {rate:.0f} Hz.')

    def _publish_target_path(self):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for p in self.positions:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def on_timer(self):
        q = self.traj[self.idx]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = K.JOINT_NAMES
        msg.position = [float(v) for v in q]
        self.joint_pub.publish(msg)
        self.idx = (self.idx + 1) % len(self.traj)


def main(args=None):
    rclpy.init(args=args)
    node = SineIKNode()
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
