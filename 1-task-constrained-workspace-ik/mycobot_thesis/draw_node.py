#!/usr/bin/env python3
"""
draw_node — thesis demo driver (ROS 2, Python).

Precomputes a sine trajectory on a chosen surface using the thesis library
(robot.py + ik.py), then streams it to /joint_states in a seamless loop. Choose the
surface with the `surface` parameter:

    horizontal  -> flat table, pen points down     (default)
    vertical    -> front-facing board, pen points away from the robot

Publishes:
  /joint_states       sensor_msgs/JointState   (robot_state_publisher -> TF)
  /target_sine_path   nav_msgs/Path            (the intended sine, base_link frame)

This is the ROS wrapper around the same math the analysis scripts use — it turns
the thesis from an offline study (matplotlib) into a live RViz demo.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from mycobot_thesis import robot as R
from mycobot_thesis import ik

# A "raised" IK branch for the paper task: the SAME end-effector poses, but a
# higher-elbow arm posture (upper arm ~0.17 m vs ~0.10 m) — the arm opened more
# toward the z-axis while the pen traces the identical sine on the table.
RAISED_PAPER_SEED = np.radians([-67.7, -20.1, -137.3, 67.4, 0.0, -157.7])

# surface presets found by the feasibility analysis (exp4_demos).
# Entry = (origin, advance u, wave v, pen axis[, preferred IK seed]).
PRESETS = {
    # side-placed horizontal table (sweeps sideways, off to the robot's left):
    'horizontal': (np.array([0.0, 0.18, 0.10]), [0, 1, 0], [1, 0, 0], [0, 0, -1]),
    # 'paper': horizontal table IN FRONT of the robot, pen writes LEFT-TO-RIGHT
    # (advance along y = left/right, wave along x = up/down the page), like writing
    # on a sheet of paper laid in front of you. Uses the RAISED posture.
    'paper':      (np.array([0.20, 0.0, 0.10]), [0, 1, 0], [1, 0, 0], [0, 0, -1],
                   RAISED_PAPER_SEED),
    # front-facing vertical board (pen points away from the robot):
    'vertical':   (np.array([0.21, 0.0, 0.12]), [0, 1, 0], [0, 0, 1], [1, 0, 0]),
}
SEEDS = [np.zeros(6),
         np.array([0.0, -0.6, 0.8, 0.0, 0.6, 0.0]),
         np.array([0.0, 0.6, -0.8, 0.0, -0.6, 0.0]),
         np.array([-0.8, -0.5, 0.9, -0.3, -0.6, 0.0])]


class DrawNode(Node):
    def __init__(self):
        super().__init__('draw_node')
        self.declare_parameter('surface', 'horizontal')
        self.declare_parameter('amplitude', 0.03)
        self.declare_parameter('length', 0.12)
        self.declare_parameter('cycles', 2.0)
        self.declare_parameter('n_points', 100)
        self.declare_parameter('rate_hz', 30.0)
        self.frame_id = 'base_link'

        surface = self.get_parameter('surface').value
        if surface not in PRESETS:
            self.get_logger().warn(f"unknown surface '{surface}', using 'horizontal'")
            surface = 'horizontal'
        entry = PRESETS[surface]
        origin, u, v, pen = entry[:4]
        preferred_seed = entry[4] if len(entry) > 4 else None
        amp = self.get_parameter('amplitude').value
        length = self.get_parameter('length').value
        cycles = self.get_parameter('cycles').value
        n = int(self.get_parameter('n_points').value)
        rate = self.get_parameter('rate_hz').value

        # --- generate sine targets on the surface ---
        u = np.asarray(u, float); v = np.asarray(v, float)
        s = np.linspace(-length / 2, length / 2, n)
        w = amp * np.sin(2 * np.pi * cycles * (s + length / 2) / length)
        self.positions = np.array([origin + si * u + wi * v for si, wi in zip(s, w)])

        # --- solve IK for every target ---
        # If the preset names a preferred posture (e.g. the raised 'paper' branch),
        # seed from it; otherwise pick the best of several seeds for the first point.
        Rt = R.target_orientation(pen, u)
        if preferred_seed is not None:
            q = np.array(preferred_seed, float)
        else:
            q = min(SEEDS, key=lambda sd: ik.damped_least_squares(self.positions[0], Rt, sd)[1]['pos_err'])
        Q, worst = [], 0.0
        for p in self.positions:
            q, info = ik.damped_least_squares(p, Rt, q, max_iters=300, tol=1e-7)
            worst = max(worst, info['pos_err'])
            Q.append(q.copy())
        Q = np.array(Q)
        self.traj = np.vstack([Q, Q[-2:0:-1]])   # seamless forward-and-back loop

        # --- publishers ---
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/target_sine_path', latched)
        self._publish_target_path()

        self.idx = 0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f"thesis draw_node up: surface='{surface}', {len(self.traj)} frames, "
            f"max IK error {worst*1000:.3f} mm, streaming at {rate:.0f} Hz.")

    def _publish_target_path(self):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for p in self.positions:
            ps = PoseStamped(); ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def on_timer(self):
        q = self.traj[self.idx]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = R.JOINT_NAMES
        msg.position = [float(v) for v in q]
        self.joint_pub.publish(msg)
        self.idx = (self.idx + 1) % len(self.traj)


def main(args=None):
    rclpy.init(args=args)
    node = DrawNode()
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
