#!/usr/bin/env python3
"""
Render the locked 'paper' demo (front-centered at x=0.20, RAISED posture, pen writes
left-to-right on a horizontal table) as an RViz-styled GIF, for the presentation.

This mirrors exactly what the live RViz demo shows:
    ros2 launch mycobot_thesis thesis_draw.launch.py surface:=paper

Output: figures/paper_raised_draw.gif
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D              # noqa: F401,E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
from mycobot_thesis import robot as R                # noqa: E402
from mycobot_thesis import ik                        # noqa: E402

FIG = os.path.join(PKG, "figures")
os.makedirs(FIG, exist_ok=True)

# locked 'paper' config (mirrors PRESETS['paper'] in draw_node.py, kept ROS-free here)
origin = np.array([0.20, 0.0, 0.10])
u = np.array([0, 1, 0.])     # advance: left-right
v = np.array([1, 0, 0.])     # wave: up/down the page
pen = [0, 0, -1]             # pen points down
seed = np.radians([-67.7, -20.1, -137.3, 67.4, 0.0, -157.7])   # raised posture


def build():
    s = np.linspace(-0.06, 0.06, 100)
    w = 0.03 * np.sin(2 * np.pi * 2 * (s + 0.06) / 0.12)
    tgt = np.array([origin + si * u + wi * v for si, wi in zip(s, w)])
    Rt = R.target_orientation(pen, u)
    q = np.array(seed, float)
    Q = []
    for p in tgt:
        q, _ = ik.damped_least_squares(p, Rt, q, max_iters=400, tol=1e-8)
        Q.append(q.copy())
    return np.array(Q), tgt


def rect():
    return np.array([origin + a * u + b * v for a, b in
                     [(-0.07, -0.05), (0.07, -0.05), (0.07, 0.05), (-0.07, 0.05), (-0.07, -0.05)]])


def main():
    Q, tgt = build()
    r = rect()
    fig = plt.figure(figsize=(7, 6), facecolor="0.15")
    ax = fig.add_subplot(111, projection="3d")

    def draw(k):
        ax.clear()
        ax.set_facecolor("0.15")
        ax.set_xlim(0, 0.32); ax.set_ylim(-0.16, 0.16); ax.set_zlim(0, 0.42)
        ax.set_xlabel("x [m]", color="w"); ax.set_ylabel("y [m]", color="w")
        ax.set_zlabel("z [m]", color="w")
        ax.tick_params(colors="0.7")
        ax.view_init(elev=22, azim=-76)
        ax.plot(r[:, 0], r[:, 1], r[:, 2], color="0.5", lw=1)
        ax.plot(tgt[:, 0], tgt[:, 1], tgt[:, 2], "--", color="#33c833", lw=1, alpha=0.5)
        ax.plot(tgt[:k + 1, 0], tgt[:k + 1, 1], tgt[:k + 1, 2], color="#e02020", lw=3)
        pts = np.array([T[:3, 3] for T in R.fk_frames(Q[k])])
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color="#4da3ff", lw=3.5, ms=5)
        ax.scatter(*pts[-1], color="w", s=35)
        ax.set_title("myCobot 280 — writing a sine left-to-right (RViz demo)",
                     color="w", fontsize=11)
        return []

    anim = FuncAnimation(fig, draw, frames=len(Q), interval=60, blit=False)
    out = os.path.join(FIG, "paper_raised_draw.gif")
    anim.save(out, writer=PillowWriter(fps=20))
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    main()
