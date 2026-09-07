#!/usr/bin/env python3
"""
Graphs for the kinematics stage:
  * fig_home_pose.png   — the arm's link frames at the home configuration (3D).
  * fig_workspace.png   — reachable end-effector cloud (random valid joint configs).

Run:  python3 analysis/plot_kinematics.py
Figures are written to ../figures/.
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")            # headless: save files, no window
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401,E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Rsine import kinematics as K   # noqa: E402

FIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "figures")
os.makedirs(FIG_DIR, exist_ok=True)


def plot_home_pose():
    frames = K.fk_frames(np.zeros(6))
    pts = np.array([T[:3, 3] for T in frames])   # 7 points base..EE

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    # links
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color="#1f77b4", lw=3, ms=6,
            label="links / joints")
    # small triad at each frame to show orientation
    for T in frames:
        o = T[:3, 3]
        for k, c in zip(range(3), ("r", "g", "b")):
            axis = T[:3, k] * 0.03
            ax.plot([o[0], o[0] + axis[0]], [o[1], o[1] + axis[1]],
                    [o[2], o[2] + axis[2]], color=c, lw=1.5)
    ax.scatter(*pts[-1], color="k", s=60, label="end-effector")
    ax.set_title("myCobot 280 — home configuration (q = 0)\n"
                 f"EE at ({pts[-1,0]:.3f}, {pts[-1,1]:.3f}, {pts[-1,2]:.3f}) m")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    _equal_aspect(ax, pts)
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=9, framealpha=0.9)
    out = os.path.join(FIG_DIR, "fig_home_pose.png")
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    return out


def plot_workspace(n=8000, seed=0):
    rng = np.random.default_rng(seed)
    P = np.empty((n, 3))
    for i in range(n):
        q = rng.uniform(K.JOINT_LIMITS[:, 0], K.JOINT_LIMITS[:, 1])
        P[i] = K.ee_position(q)

    fig = plt.figure(figsize=(11, 4.5))
    ax1 = fig.add_subplot(121, projection="3d")
    ax1.scatter(P[:, 0], P[:, 1], P[:, 2], s=2, c=P[:, 2], cmap="viridis", alpha=0.4)
    ax1.set_title(f"Reachable workspace ({n} samples)")
    ax1.set_xlabel("x [m]"); ax1.set_ylabel("y [m]"); ax1.set_zlabel("z [m]")

    ax2 = fig.add_subplot(122)
    sc = ax2.scatter(P[:, 0], P[:, 2], s=2, c=P[:, 1], cmap="coolwarm", alpha=0.4)
    ax2.set_title("Side view (x–z)")
    ax2.set_xlabel("x [m]"); ax2.set_ylabel("z [m]"); ax2.set_aspect("equal")
    ax2.grid(alpha=0.3)
    fig.colorbar(sc, ax=ax2, label="y [m]")
    out = os.path.join(FIG_DIR, "fig_workspace.png")
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    return out


def _equal_aspect(ax, pts):
    c = pts.mean(axis=0)
    r = np.abs(pts - c).max() + 0.05
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)


if __name__ == "__main__":
    print("wrote", plot_home_pose())
    print("wrote", plot_workspace())
