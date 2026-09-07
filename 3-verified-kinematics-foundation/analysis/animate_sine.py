#!/usr/bin/env python3
"""
Visual 'simulation' of the arm drawing the sine — no GUI / display needed.

Renders the arm moving through the precomputed trajectory while the drawn line
grows on the board, and writes:
  figures/sine_draw.gif       — animation you can open in VS Code
  figures/fig_draw_montage.png — 6-panel progression (shown inline in chat)

Run:  python3 analysis/animate_sine.py
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                 # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D           # noqa: F401,E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
from Rsine import kinematics as K                        # noqa: E402
from Rsine.ik import DLSIKSolver                          # noqa: E402
from Rsine.sine_path import DrawingPlane, sine_waypoints  # noqa: E402

FIG_DIR = os.path.join(PKG, "figures")
PLANE = DrawingPlane.vertical_board(x=0.14, z_center=0.21)


def build_trajectory(n=100):
    positions, _ = sine_waypoints(PLANE, 0.05, 0.20, 2.0, n)
    solver = DLSIKSolver(lam=0.04, max_iters=200, tol=1e-6)
    best = None
    for R_t in PLANE.candidate_orientations():
        Q, infos = solver.solve_trajectory(positions, R_t)
        w = max(i["pos_err"] for i in infos)
        if best is None or w < best[0]:
            best = (w, Q)
    return best[1], positions


def board_corners():
    o, u, v = PLANE.origin, PLANE.u, PLANE.v
    hw, hh = 0.12, 0.08
    return np.array([o + a * u + b * v for a, b in
                     [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh), (-hw, -hh)]])


def setup_ax(ax):
    ax.set_xlim(0, 0.30); ax.set_ylim(-0.15, 0.15); ax.set_zlim(0, 0.45)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.view_init(elev=16, azim=-72)


def draw_frame(ax, Q, target, k):
    ax.clear(); setup_ax(ax)
    c = board_corners()
    ax.plot(c[:, 0], c[:, 1], c[:, 2], color="0.6", lw=1)
    ax.plot(target[:, 0], target[:, 1], target[:, 2], "--", color="k", lw=1, alpha=0.4)
    drawn = target[:k + 1]
    ax.plot(drawn[:, 0], drawn[:, 1], drawn[:, 2], "-", color="#d62728", lw=2.5)
    pts = np.array([T[:3, 3] for T in K.fk_frames(Q[k])])
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color="#1f77b4", lw=3, ms=4)
    ax.scatter(*pts[-1], color="k", s=30)
    ax.set_title(f"myCobot 280 drawing a sine   (frame {k+1}/{len(Q)})")


def main():
    Q, target = build_trajectory(100)

    # montage
    figm = plt.figure(figsize=(15, 8))
    picks = np.linspace(0, len(Q) - 1, 6).astype(int)
    for i, k in enumerate(picks):
        ax = figm.add_subplot(2, 3, i + 1, projection="3d")
        draw_frame(ax, Q, target, k)
    figm.suptitle("Progression of the drawn sine (left→right, top→bottom)")
    figm.tight_layout()
    figm.savefig(os.path.join(FIG_DIR, "fig_draw_montage.png"), dpi=120, bbox_inches="tight")
    plt.close(figm)

    # gif
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    def update(k):
        draw_frame(ax, Q, target, k)
        return []

    anim = FuncAnimation(fig, update, frames=len(Q), interval=60, blit=False)
    gif = os.path.join(FIG_DIR, "sine_draw.gif")
    anim.save(gif, writer=PillowWriter(fps=20))
    plt.close(fig)
    print("wrote", os.path.join(FIG_DIR, "fig_draw_montage.png"))
    print("wrote", gif)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
