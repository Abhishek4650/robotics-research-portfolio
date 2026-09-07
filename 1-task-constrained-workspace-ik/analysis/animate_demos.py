#!/usr/bin/env python3
"""
Animate the myCobot 280 drawing a sine on a HORIZONTAL table (pen pointing down),
placed in the dexterous ring found by the feasibility analysis.

Writes:
  figures/horizontal_draw.gif        — animation (open in VS Code)
  figures/fig_horizontal_montage.png — 6-panel progression (shown inline)

Data flow (this is the analysis pipeline, no ROS):
  sine_points()  ->  a list of 3D targets on the table
  trace_Q()      ->  robot.jacobian + ik.damped_least_squares solve each target
                     (seed from the previous solution) -> joint angles Q
  robot.fk_frames(Q[k]) -> the arm's link positions to draw each animation frame
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

# horizontal table placement (from exp4_demos: dexterous ring, pen down)
ORIGIN = np.array([0.0, 0.18, 0.10])
U = np.array([0, 1, 0.])     # advance direction (along y)
V = np.array([1, 0, 0.])     # wave direction (along x)
PEN = [0, 0, -1]             # flange z points down, into the table
SEEDS = [np.zeros(6),
         np.array([0.0, -0.6, 0.8, 0.0, 0.6, 0.0]),
         np.array([0.0, 0.6, -0.8, 0.0, -0.6, 0.0]),
         np.array([-0.8, -0.5, 0.9, -0.3, -0.6, 0.0])]


def sine_points(amp=0.03, length=0.12, cycles=2.0, n=100):
    s = np.linspace(-length / 2, length / 2, n)
    w = amp * np.sin(2 * np.pi * cycles * (s + length / 2) / length)
    return np.array([ORIGIN + si * U + wi * V for si, wi in zip(s, w)])


def trace_Q(points):
    """Joint trajectory tracing the sine (multi-seed first point, then warm-started)."""
    Rt = R.target_orientation(PEN, U)
    # best starting config for the first waypoint
    q = min(SEEDS, key=lambda s: ik.damped_least_squares(points[0], Rt, s)[1]["pos_err"])
    q, _ = ik.damped_least_squares(points[0], Rt, q, max_iters=300, tol=1e-7)
    Q = []
    for p in points:
        q, _ = ik.damped_least_squares(p, Rt, q, max_iters=300, tol=1e-7)
        Q.append(q.copy())
    return np.array(Q)


def table_rect():
    hu, hv = 0.09, 0.05
    return np.array([ORIGIN + a * U + b * V for a, b in
                     [(-hu, -hv), (hu, -hv), (hu, hv), (-hu, hv), (-hu, -hv)]])


def setup(ax):
    ax.set_xlim(-0.05, 0.25); ax.set_ylim(0.0, 0.30); ax.set_zlim(0, 0.42)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.view_init(elev=24, azim=-62)


def draw(ax, Q, target, k):
    ax.clear(); setup(ax)
    c = table_rect()
    ax.plot(c[:, 0], c[:, 1], c[:, 2], color="0.6", lw=1)
    ax.plot(target[:, 0], target[:, 1], target[:, 2], "--", color="k", lw=1, alpha=0.4)
    drawn = target[:k + 1]
    ax.plot(drawn[:, 0], drawn[:, 1], drawn[:, 2], "-", color="#2ca02c", lw=2.6)
    pts = np.array([T[:3, 3] for T in R.fk_frames(Q[k])])
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color="#1f77b4", lw=3, ms=4)
    ax.scatter(*pts[-1], color="k", s=30)
    ax.set_title(f"myCobot 280 — sine on a horizontal table (frame {k+1}/{len(Q)})")


def main():
    target = sine_points()
    Q = trace_Q(target)
    err = max(np.linalg.norm(R.forward_kinematics(q)[:3, 3] - p)
              for q, p in zip(Q, target)) * 1000
    print(f"trajectory solved: {len(Q)} frames, max tracking error {err:.3f} mm")

    # montage
    figm = plt.figure(figsize=(15, 8))
    for i, k in enumerate(np.linspace(0, len(Q) - 1, 6).astype(int)):
        draw(figm.add_subplot(2, 3, i + 1, projection="3d"), Q, target, k)
    figm.suptitle("Progression — drawing a sine on a horizontal table")
    figm.tight_layout()
    figm.savefig(os.path.join(FIG, "fig_horizontal_montage.png"), dpi=120, bbox_inches="tight")
    plt.close(figm)

    # gif
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    anim = FuncAnimation(fig, lambda k: draw(ax, Q, target, k), frames=len(Q),
                         interval=60, blit=False)
    anim.save(os.path.join(FIG, "horizontal_draw.gif"), writer=PillowWriter(fps=20))
    plt.close(fig)
    print("wrote", os.path.join(FIG, "fig_horizontal_montage.png"))
    print("wrote", os.path.join(FIG, "horizontal_draw.gif"))


if __name__ == "__main__":
    main()
