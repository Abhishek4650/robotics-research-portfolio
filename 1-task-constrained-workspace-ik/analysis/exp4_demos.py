#!/usr/bin/env python3
"""
Experiment 4 — sine tracing demonstrations on two surfaces, placed using the
feasibility/manipulability analysis:
    * a FRONT-facing VERTICAL board (pen points away from the robot, +x) — placed
      in the outer region where the front direction is feasible;
    * a HORIZONTAL table (pen points down, -z) — placed in the dexterous ring.

Each waypoint is solved with DLS IK seeded from the previous solution; we report
the tracking error and render both drawn sines in 3D.

Run:  python3 analysis/exp4_demos.py -> figures/fig_demos.png
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402
from mpl_toolkits.mplot3d import Axes3D              # noqa: F401,E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
from mycobot_thesis import robot as R                # noqa: E402
from mycobot_thesis import ik                        # noqa: E402

FIG = os.path.join(PKG, "figures")
os.makedirs(FIG, exist_ok=True)


def sine_points(origin, u, v, amp, length, cycles, n=120):
    u = np.asarray(u, float); v = np.asarray(v, float)
    s = np.linspace(-length / 2, length / 2, n)
    w = amp * np.sin(2 * np.pi * cycles * (s + length / 2) / length)
    return np.array([origin + si * u + wi * v for si, wi in zip(s, w)])


SEEDS = [np.zeros(6),
         np.array([0.0, -0.6, 0.8, 0.0, 0.6, 0.0]),
         np.array([0.0, 0.6, -0.8, 0.0, -0.6, 0.0]),
         np.array([0.8, -0.5, 0.9, 0.3, -0.6, 0.0]),
         np.array([-0.8, -0.5, 0.9, -0.3, -0.6, 0.0])]


def _solve_multiseed(p, Rt):
    """Best config reaching (p, Rt) over several seeds."""
    best, best_err = None, np.inf
    for q0 in SEEDS:
        q, info = ik.damped_least_squares(p, Rt, q0, max_iters=300, tol=1e-7)
        err = info["pos_err"] + 0.05 * info["ori_err"]
        if err < best_err:
            best, best_err = q, err
    return best


def trace(points, pen_axis, advance):
    """First waypoint via multi-seed; the rest seeded from the previous solution."""
    Rt = R.target_orientation(pen_axis, advance)
    q = _solve_multiseed(points[0], Rt)
    ee, worst = [], 0.0
    for p in points:
        q, info = ik.damped_least_squares(p, Rt, q, max_iters=300, tol=1e-7)
        worst = max(worst, info["pos_err"])
        ee.append(R.forward_kinematics(q)[:3, 3])
    return np.array(ee), worst * 1000, q


def find_placement(centers, u, v, pen_axis, advance, amp=0.025, length=0.10):
    """Pick the center whose small sine tracks best (uses the feasibility idea)."""
    best = None
    for o in centers:
        pts = sine_points(np.asarray(o, float), u, v, amp, length, 2.0, n=80)
        ee, err, _ = trace(pts, pen_axis, advance)
        if best is None or err < best[0]:
            best = (err, np.asarray(o, float), pts, ee)
    return best   # (err_mm, origin, points, ee)


def board_rect(origin, u, v, hu, hv):
    u = np.asarray(u, float); v = np.asarray(v, float)
    return np.array([origin + a * u + b * v for a, b in
                     [(-hu, -hv), (hu, -hv), (hu, hv), (-hu, hv), (-hu, -hv)]])


def main():
    # ---- FRONT-facing vertical board (pen +x, away from robot), outer region ----
    vu, vv = np.array([0, 1, 0.]), np.array([0, 0, 1.])   # advance y, wave z
    v_centers = [(x, 0.0, z) for x in (0.19, 0.21, 0.23) for z in (0.12, 0.16, 0.20)]
    verr, vo, vpts, vee = find_placement(v_centers, vu, vv,
                                         pen_axis=[1, 0, 0], advance=[0, 1, 0])

    # ---- horizontal table (pen -z, down) in the dexterous ring ----
    hu, hv = np.array([0, 1, 0.]), np.array([1, 0, 0.])   # advance y, wave x
    h_centers = [(0.0, 0.18, 0.10), (0.0, 0.20, 0.10), (0.0, 0.22, 0.12),
                 (0.15, 0.12, 0.10), (0.12, 0.15, 0.10), (0.18, 0.0, 0.10),
                 (0.0, 0.18, 0.06)]
    herr, ho, hpts, hee = find_placement(h_centers, hu, hv,
                                         pen_axis=[0, 0, -1], advance=[0, 1, 0])

    print(f"FRONT vertical board  : center {vo.round(3)}  max tracking error {verr:.3f} mm")
    print(f"HORIZONTAL table      : center {ho.round(3)}  max tracking error {herr:.3f} mm")

    _plot(vo, vu, vv, vpts, vee, verr, ho, hu, hv, hpts, hee, herr)
    print("figure ->", os.path.join(FIG, "fig_demos.png"))


def _plot(vo, vu, vv, vpts, vee, verr, ho, hu, hv, hpts, hee, herr):
    fig = plt.figure(figsize=(14, 6))
    home = np.array([T[:3, 3] for T in R.fk_frames(np.zeros(6))])

    def arm(ax):
        ax.plot(home[:, 0], home[:, 1], home[:, 2], "-o", color="#1f77b4",
                lw=2, ms=3, alpha=0.4, label="arm (home)")

    ax1 = fig.add_subplot(121, projection="3d")
    rect = board_rect(vo, vu, vv, 0.08, 0.06)
    ax1.plot(rect[:, 0], rect[:, 1], rect[:, 2], color="0.6", lw=1)
    ax1.plot(vpts[:, 0], vpts[:, 1], vpts[:, 2], "k--", lw=1, label="target")
    ax1.plot(vee[:, 0], vee[:, 1], vee[:, 2], color="#d62728", lw=2.2, label="drawn")
    arm(ax1)
    ax1.set_title(f"(a) FRONT-facing vertical board — pen points away\n"
                  f"max error {verr:.2f} mm")
    ax1.set_xlabel("x [m]"); ax1.set_ylabel("y [m]"); ax1.set_zlabel("z [m]")
    ax1.legend(loc="upper left", fontsize=8); ax1.view_init(elev=18, azim=-60)

    ax2 = fig.add_subplot(122, projection="3d")
    rect = board_rect(ho, hu, hv, 0.08, 0.06)
    ax2.plot(rect[:, 0], rect[:, 1], rect[:, 2], color="0.6", lw=1)
    ax2.plot(hpts[:, 0], hpts[:, 1], hpts[:, 2], "k--", lw=1, label="target")
    ax2.plot(hee[:, 0], hee[:, 1], hee[:, 2], color="#2ca02c", lw=2.2, label="drawn")
    arm(ax2)
    ax2.set_title(f"(b) HORIZONTAL table — pen points down\n"
                  f"max error {herr:.2f} mm")
    ax2.set_xlabel("x [m]"); ax2.set_ylabel("y [m]"); ax2.set_zlabel("z [m]")
    ax2.legend(loc="upper left", fontsize=8); ax2.view_init(elev=26, azim=-60)

    fig.suptitle("Sine tracing on two surfaces, placed by the feasibility analysis",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_demos.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
