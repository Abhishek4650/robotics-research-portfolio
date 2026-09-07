#!/usr/bin/env python3
"""
Reachable-workspace analysis for the myCobot 280 — generated AND verified.

"Reachable workspace" = every end-effector position the arm can attain with some
joint configuration inside the URDF limits. We sample it by forward kinematics over
random valid joint vectors, then VERIFY the cloud three independent ways:

  1. ikpy cross-check : ikpy's FK on the same joints must match ours (independent).
  2. reach bound      : no sampled point may exceed the arm's geometric max reach.
  3. limit sanity     : sampling only uses joint vectors inside the URDF limits.

We also overlay the drawing board + sine so it is visually clear the demo path
lies well inside the reachable set.

Run:  python3 analysis/workspace.py   ->  figures/fig_reachable_workspace.png
"""
import os
import sys
import logging
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt           # noqa: E402
from mpl_toolkits.mplot3d import Axes3D    # noqa: F401,E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
from Rsine import kinematics as K                      # noqa: E402
from Rsine.sine_path import DrawingPlane, sine_waypoints  # noqa: E402

FIG_DIR = os.path.join(PKG, "figures")
ARM_URDF = os.path.join(PKG, "urdf", "mycobot_280_arm.urdf")
os.makedirs(FIG_DIR, exist_ok=True)

PLANE = DrawingPlane.vertical_board(x=0.14, z_center=0.21)


def geometric_max_reach():
    """Upper bound on |EE - base|: sum of the link translation lengths in the chain."""
    total = 0.0
    for (_, _, _, x, y, z) in K.URDF_JOINT_ORIGINS:
        total += np.linalg.norm([x, y, z])
    return total


def sample_workspace(n, seed=0):
    rng = np.random.default_rng(seed)
    Q = rng.uniform(K.JOINT_LIMITS[:, 0], K.JOINT_LIMITS[:, 1], size=(n, 6))
    P = np.array([K.ee_position(q) for q in Q])
    return Q, P


def verify(Q, P):
    from ikpy.chain import Chain
    logging.getLogger("ikpy").setLevel(logging.ERROR)
    chain = Chain.from_urdf_file(ARM_URDF, base_elements=["base_link"])
    chain.active_links_mask = [l.name in K.JOINT_NAMES for l in chain.links]
    active_idx = [i for i, a in enumerate(chain.active_links_mask) if a]

    m = min(2000, len(Q))
    worst = 0.0
    for k in range(m):
        full = np.zeros(len(chain.links))
        for j, idx in enumerate(active_idx):
            full[idx] = Q[k, j]
        worst = max(worst, np.abs(chain.forward_kinematics(full)[:3, 3] - P[k]).max())

    reach = np.linalg.norm(P, axis=1)
    horiz = np.linalg.norm(P[:, :2], axis=1)
    bound = geometric_max_reach()
    return {
        "ikpy_max_err_m": worst,
        "max_reach_m": reach.max(),
        "max_horiz_reach_m": horiz.max(),
        "geom_bound_m": bound,
        "within_bound": bool(reach.max() <= bound + 1e-9),
        "x": (P[:, 0].min(), P[:, 0].max()),
        "y": (P[:, 1].min(), P[:, 1].max()),
        "z": (P[:, 2].min(), P[:, 2].max()),
    }


def main():
    Q, P = sample_workspace(40000)
    stats = verify(Q, P)

    print("=" * 62)
    print("Reachable-workspace verification")
    print("=" * 62)
    print(f"samples                        : {len(P)}")
    print(f"ikpy cross-check max error     : {stats['ikpy_max_err_m']:.2e} m  (independent FK)")
    print(f"max reach from base (3D)       : {stats['max_reach_m']:.4f} m")
    print(f"max horizontal reach           : {stats['max_horiz_reach_m']:.4f} m")
    print(f"geometric reach upper bound    : {stats['geom_bound_m']:.4f} m")
    print(f"all points within reach bound  : {stats['within_bound']}")
    print(f"x extent [m]                   : {stats['x'][0]:+.3f} .. {stats['x'][1]:+.3f}")
    print(f"y extent [m]                   : {stats['y'][0]:+.3f} .. {stats['y'][1]:+.3f}")
    print(f"z extent [m]                   : {stats['z'][0]:+.3f} .. {stats['z'][1]:+.3f}")

    pos, _ = sine_waypoints(PLANE, 0.05, 0.20, 2.0, 120)
    _plot(P, pos, stats)
    print("figure written to", os.path.join(FIG_DIR, "fig_reachable_workspace.png"))
    return 0


def _plot(P, sine, stats):
    # thin out the cloud for a legible scatter
    idx = np.random.default_rng(1).choice(len(P), size=min(12000, len(P)), replace=False)
    Pp = P[idx]
    fig = plt.figure(figsize=(14, 4.6))

    ax = fig.add_subplot(131, projection="3d")
    ax.scatter(Pp[:, 0], Pp[:, 1], Pp[:, 2], s=1.5, c=Pp[:, 2], cmap="viridis", alpha=0.25)
    ax.plot(sine[:, 0], sine[:, 1], sine[:, 2], color="red", lw=2.5, label="sine demo")
    ax.scatter(0, 0, 0, c="k", marker="s", s=40, label="base")
    ax.set_title("Reachable workspace (3D)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")

    ax2 = fig.add_subplot(132)
    ax2.scatter(Pp[:, 0], Pp[:, 1], s=1.5, c="0.6", alpha=0.3, label="reachable cloud")
    ax2.plot(sine[:, 0], sine[:, 1], color="red", lw=2.5, label="sine demo")
    ax2.scatter(0, 0, c="k", marker="s", s=40, label="base")
    r = stats["max_horiz_reach_m"]
    th = np.linspace(0, 2 * np.pi, 200)
    ax2.plot(r * np.cos(th), r * np.sin(th), "b--", lw=1,
             label=f"max horiz. reach {r:.2f} m")
    ax2.set_title("Top view (x–y)")
    ax2.set_xlabel("x [m]"); ax2.set_ylabel("y [m]")
    ax2.set_aspect("equal"); ax2.grid(alpha=0.3)

    ax3 = fig.add_subplot(133)
    ax3.scatter(Pp[:, 0], Pp[:, 2], s=1.5, c="0.6", alpha=0.3)
    ax3.plot(sine[:, 0], sine[:, 2], color="red", lw=2.5)
    ax3.scatter(0, 0, c="k", marker="s", s=40)
    ax3.set_title("Side view (x–z)")
    ax3.set_xlabel("x [m]"); ax3.set_ylabel("z [m]")
    ax3.set_aspect("equal"); ax3.grid(alpha=0.3)

    fig.suptitle("myCobot 280 — reachable workspace (verified vs ikpy) with sine demo overlaid")
    handles, labels = ax2.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=4, fontsize=9,
               bbox_to_anchor=(0.5, -0.02))
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(os.path.join(FIG_DIR, "fig_reachable_workspace.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
