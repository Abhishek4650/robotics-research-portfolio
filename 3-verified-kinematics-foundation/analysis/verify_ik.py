#!/usr/bin/env python3
"""
Stage 2 verification + graphs:
  * Solve the sine trajectory with the analytical DLS IK.
  * Cross-check every solved joint vector against ikpy's independent FK.
  * Confirm all joints stay within limits.
  * Produce graphs: 3D drawn sine on the board, joint angles, tracking error.

Run:  python3 analysis/verify_ik.py
Figures -> ../figures/.  Trajectory cache -> ../figures/sine_trajectory.npz
"""
import os
import sys
import logging
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401,E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
from Rsine import kinematics as K         # noqa: E402
from Rsine.ik import DLSIKSolver          # noqa: E402
from Rsine.sine_path import DrawingPlane, sine_waypoints  # noqa: E402

FIG_DIR = os.path.join(PKG, "figures")
ARM_URDF = os.path.join(PKG, "urdf", "mycobot_280_arm.urdf")
os.makedirs(FIG_DIR, exist_ok=True)

# ---- demo configuration (chosen for full reachability, see analysis) -------- #
PLANE = DrawingPlane.vertical_board(x=0.14, z_center=0.21)
AMPLITUDE, LENGTH, CYCLES, N = 0.05, 0.20, 2.0, 120


def get_ikpy_chain():
    from ikpy.chain import Chain
    logging.getLogger("ikpy").setLevel(logging.ERROR)
    chain = Chain.from_urdf_file(ARM_URDF, base_elements=["base_link"])
    chain.active_links_mask = [l.name in K.JOINT_NAMES for l in chain.links]
    active_idx = [i for i, a in enumerate(chain.active_links_mask) if a]
    return chain, active_idx


def ikpy_fk_positions(chain, active_idx, Q):
    P = np.empty((len(Q), 3))
    for k, q in enumerate(Q):
        full = np.zeros(len(chain.links))
        for j, idx in enumerate(active_idx):
            full[idx] = q[j]
        P[k] = chain.forward_kinematics(full)[:3, 3]
    return P


def main():
    positions, coords = sine_waypoints(PLANE, AMPLITUDE, LENGTH, CYCLES, N)
    solver = DLSIKSolver(lam=0.04, max_iters=200, tol=1e-6)

    # A small arm can hold the pen perpendicular to the board facing only one way.
    # Try both +/- normal and keep whichever the arm actually reaches.
    best = None
    for R_t in PLANE.candidate_orientations():
        Q, infos = solver.solve_trajectory(positions, R_t)
        worst = max(i["pos_err"] for i in infos)
        if best is None or worst < best[0]:
            best = (worst, R_t, Q, infos)
    _, R_t, Q, infos = best

    achieved = np.array([K.forward_kinematics(q)[:3, 3] for q in Q])
    pos_err = np.array([i["pos_err"] for i in infos])
    ori_err = np.array([i["ori_err"] for i in infos])

    chain, active_idx = get_ikpy_chain()
    ikpy_pos = ikpy_fk_positions(chain, active_idx, Q)
    cross = np.linalg.norm(ikpy_pos - positions, axis=1)   # ikpy(our q) vs target

    within = np.all((Q >= K.JOINT_LIMITS[:, 0]) & (Q <= K.JOINT_LIMITS[:, 1]))

    print("=" * 62)
    print("Stage 2 — sine IK verification")
    print("=" * 62)
    print(f"plane: vertical board  x={PLANE.origin[0]}  z_center={PLANE.origin[2]}")
    print(f"sine : width={LENGTH} m  amp={AMPLITUDE} m  cycles={CYCLES}  pts={N}")
    print(f"pen approach axis (flange z target): {R_t[:, 2].round(3)}")
    print(f"max analytical position error : {pos_err.max()*1000:.3f} mm")
    print(f"max analytical orient error   : {ori_err.max():.3e}")
    print(f"max ikpy cross-check residual : {cross.max()*1000:.3f} mm")
    print(f"all joints within limits      : {within}")
    print(f"mean IK iters / waypoint      : {np.mean([i['iters'] for i in infos]):.1f}")

    np.savez(os.path.join(FIG_DIR, "sine_trajectory.npz"),
             Q=Q, positions=positions, achieved=achieved, coords=coords)
    _plot_3d(positions, achieved)
    _plot_joints(Q)
    _plot_error(pos_err, cross)
    print("figures written to", FIG_DIR)
    return 0


def _plot_3d(target, achieved):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    # board rectangle
    o, u, v = PLANE.origin, PLANE.u, PLANE.v
    hw, hh = LENGTH / 2 + 0.02, AMPLITUDE + 0.03
    corners = np.array([o + a * u + b * v for a, b in
                        [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh), (-hw, -hh)]])
    ax.plot(corners[:, 0], corners[:, 1], corners[:, 2], color="0.6", lw=1)
    ax.plot(target[:, 0], target[:, 1], target[:, 2], "--", color="k", lw=1.2,
            label="target sine")
    ax.plot(achieved[:, 0], achieved[:, 1], achieved[:, 2], "-", color="#d62728",
            lw=2, label="drawn (FK of IK solution)")
    # base + a home arm for context
    home = K.fk_frames(np.zeros(6))
    hp = np.array([T[:3, 3] for T in home])
    ax.plot(hp[:, 0], hp[:, 1], hp[:, 2], "-o", color="#1f77b4", lw=2, ms=3,
            alpha=0.5, label="arm (home)")
    ax.set_title("myCobot 280 drawing a sine on a vertical board")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=9, framealpha=0.9)
    ax.view_init(elev=18, azim=-70)
    fig.savefig(os.path.join(FIG_DIR, "fig_sine_3d.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


def _plot_joints(Q):
    fig, ax = plt.subplots(figsize=(9, 5))
    t = np.arange(len(Q))
    for i in range(6):
        ax.plot(t, np.degrees(Q[:, i]), label=K.JOINT_NAMES[i])
    ax.set_title("Joint angles along the sine trajectory")
    ax.set_xlabel("waypoint index"); ax.set_ylabel("joint angle [deg]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8)
    fig.savefig(os.path.join(FIG_DIR, "fig_joint_angles.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


def _plot_error(pos_err, cross):
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(pos_err * 1000, label="analytical FK error", color="#d62728")
    ax.plot(cross * 1000, label="ikpy cross-check residual", color="#1f77b4", ls="--")
    ax.set_title("Tracking accuracy along the trajectory")
    ax.set_xlabel("waypoint index"); ax.set_ylabel("position error [mm]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=9)
    fig.savefig(os.path.join(FIG_DIR, "fig_tracking_error.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
