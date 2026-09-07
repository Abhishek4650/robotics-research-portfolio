#!/usr/bin/env python3
"""
Side-by-side comparison of OUR analytical method vs the ikpy library, so the
results can be defended in the presentation.

Produces figures/fig_method_comparison.png with three panels:
  (a) verification bar chart  — max error of every agreement check (log scale).
  (b) drawn sine overlay      — target vs analytical-IK path vs ikpy-IK path.
  (c) per-waypoint EE error   — analytical IK vs ikpy IK along the trajectory.

Run:  python3 analysis/compare_methods.py
"""
import os
import sys
import logging
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
from Rsine import kinematics as K                       # noqa: E402
from Rsine.ik import DLSIKSolver                         # noqa: E402
from Rsine.sine_path import DrawingPlane, sine_waypoints # noqa: E402

FIG_DIR = os.path.join(PKG, "figures")
ARM_URDF = os.path.join(PKG, "urdf", "mycobot_280_arm.urdf")

PLANE = DrawingPlane.vertical_board(x=0.14, z_center=0.21)
AMP, LEN, CYC, N = 0.05, 0.20, 2.0, 120


def get_chain():
    from ikpy.chain import Chain
    logging.getLogger("ikpy").setLevel(logging.ERROR)
    chain = Chain.from_urdf_file(ARM_URDF, base_elements=["base_link"])
    chain.active_links_mask = [l.name in K.JOINT_NAMES for l in chain.links]
    active = [i for i, a in enumerate(chain.active_links_mask) if a]
    return chain, active


def fd_jac(q, eps=1e-6):
    J = np.zeros((6, 6)); T0 = K.forward_kinematics(q); p0, R0 = T0[:3, 3], T0[:3, :3]
    for i in range(6):
        dq = np.zeros(6); dq[i] = eps; T1 = K.forward_kinematics(q + dq)
        J[:3, i] = (T1[:3, 3] - p0) / eps
        dR = (T1[:3, :3] - R0) / eps @ R0.T
        J[3:, i] = [dR[2, 1], dR[0, 2], dR[1, 0]]
    return J


def to_plane_coords(P):
    d = P - PLANE.origin
    return np.column_stack([d @ PLANE.u, d @ PLANE.v])


def main():
    chain, active = get_chain()
    positions, coords = sine_waypoints(PLANE, AMP, LEN, CYC, N)

    # --- analytical IK (ours) --- pick reachable pen direction ---
    solver = DLSIKSolver(lam=0.04, max_iters=200, tol=1e-6)
    best = None
    for R_t in PLANE.candidate_orientations():
        Q, infos = solver.solve_trajectory(positions, R_t)
        w = max(i["pos_err"] for i in infos)
        if best is None or w < best[0]:
            best = (w, Q, infos)
    _, Q_ana, infos = best
    ee_ana = np.array([K.forward_kinematics(q)[:3, 3] for q in Q_ana])
    err_ana = np.linalg.norm(ee_ana - positions, axis=1)

    # --- ikpy IK (independent library, position-based) ---
    ee_ikpy = np.empty((N, 3))
    seed = np.zeros(len(chain.links))
    for k, p in enumerate(positions):
        seed = chain.inverse_kinematics(target_position=p, initial_position=seed)
        ee_ikpy[k] = chain.forward_kinematics(seed)[:3, 3]
    err_ikpy = np.linalg.norm(ee_ikpy - positions, axis=1)

    # --- agreement checks for the bar chart ---
    rng = np.random.default_rng(0)
    fk_dh = fk_ikpy = jac = 0.0
    for _ in range(400):
        q = rng.uniform(K.JOINT_LIMITS[:, 0], K.JOINT_LIMITS[:, 1])
        fk_dh = max(fk_dh, np.abs(K.forward_kinematics(q) - K.urdf_fk(q)).max())
        jac = max(jac, np.abs(K.jacobian(q) - fd_jac(q)).max())
        full = np.zeros(len(chain.links))
        for j, idx in enumerate(active):
            full[idx] = q[j]
        fk_ikpy = max(fk_ikpy, np.abs(chain.forward_kinematics(full)[:3, 3] -
                                      K.ee_position(q)).max())

    _plot(coords, positions, ee_ana, ee_ikpy, err_ana, err_ikpy,
          fk_dh, fk_ikpy, jac, err_ana.max(), err_ikpy.max())

    print("Comparison summary (max errors):")
    print(f"  DH-FK vs URDF-FK        : {fk_dh:.2e} m")
    print(f"  ikpy-FK vs our-FK       : {fk_ikpy:.2e} m")
    print(f"  velprop-J vs finite-diff: {jac:.2e}")
    print(f"  analytical IK tracking  : {err_ana.max()*1000:.3f} mm")
    print(f"  ikpy IK tracking        : {err_ikpy.max()*1000:.3f} mm")
    print("figure ->", os.path.join(FIG_DIR, "fig_method_comparison.png"))
    return 0


def _plot(coords, target, ee_ana, ee_ikpy, err_ana, err_ikpy,
          fk_dh, fk_ikpy, jac, ik_ana, ik_ikpy):
    fig = plt.figure(figsize=(15, 4.6))

    # (a) verification bars
    ax = fig.add_subplot(131)
    labels = ["DH-FK\nvs URDF", "ikpy-FK\nvs ours", "velprop-J\nvs FD",
              "IK ours\ntrack", "IK ikpy\ntrack"]
    vals = [fk_dh, fk_ikpy, jac, ik_ana, ik_ikpy]
    colors = ["#1f77b4", "#1f77b4", "#1f77b4", "#d62728", "#2ca02c"]
    ax.bar(labels, np.maximum(vals, 1e-17), color=colors)
    ax.set_yscale("log"); ax.set_ylabel("max error [m or rad]")
    ax.axhline(1e-3, color="0.5", ls="--", lw=1)
    ax.text(4.4, 1.3e-3, "1 mm", color="0.4", fontsize=8, ha="right")
    ax.set_title("(a) Verification — every check agrees")
    ax.tick_params(axis="x", labelsize=8)

    # (b) drawn sine overlay in plane coords
    ax2 = fig.add_subplot(132)
    ax2.plot(coords[:, 0] * 100, coords[:, 1] * 100, "k--", lw=2, label="target")
    ca = to_plane_coords(ee_ana); ci = to_plane_coords(ee_ikpy)
    ax2.plot(ca[:, 0] * 100, ca[:, 1] * 100, color="#d62728", lw=1.5, label="analytical IK")
    ax2.plot(ci[:, 0] * 100, ci[:, 1] * 100, color="#2ca02c", lw=1.2, ls=":", label="ikpy IK")
    ax2.set_title("(b) Drawn sine on the board (plane coords)")
    ax2.set_xlabel("advance s [cm]"); ax2.set_ylabel("wave w [cm]")
    ax2.set_aspect("equal"); ax2.grid(alpha=0.3)

    # (c) per-waypoint EE error
    ax3 = fig.add_subplot(133)
    ax3.plot(err_ana * 1000, color="#d62728", label="analytical IK")
    ax3.plot(err_ikpy * 1000, color="#2ca02c", ls=":", label="ikpy IK")
    ax3.set_title("(c) End-effector error per waypoint")
    ax3.set_xlabel("waypoint index"); ax3.set_ylabel("error [mm]")
    ax3.grid(alpha=0.3)

    fig.suptitle("Analytical (ours) vs ikpy — myCobot 280 sine tracing")
    handles, labels = ax2.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, fontsize=9,
               bbox_to_anchor=(0.5, -0.02))
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(os.path.join(FIG_DIR, "fig_method_comparison.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
