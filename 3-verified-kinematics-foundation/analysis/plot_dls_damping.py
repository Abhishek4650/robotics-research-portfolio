#!/usr/bin/env python3
"""
How the damping factor lambda trades accuracy against stability in the DLS solver.

The solver in Rsine/ik.py takes the step

    dq = Jt (J Jt + lam^2 I)^-1 e

Lambda is the only knob: lam -> 0 is the plain pseudo-inverse (exact, but it
explodes near singularities) and a large lam is smooth but sloppy. This script
sweeps lam over the ACTUAL sine trajectory the demo draws and produces
figures/fig_dls_damping.png with four panels:

  (a) accuracy    — max / mean end-effector position error vs lam.
  (b) cost        — mean solver iterations per waypoint vs lam.
  (c) smoothness  — largest joint jump between consecutive waypoints vs lam.
  (d) singularity — first-step |dq| for a unit error at the WORST-conditioned
                    configuration found, i.e. the failure lam is there to prevent.

The vertical marker on every panel is lam = 0.04, the value sine_ik_node.py runs.

Run:  python3 analysis/plot_dls_damping.py
Figure is written to ../figures/.
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")            # headless: save files, no window
import matplotlib.pyplot as plt  # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
from Rsine import kinematics as K                        # noqa: E402
from Rsine.ik import DLSIKSolver                          # noqa: E402
from Rsine.sine_path import DrawingPlane, sine_waypoints  # noqa: E402

FIG_DIR = os.path.join(PKG, "figures")
os.makedirs(FIG_DIR, exist_ok=True)

# same board + sine the demo draws (sine_ik_node.py defaults)
PLANE = DrawingPlane.vertical_board(x=0.14, z_center=0.21)
AMP, LEN, CYC, N = 0.05, 0.20, 2.0, 120
LAM_NODE = 0.04                          # what sine_ik_node.py actually uses
LAMBDAS = np.logspace(-3.0, 0.0, 22)     # 0.001 .. 1.0

C_MAIN, C_ALT = "#1f77b4", "#e8710a"     # blue / orange: safe under CVD
C_MARK = "#444444"


def pick_orientation():
    """Reproduce the node's choice: whichever pen direction the arm can reach."""
    positions, coords = sine_waypoints(PLANE, AMP, LEN, CYC, N)
    solver = DLSIKSolver(lam=LAM_NODE, max_iters=200, tol=1e-6)
    best = None
    for R_t in PLANE.candidate_orientations():
        _, infos = solver.solve_trajectory(positions, R_t)
        worst = max(i["pos_err"] for i in infos)
        if best is None or worst < best[0]:
            best = (worst, R_t)
    return positions, coords, best[1]


def sweep(positions, R_target):
    """Solve the whole trajectory once per lambda; collect the metrics."""
    max_err, mean_err, mean_iters, max_jump = [], [], [], []
    for lam in LAMBDAS:
        solver = DLSIKSolver(lam=lam, max_iters=200, tol=1e-6)
        Q, infos = solver.solve_trajectory(positions, R_target)
        errs = np.array([i["pos_err"] for i in infos])
        max_err.append(errs.max())
        mean_err.append(errs.mean())
        mean_iters.append(np.mean([i["iters"] for i in infos]))
        # biggest single-joint change between consecutive waypoints = jerkiness
        max_jump.append(np.abs(np.diff(Q, axis=0)).max())
    return (np.array(max_err), np.array(mean_err),
            np.array(mean_iters), np.array(max_jump))


def worst_conditioned(n=4000, seed=0):
    """Search the joint space for the most singular configuration we can find."""
    rng = np.random.default_rng(seed)
    worst_q, worst_s = None, np.inf
    for _ in range(n):
        q = rng.uniform(K.JOINT_LIMITS[:, 0], K.JOINT_LIMITS[:, 1])
        s = np.linalg.svd(K.jacobian(q), compute_uv=False)[-1]
        if s < worst_s:
            worst_s, worst_q = s, q
    return worst_q, worst_s


def singular_step(q_sing):
    """|dq| for a unit position error at q_sing, as a function of lambda.

    Also returns the undamped pseudo-inverse step for comparison — that number is
    the runaway DLS exists to tame.
    """
    J = K.jacobian(q_sing)
    e = np.zeros(6)
    e[:3] = np.linalg.svd(J, compute_uv=False)[-1] * 0.0 + 1.0  # unit pos error
    e = e / np.linalg.norm(e)
    steps = []
    for lam in LAMBDAS:
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        steps.append(np.linalg.norm(dq))
    undamped = np.linalg.norm(np.linalg.pinv(J) @ e)
    return np.array(steps), undamped


def _mark_node(ax):
    """Vertical marker for the value the demo actually runs."""
    ax.axvline(LAM_NODE, color=C_MARK, lw=1.2, ls=":", zorder=1)
    ax.annotate(f"node runs\n$\\lambda$ = {LAM_NODE}", xy=(LAM_NODE, 0.97),
                xycoords=("data", "axes fraction"), xytext=(6, 0),
                textcoords="offset points", fontsize=8, color=C_MARK,
                ha="left", va="top")


def main():
    print("choosing reachable pen orientation ...")
    positions, coords, R_target = pick_orientation()
    print(f"sweeping {len(LAMBDAS)} damping values over {N} waypoints ...")
    max_err, mean_err, mean_iters, max_jump = sweep(positions, R_target)
    print("searching for the worst-conditioned configuration ...")
    q_sing, s_min = worst_conditioned()
    steps, undamped = singular_step(q_sing)

    fig, axes = plt.subplots(2, 2, figsize=(11.5, 8.6), constrained_layout=True)
    fig.suptitle("Damped least squares: what the damping factor $\\lambda$ buys\n"
                 "myCobot 280 drawing the demo sine "
                 f"({N} waypoints, amplitude {AMP*100:.0f} cm)",
                 fontsize=13)

    # (a) accuracy ------------------------------------------------------- #
    ax = axes[0, 0]
    ax.loglog(LAMBDAS, max_err * 1000, "-o", color=C_MAIN, lw=2, ms=4,
              label="worst waypoint")
    ax.loglog(LAMBDAS, mean_err * 1000, "--s", color=C_ALT, lw=2, ms=4,
              label="mean waypoint")
    ax.set_title("(a) Accuracy — error grows with damping", fontsize=11)
    ax.set_xlabel("damping factor  $\\lambda$")
    ax.set_ylabel("position error [mm]")
    ax.legend(fontsize=9, framealpha=0.9)

    # (b) iterations ----------------------------------------------------- #
    ax = axes[0, 1]
    ax.semilogx(LAMBDAS, mean_iters, "-o", color=C_MAIN, lw=2, ms=4)
    ax.set_title("(b) Cost — heavy damping converges slowly", fontsize=11)
    ax.set_xlabel("damping factor  $\\lambda$")
    ax.set_ylabel("mean iterations per waypoint")

    # (c) smoothness ----------------------------------------------------- #
    ax = axes[1, 0]
    ax.loglog(LAMBDAS, np.degrees(max_jump), "-o", color=C_MAIN, lw=2, ms=4)
    ax.set_title("(c) Smoothness — largest joint jump along the path", fontsize=11)
    ax.set_xlabel("damping factor  $\\lambda$")
    ax.set_ylabel("max $\\Delta q$ between waypoints [deg]")

    # (d) the singularity case ------------------------------------------- #
    ax = axes[1, 1]
    ax.loglog(LAMBDAS, steps, "-o", color=C_MAIN, lw=2, ms=4,
              label="damped step")
    ax.axhline(undamped, color=C_ALT, lw=2, ls="--",
               label=f"undamped ($\\lambda$=0): {undamped:.0f} rad")
    ax.set_title(f"(d) Singularity — the runaway $\\lambda$ prevents "
                 f"($\\sigma_{{min}}$ = {s_min:.1e})", fontsize=11)
    ax.set_xlabel("damping factor  $\\lambda$")
    ax.set_ylabel("$\\|\\Delta q\\|$ for a unit error [rad]")
    ax.legend(fontsize=9, framealpha=0.9, loc="lower left")

    for ax in axes.ravel():
        ax.grid(alpha=0.25, which="both")
        ax.set_axisbelow(True)
        _mark_node(ax)

    out = os.path.join(FIG_DIR, "fig_dls_damping.png")
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)

    # numbers worth quoting in the write-up
    i = int(np.argmin(np.abs(LAMBDAS - LAM_NODE)))
    print(f"\nat lambda ~ {LAMBDAS[i]:.3f} (the node's setting):")
    print(f"  worst position error : {max_err[i]*1000:.6f} mm")
    print(f"  mean iterations      : {mean_iters[i]:.1f}")
    print(f"  max joint jump       : {np.degrees(max_jump[i]):.2f} deg")
    print(f"  singular step |dq|   : {steps[i]:.3f} rad "
          f"(undamped would be {undamped:.1f} rad)")
    print("\nwrote", out)


if __name__ == "__main__":
    main()
