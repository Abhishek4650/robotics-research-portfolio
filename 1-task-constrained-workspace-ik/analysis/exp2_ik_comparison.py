#!/usr/bin/env python3
"""
Experiment 2 — comparison of inverse-kinematics solvers on the full-pose
(position + orientation) tracing task.

Four Jacobian-based methods are compared on a common set of reachable target
poses, all seeded from the zero configuration:
    * Damped least squares (DLS)
    * Moore-Penrose pseudoinverse
    * Jacobian transpose (optimal step)
    * Levenberg-Marquardt (adaptive damping)

Metrics: success rate, mean iterations, mean solve time, final position error;
plus error-vs-iteration convergence curves for one representative pose.

Run:  python3 analysis/exp2_ik_comparison.py  ->  figures/fig_ik_comparison.png
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
from mycobot_thesis import robot as R    # noqa: E402
from mycobot_thesis import ik            # noqa: E402

FIG = os.path.join(PKG, "figures")
os.makedirs(FIG, exist_ok=True)

POS_TOL, ORI_TOL = 1e-3, 1e-2
# A bent "ready" seed (zeros = arm pointing straight up, a poor / near-boundary seed).
SEED_Q = np.array([0.0, -0.7, 1.0, 0.0, 0.6, 0.0])
COLORS = {"DLS": "#1f77b4", "Pseudoinverse": "#ff7f0e",
          "Jacobian-transpose": "#2ca02c", "Levenberg-Marquardt": "#d62728"}


def make_targets(n=150, seed=1, delta=1.0):
    """Reachable target poses within reach of the seed: FK of bounded perturbations
    of SEED_Q. This mirrors the drawing application (small moves from a nominal
    pose) and gives every solver a fair, solvable problem."""
    rng = np.random.default_rng(seed)
    T = []
    for _ in range(n):
        q = R.clamp_to_limits(SEED_Q + rng.uniform(-delta, delta, 6))
        M = R.forward_kinematics(q)
        T.append((M[:3, 3].copy(), M[:3, :3].copy()))
    return T


def benchmark(targets):
    q0 = SEED_Q
    rows = {}
    for name, solver in ik.SOLVERS.items():
        succ, iters, times, perrs = 0, [], [], []
        for p, Rt in targets:
            _, info = solver(p, Rt, q0)
            ok = info["pos_err"] < POS_TOL and info["ori_err"] < ORI_TOL
            succ += ok
            times.append(info["time_ms"]); perrs.append(info["pos_err"])
            if ok:
                iters.append(info["iters"])
        rows[name] = {
            "success": 100.0 * succ / len(targets),
            "iters": np.mean(iters) if iters else np.nan,
            "time": np.mean(times),
            "perr_mm": np.median(perrs) * 1000,
        }
    return rows


def convergence_trace(p, Rt, q0, n=120):
    """Error-vs-iteration for each method on one pose (reimplemented compactly)."""
    def err(q):
        M = R.forward_kinematics(q)
        return np.hstack([p - M[:3, 3], ik.orientation_error(M[:3, :3], Rt)])
    traces = {}
    # DLS
    q = q0.copy(); h = []
    for _ in range(n):
        e = err(q); h.append(np.linalg.norm(e)); J = R.jacobian(q)
        q = R.clamp_to_limits(q + J.T @ np.linalg.solve(J @ J.T + 0.04**2*np.eye(6), e))
    traces["DLS"] = h
    # Pseudoinverse
    q = q0.copy(); h = []
    for _ in range(n):
        e = err(q); h.append(np.linalg.norm(e)); J = R.jacobian(q)
        q = R.clamp_to_limits(q + np.linalg.pinv(J) @ e)
    traces["Pseudoinverse"] = h
    # Jacobian transpose
    q = q0.copy(); h = []
    for _ in range(n):
        e = err(q); h.append(np.linalg.norm(e)); J = R.jacobian(q)
        Je = J @ J.T @ e; a = np.dot(e, Je)/(np.dot(Je, Je)+1e-12)
        q = R.clamp_to_limits(q + a * J.T @ e)
    traces["Jacobian-transpose"] = h
    # Levenberg-Marquardt
    q = q0.copy(); h = []; lam = 1e-2
    for _ in range(n):
        e = err(q); h.append(np.linalg.norm(e)); J = R.jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2*np.eye(6), e)
        qn = R.clamp_to_limits(q + dq)
        lam = max(lam*0.7, 1e-4) if np.linalg.norm(err(qn)) < np.linalg.norm(e) else min(lam*2, 1.0)
        q = qn
    traces["Levenberg-Marquardt"] = h
    return traces


def main():
    targets = make_targets()
    rows = benchmark(targets)

    print("=" * 78)
    print(f"{'solver':22} {'success %':>10} {'mean iters':>12} "
          f"{'mean time ms':>14} {'median err mm':>15}")
    print("-" * 78)
    for name, r in rows.items():
        print(f"{name:22} {r['success']:>10.1f} {r['iters']:>12.1f} "
              f"{r['time']:>14.3f} {r['perr_mm']:>15.2e}")

    # representative pose = a mid-workspace reachable pose
    M = R.forward_kinematics(np.array([0.6, -0.5, 0.7, 0.3, -0.4, 0.2]))
    traces = convergence_trace(M[:3, 3], M[:3, :3], SEED_Q)

    _plot(rows, traces)
    print("figure ->", os.path.join(FIG, "fig_ik_comparison.png"))


def _plot(rows, traces):
    names = list(rows.keys())
    fig, ax = plt.subplots(1, 4, figsize=(17, 4.3))

    def bar(a, key, title, ylabel, log=False):
        vals = [rows[n][key] for n in names]
        a.bar(range(len(names)), vals, color=[COLORS[n] for n in names])
        a.set_title(title); a.set_ylabel(ylabel)
        a.set_xticks(range(len(names)))
        a.set_xticklabels([n.replace("-", "-\n").replace(" ", "\n") for n in names], fontsize=7)
        if log:
            a.set_yscale("log")

    bar(ax[0], "success", "(a) Success rate", "%")
    bar(ax[1], "iters", "(b) Mean iterations\n(successful)", "iterations")
    bar(ax[2], "time", "(c) Mean solve time", "ms", log=True)

    for n in names:
        ax[3].semilogy(traces[n], color=COLORS[n], label=n, lw=1.8)
    ax[3].axhline(1e-3, color="0.5", ls="--", lw=1)
    ax[3].set_title("(d) Convergence (one pose)")
    ax[3].set_xlabel("iteration"); ax[3].set_ylabel("pose error norm")
    ax[3].legend(fontsize=7, loc="upper right"); ax[3].grid(alpha=0.3)

    fig.suptitle("Inverse-kinematics solver comparison — full-pose tracing task "
                 "(myCobot 280)", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(FIG, "fig_ik_comparison.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
