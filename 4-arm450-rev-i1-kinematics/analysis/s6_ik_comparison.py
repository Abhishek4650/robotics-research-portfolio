#!/usr/bin/env python3
"""
Step 6 -- IK solver comparison (after the myCobot thesis, Exp. 2).

500 random full poses the arm can reach (position + orientation, joint limits
enforced), every solver on the same targets, two seeding regimes:
  GLOBAL  from the fixed ready pose -- a target anywhere in the workspace;
  LOCAL   from the true answer moved 3 deg per joint -- the next point of a
          traced path (what the Rsine drawing loop does).
Solvers: closed form (this arm has a spherical wrist), damped least squares
(the author's method, Rsine), pseudoinverse, Jacobian transpose, Levenberg-Marquardt,
and ikpy on the URDF. Success = within 1e-6 mm and 1e-6 rad (ikpy: 1e-2 mm,
1e-3 rad -- its optimiser stops there) and inside the joint limits.

Out: figures/fig_ik_comparison.png, IK_COMPARISON.log
"""
import os
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt            # noqa: E402

from common import R, FIG, KIN, run_ikpy   # noqa: E402
from arm450kin import ik                     # noqa: E402

N = 500


def main():
    rng = np.random.default_rng(21)
    Q = rng.uniform(R.Q_LIMITS[:, 0], R.Q_LIMITS[:, 1], size=(N, 6))
    targets = [R.fk_q(q) for q in Q]
    seeds = {"GLOBAL": [R.q_to_theta(R.READY_Q)] * N,
             "LOCAL": [R.q_to_theta(np.clip(q + np.radians(3.0) * rng.choice([-1, 1], 6), R.Q_LIMITS[:, 0] + 1e-6,
                                            R.Q_LIMITS[:, 1] - 1e-6)) for q in Q]}
    res = {}
    for regime, sd in seeds.items():
        # closed form: nearest valid branch to the seed
        ok, t, err = 0, [], []
        for T, s0 in zip(targets, sd):
            t0 = time.perf_counter()
            th = ik.analytical_best(T[:3, 3], T[:3, :3], s0)
            t.append((time.perf_counter() - t0) * 1e3)
            e = np.abs(R.fk(th) - T).max() if th is not None else np.inf
            err.append(e); ok += e < 1e-6
        res[("Closed form", regime)] = dict(ok=ok, it=[1] * N, t=t, err=err)
        for name, fn in ik.SOLVERS.items():
            ok, its, t, err = 0, [], [], []
            for T, s0 in zip(targets, sd):
                th, info = fn(T[:3, 3], T[:3, :3], s0)
                ok += info["converged"]; its.append(info["iters"]); t.append(info["time_ms"]); err.append(info["pos_err"])
            res[(name, regime)] = dict(ok=ok, it=its, t=t, err=err)
        back = np.eye(4); back[2, 3] = R.GEO["axes"][5]["point"][2] - R.GEO["tool_face_centre"][2]
        out = run_ikpy({"targets": [(T @ back).tolist() for T in targets],
                        "seeds": [list(map(float, R.theta_to_q(s))) for s in sd]})
        ok, err = 0, []
        for q, T in zip(out["q"], targets):
            Tq = R.fk_q(np.array(q)); e = np.linalg.norm(Tq[:3, 3] - T[:3, 3]); err.append(e)
            ok += e < 1e-2 and np.linalg.norm(Tq[:3, :3] - T[:3, :3]) < 1e-3 and R.within_limits(R.q_to_theta(np.array(q)))
        res[("ikpy (URDF)", regime)] = dict(ok=ok, it=[np.nan] * N, t=out["time_ms"], err=err)
    names = ["Closed form"] + list(ik.SOLVERS) + ["ikpy (URDF)"]
    lines = ["IK SOLVER COMPARISON -- %d random reachable full poses, joint limits enforced" % N, "",
             "%-22s %-7s %8s %10s %10s %14s" % ("solver", "seed", "success", "mean iter", "mean ms", "median err mm")]
    for nm in names:
        for rg in ("GLOBAL", "LOCAL"):
            r = res[(nm, rg)]
            lines.append("%-22s %-7s %7.1f%% %10s %10.2f %14.2e" % (nm, rg, 100 * r["ok"] / N,
                         "-" if np.all(np.isnan(r["it"])) else "%.1f" % np.nanmean(r["it"]), np.mean(r["t"]), np.median(r["err"])))
    lines += ["", "GLOBAL = seed at the ready pose, LOCAL = seed 3 deg from the answer (a traced path).",
              "The closed form needs no seed and is exact; the seed only picks among its valid branches."]
    txt = "\n".join(lines) + "\n"
    open(os.path.join(KIN, "IK_COMPARISON.log"), "w").write(txt)
    print(txt)

    fig, axs = plt.subplots(1, 3, figsize=(19, 6.8))
    fig.suptitle("IK solvers on ARM-450 rev I.1 (after the myCobot thesis, Exp. 2): %d random reachable full poses" % N,
                 fontsize=14, fontweight="bold")
    x = np.arange(len(names)); w = 0.38
    for k, (rg, col) in enumerate((("GLOBAL", "#3a78c2"), ("LOCAL", "#2ca25f"))):
        axs[0].bar(x + (k - 0.5) * w, [100 * res[(n, rg)]["ok"] / N for n in names], w, color=col,
                   label="%s seed" % rg.lower())
        axs[1].bar(x + (k - 0.5) * w, [np.mean(res[(n, rg)]["t"]) for n in names], w, color=col)
    axs[0].set_ylabel("success (%)"); axs[0].set_ylim(0, 105); axs[0].legend(fontsize=9)
    axs[1].set_ylabel("mean time per solve (ms)"); axs[1].set_yscale("log")
    for ax in axs[:2]:
        ax.set_xticks(x); ax.set_xticklabels(names, rotation=25, ha="right", fontsize=9); ax.grid(True, axis="y", lw=0.3)
    axs[0].set_title("success rate"); axs[1].set_title("time")
    for nm, col in zip(names, plt.cm.tab10.colors):
        e = np.sort(np.maximum(np.array(res[(nm, "GLOBAL")]["err"], float), 1e-16))
        axs[2].plot(np.log10(e), np.linspace(0, 100, len(e)), color=col, lw=2, label=nm)
    axs[2].set_xlabel("log10 position error (mm)"); axs[2].set_ylabel("% of targets"); axs[2].set_xlim(-16, 3)
    axs[2].set_title("error distribution, global seed"); axs[2].grid(True, lw=0.3); axs[2].legend(fontsize=8.5, loc="lower right")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    p = os.path.join(FIG, "fig_ik_comparison.png"); fig.savefig(p, dpi=170); plt.close(fig)
    print("wrote", p)


if __name__ == "__main__":
    main()
