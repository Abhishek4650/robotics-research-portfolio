#!/usr/bin/env python3
"""
Step 5 -- manipulability, conditioning and singularities (after the myCobot
thesis, Exp. 3).

  Yoshikawa manipulability  w = sqrt(det(J_v J_v^T))  (translational part of the
  velocity-propagation Jacobian, mm^3/rad^3) and the condition number
  kappa = sigma_max / sigma_min of J_v.

  The arm part (J1..J3 -> wrist centre W) has a closed form that the map is
  checked against:  w_W = a2 . d4 . |sin phi3| . r   (r = distance of W from the
  J1 axis, phi3 = elbow bend). Its zeros are the arm singularities; the wrist
  singularity is theta5 = 0 (J4 and J6 in line).

Out: figures/fig_manipulability.png, MANIPULABILITY.log
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt            # noqa: E402

from common import R, FIG, KIN            # noqa: E402
import batch as B                          # noqa: E402

N = 150000


def jv_W(th):
    """translational Jacobian of the wrist centre w.r.t. theta1..3 (3x3)."""
    return R.jacobian(th, tool=False)[:3, :3]


def main():
    log = ["MANIPULABILITY + SINGULARITIES", ""]
    rng = np.random.default_rng(9)
    # closed form vs velocity propagation, arm part
    err = 0.0
    for _ in range(500):
        th = R.q_to_theta(rng.uniform(R.Q_LIMITS[:, 0], R.Q_LIMITS[:, 1]))
        W = R.fk(th, tool=False)[:3, 3]
        wn = abs(np.linalg.det(jv_W(th)))
        wc = R.A2 * R.D4 * abs(np.sin(th[2] - np.pi / 2)) * np.hypot(W[0], W[1])
        err = max(err, abs(wn - wc) / max(wc, 1.0))
    log.append("arm manipulability  |det J_W| = a2 d4 |sin phi3| r : closed form vs velocity propagation, max rel. diff %.1e" % err)

    Q = rng.uniform(R.Q_LIMITS[:, 0], R.Q_LIMITS[:, 1], size=(N, 6))
    TH = R.SIGN * Q + R.DH[:, 3]
    P = B.fk_batch(TH)[:, :3, 3]
    w = np.zeros(N); kap = np.zeros(N); smin = np.zeros(N)
    for k in range(N):
        J = R.jacobian(TH[k])
        s = np.linalg.svd(J[:3], compute_uv=False)
        w[k] = np.prod(s); kap[k] = s[0] / max(s[-1], 1e-12)
        J6 = J.copy(); J6[3:] *= 100.0                         # 100 mm characteristic length
        smin[k] = np.linalg.svd(J6, compute_uv=False)[-1]
    log += ["", "over %d random poses inside the joint limits (tool face, J_v):" % N,
            "  manipulability w: median %.3g, max %.3g mm^3/rad^3" % (np.median(w), w.max()),
            "  condition number kappa: median %.1f, 95th percentile %.1f" % (np.median(kap), np.percentile(kap, 95)),
            "  6-D min singular value (100 mm scaling) < 5 %% of its median: %.1f %% of poses"
            % (100 * np.mean(smin < 0.05 * np.median(smin)))]
    th_home = R.q_to_theta(R.HOME_Q); th_ready = R.q_to_theta(R.READY_Q)
    J6h = R.jacobian(th_home); J6h[3:] *= 100
    J6r = R.jacobian(th_ready); J6r[3:] *= 100
    log += ["", "SINGULARITIES (arm straight up at home is all three at once):",
            "  wrist    theta5 = 0        J4 and J6 in line -- rank loss 1 (home, and anywhere q5 = 0)",
            "  elbow    phi3 = 0 or 180   upper arm and forearm in line -- W on the reach sphere (home)",
            "  shoulder r = 0             W on the J1 axis -- J1 moves nothing (home)",
            "  rank of J: home %d, ready pose %d; smallest singular value home %.2e, ready %.1f (100 mm scaling)"
            % (np.linalg.matrix_rank(J6h, 1e-9), np.linalg.matrix_rank(J6r, 1e-9),
               np.linalg.svd(J6h, compute_uv=False)[-1], np.linalg.svd(J6r, compute_uv=False)[-1]),
            "  -> start and work from a bent pose (the manual's ready pose q = 0, 30, -60, 0, 30, 0 deg)."]

    fig = plt.figure(figsize=(19, 10.5))
    fig.suptitle("ARM-450 rev I.1 -- manipulability and conditioning (velocity-propagation Jacobian, %d random poses)" % N,
                 fontsize=14, fontweight="bold")
    sl = np.abs(P[:, 1]) < 20
    ax = fig.add_axes([0.04, 0.53, 0.28, 0.38])
    sc = ax.scatter(P[sl, 0], P[sl, 2], c=w[sl], s=2, cmap="viridis")
    fig.colorbar(sc, ax=ax, label="w (mm^3/rad^3)"); ax.set_aspect("equal"); ax.grid(True, lw=0.3)
    ax.set_title("manipulability w, side slice |y| < 20 mm", fontsize=10.5); ax.set_xlabel("x (mm)"); ax.set_ylabel("z (mm)")
    ax = fig.add_axes([0.37, 0.53, 0.28, 0.38])
    sc = ax.scatter(P[sl, 0], P[sl, 2], c=np.log10(kap[sl]), s=2, cmap="magma_r", vmin=0, vmax=2.5)
    fig.colorbar(sc, ax=ax, label="log10 kappa"); ax.set_aspect("equal"); ax.grid(True, lw=0.3)
    ax.set_title("condition number (log10): dark = near-singular", fontsize=10.5); ax.set_xlabel("x (mm)")
    # arm manipulability over the r-z plane (closed form), joint limits masked
    rr, zz = np.meshgrid(np.linspace(-340, 340, 300), np.linspace(0, 460, 220))
    Wg = np.stack([rr.ravel(), np.zeros(rr.size), zz.ravel()], axis=1)
    best = np.full(len(Wg), np.nan)
    for th1, th2, th3, ok in B.arm_branches(Wg):
        val = R.A2 * R.D4 * np.abs(np.sin(th3 - np.pi / 2)) * np.abs(rr.ravel())
        best = np.where(ok, np.fmax(best, val), best)
    ax = fig.add_axes([0.70, 0.53, 0.28, 0.38])
    im = ax.imshow(best.reshape(rr.shape), origin="lower", extent=[-340, 340, 0, 460], cmap="viridis")
    fig.colorbar(im, ax=ax, label="a2 d4 |sin phi3| r"); ax.set_aspect("equal")
    ax.axvline(0, color="r", lw=1, ls="--"); ax.text(5, 440, "shoulder singular (r = 0)", color="r", fontsize=9)
    ax.set_title("arm part: wrist-centre manipulability (closed form)\nwhite = W not reachable inside the limits", fontsize=10.5)
    ax.set_xlabel("W x (mm)"); ax.set_ylabel("W z (mm)")
    # smallest singular value along the three singular directions from the ready pose
    ax = fig.add_axes([0.05, 0.07, 0.40, 0.36])
    for j, lab, col in ((4, "J5 (wrist, theta5 -> 0)", "#1f77b4"), (2, "J3 (elbow straightens)", "#d62728"),
                        (1, "J2 (shoulder)", "#2ca02c")):
        vals, s_ = np.linspace(R.Q_LIMITS[j, 0], R.Q_LIMITS[j, 1], 181), []
        for v in vals:
            q = R.READY_Q.copy(); q[j] = v
            J = R.jacobian(R.q_to_theta(q)); J[3:] *= 100
            s_.append(np.linalg.svd(J, compute_uv=False)[-1])
        ax.plot(np.degrees(vals), s_, color=col, lw=2, label=lab)
    ax.set_xlabel("that joint's servo angle q (deg), the others at the ready pose"); ax.set_ylabel("smallest singular value of J")
    ax.set_title("approach to the singularities from the ready pose", fontsize=10.5); ax.grid(True, lw=0.3); ax.legend(fontsize=9)
    ax = fig.add_axes([0.52, 0.05, 0.45, 0.40]); ax.axis("off")
    ax.text(0, 1, "\n".join(log), va="top", fontsize=9.2, family="monospace")
    p = os.path.join(FIG, "fig_manipulability.png"); fig.savefig(p, dpi=160); plt.close(fig)
    txt = "\n".join(log) + "\n"
    open(os.path.join(KIN, "MANIPULABILITY.log"), "w").write(txt)
    print(txt)


if __name__ == "__main__":
    main()
