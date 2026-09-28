#!/usr/bin/env python3
"""
Step 3 -- inverse kinematics: the closed-form solution and its cross-checks.

  (a) 5000 random poses inside the joint limits: FK -> all 8 closed-form
      branches -> FK again; every branch must land on the pose and the
      original configuration must be among the valid ones.
  (b) the same targets by damped least squares (the author's Rsine method) from
      the ready pose, and by ikpy on the URDF (own interpreter): do they
      agree with a closed-form branch?
  (c) figure: the 8 branches of one pose, which ones the joint limits allow.

Out: figures/fig_ik_branches.png, VERIFY_IK.log
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt            # noqa: E402

from common import R, FIG, KIN, view_basis, draw_arm, table_patch, run_ikpy   # noqa: E402
from arm450kin import ik                     # noqa: E402

N = 5000


def main():
    rng = np.random.default_rng(11)
    Q = rng.uniform(R.Q_LIMITS[:, 0], R.Q_LIMITS[:, 1], size=(N, 6))
    worst, missing, nvalid = 0.0, 0, []
    for q in Q:
        th = R.q_to_theta(q); T = R.fk(th)
        sols = ik.analytical(T[:3, 3], T[:3, :3])
        worst = max([worst] + [np.abs(R.fk(s[0]) - T).max() for s in sols])
        ok = [s[0] for s in sols if s[2]]
        nvalid.append(len(ok))
        if not any(np.abs(R.wrap(s) - R.wrap(th)).max() < 1e-6 for s in ok):
            missing += 1
    # (b) three solvers, 200 of the targets. GLOBAL: from the fixed ready pose.
    # LOCAL (as along a traced path): from the true solution moved 3 deg per joint.
    sub = Q[:200]
    targets = [R.fk_q(q) for q in sub]
    seed_th = R.q_to_theta(R.READY_Q)
    near = [np.clip(q + np.radians(3.0) * rng.choice([-1, 1], 6), R.Q_LIMITS[:, 0] + 1e-6, R.Q_LIMITS[:, 1] - 1e-6) for q in sub]

    def dls_run(seeds_th):
        ok = match = 0
        for T, s0 in zip(targets, seeds_th):
            th, info = ik.damped_least_squares(T[:3, 3], T[:3, :3], s0)
            if info["converged"]:
                ok += 1
                sols = [s[0] for s in ik.analytical(T[:3, 3], T[:3, :3]) if s[2]]
                match += any(np.abs(R.wrap(th) - s).max() < 1e-4 for s in sols)
        return ok, match

    def ikpy_run(seeds_q):
        # ikpy solves for the URDF's last link frame (on Z6 at the J6 servo point), not the tool face
        back = np.eye(4); back[2, 3] = R.GEO["axes"][5]["point"][2] - R.GEO["tool_face_centre"][2]
        kq = np.array(run_ikpy({"targets": [(T @ back).tolist() for T in targets],
                                "seeds": [list(map(float, s)) for s in seeds_q]})["q"])
        ok, err = 0, []
        for q, T in zip(kq, targets):
            Tq = R.fk_q(q); e = np.linalg.norm(Tq[:3, 3] - T[:3, 3]); err.append(e)
            ok += (e < 0.01 and np.linalg.norm(Tq[:3, :3] - T[:3, :3]) < 1e-3 and R.within_limits(R.q_to_theta(q)))
        return ok, float(np.median(err))
    g_dls, g_dls_m = dls_run([seed_th] * len(sub))
    l_dls, l_dls_m = dls_run([R.q_to_theta(q) for q in near])
    g_ik, g_ik_e = ikpy_run([R.READY_Q] * len(sub))
    l_ik, l_ik_e = ikpy_run(near)
    txt = ("INVERSE KINEMATICS VERIFICATION\n\n"
           "(a) closed form, %d random poses inside the joint limits\n"
           "    worst FK(branch) - target, all 8 branches:   %.2e mm\n"
           "    poses whose original configuration was NOT among the valid branches: %d\n"
           "    valid branches per pose (joint limits): mean %.2f, min %d, max %d; histogram %s\n\n"
           "(b) 200 of the poses, full pose (position + orientation), joint limits enforced\n"
           "    LOCAL -- seed = the true solution moved 3 deg per joint (what a traced path gives):\n"
           "      damped least squares (Rsine method): %3d / 200 converged to 1e-6 mm, %d of them on a closed-form branch\n"
           "      ikpy on the URDF (cross-check):      %3d / 200 within 0.01 mm + 1e-3 rad, median error %.2g mm\n"
           "    GLOBAL -- seed = the fixed ready pose q = (0, 30, -60, 0, 30, 0) deg, target anywhere:\n"
           "      damped least squares:                %3d / 200 (%d on a closed-form branch)\n"
           "      ikpy:                                %3d / 200, median error %.3g mm\n"
           "    -> numerical solvers are LOCAL: exact near a seed, unreliable across the workspace; the closed\n"
           "       form has no seed and returns every branch -- use it, and the numerical ones along paths.\n"
           % (N, worst, missing, np.mean(nvalid), min(nvalid), max(nvalid), np.bincount(nvalid).tolist(),
              l_dls, l_dls_m, l_ik, l_ik_e, g_dls, g_dls_m, g_ik, g_ik_e))
    open(os.path.join(KIN, "VERIFY_IK.log"), "w").write(txt)
    print(txt)

    # (c) the 8 branches of a pose reachable by two of them
    q0 = np.radians([35.0, 25.0, -50.0, 40.0, 30.0, -20.0])
    T = R.fk_q(q0)
    sols = ik.analytical(T[:3, 3], T[:3, :3])
    B = view_basis((1.0, -1.35, 0.55))
    fig, axs = plt.subplots(2, 4, figsize=(18, 10.5))
    fig.suptitle("Closed-form IK: the 8 branches for one tool pose (TCP %s mm) -- green = inside every joint limit"
                 % np.round(T[:3, 3], 1), fontsize=14, fontweight="bold")
    for ax, (th, lab, ok) in zip(axs.ravel(), sols):
        table_patch(ax, B, r=260)
        draw_arm(ax, th, B, color="#1a9641" if ok else "#c0392b", lw=4)
        tp = (B[0] @ T[:3, 3], B[1] @ T[:3, 3]); ax.plot(*tp, "*", ms=16, color="#f0a000", mec="k")
        q = np.degrees(R.theta_to_q(th))
        bad = [i + 1 for i in range(6) if abs(q[i]) > np.degrees(R.Q_RANGE[i]) + 1e-6]
        ax.set_title("%s\nq = (%s) deg\n%s" % (lab, ", ".join("%.0f" % v for v in q),
                                              "VALID" if ok else "outside the limits of J%s" % ",".join(map(str, bad))),
                     fontsize=9.5, color="#1a6e2e" if ok else "#9b2c20")
        ax.set_aspect("equal"); ax.axis("off")
        ax.set_xlim(-330, 330); ax.set_ylim(-190, 560)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    p = os.path.join(FIG, "fig_ik_branches.png"); fig.savefig(p, dpi=170); fig.savefig(p[:-4] + ".pdf"); plt.close(fig)
    print("wrote", p)


if __name__ == "__main__":
    main()
