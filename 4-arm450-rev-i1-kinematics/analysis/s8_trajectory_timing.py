#!/usr/bin/env python3
"""
Step 8 -- the sine as a TIMED trajectory on the real servos (the "future
work" of Rsine: trajectory timing against the velocity limits).

  * A pen is symmetric: the task is 5-D (tool-face position + pen axis), the
    arm has 6 joints -- one redundant DOF. The path is followed by RESOLVED
    RATE with the velocity-propagation Jacobian:
        dtheta = J5^+ dx5 + (I - J5^+ J5) z,     J5 = [J_v ; u^T J_w ; w^T J_w]
    (u, w span the plane normal to the pen axis; z pulls the joints toward the
    centre of their ranges), plus a Newton correction per point so nothing
    drifts. The start is the closed-form solution of step 7.
  * Timed at a constant pen speed; joint speeds against the ST3215 rating
    (Waveshare: 0.222 s / 60 deg at 12 V = 270 deg/s).
  * Encoder resolution (360/4096 = 0.088 deg): the tool-face error it allows
    at each point, |dx| <= sum_i |J_v,i| . dq/2 -- a floor on drawing accuracy.

Out: figures/fig_timing.png, TIMING.log
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt            # noqa: E402

from common import R, FIG, KIN            # noqa: E402
import s7_sine_demo as S7                  # noqa: E402

V_RATED = 270.0                    # deg/s, ST3215 no-load at 12 V (Waveshare wiki)
RES = 360.0 / 4096.0               # deg per encoder count
SPEEDS = [10.0, 20.0, 40.0]        # pen speeds shown (mm/s)


def task_jacobian(th, a):
    J = R.jacobian(th)
    u = np.cross(a, [1.0, 0, 0]) if abs(a[0]) < 0.9 else np.cross(a, [0, 1.0, 0]); u /= np.linalg.norm(u)
    w = np.cross(a, u)
    return np.vstack([J[:3], u @ J[3:], w @ J[3:]]), u, w


def resolved_rate(P, A, th0, k_null=0.2, newton=4):
    """follow (P[k], A[k]) from th0; returns the joint path."""
    mid = R.TH_LIMITS.mean(axis=1); half = (R.TH_LIMITS[:, 1] - R.TH_LIMITS[:, 0]) / 2
    TH = [np.array(th0, float)]
    for k in range(1, len(P)):
        th = TH[-1].copy()
        for it in range(newton + 1):
            T = R.fk(th)
            J5, u, w = task_jacobian(th, T[:3, 2])
            e_ang = np.cross(T[:3, 2], A[k])               # rotation that takes the current pen axis onto the target
            e = np.hstack([P[k] - T[:3, 3], u @ e_ang, w @ e_ang])
            if np.linalg.norm(e[:3]) < 1e-10 and np.linalg.norm(e[3:]) < 1e-12:
                break
            Jp = np.linalg.pinv(J5)
            z = -k_null * (th - mid) / half ** 2 if it == 0 else np.zeros(6)
            th = th + Jp @ e + (np.eye(6) - Jp @ J5) @ z
        TH.append(th)
    return np.array(TH)


def main():
    log = ["TIMED TRAJECTORIES + ENCODER RESOLUTION", "",
           "ST3215 (Waveshare wiki): 0.222 s/60 deg at 12 V = %.0f deg/s no-load; 4096 counts/rev = %.4f deg/count"
           % (V_RATED, RES), ""]
    res = {}
    for kind in ("vertical", "table"):
        c0, width, amp, tilt = S7.place(kind)
        P, TH_cf, A = S7.solve(kind, c0, width, amp, tilt)
        TH = resolved_rate(P, A, TH_cf[0])
        Tq = [R.fk(th) for th in TH]
        pos_err = max(np.linalg.norm(T[:3, 3] - p) for T, p in zip(Tq, P))
        ax_err = max(np.linalg.norm(T[:3, 2] - a) for T, a in zip(Tq, A))
        lim = all(R.within_limits(th) for th in TH)
        Q = np.degrees(np.array([R.theta_to_q(th) for th in TH]))
        ds = np.linalg.norm(np.diff(P, axis=0), axis=1)
        s = np.concatenate([[0], np.cumsum(ds)])
        dqds = np.abs(np.diff(Q, axis=0)) / ds[:, None]              # deg per mm of pen travel
        v_max = V_RATED / dqds.max()                                  # pen speed at which a joint hits the rating
        # encoder floor: every joint off by half a count, worst signs
        floor = np.array([np.sum(np.linalg.norm(R.jacobian(th)[:3], axis=0)) * np.radians(RES / 2) for th in TH])
        step_cf = np.abs(np.diff(np.degrees([R.theta_to_q(t) for t in TH_cf]), axis=0)).max()
        res[kind] = dict(P=P, Q=Q, s=s, dqds=dqds, floor=floor, v_max=v_max, length=s[-1])
        log += ["%s sine (%s): path %.0f mm" % (kind.upper(), "board x = %.0f" % c0[0] if kind == "vertical"
                                                 else "table z = %.0f, pen tilt %.0f deg" % (c0[2], tilt), s[-1]),
                "  resolved rate (5-D task, roll free, null space to the joint centres): error %.1e mm, pen axis %.1e; "
                "inside the limits: %s" % (pos_err, ax_err, lim),
                "  largest joint rate per mm of pen travel %.2f deg/mm (joint %d); closed-form path of step 7: largest step %.2f deg"
                % (dqds.max(), int(np.argmax(dqds.max(axis=0))) + 1, step_cf),
                "  pen speed at which a servo reaches its no-load rating: %.0f mm/s; at half the rating: %.0f mm/s"
                % (v_max, v_max / 2),
                "  one pass of the sine at half the rating: %.1f s" % (s[-1] / (v_max / 2)),
                "  encoder floor (every joint off by half a count): tool face %.2f .. %.2f mm along the path"
                % (floor.min(), floor.max()), ""]
    log += ["-> drawing faster than about half the no-load pen speed above leaves the servos no margin (the rating is",
            "   no-load; under the arm's own weight they are slower). The encoder alone limits a drawn line to about",
            "   the floor above -- a real line is also limited by gear backlash and the printed parts' play."]
    txt = "\n".join(log) + "\n"
    open(os.path.join(KIN, "TIMING.log"), "w").write(txt)
    print(txt)

    fig, axs = plt.subplots(2, 3, figsize=(19, 10.5))
    fig.suptitle("Sine as a timed trajectory on the ST3215 servos: resolved rate (roll free), joint speeds vs the rating, "
                 "encoder floor", fontsize=14, fontweight="bold")
    for r, kind in enumerate(("vertical", "table")):
        d = res[kind]
        ax = axs[r, 0]
        for j in range(6):
            ax.plot(d["s"], d["Q"][:, j], lw=2, label="q%d" % (j + 1))
        ax.set_xlabel("pen travel (mm)"); ax.set_ylabel("servo angle (deg)"); ax.grid(True, lw=0.3); ax.legend(fontsize=8, ncol=3)
        ax.set_title("%s: servo angles (resolved rate)" % kind, fontsize=11)
        ax = axs[r, 1]
        v = 20.0
        sm = (d["s"][1:] + d["s"][:-1]) / 2
        for j in range(6):
            ax.plot(sm, d["dqds"][:, j] * v, lw=1.8, label="q%d" % (j + 1))
        ax.axhline(V_RATED, color="r", ls="--", lw=1); ax.axhline(V_RATED / 2, color="r", ls=":", lw=1)
        ax.text(0, V_RATED * 0.53, "half the rating", color="r", fontsize=9)
        ax.set_yscale("log"); ax.set_ylim(0.1, 400)
        ax.set_xlabel("pen travel (mm)"); ax.set_ylabel("|servo speed| (deg/s)")
        ax.set_title("%s: servo speeds at %.0f mm/s pen speed; the rating is hit at %.0f mm/s"
                     % (kind, v, d["v_max"]), fontsize=10.5)
        ax.grid(True, which="both", lw=0.3)
        ax = axs[r, 2]
        ax.plot(d["s"], d["floor"], lw=2, color="#6a3d9a")
        ax.set_xlabel("pen travel (mm)"); ax.set_ylabel("worst tool-face error (mm)")
        ax.set_title("%s: encoder floor, +-half a count on every joint" % kind, fontsize=10.5); ax.grid(True, lw=0.3)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    p = os.path.join(FIG, "fig_timing.png"); fig.savefig(p, dpi=160); plt.close(fig)
    print("wrote", p)


if __name__ == "__main__":
    main()
