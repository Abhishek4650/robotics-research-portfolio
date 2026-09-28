#!/usr/bin/env python3
"""
Step 7 -- sine tracing (after Rsine and the thesis demos), placed with the
task-constrained workspace of step 4.

  VERTICAL BOARD in front of the arm, pen normal to it pointing away from the
  robot (+x) -- the only pen direction a vertical board allows (step 4).
  TABLE: a pen straight down is impossible (J2 + J3 + J5 give 172.5 deg of
  pitch), so the table demo uses a pen tilted TILT deg from straight down,
  leaning out along +x -- what an angled pen holder gives.

For each: the sine is placed automatically (largest one whose every point is
drawable), solved point by point with the closed-form IK (branch nearest the
previous point = continuous motion), then checked: FK error, joint limits,
largest joint step; and re-traced by damped least squares (the Rsine loop,
seeded by the previous point) and by ikpy on the URDF.

Out: figures/fig_sine_demos.png, figures/fig_sine_cad.png, figures/sine_vertical.gif,
     figures/sine_table.gif, SINE_DEMO.log, sine_*_trajectory.csv (servo angles)
"""
import csv
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt            # noqa: E402
from matplotlib import animation           # noqa: E402

from common import R, FIG, KIN, view_basis, proj, draw_arm, table_patch, run_ikpy, cad_posed, render, trimmed, joints_xyz   # noqa: E402
import batch as B                          # noqa: E402
from arm450kin import ik                     # noqa: E402

NPTS = 150
TILT = 30.0             # the table search starts here and tilts more only if it must


def sine_points(kind, c0, width, amp, cycles=2.0, n=NPTS, tilt=None):
    """vertical: along y on the board x = c0[0], centre height c0[2].
    table: along an ARC of radius c0[0] at height c0[2] (the drawable table
    region is a ring round the base), width = arc length, amplitude radial."""
    s = np.linspace(-0.5, 0.5, n)
    h = amp * np.sin(2 * np.pi * cycles * (s + 0.5))
    if kind == "vertical":
        return np.stack([np.full(n, c0[0]), c0[1] + s * width, c0[2] + h], axis=1)
    phi = s * width / c0[0]
    rho = c0[0] + h
    return np.stack([rho * np.cos(phi), rho * np.sin(phi), np.full(n, c0[2])], axis=1)


def pen_axes(kind, P, tilt=TILT):
    """per-point pen axis: +x on the board; on the table tilted `tilt` deg from
    straight down, leaning OUT along the radius (an angled pen holder)."""
    if kind == "vertical":
        return np.tile([1.0, 0.0, 0.0], (len(P), 1))
    t = np.radians(tilt)
    u = P[:, :2] / np.linalg.norm(P[:, :2], axis=1, keepdims=True)
    return np.stack([np.sin(t) * u[:, 0], np.sin(t) * u[:, 1], np.full(len(P), -np.cos(t))], axis=1)


def orient(a, roll=0.0):
    """tool frame with Z along a; X as close to the base's -z / +x as possible, turned by roll."""
    z = a / np.linalg.norm(a)
    ref = np.array([0, 0, -1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0, 0])
    x = ref - (ref @ z) * z; x /= np.linalg.norm(x)
    y = np.cross(z, x)
    c, s = np.cos(roll), np.sin(roll)
    return np.stack([c * x + s * y, -s * x + c * y, z], axis=1)


def place(kind):
    """largest drawable sine: board -- widest (width/amp = 6) over a grid of
    centres; table -- least tilt first, then the longest arc, over heights and radii."""
    if kind == "vertical":
        best = None
        for c0 in [(x0, 0.0, z0) for x0 in (250.0, 300.0, 350.0) for z0 in np.arange(100.0, 330.0, 10.0)]:
            for width in (260.0, 220.0, 200.0, 180.0, 160.0, 140.0, 120.0, 100.0, 80.0):
                if best and width <= best[1]:
                    break
                P = sine_points(kind, c0, width, width / 6.0, n=60)
                if B.pen_feasible(P, pen_axes(kind, P)).all():
                    _, TH, _ = solve(kind, c0, width, width / 6.0, None, n=60)
                    if TH is not None and smooth(TH)[0] < 8.0 and smooth(TH)[1] > 2.0:
                        best = (c0, width, width / 6.0, None)
                        break
        return best
    for tilt in (30.0, 35.0, 40.0, 45.0, 50.0):
        best = None
        for z0 in (-40.0, -20.0, 0.0, 25.0, 50.0):
            for r0 in np.arange(250.0, 390.0, 5.0):
                for width, amp in ((300.0, 12.0), (240.0, 10.0), (200.0, 8.0), (160.0, 6.0)):
                    if best and (amp, width) <= (best[2], best[1]):
                        break
                    P = sine_points(kind, (r0, 0.0, z0), width, amp, n=60)
                    if B.pen_feasible(P, pen_axes(kind, P, tilt)).all():
                        best = ((r0, 0.0, z0), width, amp, tilt)
                        break
        if best:
            return best
    return None


ROLLS = np.radians(np.arange(0.0, 360.0, 5.0))
W_STEP = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 0.5])    # a pen does not care about J6: cheaper to move


def solve(kind, c0, width, amp, tilt, n=NPTS):
    """point by point, the pen's roll FREE: among every valid (roll, branch),
    the one nearest the previous joint vector -- the redundancy of a symmetric
    pen keeps the motion smooth even past the wrist singularity (J4, J6 in line)."""
    P = sine_points(kind, c0, width, amp, n=n)
    A = pen_axes(kind, P, tilt if tilt else TILT)
    TH, th_prev = [], R.q_to_theta(R.READY_Q)
    for p, a in zip(P, A):
        best = None
        for roll in ROLLS:
            for th, lab, ok in ik.analytical(p, orient(a, roll)):
                if ok:
                    c = np.linalg.norm(W_STEP * (th - th_prev))
                    if best is None or c < best[0]:
                        best = (c, th)
        if best is None:
            return P, None, A
        TH.append(best[1]); th_prev = best[1]
    return P, np.array(TH), A


def smooth(TH):
    Q = np.array([R.theta_to_q(th) for th in TH])
    return np.degrees(np.abs(np.diff(Q, axis=0)).max()), np.degrees(np.min(R.Q_RANGE - np.abs(Q)))


def main():
    log = ["SINE TRACING (closed-form IK, continuity = nearest valid branch)", ""]
    demos = {}
    for kind in ("vertical", "table"):
        c0, width, amp, tilt = place(kind)
        P, TH, A = solve(kind, c0, width, amp, tilt)
        Tq = [R.fk(th) for th in TH]
        err = max(np.linalg.norm(T[:3, 3] - p) for T, p in zip(Tq, P))
        aerr = max(np.linalg.norm(T[:3, 2] - a) for T, a in zip(Tq, A))
        Qs = np.array([R.theta_to_q(th) for th in TH])
        lim_ok = all(R.within_limits(th) for th in TH)
        step = np.degrees(np.abs(np.diff(Qs, axis=0)).max())
        margin = np.degrees(np.min(R.Q_RANGE - np.abs(Qs)))
        # DLS re-trace (Rsine loop: seed = previous solution)
        th_d, derr, dit = TH[0].copy(), [], []
        for p, T in zip(P, Tq):
            th_d, info = ik.damped_least_squares(p, T[:3, :3], th_d)
            derr.append(info["pos_err"]); dit.append(info["iters"])
        back = np.eye(4); back[2, 3] = R.GEO["axes"][5]["point"][2] - R.GEO["tool_face_centre"][2]
        kq = np.array(run_ikpy({"targets": [(T @ back).tolist() for T in Tq],
                                "seeds": [list(map(float, R.theta_to_q(th))) for th in np.vstack([TH[:1], TH[:-1]])]})["q"])
        kerr = [np.linalg.norm(R.fk_q(q)[:3, 3] - p) for q, p in zip(kq, P)]
        demos[kind] = dict(P=P, TH=TH, Qs=Qs, c0=c0, width=width, amp=amp, derr=derr, kerr=kerr, A=A, tilt=tilt)
        log += ["%s: %s" % (kind.upper(), "board x = %.0f, pen normal +x (away from the robot)" % c0[0] if kind == "vertical"
                              else "table z = %.0f, pen tilted %.0f deg from straight down, leaning out along the radius"
                              % (c0[2], tilt)),
                ("  sine: centre (%.0f, %.0f, %.0f) mm, width %.0f mm, amplitude %.0f mm, 2 cycles, %d points"
                 % (c0[0], c0[1], c0[2], width, amp, NPTS)) if kind == "vertical" else
                ("  sine along an arc of radius %.0f mm (the drawable ring), arc length %.0f mm, radial amplitude %.0f mm, "
                 "2 cycles, %d points" % (c0[0], width, amp, NPTS)),
                "  closed form: FK error max %.1e mm, pen axis error %.1e; all points inside the joint limits: %s"
                % (err, aerr, lim_ok),
                "  motion: largest joint step between points %.2f deg; smallest margin to a limit %.1f deg (pen roll free)"
                % (step, margin),
                "  DLS re-trace (seed = previous point): max error %.1e mm, mean %.1f iterations" % (max(derr), np.mean(dit)),
                "  ikpy re-trace (URDF): max error %.1e mm" % max(kerr), ""]
        with open(os.path.join(KIN, "sine_%s_trajectory.csv" % kind), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["point", "x_mm", "y_mm", "z_mm"] + ["q%d_deg" % i for i in range(1, 7)])
            for k, (p, q) in enumerate(zip(P, Qs)):
                w.writerow([k] + ["%.3f" % v for v in p] + ["%.4f" % v for v in np.degrees(q)])
    txt = "\n".join(log)
    open(os.path.join(KIN, "SINE_DEMO.log"), "w").write(txt + "\n")
    print(txt)

    # ---- summary figure
    fig = plt.figure(figsize=(19, 12))
    fig.suptitle("Sine tracing on ARM-450 rev I.1 (after Rsine): closed-form IK, placed with the task-constrained workspace",
                 fontsize=14, fontweight="bold")
    for r, kind in enumerate(("vertical", "table")):
        d = demos[kind]
        Bv = view_basis((1.0, -1.25, 0.55))
        ax = fig.add_axes([0.02, 0.52 - 0.48 * r, 0.30, 0.40])
        table_patch(ax, Bv, r=280)
        for k in (0, NPTS // 3, 2 * NPTS // 3, NPTS - 1):
            draw_arm(ax, d["TH"][k], Bv, color="#1f4e99", lw=3, alpha=0.35 + 0.2 * (k == NPTS - 1))
        pp = proj(d["P"], Bv); ax.plot(pp[:, 0], pp[:, 1], color="#d62728", lw=2)
        ax.set_aspect("equal"); ax.axis("off")
        ax.set_title("%s -- %d points, 4 of the poses" % ("vertical board, pen +x" if kind == "vertical" else
                                                          "table z = %.0f, pen tilted %.0f deg" % (d["c0"][2], d["tilt"]), NPTS),
                     fontsize=10.5)
        ax = fig.add_axes([0.37, 0.55 - 0.48 * r, 0.33, 0.36])
        for j in range(6):
            ax.plot(np.degrees(d["Qs"][:, j]), lw=2, label="q%d (+-%g)" % (j + 1, np.degrees(R.Q_RANGE[j])))
        ax.set_xlabel("point"); ax.set_ylabel("servo angle (deg)"); ax.grid(True, lw=0.3)
        ax.legend(fontsize=8, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.15))
        ax.set_title("servo angles along the path (all inside the limits)", fontsize=10.5)
        ax = fig.add_axes([0.76, 0.55 - 0.48 * r, 0.22, 0.36])
        ax.semilogy(np.maximum(d["derr"], 1e-16), label="damped least squares", lw=1.5)
        ax.semilogy(np.maximum(d["kerr"], 1e-16), label="ikpy (URDF)", lw=1.5)
        ce = [np.linalg.norm(R.fk(th)[:3, 3] - p) for th, p in zip(d["TH"], d["P"])]
        ax.semilogy(np.maximum(ce, 1e-16), label="closed form", lw=1.5)
        ax.set_xlabel("point"); ax.set_ylabel("tracking error (mm)"); ax.grid(True, lw=0.3); ax.legend(fontsize=8)
        ax.set_title("tracking error, three solvers", fontsize=10.5)
    p = os.path.join(FIG, "fig_sine_demos.png"); fig.savefig(p, dpi=160); plt.close(fig)

    # ---- the real CAD drawing (4 frames of the vertical board)
    fig = plt.figure(figsize=(19, 7))
    fig.suptitle("The CAD arm posed by the closed-form IK along the vertical-board sine: pen (black) normal to the board, "
                 "the drawn curve in red", fontsize=13, fontweight="bold")
    d = demos["vertical"]
    import trimesh
    for k, i in enumerate((0, NPTS // 4, NPTS // 2, 3 * NPTS // 4, NPTS - 1)):
        segs = [trimesh.creation.cylinder(radius=1.8, segment=[d["P"][j], d["P"][j + 1]], sections=8) for j in range(i)]
        extra, cols = {}, {}
        if segs:
            m = trimesh.util.concatenate(segs); m.apply_translation([-0.8, 0, 0])
            extra["path"] = (np.asarray(m.vertices), np.asarray(m.faces)); cols["path"] = "#d62728"
        # the pen: from the tool face back along -a would sit inside the flange; draw it as the red
        # stub the tool face carries TOWARD the board is the path point itself -- show a 50 mm pen body
        pen = trimesh.creation.cylinder(radius=3.0, segment=[d["P"][i] - 50 * d["A"][i], d["P"][i]], sections=12)
        extra["pen"] = (np.asarray(pen.vertices), np.asarray(pen.faces)); cols["pen"] = "#1a1a1a"
        x0 = d["c0"][0]
        board = trimesh.creation.box(extents=[2.0, 300.0, 200.0])
        board.apply_translation([x0 + 1.2, 0.0, d["c0"][2]])
        extra["board"] = (np.asarray(board.vertices), np.asarray(board.faces)); cols["board"] = "#f3ecd2"
        M, cc = cad_posed(np.degrees(d["Qs"][i]), extra, cols)
        for nm in cc:
            if nm not in ("path", "pen", "board"):
                cc[nm] = "#b9c0c8"
        png = os.path.join(FIG, "_sine_%d.png" % k)
        render(M, cc, png, view=(-0.25, -1.4, 0.4), size=(1100, 1100), zoom=1.0)
        ax = fig.add_axes([0.005 + 0.199 * k, 0.03, 0.19, 0.84]); ax.imshow(trimmed(png), interpolation="none"); ax.axis("off")
        ax.set_title("point %d / %d" % (i + 1, NPTS), fontsize=10)
        os.remove(png)
    p = os.path.join(FIG, "fig_sine_cad.png"); fig.savefig(p, dpi=150); plt.close(fig)

    # ---- animations (stick figure, projected -- this machine has no screen for RViz)
    for kind in ("vertical", "table"):
        d = demos[kind]
        Bv = view_basis((1.0, -1.25, 0.55))
        fig, ax = plt.subplots(figsize=(6.4, 6.4))
        pp = proj(d["P"], Bv)
        J0 = proj(np.vstack([joints_xyz(th) for th in d["TH"]]), Bv)
        ax.set_xlim(J0[:, 0].min() - 60, J0[:, 0].max() + 60); ax.set_ylim(J0[:, 1].min() - 60, J0[:, 1].max() + 60)
        ax.set_aspect("equal"); ax.axis("off")
        frames = list(range(0, NPTS, 2)) + [NPTS - 1]

        def draw(k):
            ax.clear(); ax.set_aspect("equal"); ax.axis("off")
            ax.set_xlim(J0[:, 0].min() - 60, J0[:, 0].max() + 60); ax.set_ylim(J0[:, 1].min() - 60, J0[:, 1].max() + 60)
            table_patch(ax, Bv, r=280)
            ax.plot(pp[:, 0], pp[:, 1], color="#cccccc", lw=1, ls="--")
            ax.plot(pp[:k + 1, 0], pp[:k + 1, 1], color="#d62728", lw=2.2)
            draw_arm(ax, d["TH"][k], Bv, color="#1f4e99", lw=4)
            ax.set_title("ARM-450 rev I.1 -- %s  (point %d/%d)" % ("vertical board" if kind == "vertical" else
                                                                "table, pen tilted %.0f deg" % d["tilt"], k + 1, NPTS), fontsize=10)
        an = animation.FuncAnimation(fig, draw, frames=frames, interval=60)
        an.save(os.path.join(FIG, "sine_%s.gif" % kind), writer=animation.PillowWriter(fps=16))
        plt.close(fig)
    print("wrote fig_sine_demos.png, fig_sine_cad.png, sine_vertical.gif, sine_table.gif")


if __name__ == "__main__":
    main()
