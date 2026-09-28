#!/usr/bin/env python3
"""
Step 4 -- workspace (after the myCobot thesis, Exp. 1).

  REACHABLE workspace: every tool-face position the arm reaches with SOME
  orientation, all six joints inside their limits (Monte Carlo, 400 000
  poses), and the wrist-centre workspace that shapes it.
  TASK-CONSTRAINED ("drawable") workspace: where the tool can stand NORMAL to
  a surface -- pen straight down on a horizontal table, pen along +-x on a
  vertical board -- with its roll free (a pen is symmetric). Exact per grid
  cell from the closed-form wrist decoupling (batch.py), no sampling.

Out: figures/fig_workspace.png, figures/fig_task_workspace.png, WORKSPACE.log
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt            # noqa: E402
from matplotlib.colors import ListedColormap   # noqa: E402

from common import R, FIG, KIN, view_basis, proj, draw_arm   # noqa: E402
import batch as B                                             # noqa: E402
from arm450kin import ik                                      # noqa: E402

N_MC = 400000
STEP = 8.0                       # task-map grid (mm)
H_PLANES = [-40.0, 50.0, 150.0, 250.0, 350.0]
V_PLANES = [150.0, 250.0, 350.0]


def fibonacci_dirs(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n); th = np.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], axis=1)


def reachable_any(P, dirs):
    """position reachable with SOME tool direction (dirs sampled on the sphere)."""
    ok = np.zeros(len(P), bool)
    for a in dirs:
        todo = ~ok
        if not todo.any():
            break
        ok[todo] = B.pen_feasible(P[todo], a)
    return ok


def self_checks():
    rng = np.random.default_rng(3)
    TH = np.array([R.q_to_theta(q) for q in rng.uniform(R.Q_LIMITS[:, 0], R.Q_LIMITS[:, 1], size=(300, 6))])
    Tb = B.fk_batch(TH)
    e1 = max(np.abs(Tb[k] - R.fk(TH[k])).max() for k in range(len(TH)))
    # pen_feasible vs the full closed-form IK (roll swept) on random pens
    Qs = rng.uniform(R.Q_LIMITS[:, 0], R.Q_LIMITS[:, 1], size=(200, 6))
    Ts = [R.fk_q(q) for q in Qs]                                   # 200 pens the arm really holds
    P = np.vstack([np.array([T[:3, 3] for T in Ts]), rng.uniform([-450, -450, -100], [450, 450, 560], size=(200, 3))])
    A = np.vstack([np.array([T[:3, 2] for T in Ts]), rng.normal(size=(200, 3))])
    A /= np.linalg.norm(A, axis=1, keepdims=True)
    fb = B.pen_feasible(P, A)
    fr = []
    for p, a in zip(P, A):
        x = np.cross(a, [1.0, 0, 0]) if abs(a[0]) < 0.9 else np.cross(a, [0, 1.0, 0]); x /= np.linalg.norm(x)
        y = np.cross(a, x)
        hit = False
        for psi in np.radians(np.arange(0, 360, 5)):
            Rt = np.stack([np.cos(psi) * x + np.sin(psi) * y, -np.sin(psi) * x + np.cos(psi) * y, a], axis=1)
            if any(s[2] for s in ik.analytical(p, Rt)):
                hit = True; break
        fr.append(hit)
    agree = np.mean(fb == np.array(fr))
    return e1, agree, int(np.sum(fb)), int(np.sum(fr))


def min_tilt(Pg, normal, tilts=(0, 5, 10, 15, 20, 25, 30, 40, 50, 60, 75, 90), naz=24):
    """smallest angle (deg) between the tool axis and the surface normal that
    some branch can hold at each point (roll free); nan = not even at 90."""
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    u = np.cross(n, [1.0, 0, 0]) if abs(n[0]) < 0.9 else np.cross(n, [0, 1.0, 0]); u /= np.linalg.norm(u)
    v = np.cross(n, u)
    best = np.full(len(Pg), np.nan)
    for t in tilts:
        todo = np.isnan(best)
        if not todo.any():
            break
        tr = np.radians(t)
        for az in (np.radians(np.arange(0, 360, 360 / naz)) if t else [0.0]):
            a = np.cos(tr) * n + np.sin(tr) * (np.cos(az) * u + np.sin(az) * v)
            idx = np.where(np.isnan(best))[0]
            ok = B.pen_feasible(Pg[idx], a)
            best[idx[ok]] = t
    return best


def main():
    e1, agree, nb, nr = self_checks()
    log = ["WORKSPACE", "", "self-checks: batch FK vs model FK %.1e mm; pen feasibility batch vs full IK (roll swept 5 deg) "
           "agree on %.1f %% of 400 pens (200 held by real poses + 200 random; %d vs %d feasible)" % (e1, 100 * agree, nb, nr), ""]
    rng = np.random.default_rng(5)
    Q = rng.uniform(R.Q_LIMITS[:, 0], R.Q_LIMITS[:, 1], size=(N_MC, 6))
    TH = R.SIGN * Q + R.DH[:, 3]              # theta = sign * q + offset, all poses at once
    T = B.fk_batch(TH)
    P = T[:, :3, 3]
    Wc = B.fk_batch(TH, upto=5, tool=False)[:, :3, 3]
    r = np.hypot(P[:, 0], P[:, 1])
    dsh = np.linalg.norm(P - np.array([0, 0, R.D1]), axis=1)
    above = P[:, 2] > R.TABLE_Z
    vox = 10.0
    occ = np.unique(np.floor(P[above] / vox).astype(int), axis=0)
    log += ["REACHABLE (tool-flange face, %d random poses inside the joint limits)" % N_MC,
            "  horizontal reach from the J1 axis: max %.1f mm" % r.max(),
            "  distance from the shoulder (0, 0, %.0f): max %.1f mm  (a2 + d4 + d_T = %.1f)" % (R.D1, dsh.max(), R.A2 + R.D4 + R.D_TOOL),
            "  height: %.1f .. %.1f mm; below the table top (z < %.1f): %.1f %% of poses"
            % (P[:, 2].min(), P[:, 2].max(), R.TABLE_Z, 100 * np.mean(~above)),
            "  volume above the table (occupied %.0f mm voxels): %.2f litres" % (vox, len(occ) * vox ** 3 / 1e6),
            "  wrist centre: r max %.1f mm, z %.1f .. %.1f mm" % (np.hypot(Wc[:, 0], Wc[:, 1]).max(), Wc[:, 2].min(), Wc[:, 2].max()), ""]

    # ---- figure: reachable workspace
    fig = plt.figure(figsize=(18, 8.2))
    fig.suptitle("ARM-450 rev I.1 -- reachable workspace (tool-flange face, %d random poses, every joint inside its limits)"
                 % N_MC, fontsize=14, fontweight="bold")
    sel = rng.choice(N_MC, 60000, replace=False)
    ax = fig.add_axes([0.03, 0.08, 0.30, 0.80])
    ax.scatter(P[sel, 0], P[sel, 1], s=0.3, c=P[sel, 2], cmap="viridis", alpha=0.5)
    t = np.linspace(0, 2 * np.pi, 200)
    ax.plot(r.max() * np.cos(t), r.max() * np.sin(t), "k--", lw=0.8)
    ax.add_patch(plt.Circle((0, 0), 64, color="#777", alpha=0.6))
    ax.set_aspect("equal"); ax.set_title("top view (x-y), colour = height; dashed: max reach %.0f mm; grey: foot" % r.max(), fontsize=10)
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.grid(True, lw=0.3)
    ax = fig.add_axes([0.36, 0.08, 0.30, 0.80])
    xs = np.where(np.abs(P[:, 1]) < 15)[0]
    ax.scatter(P[xs, 0], P[xs, 2], s=0.6, c="#3a78c2", alpha=0.35, label="tool face, |y| < 15 mm")
    ws = np.where(np.abs(Wc[:, 1]) < 15)[0]
    ax.scatter(Wc[ws, 0], Wc[ws, 2], s=0.6, c="#e08030", alpha=0.35, label="wrist centre W, |y| < 15 mm")
    ax.axhline(R.TABLE_Z, color="#b59f68", lw=3); ax.text(-440, R.TABLE_Z + 8, "table top", fontsize=9)
    for q, c in ((R.HOME_Q, "#222"), (R.READY_Q, "#b0181a")):
        J = [R.fk_frames(R.q_to_theta(q))[i][:3, 3] for i in (0, 2, 3, 5, 7)]
        ax.plot([p[0] for p in J], [p[2] for p in J], "-o", color=c, lw=2.5, ms=4)
    ax.set_aspect("equal"); ax.set_xlim(-470, 470); ax.set_ylim(-130, 580); ax.grid(True, lw=0.3)
    ax.set_title("side slice (x-z, |y| < 15 mm): tool face (blue), wrist centre (orange);\nhome (black) and ready pose (red)", fontsize=10)
    ax.set_xlabel("x (mm)"); ax.set_ylabel("z (mm)"); ax.legend(loc="lower center", fontsize=8, markerscale=12)
    ax = fig.add_axes([0.69, 0.08, 0.30, 0.80])
    Bv = view_basis((1.0, -1.3, 0.6))
    Pp = proj(P[sel], Bv)
    ax.scatter(Pp[:, 0], Pp[:, 1], s=0.2, c=P[sel, 2], cmap="viridis", alpha=0.35)
    draw_arm(ax, R.q_to_theta(R.READY_Q), Bv, color="#b0181a")
    ax.set_aspect("equal"); ax.axis("off"); ax.set_title("3-D view (projected), ready pose drawn", fontsize=10)
    p = os.path.join(FIG, "fig_workspace.png"); fig.savefig(p, dpi=170); plt.close(fig)

    # ---- task-constrained maps
    dirs = fibonacci_dirs(400)
    g = np.arange(-460, 461, STEP)
    fig, axs = plt.subplots(2, 5, figsize=(24, 12))
    fig.suptitle("Task-constrained workspace (after the myCobot thesis): where the tool can stand NORMAL to a surface, roll free\n"
                 "top: horizontal tables -- colour = least pen tilt from straight down that some pose can hold (grey: reachable only);  "
                 "bottom: vertical boards -- green front pen (+x), blue back pen (-x). Exact per %g mm cell." % STEP,
                 fontsize=13, fontweight="bold")
    cm = ListedColormap(["white", "#c9ccd1", "#2ca25f", "#1f78b4"])
    log.append("TASK-CONSTRAINED (drawable) -- share of the reachable cells where the tool can stand normal to the surface")
    log.append("  NOTE: straight down needs 180 deg of pitch; J2 + J3 + J5 give at most 54 + 72 + 46.5 = %.1f deg" %
               np.degrees(R.Q_RANGE[1] + R.Q_RANGE[2] + R.Q_RANGE[4]))
    tilt_cm = plt.get_cmap("RdYlGn_r")
    least = {}
    for k, z0 in enumerate(H_PLANES):
        X, Y = np.meshgrid(g, g)
        Pg = np.stack([X.ravel(), Y.ravel(), np.full(X.size, z0)], axis=1)
        rea = reachable_any(Pg, dirs)
        down = B.pen_feasible(Pg, [0, 0, -1.0])
        mt = min_tilt(Pg, [0, 0, -1.0])
        rea |= np.isfinite(mt) | down        # every direction tried counts towards "reachable"
        ax = axs[0, k]
        ax.imshow(np.where(rea, 0.5, np.nan).reshape(X.shape), origin="lower", extent=[g[0], g[-1], g[0], g[-1]],
                  cmap=ListedColormap(["#d7d9dd"]), interpolation="nearest")
        im = ax.imshow(mt.reshape(X.shape), origin="lower", extent=[g[0], g[-1], g[0], g[-1]], cmap=tilt_cm, vmin=0, vmax=90,
                       interpolation="nearest")
        ax.add_patch(plt.Circle((0, 0), 64, color="#555", alpha=0.5))
        share = 100 * down.sum() / max(rea.sum(), 1)
        ok30 = np.nanmin(mt) if np.isfinite(mt).any() else np.nan
        least[z0] = ok30
        ax.set_title("table z = %g mm\nstraight down %.0f %%, least tilt %s deg"
                     % (z0, share, "%.0f" % ok30 if np.isfinite(ok30) else "-"), fontsize=10.5)
        ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
        log.append("  horizontal z = %6.1f  pen straight down: %5.1f %% of %5d reachable cells; least tilt that works %s deg; "
                   "<= 30 deg on %.1f %%, <= 45 deg on %.1f %%"
                   % (z0, share, rea.sum(), "%.0f" % ok30 if np.isfinite(ok30) else "-",
                      100 * np.sum(mt <= 30) / max(rea.sum(), 1), 100 * np.sum(mt <= 45) / max(rea.sum(), 1)))
    cb = fig.add_axes([0.985, 0.55, 0.006, 0.33]); fig.colorbar(im, cax=cb); cb.set_title("deg", fontsize=8)
    for k, x0 in enumerate(V_PLANES):
        Yv, Zv = np.meshgrid(g, np.arange(-100, 581, STEP))
        Pg = np.stack([np.full(Yv.size, x0), Yv.ravel(), Zv.ravel()], axis=1)
        rea = reachable_any(Pg, dirs)
        front = B.pen_feasible(Pg, [1.0, 0, 0])
        back = B.pen_feasible(Pg, [-1.0, 0, 0])
        rea |= front | back
        img = np.where(front & back, 3, np.where(front, 2, np.where(back, 3, np.where(rea, 1, 0)))).reshape(Yv.shape)
        ax = axs[1, k]
        ax.imshow(img, origin="lower", extent=[g[0], g[-1], -100, 580], cmap=cm, vmin=0, vmax=3, interpolation="nearest")
        ax.axhline(R.TABLE_Z, color="#b59f68", lw=2)
        sf, sb = 100 * front.sum() / max(rea.sum(), 1), 100 * back.sum() / max(rea.sum(), 1)
        ax.set_title("board x = %g mm, pen normal\nfront (+x) %.1f %%, back (-x) %.1f %%" % (x0, sf, sb), fontsize=10.5)
        ax.set_xlabel("y (mm)"); ax.set_ylabel("z (mm)")
        log.append("  vertical   x = %6.1f  pen +x (front): %5.1f %%, pen -x (back): %5.1f %% of %6d reachable cells"
                   % (x0, sf, sb, rea.sum()))
    # one more panel: radial board, pen pointing outward along the radius
    ax = axs[1, 3]
    rg, zg = np.meshgrid(np.arange(0, 461, STEP / 2), np.arange(-100, 581, STEP / 2))
    Pg = np.stack([rg.ravel(), np.zeros(rg.size), zg.ravel()], axis=1)
    rea = reachable_any(Pg, dirs)
    out_ = B.pen_feasible(Pg, [1.0, 0, 0]); down = B.pen_feasible(Pg, [0, 0, -1.0])
    img = np.where(out_ & down, 3, np.where(out_, 2, np.where(down, 3, np.where(rea, 1, 0)))).reshape(rg.shape)
    front_x = Pg[out_, 0].min() if out_.any() else np.nan
    ax.imshow(img, origin="lower", extent=[0, 460, -100, 580], cmap=cm, vmin=0, vmax=3, interpolation="nearest")
    ax.axhline(R.TABLE_Z, color="#b59f68", lw=2)
    ax.set_title("x-z half plane (y = 0)\ntool along +x (green) / straight down (blue)", fontsize=10.5)
    ax.set_xlabel("x (mm)"); ax.set_ylabel("z (mm)")
    axs[1, 4].axis("off")
    summ = ["WHAT IT MEANS", "",
            "Straight down is impossible:", "J2 + J3 + J5 tilt at most", "54 + 72 + 46.5 = 172.5 deg < 180.", "",
            "Table drawing needs a TILTED pen:",
            "least tilt %.0f deg at z = %g," % (least[H_PLANES[0]], H_PLANES[0]),
            "%.0f deg at z = %g, more higher up." % (least[H_PLANES[1]], H_PLANES[1]),
            "-> an angled pen holder,", "   or a sloped board.", "",
            "Vertical board: only the FRONT pen", "(pointing away from the robot),",
            "tool face from x = %.0f mm out;" % front_x,
            "the back pen (toward the robot)", "is never possible."]
    axs[1, 4].text(0.02, 0.98, "\n".join(summ), va="top", fontsize=10.5, family="monospace", transform=axs[1, 4].transAxes)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    p = os.path.join(FIG, "fig_task_workspace.png"); fig.savefig(p, dpi=150); plt.close(fig)
    txt = "\n".join(log) + "\n"
    open(os.path.join(KIN, "WORKSPACE.log"), "w").write(txt)
    print(txt)


if __name__ == "__main__":
    main()
