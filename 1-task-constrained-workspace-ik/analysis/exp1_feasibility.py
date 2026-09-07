#!/usr/bin/env python3
"""
Experiment 1 — Task-constrained (orientation-feasible) workspace.

Answers two research questions:
  Q1  Which pen directions are feasible where?  (the "front vs back" map)
  Q2  Can the arm trace on any surface anywhere, incl. a HORIZONTAL table?

Method: over a grid of end-effector target positions we test whether the DLS IK
converges with the pen held normal to the surface, for each candidate normal,
from several seeds (to explore different IK branches). We classify each cell and
compare the orientation-constrained (dexterous) workspace to the raw reachable
workspace.

Run:  python3 analysis/exp1_feasibility.py  ->  figures/fig_feasibility.png
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402
from matplotlib.colors import ListedColormap        # noqa: E402
from matplotlib.patches import Patch                 # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
from mycobot_thesis import robot as R                # noqa: E402
from mycobot_thesis import ik                        # noqa: E402

FIG = os.path.join(PKG, "figures")
os.makedirs(FIG, exist_ok=True)

# seeds spanning different elbow/wrist branches (better feasibility coverage)
SEEDS = [np.zeros(6),
         np.array([0.0, -0.6, 0.8, 0.0, 0.6, 0.0]),
         np.array([0.0, 0.6, -0.8, 0.0, -0.6, 0.0]),
         np.array([0.8, -0.3, 0.5, 0.5, -0.5, 0.0])]
POS_TOL = 2e-3       # 2 mm
ORI_TOL = 3e-2


# A feasible DLS converges in <25 iters; 70 is plenty and keeps infeasible
# cells cheap.
FEAS_ITERS = 70


def pos_reachable(p):
    for q0 in SEEDS:
        _, info = ik.damped_least_squares(p, None, q0, position_only=True,
                                          max_iters=FEAS_ITERS)
        if info["pos_err"] < POS_TOL:
            return True
    return False


def ori_reachable(p, normal, advance):
    Rt = R.target_orientation(normal, advance)
    for q0 in SEEDS:
        _, info = ik.damped_least_squares(p, Rt, q0, max_iters=FEAS_ITERS)
        if info["pos_err"] < POS_TOL and info["ori_err"] < ORI_TOL:
            return True
    return False


# ------- Panel A: vertical surface (pen +/- x) over an x-z slice at y=0 -------
def vertical_slice(nx=44, nz=44):
    xs = np.linspace(-0.05, 0.32, nx)
    zs = np.linspace(0.0, 0.45, nz)
    cat = np.zeros((nz, nx), int)   # 0 none,1 pos-only,2 back,3 front,4 both
    for j, z in enumerate(zs):
        for i, x in enumerate(xs):
            p = np.array([x, 0.0, z])
            if not pos_reachable(p):
                cat[j, i] = 0; continue
            front = ori_reachable(p, [1, 0, 0], [0, 0, 1])   # +x, pen away from robot
            back = ori_reachable(p, [-1, 0, 0], [0, 0, 1])   # -x, pen toward robot
            cat[j, i] = 4 if (front and back) else 3 if front else 2 if back else 1
    return xs, zs, cat


# ------- Panel B: horizontal surface (pen -z, table) over x-y at height z ------
def horizontal_slice(z_table, nx=44, ny=44):
    xs = np.linspace(-0.32, 0.32, nx)
    ys = np.linspace(-0.32, 0.32, ny)
    cat = np.zeros((ny, nx), int)   # 0 none,1 pos-only,2 drawable(pen down)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            p = np.array([x, y, z_table])
            if not pos_reachable(p):
                cat[j, i] = 0; continue
            down = ori_reachable(p, [0, 0, -1], [1, 0, 0])   # pen pointing down
            cat[j, i] = 2 if down else 1
    return xs, ys, cat


def main():
    print("Computing vertical-surface feasibility (x-z slice)...")
    xs, zs, catV = vertical_slice()
    print("Computing horizontal-surface feasibility at two table heights...")
    hx, hy, catH1 = horizontal_slice(0.06)
    _, _, catH2 = horizontal_slice(0.12)

    # summary numbers
    reachV = catV >= 1
    drawableV = catV >= 2
    fracV = 100.0 * drawableV.sum() / max(reachV.sum(), 1)
    frac_front = 100.0 * (catV >= 3).sum() / max(reachV.sum(), 1)
    reachH = catH1 >= 1
    fracH = 100.0 * (catH1 == 2).sum() / max(reachH.sum(), 1)
    print(f"  vertical  : {fracV:5.1f}% of reachable cells allow a perpendicular pen "
          f"({frac_front:.1f}% allow the FRONT/away direction)")
    print(f"  horizontal(z=0.06): {fracH:5.1f}% of reachable cells allow pen-down drawing")

    _plot(xs, zs, catV, hx, hy, catH1, catH2, fracV, frac_front, fracH)
    print("figure ->", os.path.join(FIG, "fig_feasibility.png"))


def _plot(xs, zs, catV, hx, hy, catH1, catH2, fracV, frac_front, fracH):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2))

    cmapV = ListedColormap(["#f2f2f2", "#c9d9ef", "#f2c14e", "#d1495b", "#2e8b57"])
    axes[0].pcolormesh(xs, zs, catV, cmap=cmapV, vmin=0, vmax=4, shading="auto")
    axes[0].scatter(0, 0, c="k", marker="s", s=40)
    axes[0].set_title("(a) Vertical surface — pen direction feasibility\n(x-z slice, y=0)")
    axes[0].set_xlabel("x [m]"); axes[0].set_ylabel("z [m]"); axes[0].set_aspect("equal")
    legV = [Patch(fc="#f2f2f2", ec="0.5", label="unreachable"),
            Patch(fc="#c9d9ef", ec="0.5", label="position only (no ⊥ pen)"),
            Patch(fc="#f2c14e", ec="0.5", label="BACK only (toward robot)"),
            Patch(fc="#d1495b", ec="0.5", label="FRONT only (away)"),
            Patch(fc="#2e8b57", ec="0.5", label="both directions")]
    axes[0].legend(handles=legV, loc="upper left", fontsize=7, framealpha=0.95)

    cmapH = ListedColormap(["#f2f2f2", "#c9d9ef", "#2e8b57"])
    for ax, cat, zt in ((axes[1], catH1, 0.06), (axes[2], catH2, 0.12)):
        ax.pcolormesh(hx, hy, cat, cmap=cmapH, vmin=0, vmax=2, shading="auto")
        ax.scatter(0, 0, c="k", marker="s", s=40)
        ax.set_title(f"({'b' if zt == 0.06 else 'c'}) Horizontal table at z={zt} m\n"
                     "pen-down feasibility (x-y)")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_aspect("equal")
    legH = [Patch(fc="#f2f2f2", ec="0.5", label="unreachable"),
            Patch(fc="#c9d9ef", ec="0.5", label="position only"),
            Patch(fc="#2e8b57", ec="0.5", label="drawable (pen down)")]
    axes[2].legend(handles=legH, loc="upper right", fontsize=7, framealpha=0.95)

    fig.suptitle("Task-constrained workspace: where the myCobot 280 can hold the pen "
                 "normal to the surface", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(os.path.join(FIG, "fig_feasibility.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
