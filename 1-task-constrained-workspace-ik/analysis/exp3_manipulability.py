#!/usr/bin/env python3
"""
Experiment 3 — manipulability and conditioning across the workspace.

For each reachable cell of a workspace slice we solve position IK, then evaluate
at that configuration:
    * Yoshikawa manipulability   w = sqrt(det(J_v J_v^T))   (translational)
    * condition number           k = sigma_max / sigma_min  of J_v
Low w / high k mark near-singular, poorly-conditioned regions where drawing is
inaccurate and control is twitchy; high w / low k mark good drawing zones.

Run:  python3 analysis/exp3_manipulability.py -> figures/fig_manipulability.png
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

SEEDS = [np.zeros(6),
         np.array([0.0, -0.6, 0.8, 0.0, 0.6, 0.0]),
         np.array([0.0, 0.6, -0.8, 0.0, -0.6, 0.0])]
POS_TOL = 2e-3


def config_at(p):
    """Return a joint config reaching p (position IK), or None if unreachable."""
    best = None
    for q0 in SEEDS:
        q, info = ik.damped_least_squares(p, None, q0, position_only=True, max_iters=80)
        if info["pos_err"] < POS_TOL:
            if best is None or R.manipulability(q, "pos") > R.manipulability(best, "pos"):
                best = q
    return best


def slice_maps(points, shape):
    W = np.full(shape, np.nan)
    K = np.full(shape, np.nan)
    for idx, p in points:
        q = config_at(p)
        if q is not None:
            W[idx] = R.manipulability(q, "pos")
            K[idx] = min(R.condition_number(q, "pos"), 1e3)
    return W, K


def horizontal_points(z, nx=42, ny=42):
    xs = np.linspace(-0.30, 0.30, nx); ys = np.linspace(-0.30, 0.30, ny)
    pts = [((j, i), np.array([x, y, z])) for j, y in enumerate(ys) for i, x in enumerate(xs)]
    return xs, ys, pts, (ny, nx)


def vertical_points(y, nx=42, nz=42):
    xs = np.linspace(-0.02, 0.30, nx); zs = np.linspace(0.0, 0.42, nz)
    pts = [((j, i), np.array([x, y, z])) for j, z in enumerate(zs) for i, x in enumerate(xs)]
    return xs, zs, pts, (nz, nx)


def main():
    print("manipulability map — horizontal slice (z=0.10)...")
    hx, hy, hpts, hshape = horizontal_points(0.10)
    Wh, Kh = slice_maps(hpts, hshape)
    print("manipulability map — vertical slice (y=0)...")
    vx, vz, vpts, vshape = vertical_points(0.0)
    Wv, Kv = slice_maps(vpts, vshape)

    print(f"  peak translational manipulability: {np.nanmax(Wh):.4f} (horizontal), "
          f"{np.nanmax(Wv):.4f} (vertical)")
    _plot(hx, hy, Wh, Kh, vx, vz, Wv)
    print("figure ->", os.path.join(FIG, "fig_manipulability.png"))


def _plot(hx, hy, Wh, Kh, vx, vz, Wv):
    fig, ax = plt.subplots(1, 3, figsize=(16, 5))

    m0 = ax[0].pcolormesh(hx, hy, Wh, cmap="viridis", shading="auto")
    ax[0].set_title("(a) Manipulability w\nhorizontal slice z=0.10 m")
    ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]"); ax[0].set_aspect("equal")
    fig.colorbar(m0, ax=ax[0], shrink=0.8, label="w = √det(Jv Jvᵀ)")

    m1 = ax[1].pcolormesh(hx, hy, Kh, cmap="magma_r", shading="auto", vmin=1, vmax=60)
    ax[1].set_title("(b) Condition number κ\nhorizontal slice z=0.10 m")
    ax[1].set_xlabel("x [m]"); ax[1].set_ylabel("y [m]"); ax[1].set_aspect("equal")
    fig.colorbar(m1, ax=ax[1], shrink=0.8, label="κ (1 = isotropic)")

    m2 = ax[2].pcolormesh(vx, vz, Wv, cmap="viridis", shading="auto")
    ax[2].scatter(0, 0, c="w", marker="s", s=30, edgecolors="k")
    ax[2].set_title("(c) Manipulability w\nvertical slice y=0")
    ax[2].set_xlabel("x [m]"); ax[2].set_ylabel("z [m]"); ax[2].set_aspect("equal")
    fig.colorbar(m2, ax=ax[2], shrink=0.8, label="w")

    fig.suptitle("Manipulability and conditioning of the myCobot 280 "
                 "(brighter = more dexterous, further from singularity)", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(FIG, "fig_manipulability.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
