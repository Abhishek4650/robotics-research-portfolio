#!/usr/bin/env python3
"""
Render the book's equations as crisp PNG images IN THE USER'S REFERENCE NOTATION
(modified-DH left-super/subscripts, velocity-propagation symbols, dotted rates).

matplotlib's mathtext handles left-prescripts / dots / hats cleanly (pandoc's Word
math boxes them), and a small custom renderer draws the matrices. Images -> book/eq/.
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402

EQ = os.path.join(os.path.dirname(os.path.abspath(__file__)), "eq")
os.makedirs(EQ, exist_ok=True)
plt.rcParams["mathtext.fontset"] = "cm"   # Computer-Modern look (like a textbook)


def eq(name, latex, fontsize=24):
    """Single-line equation via mathtext (robust: tiny fig + tight bbox crop)."""
    fig = plt.figure(figsize=(0.01, 0.01))
    fig.text(0.5, 0.5, f"${latex}$", fontsize=fontsize, ha="center", va="center")
    fig.savefig(os.path.join(EQ, name + ".png"), dpi=200, bbox_inches="tight",
                pad_inches=0.08, facecolor="white")
    plt.close(fig)


def matrix_eq(name, lhs, rows, fontsize=19, col_w=2.8, lhs_w=3.0):
    """Render  lhs = [ rows ]  with real brackets.

    Deterministic sizing: D inches per data unit so font (points) always fits.
    """
    nr, nc = len(rows), len(rows[0])
    D, row_h, gap = 0.55, 1.0, 0.7
    mat_w = nc * col_w
    W = lhs_w + gap + mat_w + 1.0
    H = nr * row_h + 1.0
    fig = plt.figure(figsize=(W * D, H * D))
    ax = fig.add_axes([0, 0, 1, 1]); ax.axis("off")
    ax.set_xlim(0, W); ax.set_ylim(0, H)
    ycen = H / 2.0
    ax.text(0.15, ycen, rf"${lhs} =$", fontsize=fontsize, ha="left", va="center")
    mat_left = lhs_w + gap
    xs = [mat_left + col_w * (c + 0.5) for c in range(nc)]
    ys = [ycen + ((nr - 1) / 2.0 - i) * row_h for i in range(nr)]
    for i, row in enumerate(rows):
        for c, cell in enumerate(row):
            ax.text(xs[c], ys[i], rf"${cell}$", fontsize=fontsize, ha="center", va="center")
    top, bot = ycen + nr * row_h / 2.0, ycen - nr * row_h / 2.0
    xl, xr = mat_left, mat_left + mat_w
    for xb, sgn in ((xl, 1), (xr, -1)):
        ax.plot([xb, xb], [bot, top], color="k", lw=1.8)
        ax.plot([xb, xb + sgn * 0.22], [top, top], color="k", lw=1.8)
        ax.plot([xb, xb + sgn * 0.22], [bot, bot], color="k", lw=1.8)
    fig.savefig(os.path.join(EQ, name + ".png"), dpi=200, facecolor="white")
    plt.close(fig)


def main():
    # (1) Modified-DH link transform, user's notation  ^{i-1}_i T
    matrix_eq("dh_transform", r"^{i-1}_{i}T", [
        [r"c\theta_i", r"-s\theta_i", r"0", r"a_{i-1}"],
        [r"s\theta_i c\alpha_{i-1}", r"c\theta_i c\alpha_{i-1}", r"-s\alpha_{i-1}", r"-s\alpha_{i-1} d_i"],
        [r"s\theta_i s\alpha_{i-1}", r"c\theta_i s\alpha_{i-1}", r"c\alpha_{i-1}", r"c\alpha_{i-1} d_i"],
        [r"0", r"0", r"0", r"1"],
    ], col_w=3.2)

    # (2) forward kinematics as a product of link transforms
    eq("fk_product",
       r"^{0}_{N}T = {}^{0}_{1}T\; {}^{1}_{2}T\; {}^{2}_{3}T \cdots {}^{N-1}_{N}T")

    # (3) angular-velocity propagation
    eq("velprop_omega",
       r"^{i+1}\omega_{i+1} = {}^{i+1}_{i}R\,{}^{i}\omega_i + \dot{\theta}_{i+1}\,{}^{i+1}\hat{Z}_{i+1}")

    # (4) linear-velocity propagation
    eq("velprop_v",
       r"^{i+1}v_{i+1} = {}^{i+1}_{i}R\left({}^{i}v_i + {}^{i}\omega_i \times {}^{i}P_{i+1}\right) + \dot{d}_{i+1}\,{}^{i+1}\hat{Z}_{i+1}")

    # (5) geometric Jacobian column (revolute)
    matrix_eq("jacobian_col", r"J_i", [
        [r"\,^{0}\hat{Z}_i \times \left(\,^{0}P_e - {}^{0}P_i\right)"],
        [r"^{0}\hat{Z}_i"],
    ], col_w=8.0, lhs_w=1.4)

    # (6) damped least-squares IK update
    eq("dls_update",
       r"q_{k+1} = q_k + J^{\top}\!\left(J J^{\top} + \lambda^{2} I\right)^{-1} e")

    # (7) manipulability and condition number
    eq("manip",
       r"w = \sqrt{\det\!\left(J_v J_v^{\top}\right)}, \qquad \kappa = \frac{\sigma_{\max}(J_v)}{\sigma_{\min}(J_v)}")

    # (8) task-constrained (drawable) workspace set
    eq("drawable_set",
       r"\mathcal{D}(n) = \left\{\, p \in \mathcal{R} : \exists\, q,\; \mathrm{fk}(q)=p,\; \hat{z}(q) = \pm n \,\right\}")

    # (9) the four DH parameters (meanings are given in the prose table)
    eq("dh_params", r"\left(\ \alpha_{i-1},\ \ a_{i-1},\ \ d_i,\ \ \theta_i\ \right)")

    # (10) notation key for the preface (frame indices, dot, hat)
    eq("notation_key",
       r"^{i-1}_{i}T \quad\ \ ^{i}\omega_i \quad\ \ \dot{\theta} \quad\ \ \hat{Z}", fontsize=26)

    print("rendered equation images to", EQ)


if __name__ == "__main__":
    main()
