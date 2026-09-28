#!/usr/bin/env python3
"""
Step 1 -- the Modified (Craig) DH table, DERIVED from the rev I.1 CAD.

Input: the six servo axes of the assembled arm (point + direction, measured
on the CAD) and the tool-flange face. Frames by the convention's rules:
  * Z_i along joint axis i, right-hand rule, "+Z up" at home (vertical joints
    +z, pitch joints +y);
  * X_i along the common normal from Z_i to Z_{i+1}; where the two axes
    intersect, X_i = +-(Z_i x Z_{i+1}), the sign nearest X_{i-1};
  * origin O_i where that normal meets Z_i; for parallel axes on the line
    X_{i-1} (so d_i = 0); frame 6 = frame 5 turned onto Z_6 (d_6 = 0);
  * Y_i completes the right-handed set.
Then every ^{i-1}_iT is read back into (alpha_{i-1}, a_{i-1}, d_i, theta_i),
checked to have exactly the Craig form, and compared with the table the
model uses (arm450kin/robot.py).

Out: figures/fig_dh_frames.png, figures/fig_dh_table.png, DH_TABLE.md, dh_table.csv
"""
import csv
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt            # noqa: E402

from common import R, FIG, KIN, cad_posed, frame_meshes, render, trimmed   # noqa: E402

AX = [(np.array(a["point"]), np.array(a["axis"]) / np.linalg.norm(a["axis"])) for a in R.GEO["axes"]]


def chosen_z(w):
    """frame rule: +Z up for vertical joints, +y for horizontal ones."""
    if abs(w[2]) > 0.5:
        return w if w[2] > 0 else -w
    return w if w[1] > 0 else -w


def closest(p1, d1, p2, d2):
    """closest points on two lines (None, None if parallel)."""
    n = np.cross(d1, d2)
    if np.linalg.norm(n) < 1e-9:
        return None, None
    A = np.array([d1, -d2, n]).T
    t = np.linalg.solve(A, p2 - p1)
    return p1 + t[0] * d1, p2 + t[1] * d2


def pick_sign(x, x_prev):
    c = x @ x_prev
    if abs(c) > 1e-9:
        return x if c > 0 else -x
    for ref in (np.array([1.0, 0, 0]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0])):   # tie-break
        if abs(x @ ref) > 1e-9:
            return x if x @ ref > 0 else -x
    return x


def derive():
    Z = [np.array([0, 0, 1.0])] + [chosen_z(w) for _, w in AX]
    P = [np.zeros(3)] + [p for p, _ in AX]
    X = [np.array([1.0, 0, 0])]
    O = [np.zeros(3)]
    for i in range(1, 7):
        if i < 6:
            a, b = closest(P[i], Z[i], P[i + 1], Z[i + 1])
            if a is None:                                   # parallel: normal between the lines
                v = (P[i + 1] - P[i]) - ((P[i + 1] - P[i]) @ Z[i]) * Z[i]
                x = v / np.linalg.norm(v)
                o, _ = closest(O[i - 1], X[i - 1], P[i], Z[i])   # on the line X_{i-1}: d_i = 0
            elif np.linalg.norm(a - b) < 1e-9:              # intersecting
                x = pick_sign(np.cross(Z[i], Z[i + 1]) / np.linalg.norm(np.cross(Z[i], Z[i + 1])), X[i - 1])
                o = a
            else:                                           # skew
                x = (b - a) / np.linalg.norm(b - a); o = a
        else:                                               # last frame: d_6 = 0, X_6 = X_5
            x = X[5] - (X[5] @ Z[6]) * Z[6]; x /= np.linalg.norm(x)
            o, _ = closest(O[5], X[5], P[6], Z[6]) if np.linalg.norm(np.cross(X[5], Z[6])) > 1e-9 else (O[5], None)
        X.append(x); O.append(o)
    T = []
    for i in range(7):
        M = np.eye(4); M[:3, 0] = X[i]; M[:3, 2] = Z[i]; M[:3, 1] = np.cross(Z[i], X[i]); M[:3, 3] = O[i]
        T.append(M)
    rows = []
    for i in range(1, 7):
        M = np.linalg.inv(T[i - 1]) @ T[i]
        a = M[0, 3]
        al = np.arctan2(-M[1, 2], M[2, 2])
        th = np.arctan2(-M[0, 1], M[0, 0])
        d = M[2, 3] / np.cos(al) if abs(np.cos(al)) > 0.5 else -M[1, 3] / np.sin(al)
        err = np.abs(R.craig_T(al, a, d, th) - M).max()
        assert err < 1e-9, "frame %d -> %d is not a Craig transform (%.2e)" % (i - 1, i, err)
        rows.append((al, a, d, th))
    sign = np.array([np.sign(Z[i] @ AX[i - 1][1]) for i in range(1, 7)])
    return np.array(rows), sign, T


def main():
    rows, sign, T = derive()
    diff = np.abs(rows - R.DH).max()
    print("DH table derived from the CAD axes vs the model's table: max difference %.2e" % diff)
    assert diff < 1e-9 and np.all(sign == R.SIGN)
    print("servo-to-DH signs %s (theta = sign * q + offset)" % sign.astype(int))
    th0 = R.q_to_theta(np.zeros(6))
    Tm = R.fk_frames(th0)
    worst = max(np.abs(T[i] - Tm[i]).max() for i in range(7))
    print("derived frames vs model frames at home: %.2e mm" % worst)
    head = ["i", "alpha_{i-1} (deg)", "a_{i-1} (mm)", "d_i (mm)", "theta_i", "theta from servo q", "servo range", "joint"]
    lines = []
    for (i, al, a, d, th), s, nm, lim in zip(R.dh_table_rows(), R.SIGN, R.NAMES, np.degrees(R.Q_RANGE)):
        lines.append([i, "%g" % round(al, 6), "%.3f" % a, "%.3f" % d, th,
                      "th%d = %sq%d%s" % (i, "-" if s < 0 else "", i,
                                          "" if abs(R.DH[i - 1, 3]) < 1e-9 else " %+g deg" % np.degrees(R.DH[i - 1, 3])),
                      "q%d +-%g deg" % (i, lim), nm])
    with open(os.path.join(KIN, "dh_table.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(head); w.writerows(lines)
    md = ["# ARM-450 rev I.1 -- Modified (Craig) DH table", "",
          "Derived from the CAD servo axes (`analysis/s1_dh_from_cad.py`); frame i on link i;",
          "`^{i-1}_iT = Rot_x(alpha_{i-1}) Trans_x(a_{i-1}) Rot_z(theta_i) Trans_z(d_i)`.", "",
          "| " + " | ".join(head) + " |", "|" + "---|" * len(head)] + \
         ["| " + " | ".join(str(c) for c in l) + " |" for l in lines] + \
         ["", "Tool: `^6_TT = Trans_z(d_T)`, d_T = %.3f mm (wrist centre -> tool-flange face)." % R.D_TOOL,
          "Base frame 0 = CAD world: origin on the J1 axis at the base's top face, Z up; the table top is z = %.1f." % R.TABLE_Z,
          "Wrist centre W = O4 = O5 = O6 (J4, J5, J6 meet): spherical wrist -> closed-form IK.",
          "Home (all servos at 0) = arm straight up; ^0_TT = Trans_z(%.3f), tool frame parallel to the base frame." % (R.D1 + R.A2 + R.D4 + R.D_TOOL)]
    open(os.path.join(KIN, "DH_TABLE.md"), "w").write("\n".join(md) + "\n")
    print("\n".join(md))

    # ---- figure 1: CAD with the frames + schematic with dimensions
    fig = plt.figure(figsize=(17.0, 11.0))
    fig.suptitle("ARM-450 rev I.1 -- Modified (Craig) DH frames, derived from the CAD (home: all servos at 0)",
                 fontsize=15, fontweight="bold")
    extra, ecol = {}, {}
    for i in (0, 2, 3, 5, 7):
        m, c = frame_meshes(Tm[i], "F%d" % i, length=55.0 if i else 70.0, r=1.6)
        extra.update(m); ecol.update(c)
    M, cols = cad_posed(np.zeros(6), extra, ecol)
    for k in cols:
        if not k.startswith("F"):
            cols[k] = "#cfd3d8"
    png = os.path.join(FIG, "_dh_cad.png")
    render(M, cols, png, view=(1.0, -1.1, 0.35), size=(1300, 2200), zoom=1.05)
    ax = fig.add_axes([0.01, 0.03, 0.40, 0.88]); ax.imshow(trimmed(png), interpolation="none"); ax.axis("off")
    ax.set_title("frames on the CAD: X red, Y green, Z blue (O0 base, O1=O2 shoulder, O3 elbow,\nO4=O5=O6 wrist centre, T tool face)",
                 fontsize=10)
    ax = fig.add_axes([0.45, 0.06, 0.52, 0.84])
    J = [np.array([0, 0, R.TABLE_Z])] + [Tm[i][:3, 3] for i in (0, 2, 3, 5, 7)]
    xs, zs = [p[0] for p in J], [p[2] for p in J]
    ax.plot([0, 0], [R.TABLE_Z, 0], color="#666", lw=14, solid_capstyle="butt")
    ax.plot(xs[1:], zs[1:], "-", color="#1f4e99", lw=5)
    ax.plot(xs[2:5], zs[2:5], "ko", ms=8)
    ax.axhline(R.TABLE_Z, color="#b59f68", lw=3); ax.text(-150, R.TABLE_Z - 18, "table top  z = %.1f" % R.TABLE_Z, fontsize=10)
    labels = {0: "O0 (base)", 2: "O1 = O2 (shoulder)", 3: "O3 (elbow)", 5: "O4 = O5 = O6 (wrist centre W)", 7: "T (tool-flange face)"}
    for i, lab in labels.items():
        o = Tm[i][:3, 3]
        ax.annotate("", xy=(o[0] + 40 * Tm[i][0, 0], o[2] + 40 * Tm[i][2, 0]), xytext=(o[0], o[2]),
                    arrowprops=dict(arrowstyle="-|>", color="#d02020", lw=2))
        ax.annotate("", xy=(o[0] + 40 * Tm[i][0, 2], o[2] + 40 * Tm[i][2, 2]), xytext=(o[0], o[2]),
                    arrowprops=dict(arrowstyle="-|>", color="#2050e0", lw=2))
        ax.text(o[0] + 48, o[2] + 4, lab, fontsize=10.5, va="center")
    ax.text(95, 300, "Z blue, X red; Y into the page\n(pitch joints J2, J3, J5: Z along +y,\n drawn as the axle symbol)", fontsize=9.5)
    for i in (2, 3, 5):
        o = Tm[i][:3, 3]; ax.plot(o[0], o[2], "o", ms=16, mfc="none", mec="#2050e0", mew=2); ax.plot(o[0], o[2], "x", ms=9, color="#2050e0", mew=2)
    def dim(z0, z1, x, text):
        ax.annotate("", xy=(x, z1), xytext=(x, z0), arrowprops=dict(arrowstyle="<->", color="k", lw=1.2))
        ax.text(x - 8, (z0 + z1) / 2, text, fontsize=11, ha="right", va="center")
    dim(0, R.D1, -40, "d1 = %.3f" % R.D1)
    dim(R.D1, R.D1 + R.A2, -40, "a2 = %.3f" % R.A2)
    dim(R.D1 + R.A2, R.D1 + R.A2 + R.D4, -40, "d4 = %.3f" % R.D4)
    dim(R.D1 + R.A2 + R.D4, R.D1 + R.A2 + R.D4 + R.D_TOOL, -40, "d_T = %.3f" % R.D_TOOL)
    ax.set_xlim(-230, 330); ax.set_ylim(R.TABLE_Z - 40, 560); ax.set_aspect("equal"); ax.grid(True, lw=0.3)
    ax.set_xlabel("x (mm)"); ax.set_ylabel("z (mm)")
    ax.set_title("side view (x-z), home pose -- the link lengths of the table", fontsize=11)
    p = os.path.join(FIG, "fig_dh_frames.png"); fig.savefig(p, dpi=200); fig.savefig(p[:-4] + ".pdf"); plt.close(fig)
    os.remove(png)

    # ---- figure 2: the table + the transform in the convention's notation
    fig = plt.figure(figsize=(14.0, 7.5))
    fig.suptitle("ARM-450 rev I.1 -- Modified (Craig) DH parameters", fontsize=15, fontweight="bold")
    ax = fig.add_axes([0.02, 0.38, 0.96, 0.52]); ax.axis("off")
    cell = [["%d" % l[0], l[1], l[2], l[3], r"$\theta_%d$%s" % (l[0], l[4][3:].replace("deg", r"$^\circ$")),
             l[5].replace("th%d" % l[0], r"$\theta_%d$" % l[0]).replace("deg", r"$^\circ$"),
             l[6].replace("+-", r"$\pm$").replace("deg", r"$^\circ$"), l[7]] for l in lines]
    t = ax.table(cellText=cell, colLabels=["i", r"$\alpha_{i-1}$ (deg)", r"$a_{i-1}$ (mm)", r"$d_i$ (mm)", r"$\theta_i$",
                                          r"$\theta_i$ from servo $q_i$", "servo range", "joint"], loc="center", cellLoc="center",
                 colWidths=[0.04, 0.11, 0.10, 0.10, 0.16, 0.18, 0.12, 0.17])
    t.auto_set_font_size(False); t.set_fontsize(11); t.scale(1, 2.0)
    for (r, c), ce in t.get_celld().items():
        if r == 0:
            ce.set_facecolor("#dde3ea")
    fig.text(0.04, 0.30, r"$^{i-1}_{\ \ i}T = Rot_x(\alpha_{i-1})\,Trans_x(a_{i-1})\,Rot_z(\theta_i)\,Trans_z(d_i)$"
             r"$\qquad ^{6}_{T}T = Trans_z(d_T),\ d_T = %.3f$ mm" % R.D_TOOL, fontsize=15)
    fig.text(0.04, 0.20, r"$^{0}_{T}T = \,^{0}_{1}T\ ^{1}_{2}T\ ^{2}_{3}T\ ^{3}_{4}T\ ^{4}_{5}T\ ^{5}_{6}T\ ^{6}_{T}T$"
             r"$\qquad$ home: $^{0}_{T}T = Trans_z(%.3f)$" % (R.D1 + R.A2 + R.D4 + R.D_TOOL), fontsize=15)
    fig.text(0.04, 0.08, "Derived from the CAD servo axes and checked: every link transform has the exact Craig form; "
             "the frames match the model's to %.0e mm.\nSpherical wrist (J4, J5, J6 meet at W, %.3f mm above the base) -> "
             "closed-form IK." % (max(worst, 1e-16), R.D1 + R.A2 + R.D4), fontsize=11)
    p = os.path.join(FIG, "fig_dh_table.png"); fig.savefig(p, dpi=200); fig.savefig(p[:-4] + ".pdf"); plt.close(fig)
    print("wrote fig_dh_frames.png, fig_dh_table.png, DH_TABLE.md, dh_table.csv")


if __name__ == "__main__":
    main()
