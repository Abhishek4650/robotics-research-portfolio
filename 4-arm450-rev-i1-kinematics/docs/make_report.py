#!/usr/bin/env python3
"""
ARM450_KINEMATICS.pdf -- the kinematics report in the author's notation
(Modified / Craig DH, velocity propagation), every number read from the logs
the analysis scripts wrote, every figure theirs.
"""
import os
import re
import textwrap

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                        # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages   # noqa: E402

DOC = os.path.dirname(os.path.abspath(__file__))
KIN = os.path.dirname(DOC)
FIG = os.path.join(KIN, "figures")
import sys                                              # noqa: E402
sys.path.insert(0, KIN); sys.dont_write_bytecode = True
from arm450kin import robot as R                        # noqa: E402

OUT = os.path.join(DOC, "ARM450_KINEMATICS.pdf")
A4L = (11.69, 8.27)
W = lambda s, n=118: "\n".join(textwrap.fill(p, n) if p.strip() else "" for p in s.split("\n"))


def txt(fn):
    p = os.path.join(KIN, fn)
    return open(p).read() if os.path.exists(p) else ""


def g(rx, s, default="?"):
    m = re.search(rx, s)
    return m.group(1) if m else default


def page(pdf, title, body=None, fs=11, img=None, img_box=(0.03, 0.03, 0.94, 0.80)):
    fig = plt.figure(figsize=A4L)
    fig.text(0.04, 0.95, title, fontsize=17, fontweight="bold", va="top")
    if body:
        fig.text(0.04, 0.89, body, fontsize=fs, va="top", family="DejaVu Sans")
    if img:
        ax = fig.add_axes(img_box); ax.imshow(plt.imread(os.path.join(FIG, img)), interpolation="none"); ax.axis("off")
    pdf.savefig(fig); plt.close(fig)
    return fig


def matrix(fig, x, y, lhs, M, w=0.105, h=0.045, fs=14):
    """lhs = [ M ] drawn with mathtext cells (matplotlib has no bmatrix)."""
    n, m = len(M), len(M[0])
    fig.text(x, y - (n - 1) * h / 2, lhs + r"$\;=$", fontsize=fs + 2, va="center", ha="right")
    x0 = x + 0.012
    for i, row in enumerate(M):
        for j, c in enumerate(row):
            fig.text(x0 + (j + 0.5) * w, y - i * h, c, fontsize=fs, ha="center", va="center")
    ax = fig.add_axes([0, 0, 1, 1], facecolor="none"); ax.axis("off"); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    top, bot = y + h * 0.6, y - (n - 1) * h - h * 0.6
    for xx, d in ((x0, 1), (x0 + m * w, -1)):
        ax.plot([xx + d * 0.006, xx, xx, xx + d * 0.006], [top, top, bot, bot], color="k", lw=1.3)


def main():
    fk, iklog, cmp_, ws, man, sine, tim = (txt(f) for f in ("VERIFY_FK.log", "VERIFY_IK.log", "IK_COMPARISON.log",
                                                             "WORKSPACE.log", "MANIPULABILITY.log", "SINE_DEMO.log",
                                                             "TIMING.log"))
    with PdfPages(OUT) as pdf:
        # ---------------------------------------------------------------- title
        fig = plt.figure(figsize=A4L)
        fig.text(0.5, 0.80, "ARM-450 rev I.1", ha="center", fontsize=30, fontweight="bold")
        fig.text(0.5, 0.73, "Kinematics: DH table, forward and inverse kinematics, Jacobian, workspace",
                 ha="center", fontsize=16)
        fig.text(0.5, 0.685, "in the Modified (Craig) DH convention with the velocity-propagation Jacobian -- "
                 "after the myCobot 280 thesis and Rsine", ha="center", fontsize=12, style="italic")
        key = [
            "Geometry taken from the rev I.1 CAD (servo axes of the assembled arm, tool-flange face):",
            "   d1 = %.3f   a2 = %.3f   d4 = %.3f   d_T = %.3f mm;  J4, J5, J6 meet in one point (spherical wrist)"
            % (R.D1, R.A2, R.D4, R.D_TOOL),
            "Forward kinematics = the CAD to %s mm and the URDF (ikpy) to %s mm over 2000 random poses"
            % (g(r"vs CAD \(product.*?max ([\d.e+-]+)", fk), g(r"vs URDF \(ikpy.*?max ([\d.e+-]+)", fk)),
            "Jacobian by velocity propagation = the z x r form to %s; = central differences to %s (relative)"
            % (g(r"z_i x \(p_e - p_i\)\s+max ([\d.e+-]+)", fk), g(r"central differences \(relative\)\s+max ([\d.e+-]+)", fk)),
            "Inverse kinematics in CLOSED FORM: up to 8 branches, exact to %s mm on 5000 random poses"
            % g(r"all 8 branches:\s+([\d.e+-]+)", iklog),
            "Reach: %s mm from the J1 axis, %s mm from the shoulder; tool face %s"
            % (g(r"from the J1 axis: max ([\d.]+)", ws), g(r"from the shoulder.*?max ([\d.]+)", ws),
               g(r"height: ([-\d. .]+ mm)", ws)),
            "Pen STRAIGHT DOWN is impossible: J2 + J3 + J5 pitch at most 172.5 deg; a table needs a tilted pen",
            "Sine traced on a vertical board and on the table ring: closed form exact, DLS and ikpy re-trace to 1e-6 .. 1e-4 mm",
        ]
        fig.text(0.07, 0.58, "\n".join(key), fontsize=11.5, va="top", linespacing=1.7)
        fig.text(0.5, 0.06, "files: Arm_450_new_design/KINEMATICS/ -- arm450kin/ (model, IK), analysis/ (s1..s7), "
                 "figures/, docs/", ha="center", fontsize=10, color="#555")
        pdf.savefig(fig); plt.close(fig)

        # ---------------------------------------------------------- convention
        fig = plt.figure(figsize=A4L)
        fig.text(0.04, 0.95, "1  The convention: Modified (Craig) DH", fontsize=17, fontweight="bold", va="top")
        fig.text(0.04, 0.88, W("Frame {i} is attached to link i. Frames use only X and Z: Z_i along joint axis i (right-hand "
                               "rule, +Z up at home -- here the vertical joints J1, J4, J6 point +z and the pitch joints J2, "
                               "J3, J5 point +y, so a positive angle leans the arm forward / turns it counter-clockwise seen "
                               "from above); X_i along the common normal from Z_i to Z_{i+1}; Y_i completes the right-handed "
                               "set.", 125), fontsize=11.5, va="top")
        defs = [r"$a_{i-1}$   link length: the mutual-perpendicular distance between $Z_{i-1}$ and $Z_i$, measured along $X_{i-1}$",
                r"$\alpha_{i-1}$   link twist: the angle from $Z_{i-1}$ to $Z_i$, measured about $X_{i-1}$",
                r"$d_i$   link offset: the distance from $X_{i-1}$ to $X_i$, measured along $Z_i$  (prismatic variable)",
                r"$\theta_i$   joint angle: the angle from $X_{i-1}$ to $X_i$, measured about $Z_i$  (revolute variable)"]
        for k, d in enumerate(defs):
            fig.text(0.07, 0.74 - 0.055 * k, d, fontsize=13)
        fig.text(0.07, 0.45, r"$^{i-1}_{\ \ \,i}T = Rot_x(\alpha_{i-1})\ Trans_x(a_{i-1})\ Rot_z(\theta_i)\ Trans_z(d_i)$",
                 fontsize=15)
        matrix(fig, 0.30, 0.34, r"$^{i-1}_{\ \ \,i}T$",
               [[r"$c\theta_i$", r"$-s\theta_i$", r"$0$", r"$a_{i-1}$"],
                [r"$s\theta_i\,c\alpha_{i-1}$", r"$c\theta_i\,c\alpha_{i-1}$", r"$-s\alpha_{i-1}$", r"$-s\alpha_{i-1}\,d_i$"],
                [r"$s\theta_i\,s\alpha_{i-1}$", r"$c\theta_i\,s\alpha_{i-1}$", r"$c\alpha_{i-1}$", r"$c\alpha_{i-1}\,d_i$"],
                [r"$0$", r"$0$", r"$0$", r"$1$"]], w=0.12)
        fig.text(0.04, 0.08, "arm450kin/robot.py: craig_T() is exactly this matrix; the analysis derives the table from the "
                 "CAD with these rules (analysis/s1_dh_from_cad.py).", fontsize=10.5, color="#444")
        pdf.savefig(fig); plt.close(fig)

        page(pdf, "2  The DH table of ARM-450 rev I.1 (derived from the CAD)", img="fig_dh_table.png",
             img_box=(0.02, 0.02, 0.96, 0.86))
        page(pdf, "2  Frames on the CAD and the link lengths", img="fig_dh_frames.png", img_box=(0.02, 0.02, 0.96, 0.88))

        # ------------------------------------------------------------------ FK
        fig = plt.figure(figsize=A4L)
        fig.text(0.04, 0.95, "3  Forward kinematics", fontsize=17, fontweight="bold", va="top")
        fig.text(0.06, 0.86, r"$^{0}_{T}T(\theta) = \ ^{0}_{1}T\,(\theta_1)\ ^{1}_{2}T\,(\theta_2)\ ^{2}_{3}T\,(\theta_3)\ "
                 r"^{3}_{4}T\,(\theta_4)\ ^{4}_{5}T\,(\theta_5)\ ^{5}_{6}T\,(\theta_6)\ ^{6}_{T}T$", fontsize=16)
        fig.text(0.06, 0.78, r"$^{6}_{T}T = Trans_z(d_T),\quad d_T = %.3f$ mm (wrist centre to the tool-flange face)" % R.D_TOOL,
                 fontsize=14)
        fig.text(0.06, 0.71, r"servo command $q_i$ to DH angle:  $\theta_i = s_i\,q_i + \theta_{i,0}$,  "
                 r"$s = (-1, -1, +1, -1, +1, -1)$,  $\theta_{i,0} = (0, -90^\circ, +90^\circ, 0, 0, 0)$", fontsize=14)
        fig.text(0.06, 0.64, r"home (all $q_i = 0$, arm straight up):  $^{0}_{T}T = Trans_z(%.3f)$ -- tool frame parallel to the base"
                 % (R.D1 + R.A2 + R.D4 + R.D_TOOL), fontsize=14)
        fig.text(0.06, 0.53, "VERIFICATION (VERIFY_FK.log)\n" + "\n".join(l for l in fk.splitlines()[2:] if l.strip()),
                 fontsize=10.5, family="monospace", va="top")
        fig.text(0.06, 0.18, W("Three independent references: the CAD itself (the assembly moved by the product of "
                               "exponentials of its own measured servo axes -- no DH anywhere), the URDF that RViz uses "
                               "(through ikpy), and for the Jacobian the textbook z x r form and finite differences.", 130),
                 fontsize=11, va="top")
        pdf.savefig(fig); plt.close(fig)
        page(pdf, "3  FK verified: the DH frame sits on the CAD's tool face in every pose", img="fig_fk_verification.png",
             img_box=(0.02, 0.02, 0.96, 0.88))

        # ------------------------------------------------------------ Jacobian
        fig = plt.figure(figsize=A4L)
        fig.text(0.04, 0.95, "4  Jacobian by velocity propagation (Craig)", fontsize=17, fontweight="bold", va="top")
        eqs = [r"$^{i+1}\omega_{i+1} = \ ^{i+1}_{\ \ \ i}R\ ^{i}\omega_i + \dot\theta_{i+1}\ ^{i+1}\hat Z_{i+1}$",
               r"$^{i+1}v_{i+1} = \ ^{i+1}_{\ \ \ i}R\ (\,^{i}v_i + \,^{i}\omega_i \times \,^{i}P_{i+1}) + \dot d_{i+1}\ ^{i+1}\hat Z_{i+1}$",
               r"revolute joint: drop the $\dot d$ term; prismatic: drop the $\dot\theta$ term (kept in the code)",
               r"tool face:  $^{6}v_T = \,^{6}v_6 + \,^{6}\omega_6 \times (0, 0, d_T)^T$",
               r"base frame:  $^{0}v_T = \,^{0}_{6}R\ ^{6}v_T,\ \ ^{0}\omega_6 = \,^{0}_{6}R\ ^{6}\omega_6$ ;  "
               r"column $k$ of $J$ = the tip twist for $\dot\theta_k = 1$"]
        for k, e in enumerate(eqs):
            fig.text(0.06, 0.84 - 0.08 * k, e, fontsize=15)
        fig.text(0.06, 0.40, W("Implemented in arm450kin/robot.py jacobian(): each column is carried frame by frame from the "
                               "base to the tip exactly as above, then expressed in the base frame. Checked against the "
                               "geometric form z_i x (p_T - p_i); z_i and against central differences of the FK:", 125),
                 fontsize=11.5, va="top")
        fig.text(0.08, 0.28, "\n".join(l for l in fk.splitlines() if "Jacobian" in l or "J vs" in l), fontsize=11,
                 family="monospace", va="top")
        fig.text(0.06, 0.18, r"manipulability (Yoshikawa)  $w = \sqrt{\det(J_v J_v^T)}$ ,  condition number  "
                 r"$\kappa = \sigma_{max} / \sigma_{min}$", fontsize=14)
        pdf.savefig(fig); plt.close(fig)

        # ------------------------------------------------------------------ IK
        fig = plt.figure(figsize=A4L)
        fig.text(0.04, 0.95, "5  Inverse kinematics in closed form (spherical wrist, Pieper)", fontsize=17,
                 fontweight="bold", va="top")
        steps = [r"1.  wrist centre:   $W = p - d_T\ ^{0}_{T}R\ \hat z$         (J4, J5, J6 meet at $W$)",
                 r"2.  base:   $\theta_1 = atan2(W_y, W_x)$  or  $+\pi$ (arm reaching back)",
                 r"3.  arm plane:  $r = \pm\sqrt{W_x^2 + W_y^2},\ \ s = W_z - d_1,\ \ "
                 r"D = \frac{r^2 + s^2 - a_2^2 - d_4^2}{2\,a_2\,d_4}$",
                 r"      $\varphi_3 = \pm\arccos D,\ \ \varphi_2 = atan2(r, s) - atan2(d_4 \sin\varphi_3,\ a_2 + d_4\cos\varphi_3)$",
                 r"      $\theta_2 = \varphi_2 - 90^\circ,\ \ \theta_3 = \varphi_3 + 90^\circ$   ($\varphi$ = lean from vertical)",
                 r"4.  wrist:   $Rot_x(90^\circ)^T\ ^{0}_{3}R^T\ ^{0}_{6}R = Rot_z(\theta_4)\,Rot_y(\theta_5)\,Rot_z(\theta_6) = M$"
                 r"   (Z-Y-Z Euler)",
                 r"      $\theta_5 = atan2(\pm\sqrt{M_{13}^2 + M_{23}^2},\ M_{33}),\ \ \theta_4 = atan2(\pm M_{23}, \pm M_{13}),\ \ "
                 r"\theta_6 = atan2(\pm M_{32}, \mp M_{31})$",
                 r"5.  2 (shoulder) x 2 (elbow) x 2 (wrist) = up to 8 solutions; the joint limits keep the valid ones"]
        for k, e in enumerate(steps):
            fig.text(0.05, 0.85 - 0.065 * k, e, fontsize=13.5)
        fig.text(0.05, 0.30, "\n".join(iklog.splitlines()[2:]), fontsize=9.3, family="monospace", va="top")
        pdf.savefig(fig); plt.close(fig)
        page(pdf, "5  The 8 branches of one pose", img="fig_ik_branches.png", img_box=(0.02, 0.02, 0.96, 0.88))

        fig = plt.figure(figsize=A4L)
        fig.text(0.04, 0.95, "6  Numerical IK and the solver comparison (after the thesis, Exp. 2)", fontsize=17,
                 fontweight="bold", va="top")
        meths = [r"damped least squares (Rsine):  $\Delta\theta = J^T (J J^T + \lambda^2 I)^{-1} e$",
                 r"pseudoinverse:  $\Delta\theta = J^{\dagger} e$",
                 r"Jacobian transpose:  $\Delta\theta = \alpha J^T e,\ \ \alpha = \langle e, J J^T e\rangle / \|J J^T e\|^2$",
                 r"Levenberg-Marquardt:  $(J^T J + \lambda\,\mathrm{diag}(J^T J))\,\Delta\theta = J^T e$ , $\lambda$ adapted",
                 r"error  $e = (p^* - p,\ \ w\,\frac{1}{2}\sum_k x_k \times x_k^*)$ , $w$ = 100 mm/rad; joint limits clamped"]
        for k, e in enumerate(meths):
            fig.text(0.05, 0.86 - 0.06 * k, e, fontsize=13.5)
        fig.text(0.05, 0.52, cmp_, fontsize=9.6, family="monospace", va="top")
        pdf.savefig(fig); plt.close(fig)
        page(pdf, "6  Success, time and accuracy of the solvers", img="fig_ik_comparison.png", img_box=(0.02, 0.05, 0.96, 0.82))

        # ----------------------------------------------------------- workspace
        page(pdf, "7  Reachable workspace", body=W("\n".join(ws.split("TASK-CONSTRAINED")[0].splitlines()[2:]), 150), fs=9.5,
             img="fig_workspace.png", img_box=(0.02, 0.02, 0.96, 0.62))
        page(pdf, "7  Task-constrained (drawable) workspace -- where the pen can stand normal to a surface",
             img="fig_task_workspace.png", img_box=(0.02, 0.02, 0.96, 0.88))
        page(pdf, "7  What the task workspace says", body="TASK-CONSTRAINED" + ws.split("TASK-CONSTRAINED")[1], fs=9.3)
        page(pdf, "8  Manipulability and singularities (after the thesis, Exp. 3)", img="fig_manipulability.png",
             img_box=(0.02, 0.02, 0.96, 0.88))
        page(pdf, "9  Sine tracing (after Rsine)", body=sine, fs=9.6)
        page(pdf, "9  Sine tracing: paths, servo angles, errors", img="fig_sine_demos.png", img_box=(0.02, 0.02, 0.96, 0.88))
        page(pdf, "9  The CAD arm drawing on the board (animations: figures/sine_vertical.gif, sine_table.gif)",
             img="fig_sine_cad.png", img_box=(0.02, 0.10, 0.96, 0.70))
        fig = plt.figure(figsize=A4L)
        fig.text(0.04, 0.95, "10  The sine as a timed trajectory on the ST3215 servos", fontsize=17, fontweight="bold", va="top")
        eqs = [r"a pen is symmetric: task = tool-face position + pen axis = 5-D, arm = 6 joints -> 1 redundant DOF",
               r"$J_5 = \left[\,J_v\ ;\ u^T J_\omega\ ;\ w^T J_\omega\,\right]$  ($u, w \perp$ pen axis),   "
               r"$\Delta\theta = J_5^{\dagger}\,\Delta x_5 + (I - J_5^{\dagger} J_5)\,z$",
               r"$z = -k\,(\theta - \theta_{mid}) / \theta_{half}^2$ keeps the joints near the middle of their ranges; "
               r"Newton correction per point",
               r"servo speed $\dot q_i = \frac{dq_i}{ds}\,v$  against the ST3215 rating 0.222 s/60$^\circ$ = 270$^\circ$/s",
               r"encoder floor  $|\delta x| \leq \sum_i \|J_{v,i}\|\,\frac{\Delta q}{2}$,  $\Delta q = 360^\circ/4096$"]
        for k, e in enumerate(eqs):
            fig.text(0.05, 0.86 - 0.075 * k, e, fontsize=13.5)
        fig.text(0.05, 0.47, tim, fontsize=9.3, family="monospace", va="top")
        pdf.savefig(fig); plt.close(fig)
        page(pdf, "10  Servo angles, servo speeds and the encoder floor along both sines", img="fig_timing.png",
             img_box=(0.02, 0.02, 0.96, 0.88))
        page(pdf, "11  Conclusions", body=W(
            "* The Modified-DH table of rev I.1 follows from the CAD with the convention's frame rules and reproduces the CAD and "
            "the URDF to 1e-13 mm; the velocity-propagation Jacobian matches the geometric form and finite differences.\n\n"
            "* The arm has a spherical wrist, so its IK is closed-form: up to 8 exact branches in about 0.2 ms, no seed. "
            "The numerical solvers (DLS, pseudoinverse, Levenberg-Marquardt, ikpy) are exact only from a nearby seed: "
            "good for following a path, unreliable for a target anywhere. Use the closed form to start, a numerical "
            "solver (or the closed form with branch continuity) along a path.\n\n"
            "* Home (straight up) is singular three ways (shoulder, elbow, wrist): work from the bent ready pose.\n\n"
            "* The joint limits (set by the parts' clearances) cap the pitch at J2 + J3 + J5 = 172.5 deg: the pen can never "
            "point straight down. Table drawing needs a tilted pen (least tilt ~15 deg near the table) and happens on a ring "
            "round the base; a vertical board works with the pen pointing away from the robot, from ~190 mm out.\n\n"
            "* Both sines were traced smoothly by using the free roll of a symmetric pen (largest joint step < 3 deg) -- "
            "the same redundancy idea as the thesis's raised-elbow branch.\n\n"
            "* Timed on the ST3215 (270 deg/s no-load): the board sine can run at ~%s mm/s pen speed with the servos at half "
            "their rating; the encoder (0.088 deg) alone sets a ~%s mm floor on drawing accuracy -- backlash and the printed "
            "parts' play come on top." % (g(r"VERTICAL.*?at half the rating: (\d+) mm/s", tim.replace("\n", " ")),
                                          g(r"VERTICAL.*?tool face [\d.]+ \.\. ([\d.]+) mm", tim.replace("\n", " "))), 130), fs=12)
    print("written", OUT)


if __name__ == "__main__":
    main()
