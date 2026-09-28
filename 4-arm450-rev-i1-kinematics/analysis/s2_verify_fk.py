#!/usr/bin/env python3
"""
Step 2 -- forward kinematics and the Jacobian, verified three independent ways.

  (a) DH FK  vs  the CAD itself: the release assembly moved by the product of
      exponentials of its measured servo axes (REV_H make_rule6_pages) -- no
      DH involved. 2000 random poses inside the joint limits.
  (b) DH FK  vs  the URDF (EDITABLE_CAD/urdf/arm450.urdf) through ikpy -- the
      same chain RViz shows.
  (c) velocity-propagation Jacobian  vs  the textbook z_i x (p_e - p_i) form
      and vs central finite differences of the FK.

Out: figures/fig_fk_verification.png, VERIFY_FK.log
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt            # noqa: E402

from common import R, FIG, KIN, cad_meshes, cad_posed, frame_meshes, render, trimmed   # noqa: E402

N = 2000


def rot_err(Ra, Rb):
    """|| Ra^T Rb - I ||_F  (~ rotation angle x sqrt2; arccos of the trace
    cannot resolve below ~1e-8 rad)."""
    return float(np.linalg.norm(Ra.T @ Rb - np.eye(3)))


def main():
    rng = np.random.default_rng(7)
    Q = rng.uniform(R.Q_LIMITS[:, 0], R.Q_LIMITS[:, 1], size=(N, 6))
    C = cad_meshes()
    T_home = R.fk_q(np.zeros(6))
    e_cad_p, e_cad_r = [], []
    for q in Q:
        G = C["R6"].body_frames(C["axes"], np.degrees(q))
        Tc, Td = G[6] @ T_home, R.fk_q(q)
        e_cad_p.append(np.linalg.norm(Tc[:3, 3] - Td[:3, 3])); e_cad_r.append(rot_err(Tc[:3, :3], Td[:3, :3]))
    # (b) URDF through ikpy (own interpreter: see ikpy_bridge.py)
    from common import run_ikpy
    Tu_all = np.array(run_ikpy({"q": Q.tolist()})["T"])
    F6 = np.eye(4); F6[2, 3] = R.GEO["axes"][5]["point"][2] - (R.D1 + R.A2 + R.D4)   # URDF flange frame on Z6
    e_urdf_p, e_urdf_r = [], []
    for q, Tu in zip(Q, Tu_all):
        Td = R.fk_q(q, tool=False) @ F6
        e_urdf_p.append(np.linalg.norm(Tu[:3, 3] - Td[:3, 3])); e_urdf_r.append(rot_err(Tu[:3, :3], Td[:3, :3]))
    # (c) Jacobians
    e_geo, e_num = [], []
    for q in Q[:500]:
        th = R.q_to_theta(q)
        J = R.jacobian(th)
        e_geo.append(np.abs(J - R.jacobian_geometric(th)).max())
        Jn = np.zeros((6, 6)); h = 1e-6
        for k in range(6):
            a, b = th.copy(), th.copy(); a[k] += h; b[k] -= h
            Ta, Tb = R.fk(a), R.fk(b)
            Jn[:3, k] = (Ta[:3, 3] - Tb[:3, 3]) / (2 * h)
            S = (Ta[:3, :3] - Tb[:3, :3]) / (2 * h) @ R.fk(th)[:3, :3].T
            Jn[3:, k] = [S[2, 1], S[0, 2], S[1, 0]]
        e_num.append(np.abs(J - Jn).max() / max(1.0, np.abs(J).max()))
    rows = [("DH FK vs CAD (product of exponentials), position", max(e_cad_p), "mm"),
            ("DH FK vs CAD, orientation", max(e_cad_r), "rad"),
            ("DH FK vs URDF (ikpy %s), position" % __import__("ikpy").__version__, max(e_urdf_p), "mm"),
            ("DH FK vs URDF, orientation", max(e_urdf_r), "rad"),
            ("velocity-propagation J vs z_i x (p_e - p_i)", max(e_geo), "mm/rad"),
            ("velocity-propagation J vs central differences (relative)", max(e_num), "-")]
    txt = "FORWARD KINEMATICS + JACOBIAN VERIFICATION (%d random poses inside the joint limits; Jacobian 500)\n\n" % N
    txt += "\n".join("  %-60s max %.2e %s" % r for r in rows)
    open(os.path.join(KIN, "VERIFY_FK.log"), "w").write(txt + "\n")
    print(txt)

    fig = plt.figure(figsize=(17, 11))
    fig.suptitle("ARM-450 rev I.1 -- forward kinematics verified: DH vs the CAD, vs the URDF, Jacobian two ways",
                 fontsize=15, fontweight="bold")
    data = [(e_cad_p, "DH vs CAD: TCP position error (mm)"), (e_urdf_p, "DH vs URDF (ikpy): flange position error (mm)"),
            (e_geo, "Jacobian: velocity propagation vs z x r (abs.)"), (e_num, "Jacobian vs central differences (rel.)")]
    for k, (d, t) in enumerate(data):
        ax = fig.add_axes([0.05 + 0.24 * k, 0.60, 0.19, 0.28])
        d = np.maximum(np.array(d), 1e-17)
        ax.hist(np.log10(d), bins=40, color="#3a78c2")
        ax.set_title(t + "\nmax %.1e" % d.max(), fontsize=10); ax.set_xlabel("log10(error)"); ax.set_ylabel("poses")
    # six random poses: the CAD with the DH-predicted tool frame on its flange face
    for k, q in enumerate(Q[:6]):
        T = R.fk_q(q)
        m, c = frame_meshes(T, "TCP", length=60.0, r=2.0)
        M, cols = cad_posed(np.degrees(q), m, c)
        for nm in cols:
            if not nm.startswith("TCP"):
                cols[nm] = "#c6ccd3"
        png = os.path.join(FIG, "_fk_%d.png" % k)
        render(M, cols, png, view=(1.0, -1.3, 0.55), size=(900, 1100), zoom=1.0)
        ax = fig.add_axes([0.01 + 0.165 * k, 0.03, 0.16, 0.44]); ax.imshow(trimmed(png), interpolation="none"); ax.axis("off")
        ax.set_title("q = (%s) deg" % ", ".join("%.0f" % v for v in np.degrees(q)), fontsize=8)
        os.remove(png)
    fig.text(0.5, 0.53, "the CAD posed by its own servo axes; the red/green/blue frame is the DH prediction "
             "^0_T T(q) -- it sits on the tool-flange face in every pose", ha="center", fontsize=11)
    p = os.path.join(FIG, "fig_fk_verification.png"); fig.savefig(p, dpi=170); plt.close(fig)
    print("wrote", p)


if __name__ == "__main__":
    main()
