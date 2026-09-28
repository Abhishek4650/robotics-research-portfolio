"""Shared helpers for the ARM-450 kinematics analysis: paths, a 2D-projected
stick figure (matplotlib's 3D toolkit is not usable in this install), and
the real CAD rendered at any pose (VTK, via the release's own mesh tools)."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
KIN = os.path.dirname(HERE)
FIG = os.path.join(KIN, "figures")
REV = os.path.join(os.path.dirname(KIN), "REV_H")
sys.path.insert(0, KIN)
sys.dont_write_bytecode = True

from arm450kin import robot as R          # noqa: E402

os.makedirs(FIG, exist_ok=True)


def joints_xyz(th):
    """base foot, base top, shoulder, elbow, wrist centre, tool face (mm)."""
    Ts = R.fk_frames(th)
    return np.array([[0, 0, R.TABLE_Z], [0, 0, 0], Ts[2][:3, 3], Ts[3][:3, 3], Ts[5][:3, 3], Ts[-1][:3, 3]])


def view_basis(d=(1.0, -1.25, 0.65)):
    d = np.asarray(d, float); d /= np.linalg.norm(d)
    x = np.cross([0, 0, 1.0], d); x /= np.linalg.norm(x)
    y = np.cross(d, x)
    return x, y


def proj(P, basis):
    P = np.atleast_2d(P)
    return np.stack([P @ basis[0], P @ basis[1]], axis=1)


def draw_arm(ax, th, basis, color="#1f4e99", lw=4.0, alpha=1.0, tool_axis=True, label=None):
    J = joints_xyz(th)
    P = proj(J, basis)
    ax.plot(P[:2, 0], P[:2, 1], color="#555555", lw=lw * 1.8, solid_capstyle="butt", alpha=alpha)
    ax.plot(P[1:, 0], P[1:, 1], "-", color=color, lw=lw, alpha=alpha, label=label, solid_capstyle="round")
    ax.plot(P[2:5, 0], P[2:5, 1], "o", color="k", ms=lw * 1.3, alpha=alpha)
    if tool_axis:
        T = R.fk(th)
        e = proj(np.array([T[:3, 3], T[:3, 3] + 35 * T[:3, 2]]), basis)
        ax.annotate("", xy=e[1], xytext=e[0], arrowprops=dict(arrowstyle="->", color="#c03000", lw=1.6, alpha=alpha))


def table_patch(ax, basis, r=300.0, z=None, color="#e8e2d0"):
    z = R.TABLE_Z if z is None else z
    t = np.linspace(0, 2 * np.pi, 100)
    P = proj(np.stack([r * np.cos(t), r * np.sin(t), np.full_like(t, z)], axis=1), basis)
    ax.fill(P[:, 0], P[:, 1], color=color, zorder=0, alpha=0.6)


# ------------------------------------------------------------------ CAD render
_CAD = {}


def cad_meshes():
    """the release assembly tessellated once, grouped by rigid body."""
    if "m" not in _CAD:
        sys.path.insert(0, REV)
        import make_rule6_pages as R6          # noqa: E402
        import make_rule5_views as R5          # noqa: E402
        import verify_fasteners as VF          # noqa: E402
        A, _ = R6.BA.build()
        S = {c.name: (VF.world(c), c) for c in A.children if VF.world(c) is not None}
        meshes = {k: R5.tess(v[0], 0.1, 0.3) for k, v in S.items()}
        _CAD.update(m=meshes, R6=R6, R5=R5, axes=R6.joint_axes(), cols=R5.colours(meshes.keys()))
    return _CAD


def cad_posed(q_servo_deg, extra=None, extra_cols=None):
    """{name: (V, F)} of the whole arm at servo angles q (deg), plus extras."""
    C = cad_meshes()
    G = C["R6"].body_frames(C["axes"], q_servo_deg)
    M, cols = {}, dict(C["cols"])
    for nm, (V, F) in C["m"].items():
        b = C["R6"].body(nm)
        if b is None:
            continue
        M[nm] = ((G[b][:3, :3] @ V.T).T + G[b][:3, 3], F)
    if extra:
        M.update(extra); cols.update(extra_cols)
    return M, cols


def arrow_mesh(p, d, length, r=1.6):
    import trimesh
    d = np.asarray(d, float) / np.linalg.norm(d)
    p = np.asarray(p, float)
    shaft = trimesh.creation.cylinder(radius=r, segment=[p, p + d * length * 0.78], sections=14)
    cone = trimesh.creation.cone(radius=r * 2.6, height=length * 0.22, sections=18)
    z = np.array([0, 0, 1.0])
    v = np.cross(z, d)
    if np.linalg.norm(v) < 1e-9:
        Rm = np.eye(3) if d[2] > 0 else np.diag([1, -1, -1.0])
    else:
        ang = np.arccos(np.clip(z @ d, -1, 1)); v /= np.linalg.norm(v)
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        Rm = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = p + d * length * 0.78
    cone.apply_transform(T)
    m = trimesh.util.concatenate([shaft, cone])
    return np.asarray(m.vertices), np.asarray(m.faces)


def frame_meshes(T, name, length=40.0, r=1.4):
    out, cols = {}, {}
    for k, c in zip(range(3), ("#e02020", "#20a020", "#2050e0")):
        out["%s_%d" % (name, k)] = arrow_mesh(T[:3, 3], T[:3, k], length, r)
        cols["%s_%d" % (name, k)] = c
    return out, cols


def render(M, cols, png, view=(1.0, -1.25, 0.65), focal=None, size=(1600, 1800), zoom=1.0, scale=None):
    C = cad_meshes()
    if focal is None:
        V = np.vstack([v[0] for v in M.values()])
        focal = tuple((V.min(0) + V.max(0)) / 2)
    return C["R5"].vtk_render(M, cols, png, (focal, view, (0, 0, 1)), size=size, zoom=zoom, scale=scale)


def trimmed(png):
    return cad_meshes()["R5"].trimmed(png)


def run_ikpy(payload):
    """ikpy_bridge.py in a separate interpreter with the system numpy 1.26."""
    import json
    import subprocess
    import tempfile
    import ikpy
    d = tempfile.mkdtemp(prefix="ikpy_")
    os.symlink(os.path.dirname(ikpy.__file__), os.path.join(d, "ikpy"))
    fi, fo = os.path.join(d, "in.json"), os.path.join(d, "out.json")
    json.dump(payload, open(fi, "w"))
    env = dict(os.environ, PYTHONPATH=d)
    r = subprocess.run(["python3", "-s", os.path.join(HERE, "ikpy_bridge.py"), fi, fo], env=env,
                       capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError("ikpy bridge failed:\n" + r.stderr[-2000:])
    return json.load(open(fo))
