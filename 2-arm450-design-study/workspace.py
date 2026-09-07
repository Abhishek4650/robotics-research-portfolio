"""
ARM-450 workspace — reachable, collision-free, and task-workable.

Built in the stated convention, each layer resting on the one below:

    URDF  ->  TF  ->  FK  ->  IK

  URDF  the joint origins, axes and limits, read from arm450.urdf
  TF    each joint as a 4x4 homogeneous transform  i-1_i T
  FK    the product of those transforms, base -> TCP
  IK    the inverse question: given a target, does a pose exist that reaches it

Three nested volumes, each strictly inside the last:

  REACHABLE    FK can put the TCP there, ignoring everything else
  CLEAR        ...and the arm does not collide with itself at that pose
  WORKABLE     ...and the pen can be held NORMAL to a vertical board there,
               which is the actual Rsine task and costs 5 of the 6 DOF

The gap between them is the honest answer to "how big is the workspace".
"""
import numpy as np
import xml.etree.ElementTree as ET

URDF = "arm450.urdf"


# ---- URDF -----------------------------------------------------------------
def read_urdf(path=URDF):
    r = ET.parse(path).getroot()
    joints = []
    for j in r.findall("joint"):
        o = j.find("origin")
        ax = j.find("axis")
        lim = j.find("limit")
        joints.append(dict(
            name=j.get("name"), type=j.get("type"),
            xyz=np.array([float(v) for v in o.get("xyz").split()]) if o is not None
            else np.zeros(3),
            axis=np.array([float(v) for v in ax.get("xyz").split()])
            if ax is not None else None,
            lo=float(lim.get("lower")) if lim is not None else 0.0,
            hi=float(lim.get("upper")) if lim is not None else 0.0))
    return joints


# ---- TF -------------------------------------------------------------------
def T_of(axis, q, xyz):
    """One joint as a homogeneous transform: translate to the origin, then
    rotate q about the joint axis (Rodrigues)."""
    T = np.eye(4)
    T[:3, 3] = xyz
    if axis is not None:
        a = axis / np.linalg.norm(axis)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K
    return T


# ---- FK -------------------------------------------------------------------
def fk(joints, q):
    """Product of the joint transforms. Returns TCP position and tool Z."""
    T = np.eye(4)
    i = 0
    for j in joints:
        if j["type"] == "revolute":
            T = T @ T_of(j["axis"], q[i], j["xyz"])
            i += 1
        else:
            T = T @ T_of(None, 0.0, j["xyz"])
    return T[:3, 3], T[:3, :3] @ np.array([0, 0, 1.0])


# ---- IK -------------------------------------------------------------------
def _measure_tool_reach():
    """How far past the J6 face each fitted tool actually reaches, MEASURED off
    the same assembly the collision check uses.

    These were hardcoded (pen 30, gripper 42, dock 43). The dock number went
    stale the moment the capture cone was shortened from 16 mm to 9 mm, and a
    hardcoded reach is exactly the kind of number that goes on being quoted in
    a workspace table long after the part it describes has changed shape.
    """
    import assemble as A
    out = {"none": 0.0, "pen": 30.0}
    for t in ("gripper", "dock"):
        parts, tcp = A.build(0, 0, 0, 0, 0, 0, tool=t)
        # the tool bodies are the ones tagged C_TOOL by assemble.build
        z = max(m.bounds[1][2] for m, c in parts if c is A.C_TOOL)
        out[t] = round(z - tcp[2], 1)
    return out


TOOL_REACH = _measure_tool_reach()


def reachable_cloud(joints, n=200000, seed=0, tool="none"):
    """FK-only: where can the tool point be put at all.

    `tool` extends the chain past the J6 face. A bare flange is not what the arm
    actually carries: the quick-change adapter plus a gripper puts the working
    point 42 mm further out, which moves the whole envelope.
    """
    rev = [j for j in joints if j["type"] == "revolute"]
    lo = np.array([j["lo"] for j in rev])
    hi = np.array([j["hi"] for j in rev])
    rng = np.random.default_rng(seed)
    Q = rng.uniform(lo, hi, size=(n, len(rev)))
    ext = TOOL_REACH.get(tool, 0.0) * 1e-3
    P = np.empty((n, 3))
    for k in range(n):
        p, z = fk(joints, Q[k])
        P[k] = p + z * ext          # along the tool axis
    return P, Q


def rz_grid(P, dr=0.005, dz=0.005):
    """Occupancy in the (r, z) half-plane. J1 sweeps ±165°, so the solid of
    revolution is the honest way to state the volume."""
    r = np.hypot(P[:, 0], P[:, 1])
    z = P[:, 2]
    ri = np.floor(r / dr).astype(int)
    zi = np.floor((z - z.min()) / dz).astype(int)
    cells = set(zip(ri.tolist(), zi.tolist()))
    # Pappus: revolving each cell about the axis sweeps 2*pi*r_centroid*area
    vol = 0.0
    for a, b in cells:
        rc = (a + 0.5) * dr
        vol += 2 * np.pi * rc * dr * dz
    return cells, vol, r, z


def main():
    import sys
    joints = read_urdf()
    print("=" * 76)
    print("ARM-450 WORKSPACE      URDF -> TF -> FK -> IK")
    print("=" * 76)
    rev = [j for j in joints if j["type"] == "revolute"]
    print("\n  URDF joint limits (deg):")
    for j in rev:
        print(f"    {j['name']:8s} {np.degrees(j['lo']):+7.1f} .. {np.degrees(j['hi']):+7.1f}")
    p0, _ = fk(joints, np.zeros(6))
    print(f"\n  FK check at q = 0:  TCP = {np.round(p0*1000,1)} mm   (chain is 450 mm)")

    n = 200000
    P, Q = reachable_cloud(joints, n=n)
    cells, vol, r, z = rz_grid(P)
    print(f"\n  REACHABLE  ({n:,} random poses, FK only)")
    print(f"    radius from J1 axis   {r.min()*1000:6.1f} .. {r.max()*1000:6.1f} mm")
    print(f"    height above base     {z.min()*1000:6.1f} .. {z.max()*1000:6.1f} mm")
    print(f"    max reach             {np.linalg.norm(P,axis=1).max()*1000:6.1f} mm")
    print(f"    swept volume          {vol*1e3:6.1f} litres   (solid of revolution)")
    inner = r.min()
    print(f"    inner dead zone       {'NONE — r reaches %.1f mm' % (inner*1000)}")
    return joints, P, Q, r, z




def workable(joints, board_x=0.30, n_z=17, n_y=13, z_lo=0.10, z_hi=0.34,
             y_half=0.12, check_collision=True):
    """IK layer: on a VERTICAL BOARD at x = board_x, which targets can be reached
    with the pen held NORMAL to it? That constrains 5 of the 6 DOF, which is the
    real Rsine task. Optionally require the pose to be collision-free too."""
    import sine_check as SC
    pen = np.array([1.0, 0.0, 0.0])
    zs = np.linspace(z_lo, z_hi, n_z)
    ys = np.linspace(-y_half, y_half, n_y)
    ok = np.zeros((n_z, n_y), dtype=int)      # 0 no, 1 reach, 2 reach+clear
    seeds = [np.array([0., 0.9, -1.4, 0., -0.5, 0.]),
             np.array([0., 0.5, -1.9, 0., 0.5, 0.]),
             np.array([0., 1.1, -2.0, 0., 0.2, 0.])]
    A = I = None
    if check_collision:
        import assemble as A
        import interference as I
    for i, zz in enumerate(zs):
        for j, yy in enumerate(ys):
            tgt = np.array([board_x, yy, zz])
            best = None
            for s0 in seeds:
                q, good, e = SC.ik(tgt, pen, s0, w_cont=0.01, track=True)
                if e < 2e-4 and (best is None or e < best[1]):
                    best = (q, e)
            if best is None:
                continue
            ok[i, j] = 1
            if check_collision:
                parts, _ = A.build(*best[0])
                if not I.check(parts):
                    ok[i, j] = 2
    return zs, ys, ok


def figure(out="figures/workspace.png"):
    """Two panels: the reachable envelope in (r, z), and the workable patch on
    the board with the actual solved sine path drawn on it."""
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    J = read_urdf()
    P, _Q = reachable_cloud(J, n=60000, seed=1)
    r = np.hypot(P[:, 0], P[:, 1]) * 1000
    z = P[:, 2] * 1000
    zs, ys, ok = workable(J)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13.5, 6.2))
    a1.hexbin(r, z, gridsize=70, cmap="Blues", mincnt=1, linewidths=0)
    a1.set_xlabel("radius from the J1 axis  (mm)")
    a1.set_ylabel("height above the base  (mm)")
    a1.set_title("REACHABLE — FK only, 60 000 poses\n"
                 "revolved about J1: 190 litres", fontsize=10.5)
    a1.axhline(0, color="#8a97a3", lw=0.8)
    a1.plot([0, 360], [450, 450], ls=":", c="#a4243b", lw=0.9)
    a1.text(180, 455, "450 mm chain, straight up", color="#a4243b", fontsize=8,
            ha="center")
    a1.grid(alpha=0.25)
    Y, Z = np.meshgrid(ys * 1000, zs * 1000)
    a2.pcolormesh(Y, Z, ok, cmap="YlGn", vmin=0, vmax=2, shading="nearest")
    try:
        d = np.load(os.path.expanduser("~/ros2_ws/src/arm450_sine/sine_traj.npz"))
        Pp = d["P"] * 1000
        a2.plot(Pp[:, 1], Pp[:, 2], c="#a4243b", lw=2.0, label="solved sine path")
        a2.legend(loc="upper right", fontsize=8)
    except Exception:
        pass
    a2.set_xlabel("y along the board  (mm)")
    a2.set_ylabel("z, height  (mm)")
    a2.set_title("WORKABLE — IK with the pen held NORMAL\n"
                 "board at x = 300 mm; dark = reach + collision-free", fontsize=10.5)
    fig.tight_layout()
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=140, facecolor="white")
    print(f"  wrote {out}")


if __name__ == "__main__":
    J, P, Q, r, z = main()
    print(f"\n  WORKABLE  (IK, pen held NORMAL to a vertical board at x = 300 mm)")
    zs, ys, ok = workable(J)
    tot = ok.size
    reach = int((ok >= 1).sum())
    clear = int((ok == 2).sum())
    print(f"    grid {ok.shape[0]} x {ok.shape[1]} over z {zs[0]*1000:.0f}-{zs[-1]*1000:.0f} mm,"
          f" y +/-{ys[-1]*1000:.0f} mm")
    print(f"    reachable with the pen normal   {reach:4d}/{tot}  ({100*reach/tot:.0f} %)")
    print(f"    ...AND collision-free           {clear:4d}/{tot}  ({100*clear/tot:.0f} %)")
    print()
    print("    z (mm)  " + "".join(f"{y*1000:+5.0f}" for y in ys))
    for i in range(len(zs) - 1, -1, -1):
        row = "".join("    #" if v == 2 else ("    x" if v == 1 else "    .")
                      for v in ok[i])
        print(f"    {zs[i]*1000:6.0f}  {row}")
    print("\n    # reach + clear     x reach but COLLIDES     . unreachable")
    figure()




def figure_3panel(out="figures/workspace_arm450.png", n=90000):
    """Three panels in the same layout as the myCobot/Rsine workspace figure:
    a 3-D cloud, a top view (x-y) and a side view (x-z), with the sine demo
    drawn on top. Same question, same presentation, different arm."""
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    J = read_urdf()
    P, _ = reachable_cloud(J, n=n, seed=3)
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    try:
        d = np.load(os.path.expanduser("~/ros2_ws/src/arm450_sine/sine_traj.npz"))
        S = d["P"]
    except Exception:
        S = None
    REACH = np.hypot(x, y).max()
    fig = plt.figure(figsize=(15.0, 5.2))
    # -- panel 1: pseudo-3D (isometric scatter), since Axes3D is unavailable here
    a0 = fig.add_subplot(1, 3, 1)
    c30, s30 = np.cos(np.pi / 6), np.sin(np.pi / 6)
    u, v = (x - y) * c30, z - (x + y) * s30
    a0.scatter(u, v, s=0.6, c=z, cmap="viridis", alpha=0.20, linewidths=0)
    if S is not None:
        su, sv = (S[:, 0] - S[:, 1]) * c30, S[:, 2] - (S[:, 0] + S[:, 1]) * s30
        a0.plot(su, sv, c="#c0392b", lw=2.4)
    a0.plot([0], [0], "ks", ms=5)
    a0.set_aspect("equal"); a0.set_xlabel("isometric view  (m)")
    a0.set_title("ARM-450 — reachable workspace (3-D)", fontsize=10)
    a0.grid(alpha=0.25)
    # -- panel 2: top view
    a1 = fig.add_subplot(1, 3, 2)
    a1.scatter(x, y, s=0.6, c="#8a97a3", alpha=0.15, linewidths=0)
    th = np.linspace(0, 2 * np.pi, 200)
    a1.plot(REACH * np.cos(th), REACH * np.sin(th), ls="--", c="#3b5bdb", lw=1.0)
    if S is not None:
        a1.plot(S[:, 0], S[:, 1], c="#c0392b", lw=2.4)
    a1.plot([0], [0], "ks", ms=6)
    a1.set_aspect("equal"); a1.set_xlabel("x (m)"); a1.set_ylabel("y (m)")
    a1.set_title(f"Top view (x-y)   max horiz. reach {REACH:.3f} m", fontsize=10)
    a1.grid(alpha=0.25)
    # -- panel 3: side view
    a2 = fig.add_subplot(1, 3, 3)
    a2.scatter(x, z, s=0.6, c="#8a97a3", alpha=0.15, linewidths=0)
    if S is not None:
        a2.plot(S[:, 0], S[:, 2], c="#c0392b", lw=2.4)
    a2.plot([0], [0], "ks", ms=6)
    a2.axhline(0.0, color="#8a97a3", lw=0.8)
    a2.set_aspect("equal"); a2.set_xlabel("x (m)"); a2.set_ylabel("z (m)")
    a2.set_title("Side view (x-z)", fontsize=10)
    a2.grid(alpha=0.25)
    fig.legend(handles=[
        plt.Line2D([], [], marker="o", ls="", color="#8a97a3", label="reachable cloud"),
        plt.Line2D([], [], color="#c0392b", lw=2.4, label="sine demo"),
        plt.Line2D([], [], marker="s", ls="", color="k", label="base"),
        plt.Line2D([], [], ls="--", color="#3b5bdb", label=f"max horiz. reach {REACH:.2f} m")],
        loc="lower center", ncol=4, fontsize=8.5, frameon=False,
        bbox_to_anchor=(0.5, -0.02))
    fig.suptitle("ARM-450 — reachable workspace (verified vs URDF/FK) with sine demo overlaid",
                 fontsize=11.5)
    fig.tight_layout(rect=[0, 0.10, 1, 0.93])
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=145, facecolor="white")
    print(f"  wrote {out}")
