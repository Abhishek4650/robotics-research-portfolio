"""Vectorised kinematics for the workspace maps (numpy over thousands of
points at once). Same DH model as arm450kin.robot; checked against it in
s4_workspace.py before any map is drawn."""
import numpy as np

from common import R


def craig_batch(al, a, d, th):
    th = np.asarray(th, float)
    n = th.shape[0]
    ca, sa = np.cos(al), np.sin(al)
    ct, st = np.cos(th), np.sin(th)
    T = np.zeros((n, 4, 4))
    T[:, 0, 0], T[:, 0, 1], T[:, 0, 3] = ct, -st, a
    T[:, 1, 0], T[:, 1, 1], T[:, 1, 2], T[:, 1, 3] = st * ca, ct * ca, -sa, -sa * d
    T[:, 2, 0], T[:, 2, 1], T[:, 2, 2], T[:, 2, 3] = st * sa, ct * sa, ca, ca * d
    T[:, 3, 3] = 1.0
    return T


def fk_batch(TH, upto=6, tool=True):
    """TH (n, 6) DH angles -> (n, 4, 4) of frame `upto` (tool face if tool)."""
    n = TH.shape[0]
    T = np.tile(np.eye(4), (n, 1, 1))
    for i in range(upto):
        al, a, d, _ = R.DH[i]
        T = T @ craig_batch(al, a, d, TH[:, i])
    if tool and upto == 6:
        T = T @ R.tool_T()
    return T


def lims(i):
    return R.TH_LIMITS[i, 0], R.TH_LIMITS[i, 1]


def wrap_to(th, i):
    lo, hi = lims(i)
    mid = (lo + hi) / 2
    return th - 2 * np.pi * np.round((th - mid) / (2 * np.pi))


def inside(th, i, tol=1e-9):
    lo, hi = lims(i)
    return (th >= lo - tol) & (th <= hi + tol)


def arm_branches(W):
    """closed-form theta1..3 for wrist centres W (n, 3): list of 4 branch
    tuples (th1, th2, th3, ok) with ok = reachable and within J1..J3 limits."""
    L2, L3 = R.A2, R.D4
    rho = np.hypot(W[:, 0], W[:, 1])
    base = np.arctan2(W[:, 1], W[:, 0])
    out = []
    for sh in (0, 1):
        th1 = wrap_to(base + sh * np.pi, 0)
        r = rho if sh == 0 else -rho
        s = W[:, 2] - R.D1
        D = (r * r + s * s - L2 * L2 - L3 * L3) / (2 * L2 * L3)
        reach = np.abs(D) <= 1 + 1e-12
        D = np.clip(D, -1, 1)
        for sg in (1.0, -1.0):
            ph3 = sg * np.arccos(D)
            ph2 = np.arctan2(r, s) - np.arctan2(L3 * np.sin(ph3), L2 + L3 * np.cos(ph3))
            th2 = wrap_to(ph2 - np.pi / 2, 1); th3 = wrap_to(ph3 + np.pi / 2, 2)
            ok = reach & inside(th1, 0) & inside(th2, 1) & inside(th3, 2)
            out.append((th1, th2, th3, ok))
    return out


def wrist_for_axis(th1, th2, th3, A):
    """theta4, theta5 (both wrist branches) that point the tool axis along A
    (n, 3). theta6 is free (roll about the tool axis)."""
    n = th1.shape[0]
    TH = np.zeros((n, 6)); TH[:, 0], TH[:, 1], TH[:, 2] = th1, th2, th3
    T3 = fk_batch(TH, upto=3, tool=False)
    Rx90 = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])
    m = np.einsum("ij,njk,nk->ni", Rx90.T, np.transpose(T3[:, :3, :3], (0, 2, 1)), A)   # = (c4 s5, s4 s5, c5)
    s5 = np.hypot(m[:, 0], m[:, 1])
    res = []
    for sg in (1.0, -1.0):
        th5 = wrap_to(np.arctan2(sg * s5, m[:, 2]), 4)
        th4 = wrap_to(np.where(s5 > 1e-9, np.arctan2(sg * m[:, 1], sg * m[:, 0]), 0.0), 3)
        res.append((th4, th5, inside(th4, 3) & inside(th5, 4)))
    return res


def pen_feasible(P, A):
    """tool face at P (n, 3) with the tool axis along A (n, 3) or (3,), roll
    free: True where some branch keeps J1..J5 inside their limits (J6 is a
    free roll of +-90 deg, and a pen is symmetric about its axis, so any roll
    within J6's range will do)."""
    P = np.atleast_2d(P).astype(float)
    A = np.broadcast_to(np.asarray(A, float), P.shape)
    A = A / np.linalg.norm(A, axis=1, keepdims=True)
    W = P - R.D_TOOL * A
    ok = np.zeros(P.shape[0], bool)
    for th1, th2, th3, okA in arm_branches(W):
        if not okA.any():
            continue
        for th4, th5, okW in wrist_for_axis(th1, th2, th3, A):
            ok |= okA & okW
    return ok
