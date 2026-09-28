#!/usr/bin/env python3
"""
ARM-450 rev I.1 inverse kinematics.

1. ANALYTICAL (closed form). J4, J5 and J6 meet in one point, the wrist centre
   W, so position and orientation decouple (Pieper):
     W = p - d_T . ^0_T R . Zhat                         (back off the tool)
     theta1        from the direction of W seen from above  (2 branches: front / back)
     theta2,theta3 planar 2-link arm L2 = a_2, L3 = d_4    (2 branches: elbow)
     theta4..6     ^3_6R = ^0_3R^T . ^0_6R = Rx(90) . Rz(th4) Ry(th5) Rz(th6)
                   = Z-Y-Z Euler angles                     (2 branches: wrist flip)
   -> up to 8 exact solutions, then the joint limits pick the valid ones.

2. NUMERICAL, all on the velocity-propagation Jacobian (as in the myCobot
   thesis comparison): damped least squares (the author's primary method, as in Rsine),
   pseudoinverse, Jacobian transpose, Levenberg-Marquardt.

Every solver:  th, info = solver(p_target (mm), R_target, th_seed)
info = {converged, iters, pos_err (mm), ori_err (rad), time_ms}
"""
from __future__ import annotations

import time

import numpy as np

from . import robot as R


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


# ---------------------------------------------------------------- analytical
def analytical(p, Rt, all_branches=True):
    """All closed-form solutions (DH theta, rad) for the tool pose (p, Rt).
    Returns a list of (theta[6], branch label, within_limits)."""
    p = np.asarray(p, float)
    W = p - R.D_TOOL * Rt[:, 2]                       # wrist centre
    L2, L3 = R.A2, R.D4
    out = []
    rho = np.hypot(W[0], W[1])
    base = np.arctan2(W[1], W[0]) if rho > 1e-9 else 0.0
    for sh, th1 in (("front", base), ("back", base + np.pi)):
        r = rho if sh == "front" else -rho            # signed reach along X1
        s = W[2] - R.D1
        D = (r * r + s * s - L2 * L2 - L3 * L3) / (2 * L2 * L3)
        if abs(D) > 1 + 1e-12:
            continue                                  # out of reach
        D = np.clip(D, -1.0, 1.0)
        for el, sgn in (("elbow+", 1.0), ("elbow-", -1.0)):
            ph3 = sgn * np.arccos(D)                  # lean of the forearm relative to the upper link
            ph2 = np.arctan2(r, s) - np.arctan2(L3 * np.sin(ph3), L2 + L3 * np.cos(ph3))
            th2, th3 = ph2 - np.pi / 2, ph3 + np.pi / 2
            T3 = R.fk_frames([th1, th2, th3, 0, 0, 0])[3]
            M = _rx(np.pi / 2).T @ T3[:3, :3].T @ Rt  # = Rz(th4) Ry(th5) Rz(th6)
            s5 = np.hypot(M[0, 2], M[1, 2])
            for wr, sg in (("wrist+", 1.0), ("wrist-", -1.0)):
                th5 = np.arctan2(sg * s5, M[2, 2])
                if s5 < 1e-9:                          # wrist singular: J4, J6 collinear
                    th4 = 0.0
                    th6 = np.arctan2(M[1, 0], M[0, 0]) - th4 if M[2, 2] > 0 else np.arctan2(-M[1, 0], -M[0, 0]) + th4
                else:
                    th4 = np.arctan2(sg * M[1, 2], sg * M[0, 2])
                    th6 = np.arctan2(sg * M[2, 1], -sg * M[2, 0])
                th = R.wrap(np.array([th1, th2, th3, th4, th5, th6]))
                out.append((th, "%s %s %s" % (sh, el, wr), R.within_limits(th)))
                if s5 < 1e-9:
                    break                              # the flip is the same pose
    return out if all_branches else [o for o in out if o[2]]


def analytical_best(p, Rt, th_prev=None):
    """the valid closed-form solution closest to th_prev (continuity), or None."""
    sols = [s for s in analytical(p, Rt) if s[2]]
    if not sols:
        return None
    if th_prev is None:
        th_prev = R.q_to_theta(R.READY_Q)
    return min(sols, key=lambda s: np.linalg.norm(s[0] - th_prev))[0]


# ---------------------------------------------------------------- numerical
def orientation_error(Rc, Rt):
    """0.5 sum(x_c x x_t) -- small-angle rotation vector from current to target."""
    return 0.5 * (np.cross(Rc[:, 0], Rt[:, 0]) + np.cross(Rc[:, 1], Rt[:, 1]) + np.cross(Rc[:, 2], Rt[:, 2]))


W_ORI = 100.0          # mm per rad: weighs the orientation error against the position error


def _err(th, p, Rt):
    T = R.fk(th)
    e = np.hstack([p - T[:3, 3], W_ORI * orientation_error(T[:3, :3], Rt)])
    return e, T


def _scaled_J(th):
    J = R.jacobian(th).copy()
    J[3:] *= W_ORI
    return J


def _info(th, p, Rt, it, t0, tol):
    T = R.fk(th)
    pe = float(np.linalg.norm(p - T[:3, 3]))
    oe = float(np.linalg.norm(orientation_error(T[:3, :3], Rt)))
    return dict(converged=bool(pe < tol and oe < 1e-6 and R.within_limits(th)), iters=it, pos_err=pe, ori_err=oe,
                time_ms=(time.perf_counter() - t0) * 1e3)


def _clamp(th):
    return np.clip(th, R.TH_LIMITS[:, 0], R.TH_LIMITS[:, 1])


def damped_least_squares(p, Rt, th0, lam=5.0, max_iters=300, tol=1e-6):
    """dq = J^T (J J^T + lambda^2 I)^-1 e   -- the author's method (Rsine)."""
    th, t0, it = np.array(th0, float), time.perf_counter(), 0
    for it in range(1, max_iters + 1):
        e, _ = _err(th, p, Rt)
        if np.linalg.norm(e) < tol:
            break
        J = _scaled_J(th)
        th = _clamp(th + J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e))
    return th, _info(th, p, Rt, it, t0, tol)


def pseudoinverse(p, Rt, th0, max_iters=300, tol=1e-6):
    th, t0, it = np.array(th0, float), time.perf_counter(), 0
    for it in range(1, max_iters + 1):
        e, _ = _err(th, p, Rt)
        if np.linalg.norm(e) < tol:
            break
        th = _clamp(th + np.linalg.pinv(_scaled_J(th)) @ e)
    return th, _info(th, p, Rt, it, t0, tol)


def jacobian_transpose(p, Rt, th0, max_iters=3000, tol=1e-6):
    """dq = a J^T e with the optimal step a = <e, J J^T e> / |J J^T e|^2."""
    th, t0, it = np.array(th0, float), time.perf_counter(), 0
    for it in range(1, max_iters + 1):
        e, _ = _err(th, p, Rt)
        if np.linalg.norm(e) < tol:
            break
        J = _scaled_J(th)
        g = J.T @ e
        Jg = J @ g
        a = float(e @ Jg) / max(float(Jg @ Jg), 1e-18)
        th = _clamp(th + a * g)
    return th, _info(th, p, Rt, it, t0, tol)


def levenberg_marquardt(p, Rt, th0, lam0=1e-2, max_iters=300, tol=1e-6):
    """(J^T J + lambda diag(J^T J)) dq = J^T e, lambda adapted on success / failure."""
    th, t0, it = np.array(th0, float), time.perf_counter(), 0
    lam = lam0
    e, _ = _err(th, p, Rt)
    for it in range(1, max_iters + 1):
        if np.linalg.norm(e) < tol:
            break
        J = _scaled_J(th)
        A = J.T @ J
        dq = np.linalg.solve(A + lam * np.diag(np.diag(A)) + 1e-12 * np.eye(6), J.T @ e)
        th2 = _clamp(th + dq)
        e2, _ = _err(th2, p, Rt)
        if np.linalg.norm(e2) < np.linalg.norm(e):
            th, e, lam = th2, e2, max(lam / 3.0, 1e-9)
        else:
            lam = min(lam * 5.0, 1e9)
    return th, _info(th, p, Rt, it, t0, tol)


SOLVERS = {"Damped least squares": damped_least_squares, "Pseudoinverse": pseudoinverse,
           "Jacobian transpose": jacobian_transpose, "Levenberg-Marquardt": levenberg_marquardt}
