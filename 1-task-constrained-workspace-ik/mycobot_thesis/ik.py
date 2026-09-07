#!/usr/bin/env python3
"""
Inverse-kinematics solvers for the thesis comparison. All share one interface:

    q, info = solver(p_target, R_target, q_seed)

info = {converged, iters, pos_err (m), ori_err (rad), time_ms}. Orientation error
uses the cross-product (log-map approximation) vector. All solutions are clamped
to the joint limits.
"""
from __future__ import annotations
import time
import numpy as np
from . import robot as R


def orientation_error(Rc, Rt):
    return 0.5 * (np.cross(Rc[:, 0], Rt[:, 0]) +
                  np.cross(Rc[:, 1], Rt[:, 1]) +
                  np.cross(Rc[:, 2], Rt[:, 2]))


def _pose_error(q, p_t, R_t, position_only):
    T = R.forward_kinematics(q)
    e_pos = p_t - T[:3, 3]
    if position_only:
        return e_pos, T
    return np.hstack([e_pos, orientation_error(T[:3, :3], R_t)]), T


def _info(q, p_t, R_t, iters, t0, tol):
    T = R.forward_kinematics(q)
    pe = float(np.linalg.norm(p_t - T[:3, 3]))
    oe = 0.0 if R_t is None else float(np.linalg.norm(orientation_error(T[:3, :3], R_t)))
    return {"converged": pe < tol and (R_t is None or oe < 1e-2),
            "iters": iters, "pos_err": pe, "ori_err": oe,
            "time_ms": (time.perf_counter() - t0) * 1e3}


def damped_least_squares(p_t, R_t, q0, lam=0.04, max_iters=200, tol=1e-6,
                         position_only=False):
    q = np.asarray(q0, float).copy(); t0 = time.perf_counter(); it = 0
    for it in range(1, max_iters + 1):
        e, _ = _pose_error(q, p_t, R_t, position_only)
        if np.linalg.norm(e) < tol:
            break
        J = R.jacobian(q); J = J[:3] if position_only else J
        m = J.shape[0]
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(m), e)
        q = R.clamp_to_limits(q + dq)
    return q, _info(q, p_t, R_t, it, t0, tol)


def pseudoinverse(p_t, R_t, q0, step=1.0, max_iters=200, tol=1e-6,
                  position_only=False):
    q = np.asarray(q0, float).copy(); t0 = time.perf_counter(); it = 0
    for it in range(1, max_iters + 1):
        e, _ = _pose_error(q, p_t, R_t, position_only)
        if np.linalg.norm(e) < tol:
            break
        J = R.jacobian(q); J = J[:3] if position_only else J
        dq = np.linalg.pinv(J) @ e
        q = R.clamp_to_limits(q + step * dq)
    return q, _info(q, p_t, R_t, it, t0, tol)


def jacobian_transpose(p_t, R_t, q0, max_iters=2000, tol=1e-6,
                       position_only=False):
    q = np.asarray(q0, float).copy(); t0 = time.perf_counter(); it = 0
    for it in range(1, max_iters + 1):
        e, _ = _pose_error(q, p_t, R_t, position_only)
        if np.linalg.norm(e) < tol:
            break
        J = R.jacobian(q); J = J[:3] if position_only else J
        Je = J @ J.T @ e
        alpha = np.dot(e, Je) / (np.dot(Je, Je) + 1e-12)   # optimal step
        q = R.clamp_to_limits(q + alpha * J.T @ e)
    return q, _info(q, p_t, R_t, it, t0, tol)


def levenberg_marquardt(p_t, R_t, q0, max_iters=200, tol=1e-6, lam0=1e-2,
                        position_only=False):
    """Adaptive-damping LM: grow/shrink lambda by the error trend."""
    q = np.asarray(q0, float).copy(); t0 = time.perf_counter(); lam = lam0; it = 0
    e, _ = _pose_error(q, p_t, R_t, position_only)
    err = np.linalg.norm(e)
    for it in range(1, max_iters + 1):
        if err < tol:
            break
        J = R.jacobian(q); J = J[:3] if position_only else J
        m = J.shape[0]
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(m), e)
        q_new = R.clamp_to_limits(q + dq)
        e_new, _ = _pose_error(q_new, p_t, R_t, position_only)
        if np.linalg.norm(e_new) < err:
            q, e, err = q_new, e_new, np.linalg.norm(e_new); lam = max(lam * 0.7, 1e-4)
        else:
            lam = min(lam * 2.0, 1.0)
    return q, _info(q, p_t, R_t, it, t0, tol)


SOLVERS = {
    "DLS": damped_least_squares,
    "Pseudoinverse": pseudoinverse,
    "Jacobian-transpose": jacobian_transpose,
    "Levenberg-Marquardt": levenberg_marquardt,
}


def reachable_with_orientation(p_t, R_t, seeds=None, tol=2e-3, ori_tol=1e-2):
    """True if some seed converges to p_t with orientation R_t (feasibility test)."""
    if seeds is None:
        seeds = [np.zeros(6)]
    for q0 in seeds:
        q, info = damped_least_squares(p_t, R_t, q0)
        if info["pos_err"] < tol and info["ori_err"] < ori_tol:
            return True, q
    return False, None
