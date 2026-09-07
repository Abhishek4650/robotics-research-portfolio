#!/usr/bin/env python3
"""
Damped-least-squares inverse kinematics (pure math, no ROS).

Uses the analytical FK + velocity-propagation Jacobian from `kinematics.py`:

    q_{k+1} = q_k + Jᵀ (J Jᵀ + λ² I)⁻¹ · e

with e = [ p_target - p_current ; orientation_error ]. Damping λ keeps the update
well-behaved near singularities. Solutions are clamped to the joint limits.

This is the PRIMARY (analytical) solver; `analysis/verify_ik.py` cross-checks each
solved waypoint against ikpy independently.
"""
from __future__ import annotations
import numpy as np
from . import kinematics as K


def orientation_error(R_current, R_target):
    """Cross-product orientation error (small-angle vector in the base frame)."""
    return 0.5 * (np.cross(R_current[:, 0], R_target[:, 0]) +
                  np.cross(R_current[:, 1], R_target[:, 1]) +
                  np.cross(R_current[:, 2], R_target[:, 2]))


class DLSIKSolver:
    def __init__(self, lam=0.05, max_iters=200, tol=1e-5, position_only=False):
        self.lam = lam
        self.max_iters = max_iters
        self.tol = tol
        self.position_only = position_only

    def solve(self, p_target, R_target, q_seed):
        """Solve one pose. Returns (q, info dict)."""
        q = np.asarray(q_seed, dtype=float).copy()
        it = 0
        for it in range(1, self.max_iters + 1):
            T = K.forward_kinematics(q)
            e_pos = p_target - T[:3, 3]
            if self.position_only:
                e = e_pos
                J = K.jacobian(q)[:3, :]
            else:
                e = np.hstack([e_pos, orientation_error(T[:3, :3], R_target)])
                J = K.jacobian(q)
            if np.linalg.norm(e) < self.tol:
                break
            m = J.shape[0]
            dq = J.T @ np.linalg.solve(J @ J.T + self.lam ** 2 * np.eye(m), e)
            q = K.clamp_to_limits(q + dq)
        T = K.forward_kinematics(q)
        info = {
            "iters": it,
            "pos_err": float(np.linalg.norm(p_target - T[:3, 3])),
            "ori_err": float(np.linalg.norm(orientation_error(T[:3, :3], R_target))),
        }
        return q, info

    def solve_trajectory(self, positions, R_target, q_start=None):
        """Solve a sequence of targets, seeding each from the previous solution.

        Returns Q (N,6) and a list of per-waypoint info dicts.
        """
        q = np.zeros(6) if q_start is None else np.asarray(q_start, float).copy()
        Q, infos = [], []
        for p in positions:
            q, info = self.solve(p, R_target, q)
            Q.append(q.copy())
            infos.append(info)
        return np.array(Q), infos
