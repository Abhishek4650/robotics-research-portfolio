#!/usr/bin/env python3
"""
Verify the Rsine kinematics core:
  1. Modified-DH forward kinematics  ==  URDF ground-truth FK.
  2. Velocity-propagation Jacobian   ==  finite-difference Jacobian.

Run:  python3 analysis/verify_kinematics.py
No ROS or build step required.
"""
import os
import sys
import numpy as np

# allow "import Rsine.kinematics" when run straight from the source tree
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Rsine import kinematics as K   # noqa: E402


def check_fk(n=5000, seed=99):
    rng = np.random.default_rng(seed)
    worst = 0.0
    for _ in range(n):
        q = rng.uniform(K.JOINT_LIMITS[:, 0], K.JOINT_LIMITS[:, 1])
        worst = max(worst, np.abs(K.forward_kinematics(q) - K.urdf_fk(q)).max())
    return worst


def fd_jacobian(q, eps=1e-6):
    J = np.zeros((6, 6))
    T0 = K.forward_kinematics(q)
    p0, R0 = T0[:3, 3], T0[:3, :3]
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        T1 = K.forward_kinematics(q + dq)
        J[:3, i] = (T1[:3, 3] - p0) / eps
        dR = (T1[:3, :3] - R0) / eps @ R0.T
        J[3:, i] = [dR[2, 1], dR[0, 2], dR[1, 0]]
    return J


def check_jacobian(n=500, seed=7):
    rng = np.random.default_rng(seed)
    worst = 0.0
    for _ in range(n):
        q = rng.uniform(-2.8, 2.8, 6)
        worst = max(worst, np.abs(K.jacobian(q) - fd_jacobian(q)).max())
    return worst


def main():
    fk_err = check_fk()
    j_err = check_jacobian()
    home = K.forward_kinematics(np.zeros(6))
    print("=" * 60)
    print("Rsine kinematics verification")
    print("=" * 60)
    print(f"Home end-effector position (q=0): {home[:3, 3]}")
    print(f"max |DH-FK - URDF-FK|            : {fk_err:.3e} m")
    print(f"max |velprop-J - finite-diff-J|  : {j_err:.3e}")
    ok = fk_err < 1e-9 and j_err < 1e-5
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
