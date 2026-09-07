"""
ARM-450 kinematics by MODIFIED (CRAIG) DH — the user's own convention.

Convention, exactly as in the reference notes: frame i is attached to link i,
parameters (alpha_{i-1}, a_{i-1}, d_i, theta_i), and

      a_{i-1}      mutual-perpendicular distance between Z_{i-1} and Z_i,
                   measured along X_{i-1}
      alpha_{i-1}  angle from Z_{i-1} to Z_i, measured about X_{i-1}
      d_i          distance from X_{i-1} to X_i, measured along Z_i
      theta_i      angle from X_{i-1} to X_i, measured about Z_i

               [ ct        -st       0       a      ]
  i-1_i T  =   [ st*ca    ct*ca    -sa    -sa*d     ]
               [ st*sa    ct*sa     ca     ca*d     ]
               [ 0         0        0       1       ]

HOW THE TABLE WAS OBTAINED, because two of the numbers are not what they look
like. The joint axes were taken from the URDF at q = 0 and the common normals
computed between consecutive axes. Every axis passes through the base Z line,
so every a_{i-1} is zero EXCEPT a_2 = 119 between the two parallel pitch axes.

The d's are the trap. d_4 = 181, not 119: frame 3's origin sits where X_3 meets
Z_3 (z = 209) and frame 4's sits where Z_4 meets Z_5 (z = 390), so the offset
along Z_4 is 119 + 62. Writing 119 there — the "obvious" link length — puts the
tool 193 mm out and was the first answer this file gave.

The Jacobian is built by VELOCITY PROPAGATION, base to tip, not by numerical
differencing:

      i+1_w_{i+1} = i+1_i R . i_w_i  +  thetadot_{i+1} . Zhat
      i+1_v_{i+1} = i+1_i R . ( i_v_i + i_w_i x i_P_{i+1} )

The prismatic branch is kept even though ARM-450 is all-revolute: the reference
method covers both and a joint type is then a one-line change.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

#            alpha_{i-1}   a_{i-1}    d_i     theta offset   (deg / mm)
DH = np.array([
    [   0.0,     0.0,    90.0,   180.0],   # 1  base yaw
    [  90.0,     0.0,     0.0,    90.0],   # 2  shoulder pitch
    [   0.0,   119.0,     0.0,    90.0],   # 3  elbow pitch
    [  90.0,     0.0,   181.0,   180.0],   # 4  forearm roll
    [  90.0,     0.0,     0.0,   180.0],   # 5  wrist pitch
    [  90.0,     0.0,     0.0,     0.0],   # 6  tool roll
])
REVOLUTE = [True] * 6
D_TOOL = 60.0                 # frame 6 origin -> TCP, along Z6


def craig_T(alpha, a, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([[   ct,    -st,  0.0,      a],
                     [st*ca,  ct*ca,  -sa,  -sa*d],
                     [st*sa,  ct*sa,   ca,   ca*d],
                     [  0.0,    0.0,  0.0,    1.0]])


def link_T(i, qi):
    al, a, d, off = DH[i]
    return craig_T(np.radians(al), a, d, qi + np.radians(off))


def fk(q, tool=True):
    """Base -> TCP. Position in mm."""
    T = np.eye(4)
    for i in range(6):
        T = T @ link_T(i, q[i])
    if tool:
        Tt = np.eye(4); Tt[2, 3] = D_TOOL
        T = T @ Tt
    return T


def frames(q):
    """Every intermediate transform, for the propagation."""
    out, T = [], np.eye(4)
    for i in range(6):
        Ti = link_T(i, q[i])
        T = T @ Ti
        out.append((Ti, T.copy()))
    return out


def jacobian(q):
    """6x6 Jacobian in the BASE frame, by velocity propagation.

    Each column is the tip twist for a unit rate at that joint. Carried
    frame-by-frame rather than differenced, which is the method in the notes
    and also the only version that stays exact near a singularity.
    """
    n = 6
    W = np.zeros((3, n))          # i_w_i, columns = per-joint contribution
    V = np.zeros((3, n))          # i_v_i
    Ts = frames(q)
    for i in range(n):
        Ti = Ts[i][0]
        R = Ti[:3, :3].T          # i+1_i R
        P = Ti[:3, 3]             # i_P_{i+1}, expressed in frame i
        Vn = R @ (V + np.stack([np.cross(W[:, k], P) for k in range(n)], axis=1))
        Wn = R @ W
        z = np.array([0.0, 0.0, 1.0])
        if REVOLUTE[i]:
            Wn[:, i] += z
        else:
            Vn[:, i] += z
        W, V = Wn, Vn
    # rigid tool extension along Z6
    Pt = np.array([0.0, 0.0, D_TOOL])
    V = V + np.stack([np.cross(W[:, k], Pt) for k in range(n)], axis=1)
    R0 = Ts[-1][1][:3, :3]        # base -> frame 6
    return np.vstack([R0 @ V, R0 @ W])


def jacobian_numeric(q, eps=1e-7):
    """Finite difference. Only ever used to check the propagated Jacobian."""
    J = np.zeros((6, 6))
    T0 = fk(q); p0, R0 = T0[:3, 3], T0[:3, :3]
    for k in range(6):
        dq = np.array(q, float); dq[k] += eps
        T1 = fk(dq)
        J[:3, k] = (T1[:3, 3] - p0) / eps
        dR = (T1[:3, :3] @ R0.T - np.eye(3)) / eps
        J[3:, k] = [dR[2, 1], dR[0, 2], dR[1, 0]]
    return J


def ik(p_target, z_target=None, q0=None, w_ori=0.05, iters=200, tol=1e-9):
    """Damped least-squares IK driven by the propagated Jacobian.

    Newton on the pose error with Levenberg damping, so it does not blow up at
    the J4/J6 wrist singularity where the two roll axes go collinear.
    """
    q = np.zeros(6) if q0 is None else np.array(q0, float)
    lam = 1e-2
    for _ in range(iters):
        T = fk(q)
        e = np.concatenate([p_target - T[:3, 3],
                            w_ori * 1000.0 * (np.zeros(3) if z_target is None
                                              else np.cross(T[:3, 2], z_target))])
        if np.linalg.norm(e[:3]) < tol:
            break
        J = jacobian(q)
        dq = np.linalg.solve(J.T @ J + lam * np.eye(6), J.T @ e)
        q = q + np.clip(dq, -0.2, 0.2)
    T = fk(q)
    return q, float(np.linalg.norm(p_target - T[:3, 3]))
