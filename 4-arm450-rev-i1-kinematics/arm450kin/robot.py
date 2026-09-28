#!/usr/bin/env python3
"""
ARM-450 rev I.1 kinematic model -- in the author's own convention.

MODIFIED (CRAIG) DH, frame i attached to link i, parameters
(alpha_{i-1}, a_{i-1}, d_i, theta_i):

  a_{i-1}      mutual-perpendicular distance between Z_{i-1} and Z_i, along X_{i-1}
  alpha_{i-1}  angle from Z_{i-1} to Z_i, about X_{i-1}
  d_i          distance from X_{i-1} to X_i, along Z_i
  theta_i      angle from X_{i-1} to X_i, about Z_i

                  [ c(th)          -s(th)          0            a_{i-1}         ]
  ^{i-1}_i T  =   [ s(th) c(al)    c(th) c(al)    -s(al)       -s(al) d_i       ]
                  [ s(th) s(al)    c(th) s(al)     c(al)        c(al) d_i       ]
                  [ 0              0               0            1               ]

  = Rot_x(alpha_{i-1}) . Trans_x(a_{i-1}) . Rot_z(theta_i) . Trans_z(d_i)

Frames: base frame 0 = the world frame of the CAD (Z up, X forward, origin on
the J1 axis at the base's top face). Every Z_i is chosen with the right-hand
rule and "+Z up" at home: vertical joints (J1, J4, J6) +z, pitch joints (J2,
J3, J5) +y, so a POSITIVE theta leans the arm forward (+x) / turns it
counter-clockwise seen from above.

The geometry comes from the rev I.1 CAD (cad_geometry.json, the servo axes of
the assembled arm and the tool-flange face), NOT from nominal numbers; the
table itself is derived and checked in analysis/s1_dh_from_cad.py.

Jacobian: VELOCITY PROPAGATION (Craig), base to tip:
  ^{i+1}w_{i+1} = ^{i+1}_iR . ^iw_i + thetadot_{i+1} . ^{i+1}Zhat_{i+1}
  ^{i+1}v_{i+1} = ^{i+1}_iR . (^iv_i + ^iw_i x ^iP_{i+1}) + ddot_{i+1} . ^{i+1}Zhat_{i+1}
(the prismatic branch is kept; ARM-450 is all-revolute).

Units: mm and rad inside; degrees only in the printed tables.
"""
from __future__ import annotations

import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GEO = json.load(open(os.path.join(HERE, "cad_geometry.json")))
_z = lambda k: GEO["axes"][k]["point"][2]

# ---- lengths measured on the CAD (mm) --------------------------------------
D1 = _z(1)                        # base frame -> shoulder (J2 axis height)          90.000
A2 = _z(2) - _z(1)                # shoulder -> elbow (J2 axis to J3 axis)           119.000
D4 = _z(4) - _z(2)                # elbow -> wrist centre (J3 axis to J5 axis)       221.369
D_TOOL = GEO["tool_face_centre"][2] - _z(4)   # wrist centre -> tool-flange face    88.089
TABLE_Z = GEO["table_z"]          # underside of the foot = the table top           -69.900

# ---- the Modified-DH table:  alpha_{i-1} (rad), a_{i-1} (mm), d_i (mm), theta offset (rad)
DH = np.array([
    [0.0,          0.0,  D1,  0.0],          # 1  J1 base yaw
    [-np.pi / 2,   0.0,  0.0, -np.pi / 2],   # 2  J2 shoulder pitch
    [0.0,          A2,   0.0, np.pi / 2],    # 3  J3 elbow pitch
    [np.pi / 2,    0.0,  D4,  0.0],          # 4  J4 forearm roll
    [-np.pi / 2,   0.0,  0.0, 0.0],          # 5  J5 wrist pitch
    [np.pi / 2,    0.0,  0.0, 0.0],          # 6  J6 tool roll
])
REVOLUTE = [True] * 6
NAMES = ["J1 base yaw", "J2 shoulder", "J3 elbow", "J4 forearm roll", "J5 wrist pitch", "J6 tool roll"]

# ---- servo / URDF joint angles q  <->  DH joint variables theta -------------
# theta_i = SIGN_i * q_i + offset_i. SIGN = the direction of the servo axis in the
# CAD / URDF against the DH Z_i (J1, J2, J4, J6 servos turn about -z / -y).
# Proven on 2000 random poses against the CAD in analysis/s2_verify_fk.py.
SIGN = np.array([-1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
Q_RANGE = np.radians(GEO["limits_deg"])           # +- about home, servo side
Q_LIMITS = np.stack([-Q_RANGE, Q_RANGE], axis=1)
TH_LIMITS = np.sort(np.stack([SIGN * Q_LIMITS[:, 0], SIGN * Q_LIMITS[:, 1]], axis=1), axis=1) + DH[:, 3:4]

# poses (servo q, degrees): home = straight up (singular), ready = the manual's bent pose
HOME_Q = np.zeros(6)
READY_Q = np.radians([0.0, 30.0, -60.0, 0.0, 30.0, 0.0])


def q_to_theta(q):
    return SIGN * np.asarray(q, float) + DH[:, 3]


def theta_to_q(th):
    return (np.asarray(th, float) - DH[:, 3]) * SIGN


def within_limits(th, tol=1e-9):
    th = np.asarray(th, float)
    return bool(np.all(th >= TH_LIMITS[:, 0] - tol) and np.all(th <= TH_LIMITS[:, 1] + tol))


def wrap(th):
    """each theta to the 2pi-equivalent nearest its limit window's centre."""
    th = np.asarray(th, float).copy()
    mid = TH_LIMITS.mean(axis=1)
    return th - 2 * np.pi * np.round((th - mid) / (2 * np.pi))


# ---- transforms ----------------------------------------------------------------
def craig_T(alpha, a, d, theta):
    """^{i-1}_i T, exactly the matrix of the convention."""
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
    return np.array([[ct,      -st,      0.0,  a],
                     [st * ca,  ct * ca, -sa, -sa * d],
                     [st * sa,  ct * sa,  ca,  ca * d],
                     [0.0,      0.0,      0.0, 1.0]])


def link_T(i, th_i):
    al, a, d, _ = DH[i]
    return craig_T(al, a, d, th_i)


def tool_T():
    T = np.eye(4); T[2, 3] = D_TOOL
    return T


def fk_frames(th):
    """[^0_0T, ^0_1T, ..., ^0_6T, ^0_T T] for DH joint variables theta (rad)."""
    Ts, T = [np.eye(4)], np.eye(4)
    for i in range(6):
        T = T @ link_T(i, th[i])
        Ts.append(T.copy())
    Ts.append(T @ tool_T())
    return Ts


def fk(th, tool=True):
    """^0_T T (tool-flange face centre) or ^0_6 T (wrist centre, tool=False)."""
    Ts = fk_frames(th)
    return Ts[-1] if tool else Ts[-2]


def fk_q(q, tool=True):
    return fk(q_to_theta(q), tool)


# ---- Jacobian by velocity propagation ------------------------------------------
def jacobian(th, tool=True):
    """6x6 Jacobian in the BASE frame, rows (v; w), columns = unit rate of each
    joint, propagated frame by frame (Craig):
        ^{i+1}w_{i+1} = ^{i+1}_iR ^iw_i + thetadot_{i+1} ^{i+1}Z
        ^{i+1}v_{i+1} = ^{i+1}_iR (^iv_i + ^iw_i x ^iP_{i+1}) + ddot_{i+1} ^{i+1}Z
    W[:, k], V[:, k] carry the contribution of joint k in the current frame."""
    n = 6
    W, V = np.zeros((3, n)), np.zeros((3, n))
    T0 = np.eye(4)
    for i in range(n):
        Ti = link_T(i, th[i])                 # ^{i}_{i+1}T  (frame i -> i+1)
        R = Ti[:3, :3].T                      # ^{i+1}_iR
        P = Ti[:3, 3]                         # ^iP_{i+1}
        V = R @ (V + np.cross(W.T, P).T)
        W = R @ W
        z = np.array([0.0, 0.0, 1.0])
        if REVOLUTE[i]:
            W[:, i] += z
        else:
            V[:, i] += z
        T0 = T0 @ Ti
    if tool:                                   # rigid step to the flange face along Z6
        V = V + np.cross(W.T, np.array([0.0, 0.0, D_TOOL])).T
    R06 = T0[:3, :3]
    return np.vstack([R06 @ V, R06 @ W])


def jacobian_geometric(th, tool=True):
    """z_i x (p_e - p_i); z_i -- the textbook form, used only as a cross-check."""
    Ts = fk_frames(th)
    pe = (Ts[-1] if tool else Ts[-2])[:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z, p = Ts[i + 1][:3, 2], Ts[i + 1][:3, 3]
        J[:3, i] = np.cross(z, pe - p); J[3:, i] = z
    return J


def jacobian_numeric(th, eps=1e-7, tool=True):
    J = np.zeros((6, 6))
    T0 = fk(th, tool); p0, R0 = T0[:3, 3], T0[:3, :3]
    for k in range(6):
        d = np.array(th, float); d[k] += eps
        T1 = fk(d, tool)
        J[:3, k] = (T1[:3, 3] - p0) / eps
        S = (T1[:3, :3] @ R0.T - np.eye(3)) / eps
        J[3:, k] = [S[2, 1], S[0, 2], S[1, 0]]
    return J


# ---- differential metrics (as in the myCobot thesis) ---------------------------
def manipulability(th, translational=True):
    """Yoshikawa w = sqrt(det(J J^T)); translational part in mm^3/rad^3 -> use J_v."""
    J = jacobian(th)
    Jv = J[:3] if translational else J
    return float(np.sqrt(max(np.linalg.det(Jv @ Jv.T), 0.0)))


def condition_number(th, translational=True):
    J = jacobian(th)
    s = np.linalg.svd(J[:3] if translational else J, compute_uv=False)
    return float(s[0] / s[-1]) if s[-1] > 1e-12 else np.inf


def min_singular_value(th):
    return float(np.linalg.svd(jacobian(th), compute_uv=False)[-1])


def dh_table_rows():
    """(i, alpha_{i-1} deg, a_{i-1} mm, d_i mm, theta_i text) for printing."""
    out = []
    for i, (al, a, d, off) in enumerate(DH, 1):
        o = np.degrees(off)
        th = "th%d" % i + ("" if abs(o) < 1e-9 else (" (home %+g deg)" % o))
        out.append((i, np.degrees(al), a, d, th))
    return out
