#!/usr/bin/env python3
"""
myCobot 280 kinematic model for the thesis (self-contained copy of the verified
R_sine model, extended with the differential-geometric metrics the thesis needs:
manipulability, condition number, and orientation-feasibility helpers).

Modified (Craig) DH; forward kinematics matches the URDF to ~1e-15 m and the
geometric Jacobian matches finite differences to ~2.5e-7 (verified in R_sine).
"""
from __future__ import annotations
import numpy as np

PI = np.pi

JOINT_NAMES = [
    "link1_to_link2", "link2_to_link3", "link3_to_link4",
    "link4_to_link5", "link5_to_link6", "link6_to_link6_flange",
]
JOINT_LIMITS = np.array([
    [-2.879793, 2.879793], [-2.879793, 2.879793], [-2.879793, 2.879793],
    [-2.879793, 2.879793], [-2.879793, 2.879793], [-3.05, 3.05],
])

# Modified-DH table (alpha_{i-1}, a_{i-1}, d_i, theta_offset), verified in R_sine.
DH_TABLE = np.array([
    [0.0,     0.0,     0.13056,  PI / 2],
    [PI / 2,  0.0,     0.0,     -PI / 2],
    [0.0,    -0.1104,  0.0,      0.0],
    [0.0,    -0.096,   0.06062, -PI / 2],
    [PI / 2,  0.0,     0.07318,  PI / 2],
    [-PI / 2, 0.0,     0.0456,   0.0],
])


# ---------------- transforms ----------------
def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])

def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])

def translate(x, y, z):
    T = np.eye(4); T[:3, 3] = (x, y, z); return T

def _dh_link(alpha, a, d, theta):
    return rot_x(alpha) @ translate(a, 0, 0) @ rot_z(theta) @ translate(0, 0, d)


# ---------------- FK / Jacobian ----------------
def fk_frames(q):
    q = np.asarray(q, float)
    T = np.eye(4); frames = [T.copy()]
    for i in range(6):
        al, a, d, th = DH_TABLE[i]
        T = T @ _dh_link(al, a, d, th + q[i])
        frames.append(T.copy())
    return frames

def forward_kinematics(q):
    return fk_frames(q)[-1]

def ee_position(q):
    return forward_kinematics(q)[:3, 3]

def jacobian(q):
    """6x6 geometric Jacobian in the base frame (rows 0:3 linear, 3:6 angular)."""
    frames = fk_frames(q)
    p_e = frames[-1][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = frames[i + 1][:3, 2]
        p = frames[i + 1][:3, 3]
        J[:3, i] = np.cross(z, p_e - p)
        J[3:, i] = z
    return J


# ---------------- thesis metrics ----------------
def manipulability(q, part="full"):
    """Yoshikawa manipulability  w = sqrt(det(J J^T)).

    part: 'full' (6xN), 'pos' (translational 3xN), 'rot' (rotational 3xN).
    Zero at a singularity; larger = further from singular / more dexterous.
    """
    J = jacobian(q)
    Js = {"full": J, "pos": J[:3, :], "rot": J[3:, :]}[part]
    d = np.linalg.det(Js @ Js.T)
    return float(np.sqrt(max(d, 0.0)))

def condition_number(q, part="pos"):
    """Ratio of largest/smallest singular value of J (>=1). Large => ill-conditioned."""
    J = jacobian(q)
    Js = {"full": J, "pos": J[:3, :], "rot": J[3:, :]}[part]
    s = np.linalg.svd(Js, compute_uv=False)
    smin = s[-1]
    return float(s[0] / smin) if smin > 1e-12 else np.inf

def min_singular_value(q, part="pos"):
    J = jacobian(q)
    Js = {"full": J, "pos": J[:3, :], "rot": J[3:, :]}[part]
    return float(np.linalg.svd(Js, compute_uv=False)[-1])

def clamp_to_limits(q):
    return np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

def random_q(rng):
    return rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])


# ---------------- surface / orientation helpers ----------------
def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)

def target_orientation(normal, advance):
    """Rotation with flange z along `normal` (pen axis) and x along in-plane `advance`."""
    z_t = _unit(normal)
    x_t = _unit(advance - np.dot(advance, z_t) * z_t)
    y_t = np.cross(z_t, x_t)
    return np.column_stack([x_t, y_t, z_t])
