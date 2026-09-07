#!/usr/bin/env python3
"""
myCobot 280 kinematics — Modified (Craig) DH convention.

This module is PURE MATH (no ROS dependencies) so it can be imported by both the
ROS 2 nodes and the offline analysis/plotting scripts. It implements the exact
methodology from the project notes:

  * Forward kinematics  : product of Modified-DH link transforms (base -> tip).
  * Jacobian            : velocity-propagation method (Craig), assembled as the
                          geometric Jacobian  [ z_i x (p_e - p_i) ; z_i ].

The Modified-DH table below was *identified* from the official mycobot_280.urdf
and verified to reproduce the URDF forward kinematics to ~1e-15 m (see
analysis/verify_kinematics.py). `urdf_fk` is kept as an independent ground-truth
cross-check of the DH model.

Frames / joints (all revolute, axis = local z):
    j1 link1_to_link2 ... j6 link6_to_link6_flange
End-effector frame = link6_flange.
"""
from __future__ import annotations
import numpy as np

PI = np.pi

# --------------------------------------------------------------------------- #
# Joint limits (radians) straight from the URDF <limit> tags.
# --------------------------------------------------------------------------- #
JOINT_NAMES = [
    "link1_to_link2", "link2_to_link3", "link3_to_link4",
    "link4_to_link5", "link5_to_link6", "link6_to_link6_flange",
]
JOINT_LIMITS = np.array([
    [-2.879793, 2.879793],
    [-2.879793, 2.879793],
    [-2.879793, 2.879793],
    [-2.879793, 2.879793],
    [-2.879793, 2.879793],
    [-3.05,     3.05],
])

# --------------------------------------------------------------------------- #
# Homogeneous-transform primitives.
# --------------------------------------------------------------------------- #
def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])

def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1.0]])

def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])

def translate(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T

def rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw:  R = Rz(yaw) Ry(pitch) Rx(roll)."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

# --------------------------------------------------------------------------- #
# Modified-DH model (VERIFIED: reproduces URDF FK to ~1e-15 m).
# Row order: (alpha_{i-1}, a_{i-1}, d_i, theta_offset_i)
# Modified-DH link transform:
#     i-1_T_i = Rotx(alpha_{i-1}) * Transx(a_{i-1}) * Rotz(theta_i) * Transz(d_i)
# with theta_i = theta_offset_i + q_i.
# --------------------------------------------------------------------------- #
DH_TABLE = np.array([
    # alpha_{i-1},   a_{i-1},   d_i,      theta_offset
    [0.0,            0.0,       0.13056,   PI / 2],
    [PI / 2,         0.0,       0.0,      -PI / 2],
    [0.0,           -0.1104,    0.0,       0.0],
    [0.0,           -0.096,     0.06062,  -PI / 2],
    [PI / 2,         0.0,       0.07318,   PI / 2],
    [-PI / 2,        0.0,       0.0456,    0.0],
])

def dh_link_transform(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    """Single Modified-DH link transform i-1_T_i."""
    return rot_x(alpha) @ translate(a, 0, 0) @ rot_z(theta) @ translate(0, 0, d)

def fk_frames(q) -> list[np.ndarray]:
    """Return [T_0, T_1, ..., T_6] where T_i is base->frame i (T_6 = EE).

    Uses the Modified-DH table. This is the project's primary FK.
    """
    q = np.asarray(q, dtype=float)
    T = np.eye(4)
    frames = [T.copy()]
    for i in range(6):
        alpha, a, d, th_off = DH_TABLE[i]
        T = T @ dh_link_transform(alpha, a, d, th_off + q[i])
        frames.append(T.copy())
    return frames

def forward_kinematics(q) -> np.ndarray:
    """Base -> end-effector (link6_flange) homogeneous transform (4x4)."""
    return fk_frames(q)[-1]

def ee_position(q) -> np.ndarray:
    return forward_kinematics(q)[:3, 3]

# --------------------------------------------------------------------------- #
# Jacobian — velocity-propagation method (Craig).
# For an all-revolute arm the frame-by-frame propagation of angular/linear
# velocity collapses to the geometric Jacobian:
#     J_v[:,i] = z_i x (p_e - p_i)      (linear part)
#     J_w[:,i] = z_i                    (angular part)
# where z_i and p_i are the axis and origin of joint i expressed in the base
# frame. Verified against a finite-difference Jacobian to ~2.5e-7.
# --------------------------------------------------------------------------- #
def jacobian(q) -> np.ndarray:
    """6x6 geometric Jacobian in the base frame (linear rows 0:3, angular 3:6)."""
    frames = fk_frames(q)
    p_e = frames[-1][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z_i = frames[i + 1][:3, 2]   # joint i axis (base coords)
        p_i = frames[i + 1][:3, 3]   # a point on joint i axis
        J[:3, i] = np.cross(z_i, p_e - p_i)
        J[3:, i] = z_i
    return J

# --------------------------------------------------------------------------- #
# Independent ground-truth FK straight from the URDF joint origins.
# Used only by the verification script to validate the DH model above.
# joint origin = (roll, pitch, yaw, x, y, z); rotation about child z by q_i.
# --------------------------------------------------------------------------- #
URDF_JOINT_ORIGINS = [
    (0.0,     0.0,     PI / 2,   0.0,     0.0,      0.13156),
    (0.0,     PI / 2, -PI / 2,   0.0,     0.0,     -0.001),
    (0.0,     0.0,     0.0,     -0.1104,  0.0,      0.0),
    (0.0,     0.0,    -PI / 2,  -0.096,   0.0,      0.06062),
    (PI / 2, -PI / 2,  0.0,      0.0,    -0.07318,  0.0),
    (-PI / 2, 0.0,     0.0,      0.0,     0.0456,   0.0),
]

def urdf_fk(q) -> np.ndarray:
    """Ground-truth base->EE transform built directly from URDF joint origins."""
    q = np.asarray(q, dtype=float)
    T = np.eye(4)
    for i in range(6):
        r, p, yw, x, y, z = URDF_JOINT_ORIGINS[i]
        T = T @ translate(x, y, z) @ rpy(r, p, yw) @ rot_z(q[i])
    return T

def clamp_to_limits(q) -> np.ndarray:
    """Clip a joint vector into the URDF limits."""
    return np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
