#!/usr/bin/env python3
"""
Sine path on a CONFIGURABLE drawing plane (pure math, no ROS).

A DrawingPlane is defined by an origin and two in-plane axes:
    u_axis  — the "advance" direction the pen travels along
    v_axis  — the "wave"    direction the sine oscillates along
The plane normal n = u x v. The pen (end-effector approach axis, i.e. the flange
z-axis) is driven along +/- n so it stays perpendicular to the surface; the sign
is chosen to point *into* the board (away from the robot base) by default.

Because a flat plane has a constant normal, the target orientation is constant for
the whole sine — only the target position sweeps. The same code draws on a vertical
board or a flat plate purely by changing origin/u/v.
"""
from __future__ import annotations
import numpy as np


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


class DrawingPlane:
    def __init__(self, origin, u_axis, v_axis):
        self.origin = np.asarray(origin, dtype=float)
        self.u = _unit(u_axis)                       # advance direction
        self.v = _unit(np.asarray(v_axis) - np.dot(_unit(v_axis), self.u) * self.u)  # orthogonalize
        self.n = _unit(np.cross(self.u, self.v))     # plane normal

    def point(self, s, w):
        """World position for plane coords (s along advance, w along wave)."""
        return self.origin + s * self.u + w * self.v

    def orientation_with_pen(self, pen_axis):
        """Target rotation with flange z = pen_axis (kept in-plane x aligned to advance u)."""
        z_t = _unit(pen_axis)
        x_t = _unit(self.u - np.dot(self.u, z_t) * z_t)
        y_t = np.cross(z_t, x_t)
        return np.column_stack([x_t, y_t, z_t])

    def candidate_orientations(self):
        """Both perpendicular pen directions (+/- normal). The solver picks whichever
        the arm can actually reach — a small arm can only face the surface one way."""
        return [self.orientation_with_pen(self.n),
                self.orientation_with_pen(-self.n)]

    # ---- presets -------------------------------------------------------- #
    @classmethod
    def vertical_board(cls, x=0.14, z_center=0.21):
        """Whiteboard in front of the robot: advance along y, wave along z."""
        return cls(origin=(x, 0.0, z_center), u_axis=(0, 1, 0), v_axis=(0, 0, 1))

    @classmethod
    def flat_plate(cls, z=0.10, x_center=0.16):
        """Paper on a table: advance along x, wave along y (needs pen pointing down)."""
        return cls(origin=(x_center, 0.0, z), u_axis=(1, 0, 0), v_axis=(0, 1, 0))


def sine_waypoints(plane: DrawingPlane, amplitude=0.05, length=0.20,
                   cycles=2.0, n_points=120):
    """
    Generate a sine trajectory on `plane`.

    Returns:
        positions : (N,3) world targets
        coords    : (N,2) plane coords (s, w) for plotting the intended curve
    (Target orientation is chosen by the caller via plane.candidate_orientations().)
    """
    s = np.linspace(-length / 2.0, length / 2.0, n_points)
    w = amplitude * np.sin(2.0 * np.pi * cycles * (s + length / 2.0) / length)
    positions = np.array([plane.point(si, wi) for si, wi in zip(s, w)])
    coords = np.column_stack([s, w])
    return positions, coords
