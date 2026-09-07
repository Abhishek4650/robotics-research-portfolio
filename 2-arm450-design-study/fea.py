"""
ARM-450 — 3D linear-elastic FEA on the real printed geometry.

STRATEGY: part-wise, with loads taken from the assembly-level statics.

Why not a full-assembly solve? It would need contact stiffness at every mating
face, bolt preload, and bearing radial/moment stiffness. None of those are known
here, so an assembly result would be dominated by guesses while looking
authoritative. Part-wise submodelling uses loads that ARE known (from
joint_loads.py) and returns each part's margin in isolation, which is the number
you can act on.

METHOD
  * voxelise the exported STL into a regular hex grid
  * 8-node trilinear brick elements, 2x2x2 Gauss integration
      Ke = integral B^T D B dV
  * assemble sparse K, apply Dirichlet BCs, solve K u = f
  * per-element von Mises from the stress at the element centre
  * render the surface coloured by von Mises

LIMITATIONS, stated up front:
  * voxelisation stair-steps curved surfaces, so stress AT a fillet is not
    resolved -- peak values near curved boundaries read low. Use the field to
    see WHERE load flows, and the closed-form numbers in REPORT.md for margins.
  * linear elastic, isotropic. Real FDM parts are anisotropic (see report 5).
  * no contact, no preload, no bolt modelling.
"""

import os
import numpy as np
import trimesh
import sys as _s, os as _o
_s.path.insert(0, _o.path.join(_o.path.dirname(_o.path.abspath(__file__)), 'cad'))
from params import E_MOD, RHO_SOLID   # noqa: E402
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.join(HERE, "output", "cad")

# PLA+CF, the specified link material
E0, NU = E_MOD, 0.35           # MPa, from params.MATERIAL
RHO = RHO_SOLID                # g/mm^3, from params.MATERIAL


# ---------------------------------------------------------------------------
def hex8_Ke(a, b, c, E, nu):
    """8-node trilinear brick stiffness, element size a x b x c."""
    D = E / ((1 + nu) * (1 - 2 * nu)) * np.array([
        [1 - nu, nu, nu, 0, 0, 0],
        [nu, 1 - nu, nu, 0, 0, 0],
        [nu, nu, 1 - nu, 0, 0, 0],
        [0, 0, 0, (1 - 2 * nu) / 2, 0, 0],
        [0, 0, 0, 0, (1 - 2 * nu) / 2, 0],
        [0, 0, 0, 0, 0, (1 - 2 * nu) / 2]])
    g = 1.0 / np.sqrt(3.0)
    gp = [-g, g]
    # node local coords
    nc = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                   [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], float)
    Ke = np.zeros((24, 24))
    Bs = []
    for xi in gp:
        for et in gp:
            for ze in gp:
                dN = np.zeros((3, 8))
                for i in range(8):
                    x0, y0, z0 = nc[i]
                    dN[0, i] = x0 * (1 + y0 * et) * (1 + z0 * ze) / 8
                    dN[1, i] = y0 * (1 + x0 * xi) * (1 + z0 * ze) / 8
                    dN[2, i] = z0 * (1 + x0 * xi) * (1 + y0 * et) / 8
                J = np.diag([a / 2, b / 2, c / 2])
                dNx = np.linalg.solve(J, dN)
                B = np.zeros((6, 24))
                for i in range(8):
                    B[0, 3 * i] = dNx[0, i]
                    B[1, 3 * i + 1] = dNx[1, i]
                    B[2, 3 * i + 2] = dNx[2, i]
                    B[3, 3 * i] = dNx[1, i]; B[3, 3 * i + 1] = dNx[0, i]
                    B[4, 3 * i + 1] = dNx[2, i]; B[4, 3 * i + 2] = dNx[1, i]
                    B[5, 3 * i] = dNx[2, i]; B[5, 3 * i + 2] = dNx[0, i]
                Ke += B.T @ D @ B * np.linalg.det(J)
                Bs.append(B)
    return Ke, D, Bs


def voxelise(stl_path, pitch):
    m = trimesh.load(stl_path, force="mesh")
    m.apply_translation(-m.bounds[0])
    v = m.voxelized(pitch=pitch).fill()
    idx = np.array(v.sparse_indices)
    grid = np.zeros(idx.max(axis=0) + 1, dtype=bool)
    grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    return m, grid, pitch


class Model:
    def __init__(self, grid, pitch, E=E0, nu=NU):
        self.g, self.h = grid, pitch
        nx, ny, nz = grid.shape
        self.dims = (nx, ny, nz)
        # node numbering on the (nx+1, ny+1, nz+1) lattice
        self.nn = (nx + 1, ny + 1, nz + 1)
        self.nid = np.arange(np.prod(self.nn)).reshape(self.nn)
        self.els = np.argwhere(grid)
        self.Ke, self.D, self.Bs = hex8_Ke(pitch, pitch, pitch, E, nu)
        off = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
                        [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]])
        self.edof = np.zeros((len(self.els), 24), dtype=np.int64)
        for k, e in enumerate(self.els):
            ns = self.nid[tuple((e + off).T)]
            self.edof[k] = np.repeat(ns * 3, 3) + np.tile([0, 1, 2], 8)
        self.ndof = int(np.prod(self.nn) * 3)

    def solve(self, fixed_nodes, loads):
        iK = np.repeat(self.edof, 24, axis=1).ravel()
        jK = np.tile(self.edof, (1, 24)).ravel()
        sK = np.tile(self.Ke.ravel(), len(self.els))
        K = coo_matrix((sK, (iK, jK)), shape=(self.ndof, self.ndof)).tocsc()
        f = np.zeros(self.ndof)
        for nds, vec in loads:
            if len(nds) == 0:
                continue
            for d in range(3):
                f[nds * 3 + d] += vec[d] / len(nds)
        fixed = np.unique(np.concatenate(
            [fixed_nodes * 3 + d for d in range(3)]))
        # Only nodes that belong to at least one element carry stiffness.
        # Every other lattice node leaves a zero row in K and makes the solve
        # singular -- this is the classic voxel-FEA trap.
        active = np.unique(self.edof)
        free = np.setdiff1d(active, fixed)
        u = np.zeros(self.ndof)
        u[free] = spsolve(K[free, :][:, free], f[free])
        return u

    def von_mises(self, u):
        ue = u[self.edof]
        vm = np.zeros(len(self.els))
        for B in self.Bs:                       # average over Gauss points
            eps = ue @ B.T
            sig = eps @ self.D.T
            sx, sy, sz, txy, tyz, tzx = sig.T
            vm += np.sqrt(0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2)
                          + 3 * (txy ** 2 + tyz ** 2 + tzx ** 2))
        return vm / len(self.Bs)

    def nodes_in(self, pred):
        """Node ids whose (x,y,z) in mm satisfy pred."""
        gx, gy, gz = np.meshgrid(*[np.arange(n) * self.h for n in self.nn],
                                 indexing="ij")
        mask = pred(gx, gy, gz)
        return self.nid[mask].ravel()

    def surface_quads(self, vm):
        """Exposed voxel faces + their element von Mises, for rendering."""
        nx, ny, nz = self.dims
        quads, vals = [], []
        elmap = {tuple(e): k for k, e in enumerate(self.els)}
        dirs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        for k, e in enumerate(self.els):
            for d in dirs:
                nb = tuple(e + np.array(d))
                if (0 <= nb[0] < nx and 0 <= nb[1] < ny and 0 <= nb[2] < nz
                        and self.g[nb]):
                    continue
                c = (e + 0.5) * self.h
                ax = int(np.argmax(np.abs(d)))
                s = d[ax] * self.h / 2
                u_, v_ = [i for i in range(3) if i != ax]
                q = []
                for du, dv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    p = np.zeros(3)
                    p[ax] = c[ax] + s
                    p[u_] = c[u_] + du * self.h / 2
                    p[v_] = c[v_] + dv * self.h / 2
                    q.append(p)
                quads.append(q)
                vals.append(vm[k])
        return np.array(quads), np.array(vals)
