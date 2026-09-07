"""
ARM-450 — topology optimisation of the link side wall (SIMP).

Method: Solid Isotropic Material with Penalisation.
  design variable  x_e in [0,1]  = relative density of element e
  material law     E_e = Emin + x_e^p (E0 - Emin),  p = 3
  objective        minimise compliance  c = U^T K U   (= maximise stiffness)
  constraint       sum(x_e * v_e) / V <= volfrac
  update           optimality criteria, x_new = x * (-dc/dv / lambda)^eta

The penalisation exponent p > 1 makes intermediate densities uneconomic, pushing
the design to black/white. A SENSITIVITY FILTER of radius rmin is mandatory: without
it the solution checkerboards, which is a numerical artefact of the Q4 element, not
a real structure. rmin must be >= ~1.5 elements or the mesh dependence returns.

Load case: the link is a cantilever, built in at the shoulder joint, carrying the
outboard arm plus payload at the elbow end. Domain is the 145 x 47.5 mm side wall.
"""

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def lk(E=1.0, nu=0.3):
    """Q4 plane-stress element stiffness, unit size."""
    k = np.array([1/2-nu/6, 1/8+nu/8, -1/4-nu/12, -1/8+3*nu/8,
                  -1/4+nu/12, -1/8-nu/8, nu/6, 1/8-3*nu/8])
    return E/(1-nu**2)*np.array([
        [k[0],k[1],k[2],k[3],k[4],k[5],k[6],k[7]],
        [k[1],k[0],k[7],k[6],k[5],k[4],k[3],k[2]],
        [k[2],k[7],k[0],k[5],k[6],k[3],k[4],k[1]],
        [k[3],k[6],k[5],k[0],k[7],k[2],k[1],k[4]],
        [k[4],k[5],k[6],k[7],k[0],k[1],k[2],k[3]],
        [k[5],k[4],k[3],k[2],k[1],k[0],k[7],k[6]],
        [k[6],k[3],k[4],k[1],k[2],k[7],k[0],k[5]],
        [k[7],k[2],k[1],k[4],k[3],k[6],k[5],k[0]]])


def build_filter(nelx, nely, rmin):
    """Sensitivity filter weights (Sigmund). Returns sparse H and Hs."""
    nfilter = int(nelx*nely*((2*(np.ceil(rmin)-1)+1)**2))
    iH, jH, sH = np.zeros(nfilter), np.zeros(nfilter), np.zeros(nfilter)
    cc = 0
    for i in range(nelx):
        for j in range(nely):
            row = i*nely+j
            kk1, kk2 = int(max(i-(np.ceil(rmin)-1),0)), int(min(i+np.ceil(rmin),nelx))
            ll1, ll2 = int(max(j-(np.ceil(rmin)-1),0)), int(min(j+np.ceil(rmin),nely))
            for k in range(kk1,kk2):
                for l in range(ll1,ll2):
                    col = k*nely+l
                    fac = rmin-np.sqrt((i-k)**2+(j-l)**2)
                    iH[cc], jH[cc], sH[cc] = row, col, max(0.0,fac)
                    cc += 1
    H = coo_matrix((sH[:cc],(iH[:cc],jH[:cc])),shape=(nelx*nely,nelx*nely)).tocsc()
    return H, H.sum(1)


def topopt(nelx, nely, volfrac, penal, rmin, load_nodes, fixed_dofs,
           maxiter=60, verbose=True):
    E0, Emin, nu = 1.0, 1e-9, 0.3
    ndof = 2*(nelx+1)*(nely+1)
    x = volfrac*np.ones(nelx*nely)
    KE = lk(1.0, nu)

    # element -> dof map
    edofMat = np.zeros((nelx*nely,8),dtype=int)
    for elx in range(nelx):
        for ely in range(nely):
            el = ely+elx*nely
            n1 = (nely+1)*elx+ely
            n2 = (nely+1)*(elx+1)+ely
            edofMat[el,:] = [2*n1+2,2*n1+3,2*n2+2,2*n2+3,2*n2,2*n2+1,2*n1,2*n1+1]
    iK = np.kron(edofMat,np.ones((8,1))).flatten()
    jK = np.kron(edofMat,np.ones((1,8))).flatten()

    H, Hs = build_filter(nelx,nely,rmin)

    f = np.zeros(ndof)
    for nd, val in load_nodes:
        f[nd] = val
    free = np.setdiff1d(np.arange(ndof), fixed_dofs)

    hist = []
    for it in range(maxiter):
        sK = ((KE.flatten()[np.newaxis]).T *
              (Emin+x**penal*(E0-Emin))).flatten(order='F')
        K = coo_matrix((sK,(iK,jK)),shape=(ndof,ndof)).tocsc()
        u = np.zeros(ndof)
        u[free] = spsolve(K[free,:][:,free], f[free])

        ce = (np.dot(u[edofMat].reshape(nelx*nely,8),KE) *
              u[edofMat].reshape(nelx*nely,8)).sum(1)
        c = ((Emin+x**penal*(E0-Emin))*ce).sum()
        dc = (-penal*x**(penal-1)*(E0-Emin))*ce
        dv = np.ones(nelx*nely)

        # sensitivity filtering
        dc = np.asarray((H*(x*dc))[np.newaxis].T/np.asarray(Hs))[:,0]/np.maximum(1e-3,x)

        # optimality criteria update
        l1,l2,move = 0,1e9,0.2
        while (l2-l1)/(l1+l2+1e-12) > 1e-3:
            lmid = 0.5*(l2+l1)
            xnew = np.maximum(0.0,np.maximum(x-move,
                   np.minimum(1.0,np.minimum(x+move,x*np.sqrt(-dc/dv/lmid)))))
            if xnew.sum() > volfrac*nelx*nely: l1 = lmid
            else: l2 = lmid
        change = np.abs(xnew-x).max()
        x = xnew
        hist.append(c)
        if verbose and (it % 10 == 0 or it == maxiter-1):
            print(f"    it {it:3d}  compliance {c:10.4f}  vol {x.mean():.3f}  "
                  f"change {change:.4f}")
        if change < 0.006:
            break
    return x.reshape(nelx,nely).T, hist


# ===========================================================================
if __name__ == "__main__":
    print("=" * 78)
    print("TOPOLOGY OPTIMISATION — ARM-450 link side wall")
    print("=" * 78)

    # domain: 145 mm long x 47.5 mm deep -> keep the real aspect ratio
    nelx, nely = 132, 44
    print(f"domain {nelx} x {nely} elements  (145 x 47.5 mm, aspect "
          f"{145/47.5:.2f})")

    # BC: built in along the whole left edge (the shoulder joint boss)
    fixed = np.arange(0, 2*(nely+1))
    # load: downward at the mid-height of the right edge (the elbow joint)
    tip_node = 2*((nely+1)*nelx + nely//2) + 1
    loads = [(tip_node, -1.0)]

    results = {}
    for vf in (0.25, 0.40):
        print(f"\n  volume fraction {vf:.2f}, p=3, rmin=2.0")
        xopt, hist = topopt(nelx, nely, vf, 3.0, 2.0, loads, fixed)
        results[vf] = (xopt, hist)

    # --- figure -----------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(13, 9.5),
                             gridspec_kw=dict(height_ratios=[1, 1, 0.85], hspace=0.42))
    fig.patch.set_facecolor("white")
    for ax, vf in zip(axes[:2], (0.25, 0.40)):
        xopt, _ = results[vf]
        ax.imshow(-xopt, cmap="gray", vmin=-1, vmax=0, aspect="equal",
                  interpolation="bilinear")
        ax.set_title(f"SIMP result, volume fraction {vf:.0%} — "
                     f"cantilever built in at left (shoulder), tip load at right (elbow)",
                     fontsize=11.5, fontweight="bold", color="#1b2733", loc="left")
        ax.set_xticks([]); ax.set_yticks([])

    ax = axes[2]
    for vf, col in ((0.25, "#2e7d9a"), (0.40, "#1e7d3c")):
        _, hist = results[vf]
        ax.plot(hist, color=col, lw=2.2, label=f"volfrac {vf:.0%}")
    ax.set_xlabel("iteration"); ax.set_ylabel("compliance  (lower = stiffer)")
    ax.set_yscale("log")
    ax.set_title("Convergence", fontsize=11.5, fontweight="bold",
                 color="#1b2733", loc="left")
    ax.legend(frameon=False); ax.grid(alpha=.25, ls=":")
    for s in ("top","right"): ax.spines[s].set_visible(False)

    fig.suptitle("ARM-450 — topology optimisation of the link side wall (SIMP, p=3)",
                 fontsize=14.5, fontweight="bold", color="#1b2733", x=0.02, ha="left")
    fig.tight_layout(rect=(0,0,1,0.95))
    fig.savefig("figures/opt_topology.png", dpi=200, facecolor="white")
    print("\nwrote figures/opt_topology.png")

    # --- what it means ----------------------------------------------------
    xopt, _ = results[0.40]
    top = xopt[:nely//4, :].mean()
    mid = xopt[nely//4:3*nely//4, :].mean()
    bot = xopt[3*nely//4:, :].mean()
    print("\nmaterial distribution through the depth (volfrac 40%):")
    print(f"   outer quarter, top     {top:.3f}")
    print(f"   middle half            {mid:.3f}")
    print(f"   outer quarter, bottom  {bot:.3f}")
    print(f"   -> outer/middle ratio  {(top+bot)/2/max(mid,1e-6):.2f}x")
    print("\nSIMP independently rediscovers the flange-and-web layout: material")
    print("migrates to the top and bottom surfaces, which is exactly what a")
    print("thin-walled BOX SECTION already is. The existing clamshell is the")
    print("right topology — it does not need optimising, it needs bolting shut.")
