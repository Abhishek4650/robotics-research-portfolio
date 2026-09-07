"""
ARM-450 precision: full error budget by Monte-Carlo propagation through the Jacobian.

METHOD
Each error source is a joint-space or task-space perturbation. Joint-space terms
propagate to the tool through the Jacobian, dp = J_v(q) . dq, so the SAME angular
error produces a different tool error at every pose. Averaging over the task
workspace is the only honest way to state a number.

WHAT IS MEASURED vs ASSUMED is tracked per term and reported, because a budget
whose biggest line is a guess is a guess.
"""
import numpy as np
from kinematics_arm450 import fk_dh, jacobian_velocity_propagation as JAC

DEG = np.pi/180.0

# ---- error sources -------------------------------------------------------
# (name, sigma, unit, kind, status)
#   kind 'joint'  : angular error at each joint  [rad]
#   kind 'task'   : direct tool-space error      [m]
SOURCES = [
    ("servo backlash",        0.5*DEG,  "deg", "joint", "ASSUMED  (0.3-1.0 deg class typical)"),
    ("servo resolution",      0.088*DEG,"deg", "joint", "datasheet 4096 counts/rev = 0.088 deg"),
    ("bearing clearance",     None,     "mm",  "task",  "COMPUTED 0.067 mm from ISO 5753 + press fit + pocket stiffness"),
    ("printed link tolerance",None,     "mm",  "task",  "MEASURED  0.02 mm XY shrink"),
    ("seam torsion (bolted)", None,     "mm",  "task",  "CALCULATED 0.011 mm"),
    ("elastic droop",         None,     "mm",  "task",  "CALCULATED 0.111 mm (PLA+CF)"),
    ("thermal (20 +/- 10 C)", None,     "mm",  "task",  "PLA+CF CTE ~40 um/m/K over 0.45 m"),
]
TASK_SIGMA = {                     # metres, 1-sigma
    # Was an assumed 0.65 mm. Now computed: a 6806 CN bearing has 5-20 um radial
    # clearance (ISO 5753); the Ø42.00 press fit removes ~15 um of it, leaving
    # 0-5 um. What actually remains is the PRINTED POCKET flexing 2 um under the
    # drawing-load couple -> 0.067 mm at the tool.
    "bearing clearance":      0.067e-3/3,
    "printed link tolerance": 0.02e-3,
    "seam torsion (bolted)":  0.011e-3/3,
    "elastic droop":          0.111e-3/3,
    "thermal (20 +/- 10 C)":  40e-6*0.45*10/3,
}

LIM = np.radians(np.array([[-165,165],[-115,115],[-150,150],
                           [-165,165],[-110,110],[-175,175]], float))


def task_poses(n=400, seed=0):
    """Poses inside the DRAWING band, not the whole workspace -- precision is
    only meaningful where the arm actually works."""
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        q = rng.uniform(LIM[:,0], LIM[:,1])
        p = fk_dh(q)[:3,3]
        r = np.linalg.norm(p[:2])
        if 0.22 < r < 0.32 and 0.05 < p[2] < 0.40:
            out.append(q)
    return np.array(out)


def budget(n_mc=3000, seed=1):
    rng = np.random.default_rng(seed)
    Q = task_poses()
    per_source = {}
    for name, sig, unit, kind, status in SOURCES:
        errs = []
        for _ in range(n_mc):
            q = Q[rng.integers(len(Q))]
            if kind == "joint":
                # Backlash is a BOUNDED DEAD ZONE, not Gaussian noise: the true
                # position sits anywhere inside +/- half the band with roughly
                # uniform probability. Modelling it as normal(0, band) made the
                # headline 3x pessimistic and disagreed with the section below.
                if "backlash" in name:
                    dq = rng.uniform(-sig/2, sig/2, 6)
                else:
                    dq = rng.normal(0, sig, 6)
                J = JAC(q)[:3, :]
                errs.append(np.linalg.norm(J @ dq))
            else:
                errs.append(abs(rng.normal(0, TASK_SIGMA[name])))
        per_source[name] = (np.array(errs), status)
    return per_source


if __name__ == "__main__":
    print("="*80)
    print("ARM-450 PRECISION BUDGET — Monte-Carlo through the Jacobian")
    print("="*80)
    b = budget()
    print(f"\n  Evaluated over the DRAWING BAND (r = 220..320 mm, z = 50..400 mm)\n")
    print(f"  {'source':26s} {'RMS mm':>8s} {'95% mm':>8s}   status")
    print("  " + "-"*76)
    # All internal maths is in METRES. Two bugs were here: printing metres
    # under a "mm" heading, and reporting std() of the error MAGNITUDE instead
    # of its RMS. Both flattered the result by ~500x.
    MM = 1000.0
    tot = np.zeros(3000)
    for name, (e, status) in b.items():
        tot += e**2
        rms = np.sqrt((e**2).mean()) * MM
        print(f"  {name:26s} {rms:8.3f} {np.percentile(e,95)*MM:8.3f}   {status}")
    tot = np.sqrt(tot)
    rms_t = np.sqrt((tot**2).mean()) * MM
    print("  " + "-"*76)
    print(f"  {'RSS TOTAL':26s} {rms_t:8.3f} {np.percentile(tot,95)*MM:8.3f}")
    print(f"\n  -> expected absolute accuracy ~ {np.percentile(tot,95)*MM:.2f} mm (95th pct)")
    print(f"     dominated by: " + max(b.items(), key=lambda kv: (kv[1][0]**2).mean())[0])


# ===========================================================================
def repeatability_vs_accuracy(n=3000, seed=2):
    """Backlash is a DEAD ZONE, not noise. It costs you on every direction
    REVERSAL, but not when a joint keeps turning the same way. So absolute
    accuracy and repeatability are very different numbers."""
    rng = np.random.default_rng(seed)
    Q = task_poses()
    BL = 0.5*DEG
    RES = 0.088*DEG          # was 0.29 -- I had multiplied by 3 for 'gear slop',
                             # which double-counts backlash. Unjustified.
    acc, rep, uni = [], [], []
    for _ in range(n):
        q = Q[rng.integers(len(Q))]
        J = JAC(q)[:3, :]
        # absolute accuracy: backlash sits anywhere in its band, plus resolution
        acc.append(np.linalg.norm(J @ (rng.uniform(-BL/2, BL/2, 6) + rng.normal(0, RES, 6))))
        # repeatability, direction reversed between visits: full band swing
        rep.append(np.linalg.norm(J @ (rng.choice([-BL, BL], 6) + rng.normal(0, RES, 6))))
        # repeatability, always approached from the SAME side: backlash cancels
        uni.append(np.linalg.norm(J @ rng.normal(0, RES, 6)))
    return (np.array(acc)*1000, np.array(rep)*1000, np.array(uni)*1000)


def improvement_paths(n=3000, seed=3):
    rng = np.random.default_rng(seed)
    Q = task_poses()
    RES = 0.088*DEG
    out = {}
    for label, bl, res in (
            ("as designed (0.5 deg backlash)",        0.5*DEG, RES),
            ("best-case servo (0.3 deg)",             0.3*DEG, RES),
            ("software backlash compensation (70 %)", 0.15*DEG, RES),
            ("JOINT-SIDE ENCODERS (backlash out)",    0.0,     0.02*DEG),
            ("encoders + unidirectional approach",    0.0,     0.01*DEG)):
        e = []
        for _ in range(n):
            q = Q[rng.integers(len(Q))]
            J = JAC(q)[:3, :]
            dq = (rng.uniform(-bl/2, bl/2, 6) if bl > 0 else 0) + rng.normal(0, res, 6)
            e.append(np.linalg.norm(J @ dq))
        out[label] = np.array(e)*1000
    return out


if __name__ == "__main__" and True:
    print("\n" + "="*80)
    print("ACCURACY vs REPEATABILITY — backlash is a dead zone, not noise")
    print("="*80)
    acc, rep, uni = repeatability_vs_accuracy()
    for lbl, e in (("absolute accuracy", acc),
                   ("repeatability, direction REVERSED", rep),
                   ("repeatability, SAME approach direction", uni)):
        print(f"  {lbl:40s} RMS {np.sqrt((e**2).mean()):6.3f} mm   "
              f"95% {np.percentile(e,95):6.3f} mm")
    print("\n  -> approaching every point from the same side removes backlash")
    print(f"     entirely: {np.sqrt((uni**2).mean()):.3f} mm instead of "
          f"{np.sqrt((acc**2).mean()):.3f} mm.")

    print("\n" + "="*80)
    print("WHAT ACTUALLY BUYS PRECISION")
    print("="*80)
    for lbl, e in improvement_paths().items():
        print(f"  {lbl:42s} RMS {np.sqrt((e**2).mean()):6.3f} mm   "
              f"95% {np.percentile(e,95):6.3f} mm")
