# Task-Constrained Workspace and Inverse Kinematics for Surface-Tracing with the myCobot 280

A research project (PhD-thesis chapter / conference-paper scope) studying **where**
and **how well** a small 6-DOF arm can trace curves on a surface while holding the
pen **normal to that surface**. It grows from two practical questions on the
[`Rsine`](../Rsine) project:

1. *Which pen directions are feasible where?* (the "front vs behind" question)
2. *Can it trace a sine anywhere in the workspace — including a horizontal table?*

The core message: the **reachable workspace** and the **drawable (orientation-
constrained) workspace** are very different sets, and the difference depends on the
surface orientation.

---

## 1. Package layout

```
mycobot_thesis/
├── mycobot_thesis/
│   ├── robot.py       # myCobot 280 model: modified-DH FK, Jacobian,
│   │                  #   manipulability, condition number (self-contained copy
│   │                  #   of the verified Rsine kinematics — Rsine is untouched)
│   └── ik.py          # IK solver suite: DLS, pseudoinverse, Jacobian-transpose,
│                      #   Levenberg-Marquardt, + a feasibility test
├── analysis/
│   ├── exp1_feasibility.py     # task-constrained workspace (vertical + horizontal)
│   ├── exp2_ik_comparison.py   # IK solver benchmark
│   ├── exp3_manipulability.py  # manipulability + conditioning maps
│   └── exp4_demos.py           # sine on a front vertical board AND a horizontal table
├── figures/           # generated figures
├── docs/              # research paper (PDF) + slides (PPT)   [being built]
└── paper/             # paper sources
```

`robot.py` reuses the R_sine kinematics **verified to ~1e-15 m vs the URDF**; it adds
the differential metrics the thesis needs:
- Yoshikawa manipulability `w = √det(Jᵥ Jᵥᵀ)`,
- condition number `κ = σ_max / σ_min`,
- minimum singular value (singularity proximity).

---

## 2. Experiments and key findings

Run any experiment with `python3 analysis/<name>.py` (writes to `figures/`).

### Exp 1 — Task-constrained workspace  (`fig_feasibility.png`)
Over a grid of end-effector positions, test whether DLS IK converges with the pen
held normal to the surface, for each candidate normal, from several seeds.

- **Vertical surface:** only **57.9%** of the reachable workspace allows a
  perpendicular pen; only **35.2%** supports the *front* (away-from-robot) direction.
  Near the base only the *back* direction is feasible; the *front* direction opens up
  only in the outer region (x ≳ 0.17 m). → answers Q1.
- **Horizontal table:** **63.5%** of the reachable cells allow pen-down drawing,
  forming a **ring** (a "dead zone" directly above the base, feasible at mid-radius).
  → answers Q2; horizontal drawing works, but only in that ring.

### Exp 2 — IK solver comparison  (`fig_ik_comparison.png`)
Full-pose tracing task, common seed:

| Solver | Success | Mean iters | Mean time | Note |
|--------|--------:|-----------:|----------:|------|
| Damped least squares | **98.7%** | ~76 | ~15 ms | most robust; damping caps accuracy |
| Pseudoinverse | 26% | ~8 | ~30 ms | quadratic convergence but brittle near singularities |
| Jacobian-transpose | 29% | 2000 | ~365 ms | always progresses, but linear/slow |
| Levenberg–Marquardt | 61% | ~10 | ~16 ms | fast + accurate, less robust than DLS here |

Classic accuracy-vs-robustness trade-off, quantified for this arm.

### Exp 3 — Manipulability & conditioning  (`fig_manipulability.png`)
A **low-manipulability dead zone directly above the base**, a bright **dexterous
ring** at mid-radius (best drawing region), and rising condition number toward the
workspace boundary (near-singular at full extension). Peak translational
manipulability ≈ 0.0057.

### Exp 4 — Demonstrations  (`fig_demos.png`)
Using the analysis to place the surfaces, both trace to **0.000 mm**:
- a **front-facing vertical board** (pen points away from the robot) at (0.21, 0, 0.12);
- a **horizontal table** (pen points down) at (0, 0.18, 0.10).

---

## 3. Run everything

```bash
cd ~/ros2_ws/src/mycobot_thesis
for e in exp1_feasibility exp2_ik_comparison exp3_manipulability exp4_demos; do
  python3 analysis/$e.py
done
```

Dependencies: `numpy`, `scipy`, `matplotlib` (all installed).

---

## 4. Status

Analysis + figures complete. The research-paper PDF and slide deck are generated
into `docs/` (in progress). This project is **separate** from `Rsine`, which stays
frozen for its presentation.
