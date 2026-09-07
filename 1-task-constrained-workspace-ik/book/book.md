---
title: "Kinematics of the myCobot 280 — A Worked Book"
subtitle: "Forward & inverse kinematics, the velocity-propagation Jacobian, manipulability, and the code — in reference notation"
author: "For: Abhishek Roy"
date: "2026"
---

# Preface — how to read this book

This book explains the mathematics **and** the code of the myCobot 280 projects, in
the **notation of your reference notes** (modified Denavit–Hartenberg, the
velocity-propagation method). Read a chapter, then open the file it names and match
each equation to the lines of code.

Two kinds of callouts appear throughout:

> **Key point.** The one idea you must not miss.

*Meaning.* A plain-language reading of the symbols right after each equation.

Notation used throughout (from your reference notes):

![](book/eq/notation_key.png){width=4.2in}

A **leading super/subscript** denotes the frame: the first symbol above is the
transform **from frame $i\!-\!1$ to frame $i$**; the second is an angular velocity
**of link $i$, written in frame $i$**. A **dot** is a time derivative (joint speed),
a **hat** is a unit vector (the $z$-axis).

---

# Chapter 1 — The robot and its frames

The myCobot 280 is a 6-joint (6-DOF) arm; every joint is **revolute** and turns
about its own $z$-axis. To do kinematics we attach a coordinate **frame** to each
link and describe how consecutive frames relate. We use the **modified (Craig)
Denavit–Hartenberg** convention, which needs only two axes per frame ($x$ and $z$)
and four numbers per joint:

![](book/eq/dh_params.png){width=3.2in}

| symbol | name | meaning |
|--------|------|---------|
| $\alpha_{i-1}$ | link twist  | angle from $Z_{i-1}$ to $Z_i$ about $X_{i-1}$ |
| $a_{i-1}$      | link length | distance between the two $z$-axes along $X_{i-1}$ |
| $d_i$          | link offset | distance from $X_{i-1}$ to $X_i$ along $Z_i$ |
| $\theta_i$     | joint angle | angle from $X_{i-1}$ to $X_i$ about $Z_i$ (the **variable** for a revolute joint) |

> **Key point.** For a revolute joint only $\theta_i$ changes as the motor turns;
> $\alpha_{i-1}, a_{i-1}, d_i$ are fixed by the robot's geometry.

*Meaning.* Twist and length describe how one joint axis sits relative to the next;
offset and angle describe how far and how much you travel along/around the joint
axis to reach the next link.

---

# Chapter 2 — Forward kinematics

**Question.** Given the six joint angles, where is the tool?

Each pair of frames is related by one **modified-DH link transform** — exactly the
matrix in your notes:

![](book/eq/dh_transform.png){width=6.2in}

with $c\theta=\cos\theta$, $s\theta=\sin\theta$. Chaining the six link transforms
from the base to the tool gives the full forward kinematics:

![](book/eq/fk_product.png){width=4.6in}

> **Key point.** Forward kinematics is just a **product of these link matrices**.
> One set of joint angles gives exactly one tool pose.

*Meaning.* The top-left $3\times3$ block of the tool transform $T$ is the tool's
orientation; the top-right $3\times1$ block is its position.

**In code** (`mycobot_thesis/robot.py`): `_dh_link(alpha, a, d, theta)` builds one
matrix above; `fk_frames(q)` multiplies them and returns every frame; and
`forward_kinematics(q)` returns the last one, $^{0}_{N}T$. The DH table (identified
from the URDF, verified to $10^{-15}\,\text{m}$) is the array `DH_TABLE`.

---

# Chapter 3 — The Jacobian (velocity-propagation method)

**Question.** How does the tool move when the joints move? The **Jacobian** $J(q)$
answers this, mapping joint speeds $\dot q$ to the tool's linear and angular
velocity (its *twist*).

Your notes build $J$ by **propagating velocity outward**, frame by frame. Angular
velocity carries out and gains the new joint's spin:

![](book/eq/velprop_omega.png){width=5.4in}

Linear velocity carries out, picking up the lever-arm cross-product, and gains a
prismatic slide (zero for our revolute joints):

![](book/eq/velprop_v.png){width=6.2in}

*Meaning.* The rotation $R$ carries a vector from frame $i$ into frame $i+1$; the
cross-product $\omega\times P$ is the velocity a rotating link imparts at its far
end; $\dot\theta_{i+1}$ and $\dot d_{i+1}$ add the new joint's own motion.

For an all-revolute arm this closes into the compact **geometric Jacobian**, whose
$i$-th column is

![](book/eq/jacobian_col.png){width=5.4in}

> **Key point.** Column $i$ says: *if only joint $i$ spins, the tool's linear
> velocity is $\hat Z_i\times(P_e-P_i)$ and its angular velocity is $\hat Z_i$.*

**In code** (`robot.py`): `jacobian(q)` builds exactly these columns from the frame
axes $z_i$ and origins $p_i$ returned by `fk_frames`. It is verified against a
finite-difference Jacobian to $\approx 2.5\times10^{-7}$.

---

# Chapter 4 — Inverse kinematics

**Question.** We want the tool at a desired pose — which joint angles get there?
There is no simple formula, so we iterate with **damped least squares**:

![](book/eq/dls_update.png){width=4.6in}

where $e$ is the 6-vector pose error (position + orientation) and $\lambda$ is a
small damping constant.

> **Key point.** The damping $\lambda^2 I$ is what keeps the step finite near a
> **singularity** (where $J$ loses rank and a plain inverse blows up). It trades a
> tiny steady-state error for stability.

*Meaning.* Each step multiplies the pose error by a damped inverse of the Jacobian
to get a joint correction, then repeats until the tool reaches the target.

**In code** (`mycobot_thesis/ik.py`): `damped_least_squares(...)` is the equation
above; `pseudoinverse`, `jacobian_transpose`, and `levenberg_marquardt` are the
alternatives compared in the thesis. Damped least squares was the most robust
(98.7% success); see the figure below.

![](figures/fig_ik_comparison.png){width=6.4in}

---

# Chapter 5 — Manipulability and the task-constrained workspace

Not every reachable point is equally good, and — for drawing — not every reachable
point can be reached with the pen **perpendicular to the surface**.

**Dexterity.** Yoshikawa manipulability and the condition number measure how far a
configuration is from a singularity:

![](book/eq/manip.png){width=5.6in}

*Meaning.* $w\to 0$ (and $\kappa\to\infty$) at a singularity: the arm loses a
direction of motion and IK becomes twitchy. Large $w$, small $\kappa$ = dexterous.

![](figures/fig_manipulability.png){width=6.4in}

**Drawable workspace.** Given a surface with normal $n$, the positions the pen can
reach *while staying perpendicular* form the task-constrained set

![](book/eq/drawable_set.png){width=6.0in}

> **Key point.** The **reachable** set $\mathcal R$ and the **drawable** set
> $\mathcal D(n)$ are different: $\mathcal D(n)\subset\mathcal R$, and the gap
> depends on the surface orientation.

For the myCobot: only $57.9\%$ of $\mathcal R$ allows a perpendicular pen on a
vertical surface ($35.2\%$ front-facing), and $63.5\%$ allows pen-down drawing on a
horizontal table — an **annulus** around a dead zone above the base.

![](figures/fig_feasibility.png){width=6.4in}

---

# Chapter 6 — Reading the code

Every module mirrors a chapter. The **library** is pure math (no ROS); the **nodes**
wrap it for live RViz.

**`mycobot_thesis/robot.py`** — the model (Chapters 2–3, 5).

- `DH_TABLE` — the identified modified-DH parameters $(\alpha_{i-1},a_{i-1},d_i,\theta^{\text{off}}_i)$.
- `_dh_link`, `fk_frames`, `forward_kinematics` — the link matrix and the product.
- `jacobian` — the velocity-propagation columns.
- `manipulability`, `condition_number` — the dexterity metrics.
- `target_orientation(normal, advance)` — builds the pen-perpendicular target frame.

**`mycobot_thesis/ik.py`** — the solvers (Chapter 4). `damped_least_squares` and the
three alternatives, each returning convergence info.

**`mycobot_thesis/draw_node.py`** — the ROS driver. It picks a surface preset,
generates the sine targets, solves IK for each (warm-started), and streams
`/joint_states`. The `'paper'` preset uses a **raised** IK branch so the arm is
opened toward $z$ while the pen traces the identical sine.

> **Key point.** A ROS node is a *thin wrapper*: it calls the same `robot`/`ik`
> library the offline analysis uses, then `publish()`es. Same math, live robot.

**Redundancy in action.** A 6-DOF arm can reach the same tool pose with different
postures (IK branches). Seeding the IK toward a raised branch keeps the drawn sine
identical while lifting the elbow:

![](figures/fig_posture_compare.png){width=6.4in}

**The live demo** (`ros2 launch mycobot_thesis thesis_draw.launch.py surface:=paper`)
writes the sine left-to-right on a table in front of the robot, raised posture:

![](figures/paper_raised_draw.gif){width=4.2in}

---

# Appendix — Identified DH table and commands

| $i$ | $\alpha_{i-1}$ (°) | $a_{i-1}$ (m) | $d_i$ (m) | $\theta_i^{\text{off}}$ (°) |
|----|------:|--------:|--------:|------:|
| 1 | 0   | 0       | 0.13056 | +90 |
| 2 | +90 | 0       | 0       | −90 |
| 3 | 0   | −0.1104 | 0       | 0   |
| 4 | 0   | −0.096  | 0.06062 | −90 |
| 5 | +90 | 0       | 0.07318 | +90 |
| 6 | −90 | 0       | 0.0456  | 0   |

```bash
# analysis (pure Python)
cd ~/ros2_ws/src/mycobot_thesis
python3 analysis/exp1_feasibility.py        # task-constrained workspace
python3 analysis/exp2_ik_comparison.py      # IK solver comparison
# live drawing in RViz
cd ~/ros2_ws && colcon build --packages-select mycobot_thesis && source install/setup.bash
bash src/mycobot_thesis/scripts/run_thesis_demo.sh surface:=paper
```
