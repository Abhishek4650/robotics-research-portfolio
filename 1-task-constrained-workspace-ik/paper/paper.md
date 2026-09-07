---
title: "Task-Constrained Workspace and Inverse-Kinematics Analysis for Surface-Tracing with a 6-DOF Manipulator (myCobot 280)"
author: "Abhishek Roy"
date: "2026"
---

# Abstract

Drawing or writing on a surface requires a manipulator not only to *reach* a
sequence of Cartesian points but to reach them while holding the tool **normal to
the surface**. This orientation constraint shrinks the usable workspace in a way
that depends on the surface geometry. We formalise the *task-constrained* (drawable)
workspace for surface-tracing and analyse it for the myCobot 280, a small 6-DOF
arm, using a modified Denavit–Hartenberg model whose forward kinematics matches the
manufacturer URDF to $10^{-15}\,\text{m}$. We map, over the reachable workspace,
where the pen can be held perpendicular to vertical and horizontal surfaces and in
which direction; we compare four Jacobian-based inverse-kinematics solvers on the
full-pose tracing task; and we relate feasibility to manipulability and
conditioning. We find that only $57.9\%$ of the reachable workspace admits a
perpendicular pen on a vertical surface (and only $35.2\%$ in the outward
direction), while $63.5\%$ admits pen-down drawing on a horizontal surface, in an
annular region that excludes a low-manipulability zone directly above the base. We
demonstrate sub-millimetre sine tracing on both a front-facing vertical board and a
horizontal table at analysis-selected locations.

# I. Introduction

A recurring robotics task is to trace a planar curve — a signature, a calibration
pattern, a sine wave — on a physical surface with a pen or tool. Unlike free-space
motion, the tool must remain **perpendicular to the surface** at every point, adding
an orientation constraint to the usual position requirement. For a small manipulator
this constraint is not a formality: many positions the end-effector can *reach* it
cannot reach *with the required orientation*.

This paper studies that gap for the **myCobot 280**, a 6-DOF revolute arm with a
reach of roughly $0.30\,\text{m}$. Our contributions are:

1. a verified modified-DH kinematic model and a velocity-propagation Jacobian
   (Section II);
2. a mapping of the **task-constrained workspace** — where and in which direction the
   pen can be held normal to vertical and horizontal surfaces (Section III);
3. a quantitative comparison of four inverse-kinematics solvers on the constrained
   tracing task (Section IV);
4. manipulability and conditioning maps that explain *why* the drawable region has
   the shape it does (Section V); and
5. sub-millimetre demonstrations on a front-facing vertical board and a horizontal
   table (Section VI).

# II. Kinematic Model

## A. Forward kinematics

We use the modified (Craig) Denavit–Hartenberg convention. The transform from frame
$i-1$ to frame $i$ is

$$
A_i = \mathrm{Rot}_x(\alpha_{i-1})\,\mathrm{Trans}_x(a_{i-1})\,
      \mathrm{Rot}_z(\theta_i)\,\mathrm{Trans}_z(d_i),
\tag{1}
$$

which in homogeneous form is

$$
A_i =
\begin{bmatrix}
c\theta_i & -s\theta_i & 0 & a_{i-1}\\
s\theta_i c\alpha_{i-1} & c\theta_i c\alpha_{i-1} & -s\alpha_{i-1} & -s\alpha_{i-1} d_i\\
s\theta_i s\alpha_{i-1} & c\theta_i s\alpha_{i-1} & c\alpha_{i-1} & c\alpha_{i-1} d_i\\
0 & 0 & 0 & 1
\end{bmatrix},
\tag{2}
$$

with $c\cdot=\cos$, $s\cdot=\sin$. The base-to-tool transform is the product
$T(q)=A_1 A_2 A_3 A_4 A_5 A_6$.

Because the URDF link frames are not placed on DH axes, the DH parameters were
*identified* by fitting $(\alpha_{i-1},a_{i-1},d_i,\theta_i^{\text{off}})$ so that
$T(q)$ reproduces the URDF forward kinematics; the residual is
$\max\lVert T_{\text{DH}}-T_{\text{URDF}}\rVert \approx 10^{-15}\,\text{m}$ over
$5000$ random configurations. Table I lists the result.

| $i$ | $\alpha_{i-1}$ (°) | $a_{i-1}$ (m) | $d_i$ (m) | $\theta_i^{\text{off}}$ (°) |
|----|------:|--------:|--------:|------:|
| 1 | 0   | 0       | 0.13056 | +90 |
| 2 | +90 | 0       | 0       | −90 |
| 3 | 0   | −0.1104 | 0       | 0   |
| 4 | 0   | −0.096  | 0.06062 | −90 |
| 5 | +90 | 0       | 0.07318 | +90 |
| 6 | −90 | 0       | 0.0456  | 0   |

: Table I. Identified modified-DH parameters of the myCobot 280.

## B. Jacobian by velocity propagation

Propagating link velocities outward yields, for the all-revolute arm, the geometric
Jacobian whose $i$-th column is

$$
J_i(q) =
\begin{bmatrix} z_i \times (p_e - p_i) \\ z_i \end{bmatrix},
\tag{3}
$$

where $z_i$ and $p_i$ are the axis and origin of joint $i$ in the base frame and
$p_e$ is the tool position. It maps joint rates to the tool twist,
$\begin{bmatrix} v \\ \omega \end{bmatrix} = J(q)\,\dot q$. Equation (3) matches a
finite-difference Jacobian to $\approx 2.5\times10^{-7}$.

## C. Dexterity metrics

We quantify local dexterity by the Yoshikawa manipulability and the condition number
of the translational Jacobian $J_v$ (top three rows of $J$):

$$
w(q) = \sqrt{\det\!\left(J_v J_v^{\top}\right)},
\qquad
\kappa(q) = \frac{\sigma_{\max}(J_v)}{\sigma_{\min}(J_v)} .
\tag{4}
$$

$w\to 0$ and $\kappa\to\infty$ mark singular, poorly-conditioned configurations.

# III. Task-Constrained Workspace

## A. Definition and method

Let $\mathcal{R}$ be the **reachable** workspace (positions attainable with some
joint vector within limits). Given a surface with unit normal $n$, the
**drawable** (task-constrained) workspace is

$$
\mathcal{D}(n) = \{\, p \in \mathcal{R} : \exists\, q,\ \mathrm{fk}_{\text{pos}}(q)=p,\
\hat z(q) = \pm n \,\},
\tag{5}
$$

i.e. positions the tool can reach while its approach axis $\hat z$ is aligned with
the surface normal. We estimate $\mathcal{D}(n)$ by sampling a grid of positions and
testing, for each candidate normal direction and from several seed configurations,
whether damped-least-squares IK converges with position error below
$2\,\text{mm}$ and orientation error below $3\times 10^{-2}$.

## B. Results

![Task-constrained workspace. (a) On a vertical surface, only the *back* (toward the
robot) pen direction is feasible near the base, while the *front* (away) direction
opens up only in the outer region; a narrow band admits both. (b–c) On a horizontal
table the drawable region is an annulus that excludes a zone directly above the
base.](figures/fig_feasibility.png){width=6.4in}

Figure 1 maps $\mathcal{D}$. On a **vertical** surface, only $57.9\%$ of $\mathcal{R}$
admits a perpendicular pen, and only $35.2\%$ admits the *front* (outward)
direction. This explains a practical observation: a board placed close to the robot
can only be drawn on with the pen facing *back*; to draw with the pen facing the
board from the front, the board must be placed in the outer region ($x\gtrsim
0.17\,\text{m}$). On a **horizontal** table, $63.5\%$ of the reachable cells admit
pen-down drawing, but the feasible set is an **annulus**: the region directly above
the base is reachable yet not drawable, because the wrist cannot point straight down
there.

# IV. Inverse-Kinematics Comparison

## A. Solvers

All solvers iterate $q \leftarrow q + \Delta q$ with the stacked pose error
$e = [\,p_t - p(q);\ e_{\text{ori}}\,]$, clamped to joint limits. The updates are:

$$
\Delta q_{\text{DLS}} = J^{\top}\!\left(JJ^{\top} + \lambda^{2} I\right)^{-1} e,
\qquad
\Delta q_{\text{pinv}} = J^{\dagger} e,
\tag{6}
$$

$$
\Delta q_{\text{JT}} = \alpha\, J^{\top} e,\quad
\alpha = \frac{\langle e, JJ^{\top}e\rangle}{\langle JJ^{\top}e, JJ^{\top}e\rangle},
\tag{7}
$$

with Levenberg–Marquardt using (6) with $\lambda$ adapted to the error trend. Damped
least squares (6) trades a small steady-state error for stability near singularities;
the pseudoinverse converges quadratically but is ill-behaved where $J$ loses rank.

## B. Results

On $150$ reachable target poses seeded from a common ready configuration:

| Solver | Success | Mean iters | Mean time (ms) |
|--------|-------:|-----------:|---------------:|
| Damped least squares | **98.7 %** | 76 | 15 |
| Pseudoinverse | 26 % | 8 | 30 |
| Jacobian-transpose | 29 % | 2000 | 365 |
| Levenberg–Marquardt | 61 % | 10 | 16 |

: Table II. IK solver comparison on the full-pose tracing task.

![Inverse-kinematics comparison. (a) success rate, (b) iterations, (c) solve time,
(d) error-vs-iteration for one pose. Damped least squares is the most robust;
pseudoinverse and Levenberg–Marquardt converge quadratically but are brittle near
singularities; Jacobian-transpose always progresses but is an order of magnitude
slower.](figures/fig_ik_comparison.png){width=6.4in}

Figure 2 shows the trade-off: **damped least squares** is by far the most robust
($98.7\%$), at the cost of more iterations and a damping-limited accuracy; the
**pseudoinverse** and **Levenberg–Marquardt** reach machine precision in $\sim10$
iterations on well-conditioned poses but fail more often near singularities; the
**Jacobian-transpose** is uniformly slow. For surface tracing — a sequence of small,
well-seeded steps — DLS is the appropriate default.

# V. Manipulability and Conditioning

![Manipulability $w$ and condition number $\kappa$. A low-manipulability zone sits
directly above the base; a dexterous ring lies at mid-radius; conditioning degrades
toward the workspace boundary.](figures/fig_manipulability.png){width=6.4in}

Figure 3 explains the *shape* of the drawable set. Manipulability $w$ is near zero
directly above the base (the arm is close to a shoulder/wrist singularity there) and
peaks in a mid-radius **ring**; the condition number $\kappa$ grows toward the
workspace boundary, where the arm approaches full extension. The drawable annulus of
Figure 1(b–c) coincides with this high-$w$, low-$\kappa$ ring — feasibility and good
conditioning are geometrically the same region.

# VI. Demonstrations

![Sine tracing placed by the analysis. (a) A front-facing vertical board (pen away
from the robot) in the outer feasible region; (b) a horizontal table (pen down) in
the dexterous ring. Both track the target to $0.000\,\text{mm}$.](figures/fig_demos.png){width=6.2in}

Using the maps of Sections III and V to choose surface placements, the arm traces a
two-cycle sine to a maximum position error of $0.000\,\text{mm}$ on both a
front-facing vertical board at $(0.21,0,0.12)\,\text{m}$ and a horizontal table at
$(0,0.18,0.10)\,\text{m}$ (Figure 4), confirming that the constrained-workspace maps
predict where clean drawing is possible.

# VII. Conclusion

The reachable and drawable workspaces of a surface-tracing manipulator are distinct
sets whose difference depends on the surface normal. For the myCobot 280, a
perpendicular pen is feasible over only $57.9\%$ (vertical) and $63.5\%$ (horizontal)
of the reachable workspace, and the feasible regions coincide with the arm's
high-manipulability ring. Damped least squares is the most robust IK method for the
task. Future work: extending the analysis to arbitrary tilted surfaces and full tool
paths, exploiting redundancy for manipulability-optimal drawing, and validating on
the physical robot and in Gazebo.

# References

1. J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 4th ed. Pearson, 2018.
2. T. Yoshikawa, "Manipulability of robotic mechanisms," *Int. J. Robotics Research*, 4(2), 1985.
3. C. W. Wampler, "Manipulator inverse kinematic solutions based on vector formulations and damped least-squares methods," *IEEE Trans. SMC*, 16(1), 1986.
4. Elephant Robotics, *myCobot 280 URDF / description*, 2023.
