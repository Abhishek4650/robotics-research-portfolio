# ARM-450 rev I.1 — kinematics (DH table, FK, IK, Jacobian, workspace)

In **my convention**: Modified (Craig) DH, frame i on link i, `^{i-1}_iT = Rot_x(α_{i-1}) Trans_x(a_{i-1}) Rot_z(θ_i) Trans_z(d_i)`,
Jacobian by **velocity propagation**, damped-least-squares IK cross-checked with ikpy — the method of the
myCobot 280 thesis (`~/ros2_ws/src/mycobot_thesis`) and Rsine (`~/ros2_ws/src/Rsine`), applied to the arm as
designed and released (rev I.1). Rsine, the thesis and `arm450_design` are untouched.

## Start here

| | |
|---|---|
| **Report (maths + results)** | [docs/ARM450_KINEMATICS.pdf](docs/ARM450_KINEMATICS.pdf) |
| **Slides for the guide** | [docs/ARM450_KINEMATICS_slides.pptx](docs/ARM450_KINEMATICS_slides.pptx) + how to present them: [docs/ARM450_KINEMATICS_how_to_present.pdf](docs/ARM450_KINEMATICS_how_to_present.pdf) |
| **DH table** | [DH_TABLE.md](DH_TABLE.md), [dh_table.csv](dh_table.csv) |
| **Animations** | [figures/sine_vertical.gif](figures/sine_vertical.gif), [figures/sine_table.gif](figures/sine_table.gif) |
| **RViz demo** | `source /opt/ros/jazzy/setup.bash; cd ros2; ros2 launch sine_demo.launch.py which:=vertical` (or `table`) |

## The DH table (derived from the CAD servo axes)

| i | α_{i-1} | a_{i-1} (mm) | d_i (mm) | θ_i | from servo q_i |
|---|---|---|---|---|---|
| 1 | 0 | 0 | 90.000 | θ1 | θ1 = −q1 |
| 2 | −90° | 0 | 0 | θ2 (home −90°) | θ2 = −q2 − 90° |
| 3 | 0 | 119.000 | 0 | θ3 (home +90°) | θ3 = q3 + 90° |
| 4 | +90° | 0 | 221.369 | θ4 | θ4 = −q4 |
| 5 | −90° | 0 | 0 | θ5 | θ5 = q5 |
| 6 | +90° | 0 | 0 | θ6 | θ6 = −q6 |

Tool: `^6_TT = Trans_z(88.089)` to the tool-flange face. J4, J5, J6 meet at the wrist centre (spherical wrist).

## What was done (and the result)

| step | script | result |
|---|---|---|
| 1 DH from the CAD | `analysis/s1_dh_from_cad.py` | table derived by the frame rules from the measured servo axes; every link transform has the exact Craig form |
| 2 FK + Jacobian | `analysis/s2_verify_fk.py` | FK = CAD (product of exponentials) and = URDF (ikpy) to ~3e-13 mm over 2000 poses; velocity-propagation J = z×r form to 2e-13 |
| 3 IK closed form | `analysis/s3_ik.py` | up to 8 branches, exact to 3e-13 mm on 5000 poses, original always recovered; DLS and ikpy agree |
| 4 workspace | `analysis/s4_workspace.py` | reach 405 mm from the J1 axis; **pen straight down impossible** (J2+J3+J5 = 172.5°); table needs a tilted pen (ring); board: front pen only |
| 5 manipulability | `analysis/s5_manipulability.py` | Yoshikawa w and κ maps; closed form a2·d4·\|sin φ3\|·r checked; home singular three ways |
| 6 IK solvers | `analysis/s6_ik_comparison.py` | closed form 100 % / 0.2 ms; DLS, pseudoinverse, LM, ikpy exact only from a nearby seed |
| 7 sine tracing | `analysis/s7_sine_demo.py` | vertical board + table ring, smooth (free pen roll), re-traced by DLS and ikpy to 1e-6..1e-4 mm |
| 8 timing on the servos | `analysis/s8_trajectory_timing.py` | resolved rate on the 5-D pen task (roll free); board sine at ~80 mm/s with the ST3215 at half their 270 deg/s rating; encoder floor ~0.9 mm |
| ROS 2 | `ros2/` | player + path tracer + launch; the URDF/TF chain draws both sines within 0.001 mm (`check_ros_demo.py`) |

Logs: `VERIFY_FK.log`, `VERIFY_IK.log`, `WORKSPACE.log`, `MANIPULABILITY.log`, `IK_COMPARISON.log`, `SINE_DEMO.log`, `TIMING.log`.
Trajectories (servo angles, deg): `sine_vertical_trajectory.csv`, `sine_table_trajectory.csv`.

## Use it

```python
from arm450kin import robot as R, ik
th = R.q_to_theta(q_servo_rad)          # servo angles -> DH angles
T  = R.fk(th)                           # ^0_T T, mm
J  = R.jacobian(th)                     # velocity propagation, base frame
sols = ik.analytical(T[:3, 3], T[:3, :3])        # [(theta, branch, within_limits), ...]
q = R.theta_to_q(ik.analytical_best(p, Rot, th_prev))
```

Rebuild everything: `bash run_all.sh`. ikpy runs in its own interpreter (`analysis/ikpy_bridge.py`,
`python3 -s` with the system numpy 1.26 — the user-site numpy 2.x breaks ikpy's sympy maths).
