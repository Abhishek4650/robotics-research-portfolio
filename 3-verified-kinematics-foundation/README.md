# R_sine — Drawing a Sine Wave with a myCobot 280

A ROS 2 (Jazzy) project that makes a **myCobot 280** 6-DOF arm trace a **sine wave**
on a surface, built **from first principles**: modified-DH forward kinematics, a
velocity-propagation Jacobian, and damped-least-squares inverse kinematics — every
stage verified numerically and cross-checked against an independent library (ikpy).

Everything is **Python** (no C++), organized as clean, single-responsibility modules
and nodes.

---

## 1. What it does

1. Generates a sine wave as a sequence of 3D targets on a configurable plane.
2. Solves inverse kinematics for each target (pen held perpendicular to the surface).
3. Streams the resulting joint trajectory to `/joint_states` at 30 Hz.
4. Visualizes in RViz: the robot, the **green target** sine, and the **red drawn** line.

Home end-effector (all joints = 0): **(0.061, 0.046, 0.410) m**.

---

## 2. Package layout

```
Rsine/
├── Rsine/                     # importable Python modules
│   ├── kinematics.py          # modified-DH FK + velocity-propagation Jacobian (pure math)
│   ├── ik.py                  # damped-least-squares IK solver
│   ├── sine_path.py           # configurable DrawingPlane + sine waypoint generator
│   ├── sine_ik_node.py        # ROS 2 node: precompute IK, stream /joint_states + target path
│   └── path_tracer_node.py    # ROS 2 node: accumulate the real drawn line from TF
├── analysis/                  # offline scripts (no ROS) that produce the figures
│   ├── verify_kinematics.py   # FK vs URDF (1e-15) and Jacobian vs finite diff (2.5e-7)
│   ├── verify_ik.py           # sine IK, ikpy cross-check, tracking graphs
│   ├── workspace.py           # reachable-workspace analysis (verified vs ikpy)
│   ├── compare_methods.py     # analytical vs ikpy comparison
│   ├── plot_kinematics.py     # home pose + workspace figures
│   ├── animate_sine.py        # GIF + montage of the arm drawing
│   └── make_*.py              # PPT / PDF / explainer generators
├── docs/                      # generated deliverables (see §6)
├── figures/                   # generated graphs and the animation
├── launch/rsine_sine.launch.py
├── rviz/rsine.rviz
├── urdf/mycobot_280_arm.urdf  # gripper-stripped arm URDF (for ikpy + RViz)
└── scripts/run_demo.sh        # launch helper (scrubs the VS Code snap env)
```

---

## 3. The mathematics (see `docs/mathematics.md`)

- **Convention:** Modified (Craig) Denavit–Hartenberg. Each link transform is
  `Rotx(αᵢ₋₁)·Transx(aᵢ₋₁)·Rotz(θᵢ)·Transz(dᵢ)`.
- **Identified DH table** (fitted from the URDF, verified to ~1e-15 m):

  | i | αᵢ₋₁ (°) | aᵢ₋₁ (m) | dᵢ (m) | θ offset (°) |
  |---|----------|----------|--------|--------------|
  | 1 | 0   | 0       | 0.13056 | +90 |
  | 2 | +90 | 0       | 0       | −90 |
  | 3 | 0   | −0.1104 | 0       | 0   |
  | 4 | 0   | −0.096  | 0.06062 | −90 |
  | 5 | +90 | 0       | 0.07318 | +90 |
  | 6 | −90 | 0       | 0.0456  | 0   |

- **Forward kinematics:** product of the six link transforms.
- **Jacobian (velocity propagation):** column i = `[ zᵢ × (p_e − pᵢ) ; zᵢ ]`.
- **Inverse kinematics (DLS):** `q ← q + Jᵀ(JJᵀ + λ²I)⁻¹ e`, e = position + orientation error.

---

## 4. Build

```bash
cd ~/ros2_ws
colcon build --packages-select Rsine
source install/setup.bash
```

Python deps used by the analysis scripts: `numpy`, `scipy`, `matplotlib`, `ikpy`
(cross-check), `python-pptx` + `reportlab` (deliverables). Install with
`pip install --break-system-packages ikpy python-pptx reportlab` if missing.

---

## 5. Run

**RViz simulation** (the arm draws the sine):

```bash
bash src/Rsine/scripts/run_demo.sh
# or, if not inside the VS Code snap terminal:
ros2 launch Rsine rsine_sine.launch.py
```

It is fully parameterized:

```bash
ros2 launch Rsine rsine_sine.launch.py board_x:=0.13 amplitude:=0.04 cycles:=3.0
```

> **Note (VS Code snap):** VS Code's integrated terminal is a snap sandbox that
> makes RViz crash with a `libpthread` glibc-mismatch error. `scripts/run_demo.sh`
> scrubs the offending env vars; alternatively run from a normal Ubuntu terminal.

**Regenerate all verification + figures:**

```bash
cd src/Rsine
for s in verify_kinematics verify_ik workspace compare_methods plot_kinematics animate_sine; do
  python3 analysis/$s.py
done
```

---

## 6. Deliverables (`docs/`)

| File | Purpose |
|------|---------|
| `mathematics.md` | The math, written separately |
| `R_sine_presentation.pptx` / `.pdf` | Slides for the guide (with speaker notes) |
| `R_sine_how_to_present.pdf` | Per-slide "say this / if asked" script |
| `R_sine_detailed_explanation.pdf` | Full project write-up |
| `R_sine_how_ros2_works.pdf` | How the ROS 2 nodes/topics/TF fit together |

---

## 7. Verified results

| Check | Result |
|-------|--------|
| DH forward kinematics vs URDF | ~1e-15 m |
| Velocity-propagation Jacobian vs finite differences | ~2.5e-7 |
| Analytical IK tracking of the sine | ~0.001 mm |
| ikpy cross-check (every waypoint) | ~0.001 mm |
| Reachable workspace vs ikpy FK | ~1.7e-16 m |
| Max reach (3D / horizontal) | 0.427 m / 0.301 m |

The drawing surface is **general** (vertical board or flat plate), set by a plane
origin and two in-plane axes; the solver auto-picks the reachable pen direction.
