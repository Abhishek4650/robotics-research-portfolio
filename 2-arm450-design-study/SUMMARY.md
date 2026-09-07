# ARM-450 — Summary Report

- **What it is:** a clean-sheet 6-DOF robot arm, 450 mm overall, designed from scratch to
  trace a sinusoid with the pen normal to a surface — and then to serve the
  docking / fuel-refilling work.
- **Status:** geometry, kinematics and the solved path all verified. **Mechanically
  unproven — nothing has been printed.**
- **Date:** 2026-08-21

---

## 1. Where the project stands

| | |
| --- | --- |
| Overall length | **450.0 mm** — requirement met exactly |
| Horizontal reach | 360 mm |
| **Mass** | **1365 g** against a **1450 g hard ceiling** (85 g headroom) |
| Degrees of freedom | 6, all with real hardware |
| Reachable workspace | **190 litres**, dead zone **none** |
| Workable (pen held normal, vertical board) | **20 %** — a patch ~95 × 140 mm |
| Bearings | 6 × 6806 (J1–J3) + 6 × 6706 (J4–J6) |
| Printed parts | **19 pieces, 624 g** |
| Pre-print gate | **60 passed, 0 warnings, 0 failures** |
| Geometry audit (STEP B-rep) | **52 features verified, 0 outstanding** |
| Verification rounds | **5 of 5 clean** |
| Sine tracing | **240/240 waypoints, 0.014 mm rms, 0 collisions** |

**The arm does what it was designed to do.** The kinematics are verified to the nanometre against the URDF, and Rsine runs on it unmodified.

### What changed on 2026-08-21

Three things that would each have wasted a print run:

1. **Three bearing seats did not exist.** The link halves and the base had no Ø42 pocket at
   all — each had been cut into a region an earlier cut already emptied, which CadQuery
   performs silently. The pre-print gate passed 55/55 on them because it checked the
   *parameter*, never the geometry. It now reads the STEP B-rep.
2. **The sine path was not collision-free.** 72 of 240 waypoints had the upper arm grazing
   the base. Raising the trace 20 mm cleared every pose and improved accuracy 14×.
3. **The mass model was wrong.** A 0.55 sparse-infill factor was being applied to
   hollow-modelled parts, discounting the hollowing twice and hiding ~240 g.

Also: the servo horn had nothing to drive — a horn adapter now closes that torque path —
and the wrist moved to a smaller bearing, saving 93 g.

---

## 2. What was built

**Twelve part types**, all STEP + STL, all sharing one joint interface — Ø42 pocket, 6806 bearing, Ø30 shaft — so a fit validated on one joint is valid on all six.

`base` · `turret_j1` · `link_half_tongue` ×2 · `link_half_groove` ×2 · `joint_shaft` · `shaft_clamp` · `servo_collar` · `wrist_j4_housing` · `wrist_j5_yoke` · `wrist_j6_output`

**All six joints take the same Waveshare ST3215.** One part number for the whole machine.

**Documents:** `ARM450_REPORT.pdf` (analysis) · `ARM450_KINEMATICS.pdf` (DH, FK, Jacobian, workspace) · `ARM450_DRAWINGS.pdf` (parts) · `ARM450_LESSONS.pdf` (mistakes) · `ARM450_LOG.pdf` (running log) · this summary.

---

## 3. The findings that changed the design

**The bearing was the whole problem.** The original single thrust washer has zero moment capacity — 0.2 mm of clearance became **28.9 mm of wobble** at the tool. A preloaded pair at 22 mm spacing brings that to 0.65 mm, a 44× improvement. Preload is the entire story: without it the pair only buys 4×.

**Mounts were sized for the wrong load.** They were sized for the 2.08 N·m the arm carries, when a servo can deliver 2.94 N·m of stall torque into its own mount during a collision. *Whenever an actuator can drive its structure to failure, the actuator's limit is the design load.*

**Equal links eliminate the dead zone.** |L2 − L3| is the inner radius of the reachable annulus. The user's original links were equal; a later revision broke that and opened a Ø194 mm hole.

**Stress was never the constraint.** Safety factors run 20–400 across every part. The section size is set by the servo that has to fit inside it, not by any load.

---

## 4. What is verified, and what is not

**Verified numerically:**
- DH table vs URDF: 0.0000 µm over 5,000 poses
- Jacobian vs numerical differentiation: 1.85 × 10⁻⁸ m/rad
- Chain totals 450 mm, asserted at build time
- Workspace, dead zone, task workspace: 250k-pose sweeps
- Interference across joint limits: **16 %** of poses collide, and every one is arm-vs-base, none arm-vs-arm
- FEA on all parts: SF 20–400
- URDF extracted from the CAD with real meshes and inertias

**Not verified — and this is the honest gap:**
- **The 0.65 mm wobble is calculated, never measured.** It is the single most valuable number to obtain.
- Four of six joint parts (J1, J4, J5, J6) are **first-pass**: no shaft fit check, no servo horn clearance, no cable routing.
- **Servo mounting-hole and horn bolt patterns are placeholders.** The user's ST3215 STL is an outer shell with no holes modelled.
- Horizontal-table tracing **does not work** and needs a wrist rethink.

---

## 5. Against the guide's brief

**Sine motion — achievable now.** Vertical board at 240–300 mm, 0.028 mm rms.

**docking — yes, but by geometry rather than by accuracy.** Absolute accuracy is
backlash-limited at roughly 5 mm and no structural change fixes that; it would take
joint-side encoders. So the docking tool does not ask the arm to be accurate. Its capture
cone is a **Ø34 mouth over a Ø16 throat**, and the measured envelope — probe mesh swept
sideways against target mesh until they foul — is **±9.75 mm**, about **2× the arm's
backlash-limited error and 7× its repeatable error.** The cone converts a positioning
problem into a geometry problem, which is the standard answer in docking hardware and the
reason the mouth is as oversized as it is.

The latch needs no seventh actuator either: it is a bayonet and **J6 turns it.**

**"Space grade with minor adjustment" — only in the geometric sense.** The kinematics, joint architecture and fastening scheme all survive a material swap. PLA does not (outgassing, 60 °C glass transition), hobby servos do not (no vacuum lubrication), and wet-grease bearings do not. Re-made in Al 6061 the same design is 1.13 kg and about 10× stiffer. Worth agreeing with the guide which meaning is intended before promising it.

---

## 6. Next three steps

1. **Measure the wobble.** Assemble one joint, dial indicator at 360 mm, push both ways, record the hysteresis before and after preload. That single before/after pair tests the entire bearing theory.
2. **Measure a real ST3215** and correct four numbers in `cad/params.py`. Every mounting hole regenerates.
3. **Reprint the link halves at 119 mm.** They were shortened from 145 mm to buy wrist length; reach is unchanged.
