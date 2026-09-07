# ARM-450 — Project Review Dossier

**A 450 mm printed 6-DOF manipulator for sinusoidal tracing and probe-and-drogue docking**

Prepared for technical inspection · 2026-08-28 · **status: pre-hardware**

> **Read this first.** Nothing in this project has been printed or measured. Every number
> is analytical or simulated, and the document says so at each point. That is deliberate:
> a prediction written down before the test is evidence; the same number written after is
> not. §10 lists seven measurements not yet made, each with its falsification condition.

---

## 1. The idea, and why this design and not another

### 1.1 The problem

The arm is the manipulation element of a maglev-floated chaser/target capture
project. Two demonstrations were required:

1. **Sinusoidal trace** on a board — a *precision* demonstration. The traced curve is a
   permanent, visible record of repeatability, and it exercises five joints at once.
2. **Probe-and-drogue docking** with fluid transfer — a *capture tolerance* demonstration.

These fail for opposite reasons, and that shaped everything.

### 1.2 Why 6-DOF serial, and not the alternatives

| architecture | rejected because |
| --- | --- |
| **SCARA (4-DOF)** | cannot orient a tool arbitrarily. Docking needs the probe axis aligned with the port in 3-D; SCARA holds the tool vertical always. |
| **Delta / parallel** | excellent stiffness and speed, but a small workspace relative to its footprint, and no natural way to put a *rotating* tool at the end — the docking latch is driven by tool roll. |
| **Cartesian gantry** | the stiffest option and the easiest to build accurately, but it is not a manipulator. It cannot represent a spacecraft arm, which is the point of the project. |
| **5-DOF serial** | one short. A 5-DOF arm can position and point, but cannot roll about the tool axis — and **tool roll is what latches the docking bayonet.** Removing J6 would force a seventh actuator back in elsewhere. |
| **7-DOF redundant** | the honest answer for a real space arm (obstacle avoidance, singularity escape). Rejected here on mass: a seventh ST3215 is 60 g plus driver and wiring, against a 1450 g budget already tight. |

**6-DOF serial is the minimum that does both tasks.** That is the argument — not that it is
best in the abstract.

### 1.3 Why these particular design choices

**One joint interface on all six axes.** Ø42 pocket · 6806 bearing · Ø30 shaft, everywhere.
The alternative — sizing each joint to its own load — is what an engineer's instinct says.
It was rejected because a printed bearing pocket must be *characterised on the machine that
prints it*, and six different pockets means six characterisations. With one, a single 0.9 h
test coupon gates the entire 21 h print. The cost is a wrist carrying more bearing than it
needs, and that cost was measured, not assumed: swapping the wrist to a 4 mm-wide 6706
saved 24 g and cost **no** stiffness, because moment stiffness goes as bearing *spacing*
squared and is independent of bearing width.

**Clamshell links, split on the neutral axis.** A closed box section 47.5 × 29.0 mm in a
2.4 mm wall. An open C-section of the same envelope has roughly two orders of magnitude
less torsional stiffness, and the payload hangs off-axis. The seam bolts are what close the
section — they are structural, not cosmetic.

**Two bearings per joint, not one.** A single deep-groove bearing is a pivot: it holds the
shaft centred and lets the joint tilt. Two spaced apart form a couple that resists bending.
An earlier revision of this arm used a single thrust washer per joint and it is recorded as
the largest error in the project — it produced the wobble and the limit-cycle oscillation
that started the redesign.

```
tool error at 360 mm reach
  single thrust washer                28.90 mm
  bearing pair, no preload             6.95 mm
  bearing pair, some preload           1.44 mm
  preloaded pair, 40 mm apart          0.45 mm   <- the design
```

**Docking latched by J6, with no seventh actuator.** J6 has ±175° of travel and does
nothing during a docking approach — a probe is rotationally symmetric about its own axis.
Rolling it 30° turns a three-lug bayonet. The motion was already free.

---

## 2. Mathematics

### 2.1 Kinematic chain

```
world → base_link → link1 → link2 → link3 → link4 → link5 → link6 → tcp
        J1 Z(yaw)  J2 Y    J3 Y    J4 Z    J5 Y    J6 Z
  z, mm:    50      40      119     119     62      30      30
```

Total **450.00 mm** base to tool face, verified to 1 × 10⁻⁶ mm against the parameter file.

### 2.2 Forward kinematics — modified (Craig) DH

Frame *i* is attached to link *i*, parameters (α<sub>i−1</sub>, a<sub>i−1</sub>, d<sub>i</sub>, θ<sub>i</sub>):

```
        ⎡ cθ         −sθ        0      a      ⎤
i-1_i T=⎢ sθ·cα     cθ·cα     −sα   −sα·d     ⎥
        ⎢ sθ·sα     cθ·sα      cα    cα·d     ⎥
        ⎣ 0          0          0      1      ⎦
```

| i | α<sub>i−1</sub> | a<sub>i−1</sub> (mm) | d<sub>i</sub> (mm) | θ<sub>i</sub> |
| --- | --- | --- | --- | --- |
| 1 | 0° | 0 | 90 | q₁ + 180° |
| 2 | 90° | 0 | 0 | q₂ + 90° |
| 3 | 0° | **119** | 0 | q₃ + 90° |
| 4 | 90° | 0 | **181** | q₄ + 180° |
| 5 | 90° | 0 | 0 | q₅ + 180° |
| 6 | 90° | 0 | 0 | q₆ |

Tool: 60 mm along Z₆.

**How the table was obtained, and the trap in it.** The joint axes were taken from the URDF
at q = 0 and the common normals computed between consecutive axes. Every axis passes
through the base Z line, so every `a` is zero *except* the perpendicular between the two
parallel pitch axes. **d₄ = 181, not 119**: frame 3's origin sits where X₃ meets Z₃
(z = 209) and frame 4's where Z₄ meets Z₅ (z = 390), so the offset along Z₄ is 119 + 62.
Writing the "obvious" link length there puts the tool **193 mm** out — which is exactly what
the first version did.

### 2.3 Jacobian — velocity propagation, not differencing

```
i+1_ω_{i+1} = i+1_i R · i_ω_i  +  θ̇_{i+1} · Ẑ
i+1_v_{i+1} = i+1_i R · ( i_v_i + i_ω_i × i_P_{i+1} )
```

Propagated base to tip. This matters at the J4/J6 wrist singularity, where the two roll
axes go collinear and a finite-difference Jacobian degrades.

### 2.4 Inverse kinematics

Bounded least-squares on position plus tool-axis direction. Two terms are non-obvious and
both were arrived at by failure:

**J6 is locked.** A pen is rotationally symmetric about the tool axis, so J6 is pure
redundancy. Free, the solve is under-determined and the solver wanders 135° between
adjacent waypoints. Locking it makes an exactly-determined 5-DOF problem.

**Null-space damping on J4 and J6 only.** When J5 → 0 those two roll axes become collinear
and only their *sum* is determined. Damping all six joints equally suppressed the wander
and destroyed accuracy — 0.02 → 8.8 mm rms — because it also fought the joints doing the
task.

### 2.5 Statics

Gravity torque from a 60 000-pose sweep of the reachable configuration space, not a single
worst-case pose:

| joint | peak N·m | servo | stall N·m | margin |
| --- | --- | --- | --- | --- |
| J1 | 0.00 | ST3215 | 2.94 | — (axis vertical) |
| **J2** | **2.74** | ST3250 | **4.90** | **1.79×** |
| J3 | 1.39 | ST3215 | 2.94 | 2.12× |
| J4 | 0.42 | ST3215 | 2.94 | 7.0× |
| J5 | 0.42 | ST3215 | 2.94 | 7.0× |
| J6 | 0.09 | ST3215 | 2.94 | 32× |

### 2.6 Compliance

Four closed-form terms, each stated so a disagreement is traceable: forearm bending
(point + UDL), upper-arm bending, upper-arm **slope × outboard length**, and shell torsion
from the payload's lateral offset.

```
box section:  I = (b·h³ − (b−2t)(h−2t)³)/12
torsion:      J = 4·Am²·t / perimeter        (Bredt, closed thin tube)
cantilever:   δ = P·L³/(3EI)      UDL: δ = w·L⁴/(8EI)
```

---

## 3. Software architecture

```
cad/params.py          SINGLE SOURCE OF TRUTH — geometry, material, print settings
   │                   (MATERIAL = "PLA" drives E, ρ, yield, Tg, shrinkage)
   ├── cad/*.py        CadQuery part models → STEP + STL
   │
   ├── audit_parts.py  reads the STEP B-rep back; are the features PHYSICALLY there?
   ├── dim_audit.py    every length, bore, fillet, chamfer, one at a time
   ├── mate_check.py   do holes in parts that bolt together actually line up?
   ├── final_audit.py  printability: walls, overhangs, build volume, orphan files
   │
   ├── fea.py          hex8 voxel FEA on the real printed geometry
   ├── creep_fatigue.py Findley creep + Basquin S-N
   ├── compliance_budget.py  independent closed-form cross-check
   ├── joint_loads.py  gravity torque sweep, parameterised by tool reach
   ├── verify_configs.py  the whole load suite × 5 payload configurations
   │
   ├── dh_kinematics.py  modified-DH FK + velocity-propagation Jacobian + IK
   ├── check_ikpy.py     three-way cross-check against an outside package
   │
   ├── assemble.py     every part at its true mating position
   ├── interference.py exact mesh-mesh collision (FCL)
   ├── plan_sine.py    offline IK for the trajectory
   └── preflight.py    the 8-stage gate — one verdict
```

**The design rule the whole codebase follows:** *every check reads the exported geometry,
never the parameters used to author it.* §4 explains why.

---

## 4. ROS 2 architecture

```
                    ┌──────────────────┐
   sine_traj.npz ──►│  arm450_sine     │──► /joint_states      (sensor_msgs)
   (solved offline) │  sine_node       │──► /arm450/trajectory (JointTrajectory,
                    └──────────────────┘        latched TRANSIENT_LOCAL)
                                        └────► /sine_path      (Marker)
                             │
   arm450_meshes.urdf ──► robot_state_publisher ──► /tf, /tf_static
                             │
                          RViz 2  (3 configs: whole arm · wrist · side-on)
```

**IK is solved offline and replayed.** A least-squares solve per waypoint inside the node
would block the executor. `plan_sine.py` writes 240 waypoints; the node replays them.

Three features exist because simulation exposed the need:

| feature | what it prevents |
| --- | --- |
| **cosine approach ramp** | the node used to publish waypoint 0 on its first tick — invisible in RViz, a full-speed slam on real servos |
| **ping-pong playback** | modular wrapping is a discontinuity, not a loop: J4 jumps 43.3° in one 40 ms tick and the tool teleports 139.8 mm |
| **latched JointTrajectory** | `/joint_states` is a visualisation stream a controller will not follow |

Packages: `arm450_description` (URDF, meshes, RViz), `arm450_sine` (node, launch).
`Rsine` is a separate, frozen package — imported read-only for its plane geometry, never
written to.

---

## 5. Simulation — parameters and results

### 5.1 Trajectory

| parameter | value |
| --- | --- |
| board distance | 300 mm (flange) / **342 mm (gripper tip)** |
| trace centre height | 220 mm |
| amplitude | 40 mm |
| cycles | 2 |
| span | 140 mm |
| waypoints | 240 |
| playback | 25 Hz → 9.6 s per pass, ping-pong |

### 5.2 Results

| metric | value |
| --- | --- |
| position error, flange | **0.014 mm rms**, 0.217 max |
| position error, tool tip | **0.055 mm rms**, 0.834 max |
| tool tilt | 0.093° rms |
| largest joint step between waypoints | 2.09° |
| **collisions, exact mesh** | **0 of 240** |
| with gripper fitted | 0 of 60 |
| with docking probe fitted | 0 of 60 |
| peak joint rate | 0.91 rad/s vs ~2.3 loaded ST3215 |

### 5.3 Workspace

Monte-Carlo FK, 200 000 poses inside the URDF joint limits, binned into a 5 mm (r, z) grid
and revolved:

| fitted | reach past J6 | max reach | swept volume |
| --- | --- | --- | --- |
| bare face | — | 450 mm | 188 L |
| pen | 30 mm | 480 mm | 239 L |
| **gripper** | **42 mm** | **492 mm** | **262 L** |
| dock probe | 36 mm | 486 mm | 251 L |

### 5.4 Cases that do not work

**Horizontal-plate tracing fails.** Swept 12 plate positions; best is **11.7 mm rms**
against 0.020 on the vertical board — 585× worse. **J2 is hard against its +115° stop**: to
reach out over a table *and* tip the tool down, the shoulder runs out of travel. A 90° bent
tool adapter was tried and improved it to 8.1 mm, at which point J2 *and* J5 both saturate.

It is a joint-range limit, not a solver or tool problem, and it traces to J5 being narrowed
from ±110° to **−91°/+49°** to stop the J4 housing striking the J6 body — a real collision
found by exact-mesh sweep. **The collision fix cost the horizontal capability, and that is
a trade made knowingly.**

---

## 6. Structural analysis

### 6.1 FEA

Hex8 voxel FEA on the **actual printed geometry**, with loads taken from the assembly-level
statics rather than guessed.

| | |
| --- | --- |
| method | part-wise submodelling, hex8 elements |
| loads | from `joint_loads.py`, not assumed |
| worst p99 von Mises safety factor | **23.0** across all parts |
| working stress, worst case | 0.437 MPa against 55 MPa yield |

**Why part-wise and not assembly-level:** an assembly FEA needs joint stiffness, bolt
preload, and bearing radial/moment stiffness — none of which is known to better than a
factor of two before hardware. Part-wise uses loads that *are* known and returns each
part's margin in isolation. **This is a stated limitation, not an oversight** (§10.2).

### 6.2 Creep

Findley power law, 1 year of continuous load:

| | |
| --- | --- |
| working stress | 0.437 MPa |
| stress ratio | 0.0064 of yield |
| **strain at 1 year** | **0.097 %** (plain PLA) |
| limit | 0.5 % |

The absolute strain is tiny but it is several times the *elastic* strain, and the Findley
fit is being used two orders of magnitude below its calibration range. **Actionable
outcome: park the arm folded when idle, not extended.**

### 6.3 Fatigue

Basquin S-N. **No endurance limit is assumed** — polymers do not have one, so a life target
is used instead of a fatigue limit.

| | |
| --- | --- |
| stress amplitude | 0.22 MPa |
| **cycles to failure** | **8.0 × 10³¹** |
| target | 1 × 10⁸ |

Not limiting, because the arm is stiffness-driven, not strength-driven.

### 6.4 Stiffness and the precision budget

| config | structural | 0.5° backlash | combined |
| --- | --- | --- | --- |
| bare | 0.071 mm | 1.88 mm | 1.88 mm |
| gripper | 0.108 mm | 2.24 mm | 2.24 mm |
| **gripper + 225 g** | **0.197 mm** | 2.24 mm | **2.25 mm** |

> **Backlash exceeds structural deflection by roughly 20× in every configuration.**
> This arm is **servo-limited, not structure-limited.** No change to the printed parts
> moves the number. Accuracy work belongs in joint-side feedback.

### 6.5 Mass

| group | mass |
| --- | --- |
| printed structure (PLA, 90 % effective fill) | 599 g |
| servos, 6 × | 360 g |
| bearings, 12 × | 176 g |
| aluminium shafts, 6 × | 205 g |
| **structure subtotal** | **1340 g** |
| fasteners and heat-set inserts | 244 g |
| **ASSEMBLED** | **1584 g** |
| budget | 1450 g |
| **over by** | **134 g** |

**This is an open item and are not hiding it.** See §9 Q11.

---

## 7. Verification method — the part are most confident about

### 7.1 The principle

**Every check reads the exported STEP or mesh, never the parameters used to author it.**

An early gate checked `BEARING_POCKET_D == 42.0` and passed 55/55 while **three bearing
pockets did not physically exist**: a hollowing operation had removed the material the
pocket was cut into, and subtracting a Ø42 pocket from an already-empty region succeeds
silently and changes nothing.

> A boolean that removes nothing does not fail. It returns the same solid, and every step
> downstream — render, drawing, mass, collision — treats the result as correct.

### 7.2 The eight-stage gate

| stage | question | criterion | result |
| --- | --- | --- | --- |
| 1 Design check | do the features physically exist? | STEP B-rep audit | 52 features, 0 missing |
| 2 FEA | does anything yield? | p99 SF ≥ 3 | 23.0 |
| 2 Load cases | with and without a tool? | 6 criteria × 5 configs | 30/30 |
| 3 Creep + fatigue | does it fail slowly? | < 0.5 %/y, > 1e8 | pass |
| 4 Stiffness | does the tool stay put? | ≤ 0.30 mm | 0.197 |
| 5 URDF | does the model match the machine? | parses, mass, meshes | pass |
| 6 Path tracing | does the real trajectory work? | 0 collisions | 240/240 |
| 7 Simulation | will ROS work on hardware? | ramp, topic, rate | pass |
| 8 End effectors | do the tools fit, does it latch? | 32 checks + a mate | pass |

**Current: 25 passed, 1 failure** — the mass overage in §6.5.

### 7.3 Independent cross-checks

Nothing important rests on one calculation:

| quantity | primary | cross-check | agreement |
| --- | --- | --- | --- |
| forward kinematics | Craig DH | URDF chain **and ikpy** | 2 × 10⁻¹³ mm |
| Jacobian | velocity propagation | finite difference | 1.8 × 10⁻⁵ |
| compliance | `verify_configs` | `compliance_budget.py` | 0.098 mm both (CF case) |
| mass | STEP volumes × density | per-part geometry | — |
| bearing fit | parametric | virtual press into every pocket | light press, 10/10 |

**Three implementations sharing no code cannot agree on the same mistake.** That is why
ikpy is in the loop: agreement between our DH table and our URDF would only prove the table
copies the URDF faithfully, *errors included*.

### 7.4 What the method returns

The value of reading exported geometry rather than authoring parameters is not theoretical.
Fifteen defects were identified by this route that had survived visual inspection of the
same parts, and they fall into four recurring classes:

**Operations that succeed without effect.** A boolean subtraction from a region that is
already void returns the original solid unchanged, with no error. Three bearing pockets were
absent for this reason while a parameter check reported every dimension correct.

**Assemblies that are not connected.** A union of two bodies that do not touch also succeeds;
it simply returns a compound. The docking probe exported as four separate solids, three of
them latch lugs suspended clear of the bore they were intended to grow from.

**Dimensions never checked against what they cut.** A capture groove specified at 3.30 mm
depth was applied to a wall 1.65 mm thick, severing the feature it was cut into.

**Relationships that appear in no single-part view.** The capture cone was specified deeper
than the mating spigot was long, so probe and target could not engage at any insertion depth
or roll angle. Neither part is incorrect in isolation; the defect exists only in the pair.

Two further categories were found in the documentation rather than the geometry: drawings
asserting features the parts did not carry, and constants duplicated across files that had
since diverged.

The common property is that **none of these failed loudly.** Each produced a plausible
artefact — a render, a mass figure, a passing check — that was wrong. Detection required
interrogating the exported representation rather than the intent behind it.

---

## 8. Space vs ground manipulators

### 8.1 The differences that matter

| | ground manipulator | space manipulator |
| --- | --- | --- |
| **Gravity** | dominates sizing; the arm must hold itself up | ~0 g. Sizing is driven by **inertia and dynamics**, not weight |
| **What limits payload** | actuator torque against gravity | reaction on the base — the arm moves the *spacecraft* |
| **Base** | bolted to a rigid floor | free-floating. Moving the arm rotates the satellite; momentum must be managed |
| **Structure** | stiff enough not to sag | stiff enough not to *oscillate* — no damping in vacuum |
| **Thermal** | 20 ± 15 °C | −120 to +120 °C cycling; CTE mismatch drives joint clearances |
| **Lubrication** | wet grease | dry film (MoS₂) or PFPE — wet grease outgasses and creeps |
| **Materials** | anything | TML < 1 %, CVCM < 0.1 %; no trapped volumes |
| **Radiation** | none | TID-rated electronics, latch-up immunity |
| **Backlash** | tolerable | often unacceptable — no operator to correct in the loop |
| **Repair** | trivial | none. Single-fault tolerance or redundancy |
| **Qualification** | "it works" | vibration, shock, thermal-vac, EMC, life test |
| **Control** | position | often **impedance/force** — contact dynamics with a free target |

### 8.2 The one that surprises people

**A space arm's hardest problem is not holding the load — it is that there is nothing to
push against.** On the ground the floor absorbs every reaction. In orbit, Newton's third
law means accelerating the arm rotates the spacecraft. Canadarm2 moves slowly for this
reason, and free-flying servicers must plan trajectories that are *reactionless* or
compensate with reaction wheels.

**ARM-450 cannot demonstrate this**, because it is bolted down. That is the honest limit of
the analogy, and the maglev float is precisely the attempt to recover it.

### 8.3 How ARM-450 relates

Of sixteen subsystems compared against a flight equivalent: **3 survive unchanged, 2 need
rework, 11 must be replaced.**

**Survives:**
- **The kinematics.** A 6R chain with this layout is unaffected by what it is made of. URDF, IK and workspace transfer directly.
- **The joint architecture.** One bore, one shaft, one bearing family is good practice at any grade; only part numbers and lubricant change.
- **The fastening scheme.** Bolt patterns and load paths transfer; flight versions get locking compound, staking and vented screws in the same places.

**Does not:**
- **The material is the wall.** PLA's glass transition is ~60 °C; a sun-facing surface in LEO exceeds it. It outgasses. Re-made in Al 6061-T6 the same geometry is **≈1.13 kg — lighter than printed — and about 10× stiffer.** The swap is an improvement that costs money, not a compromise.
- **The actuators.** Hobby serial servos: plastic gears, wet grease, no radiation tolerance, no joint-side feedback. This single choice sets the 2.2 mm precision figure.
- **Vacuum compatibility was never designed in.** The printed parts contain closed internal volumes with no vent path.
- **Nothing has been qualified.**

> **The honest framing:** ARM-450 is a **kinematic and architectural prototype** of a space
> manipulator, and a **functional demonstrator** of a docking interface. It is not a flight
> article. What transfers is the geometry, the joint scheme, the docking principle — and
> the verification method, which is grade-independent.

### 8.4 What genuinely is space-relevant

**The docking interface.** Capture tolerance was solved as *geometry*, not accuracy: a Ø34
mouth over a Ø16 throat gives a **measured ±9.75 mm** envelope against ≈2.2 mm of arm
error. This is exactly how real docking hardware works, and it is the transferable idea.

**Fluid transfer.** A radial O-ring in the spigot, compressed by the bayonet itself, with a
Ø8 pass-through in both halves — the architecture of an orbital refuelling coupling at
demonstration scale.

**No seventh actuator.** Using an existing DOF to drive a mechanism is a mass-saving
technique that matters far more in orbit than on a bench.

---

## 9. Anticipated questions

*Written to be answered honestly. Where the answer is "this does not know", it says so.*

---

**Q1. Has the arm been built? What has actually been verified?**

No. Nothing is printed. Everything is analytical or simulated, and §10 lists the seven
measurements outstanding with their predictions and falsification conditions. What *is*
verified is that the geometry is self-consistent, the loads are within margin, the
trajectory is collision-free on exact meshes, and the kinematics agrees with two
independent implementations. That is design verification, not hardware validation, and we
do not claim otherwise.

---

**Q2. A safety factor of 23 — is the arm massively over-designed?**

Yes on strength, and deliberately. The arm is **stiffness-driven, not strength-driven** —
sized by how far the tool deflects, not by whether anything breaks. Once the wall is thick
enough to be stiff, it is far more than thick enough to be strong. Reporting SF 23 is
reporting that strength was never the binding constraint. The binding constraint is
**backlash**, which beats structural deflection by 20×.

---

**Q3. Then why not thin the walls and save mass?**

Because bending stiffness goes as **t³**. Dropping the wall from 2.4 to 1.8 mm — a 25 %
reduction — costs 58 % of the stiffness. And at a 0.6 mm nozzle, 2.4 mm is exactly four
perimeters: thinner walls stop being solid extrusion and start containing infill voids,
which is a different material, not a thinner one.

---

**Q4. Why plain PLA rather than an engineering polymer?**

Availability and print reliability. The full load suite was re-run for it — **10 of 10
pass**, worst deflection 0.197 mm against a 0.30 mm limit — and at the tool the difference
from carbon-filled is **2.24 vs 2.25 mm**, because backlash dominates. PLA is not a
compromise for *this* demonstrator. It would be for a flight article, and §8.3 says so.

---

**Q5. 2.2 mm positional accuracy is poor for a manipulator.**

It is, and it is honest. It is set almost entirely by **servo backlash** — the printed
structure contributes 0.197 mm, about 2 % of the total. The arm uses hobby serial servos
with no joint-side encoder. Improving it means adding feedback at the joint, not stiffening
the arm; a magnetic encoder on each output shaft would take it to ~0.1 mm.

**And it is why the docking interface has a ±9.75 mm capture cone.** designed the
interface around the accuracy have rather than pretending to accuracy this does not.

---

**Q6. Simulation reports 0.055 mm rms. Is that the claimed accuracy?**

**No.** That number validates the *kinematics* — the solver puts the tool where it is
asked. The hardware prediction is **1–3 mm rms**, recorded in §10 with its falsification
condition. If the trace measures under 0.5 mm, our backlash model is wrong and will say
so.

---

**Q7. Why two bearings per joint? Is that not wasteful?**

A single deep-groove bearing is a pivot — it permits tilt. Two spaced apart resist bending.
The measured cost of removing one: tool error goes **0.45 mm → 28.90 mm**, 64× worse. An
earlier revision of this arm made exactly that mistake with a single thrust washer, and it
is documented as the largest error in the project.

---

**Q8. Why is the wrist over-engineered with 42 mm bearings?**

It is not, any more — that was measured and fixed. The wrist runs 6706 (Ø37 × 4) against
6806 (Ø42 × 7) at the shoulder: **24 g saved, no stiffness lost**, because moment stiffness
goes as bearing *spacing* squared and is independent of width. What remains is the cost of
commonality: one bore diameter across all six joints, so one fit characterisation and one
spare part number.

---

**Q9. Why is the J5 range asymmetric, −91° to +49°?**

Because an exact-mesh sweep found the **J4 housing striking the J6 output body** over the
excluded range. It was specified as ±110°; the collision check narrowed it, and the URDF
carries the narrowed limit while the collision checker parses limits *from the URDF* rather
than holding a second copy.

**It cost us horizontal-plate tracing** (§5.4). The trade was made knowingly: an arm that
cannot trace a table is better than an arm that destroys its own wrist.

---

**Q10. Can it trace on a table?**

No. Best of 12 plate positions is **11.7 mm rms** against 0.020 on a vertical board.
**J2 saturates at +115°.** A 90° bent tool adapter was tried; then J2 *and* J5 both
saturate. It is a joint-range limit. The workaround costing nothing is to mount the arm on
its side, which makes the table a vertical board in the arm's own frame.

---

**Q11. The mass budget is 1450 g and the arm is 1584 g. Explain.**

The original figure counted structure, actuators, bearings and shafts. **It had no fastener
term** — 40 brass heat-set inserts, six M5 × 90 preload bolts and roughly a hundred screws,
244 g in total, 15 % of the arm and entirely absent. It was identified, quantified every fastener
from its own geometry, and put it in the gate as a **hard failure**, which is why the gate
currently reads NO-GO.

Available savings were costed: aluminium shims (−19 g), M4 × 80 preload bolts instead of
M5 × 90 (−41 g), titanium seam screws (−15 g), nylon washers (−3 g) — **78 g**, leaving
~56 g still over. Closing it entirely requires lightening the base, which invalidates its
FEA until re-run.

**The overage is reported, not absorbed, and the budget has not been quietly raised.**

---

**Q12. What if the bearing does not fit the printed pocket?**

That is the single largest risk and it has a dedicated test. The **fit coupon** carries
three pockets at Ø41.95 / 42.00 / 42.05, three insert-boss sizes and a seam sample; it
prints in 0.9 h and gates the ~19 h of parts that contain a bearing pocket. Whichever
diameter gives a firm thumb press sets `BRG_FIT`, everything re-exports, and the structural
parts print.

Plain PLA shrinks 0.40 % — **0.17 mm on a Ø42 bore, larger than the coupon's whole 0.10 mm
bracket** — so all three may come out tight. That is the coupon working: set XY compensation
≈ −0.15 mm and run it again. Two coupon prints are budgeted two coupon prints.

---

**Q13. On what basis is the FEA considered correct?**

It is not relied on alone. It is cross-checked against a **closed-form compliance budget**
written from first principles with every formula stated, and the two agree. The FEA is also
deliberately **part-wise, not assembly-level**, because assembly FEA needs joint stiffness
and bolt preload that are not known before hardware. §10.2 lists this as a limitation.

---

**Q14. Why offline IK? Is that not a limitation?**

For a repeated demonstration trajectory it is the correct choice: a least-squares solve per
waypoint inside the ROS node would block the executor, and the path is identical every run.
The node also publishes the solved path as a latched `JointTrajectory`, so a real controller
can consume it directly. **For reactive tasks — following a moving target — this would need
to become online IK**, and that is named in the future scope.

---

**Q15. What happens if a servo fails mid-trace?**

The arm is **single-string** — there is no redundancy, and it would stop. On a bench that is
acceptable. §8.1 lists redundancy as one of the eleven subsystems that must be replaced for
flight.

---

**Q16. Is this actually space-relevant, or is it a robot arm with a story attached?**

Three things transfer and each is defensible: the **kinematics and joint architecture**
(unaffected by material), the **docking principle** (capture tolerance solved as geometry,
which is how real docking hardware works), and the **verification method** (grade-
independent). Eleven of sixteen subsystems do not transfer, and §8.3 names every one.

It is preferable to state that than claim more.

---

**Q17. What is the weakest element of the work?**

**That backlash is assumed and not measured, and it dominates the error budget.** The
headline precision figure is therefore the least evidenced number in the document. It is
first in the measurement plan.

Second: the **fit coupon is unprinted**, so this does not yet know what this printer does to a
Ø42 bore.

---

**Q18. What would be done differently from the outset?**

Two things. **Interrogate the geometry from day one** — the parameter-checking gate passed
55/55 while three bearing pockets did not exist, and everything since has been reading the
exported STEP instead. And **put every material and print constant in one place from the
start**: E, density and yield were hardcoded in twelve files, so switching material meant
finding all twelve, and missing one would have left the gate silently mixing two materials.

---

## 10. Measurement plan

No part of this design has yet been built, so every figure in the preceding sections is
analytical or simulated. The measurements below are the ones that convert those figures
into evidence, and they are listed here in the form they will be executed.

Each carries a prediction **and a falsification condition**, both fixed in writing before
the measurement is taken. The prediction states what the analysis expects; the
falsification condition states the result that would show the analysis to be wrong. A test
that cannot fail confirms nothing, so a measurement with no falsification condition is not
recorded as a test. The final column is left blank and is completed when the measurement
is made.

### 10.1 Planned measurements

| # | measurement | method | prediction | falsified if | result |
| --- | --- | --- | --- | --- | --- |
| 1 | Ø42 bore fit | fit coupon, thumb press | Ø42.00 correct, or −0.15 XY comp | drops in free / will not start | ______ |
| 2 | **servo backlash** | dial indicator at the tool | 0.3–1.0° per joint, 1.9–2.2 mm at tip | < 0.5 mm or > 5 mm | ______ |
| 3 | sine trace accuracy | trace on paper, scan, fit | 1–3 mm rms | < 0.5 mm — backlash model wrong | ______ |
| 4 | docking latch | bench, by hand, roll J6 30° | latches and releases repeatably | lugs bind or release under load | ______ |
| 5 | static deflection | 300 g at full extension | 0.197 mm | > 0.30 mm — seam not closing | ______ |
| 6 | J2 torque and case temp | bus current + IR after 3 min | 2.74 N·m, 44 °C | stalls, or case > 60 °C | ______ |
| 7 | assembled mass | scale | 1584 ± 50 g | outside 1534–1634 g — volume or density model wrong | ______ |

### 10.2 Stated limitations

1. No hardware. Every number is analytical or simulated.
2. **FEA is part-wise, not assembly-level** — joint stiffness, bolt preload and contact are not modelled.
3. **Backlash is assumed, not measured**, and it dominates the error budget.
4. Servo case-screw positions are from a datasheet, not a measured servo.
5. Creep uses a Findley fit two orders of magnitude below its calibration range.
6. **Print anisotropy is not modelled** — layer adhesion is typically 40–70 % of in-plane strength; the FEA is isotropic.
7. The docking mate is verified as rigid-body geometry — no friction, compliance or misalignment dynamics.
8. The fluid line is not in any collision model; internal routing through the joints was measured as **impossible** (Ø3.6 mm free channel against a Ø6 tube).

---

## 11. Future scope

### 11.1 Immediate — weeks

| | |
| --- | --- |
| **Print and measure** | close all seven items in §10. Until then this is a design, not a machine. |
| **Joint-side encoders** | the single highest-value change. Magnetic encoders on each output shaft would take precision from ~2.2 mm to ~0.1 mm — a **20× improvement** for roughly 30 g and six I²C devices, and it addresses the one thing no structural change can. |
| **Fluid transfer** | the sealed target is designed and verified; the tube route is not. Internal routing is ruled out by measurement; external needs a designed path. |

### 11.2 Medium — months

**Online IK** for reactive tasks — following a moving target rather than replaying a solved
path. Requires the solve to leave the ROS node's executor thread.

**Wrist redesign** to recover J5 travel and with it horizontal-plate tracing. The constraint
is the J4-housing/J6-body collision; a re-proportioned yoke would buy back the range.

**Force/impedance control.** Docking with a *free-floating* target is a contact-dynamics
problem, not a position problem. This is the single biggest control-side gap between this
arm and a real servicer.

**Aluminium version.** Same geometry in Al 6061-T6: ≈1.13 kg, ~10× stiffer, no glass
transition at 60 °C. It also removes the mass overage entirely.

### 11.3 Long — the free-floating path

**Reactionless trajectory planning.** Once the base floats, arm motion rotates the platform.
Planning motions with zero net angular momentum — or compensating with reaction wheels — is
the defining problem of free-flying manipulation, and it cannot be studied on a bolted-down
arm. The maglev platform is what makes it accessible.

**Full refuelling sequence:** approach → capture in the cone → latch by J6 → seal → transfer
→ purge → unlatch → retreat, closed-loop and autonomous.

**Vacuum-compatible revision:** vented parts, dry-film bearing lubricant, outgassing-
qualified materials. This is a redesign, not a modification, and §8.3 says which eleven
subsystems it touches.

---

## 12. Supporting documents

| document | contents |
| --- | --- |
| `ARM450_PAPER.pdf` | the work as a research paper |
| `ARM450_REPORT.pdf` | full engineering report — FEA, creep, fatigue, stiffness |
| `ARM450_KINEMATICS.pdf` | DH derivation and the three-way cross-check |
| `ARM450_PREFLIGHT.pdf` | the eight-stage gate |
| `ARM450_DRAWINGS_ISO.pdf` / `_3VIEW.pdf` | dimensioned sheets, 19 parts |
| `ARM450_BUY.pdf` | procurement and print order |
| `ARM450_SLICING.pdf` | print settings, and the two that are not preferences |
| `ARM450_END_EFFECTORS.pdf` | gripper and docking probe |
| `ARM450_WORKSPACE.pdf` | reach and swept volume |
| `ARM450_LESSONS.pdf` | **the mistakes and the rules they produced** |
| `ARM450_LOG.pdf` | dated record of every change and why |
| `ARM450_COMMANDS.pdf` | every command, generated from the source |

**Reproduce everything:**

```bash
python3 preflight.py        # the 8-stage gate, one verdict
python3 check_ikpy.py       # kinematics, three ways
python3 verify_configs.py   # load suite × 5 payload configurations
python3 dim_audit.py        # every dimension, one at a time
python3 mate_check.py       # do mating parts line up
python3 final_audit.py      # printability
```
