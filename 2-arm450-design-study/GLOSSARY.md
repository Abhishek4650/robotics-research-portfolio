# ARM-450 — Glossary

Every term used across the ARM-450 documents, in plain language, with the number it
actually took in this project.

---

## 1. The verification words

### Gate
A **single program that runs every check and gives one verdict.** `preflight.py`. The idea
is that you never have to remember which of forty checks you last ran — you run the gate
and it tells you GO or NO-GO. Borrowed from manufacturing, where a "gate" is a point work
cannot pass until it is signed off.

### Check
One question with a yes/no answer and a stated limit. *"Is the worst safety factor at least
3?"* — not *"is it strong?"* A check you cannot fail is not a check.

### Stage
A group of related checks. ARM-450 has eight, run in order — design geometry, FEA, creep
and fatigue, stiffness, URDF, path tracing, simulation, end effectors. They are ordered so
the cheap checks fail first: there is no point running a 4-minute collision sweep if a bolt
hole is missing.

### Gate passed / GO
**Every hard check passed.** In ARM-450 it means: the features physically exist in the CAD,
nothing yields, the arm is stiff enough, the model matches the machine, the trajectory does
not collide, and the ROS side will work on hardware.

**What it does not mean:** that the arm works. Nothing has been printed. GO means *the
design is self-consistent and inside its own limits* — no more.

### NO-GO
At least one hard check failed. The gate names it and stops. ARM-450 currently reads NO-GO
on one item: assembled mass 1584 g against a 1450 g budget.

### Hard check vs note
A **hard** check failing means NO-GO. A **note** is reported but does not block — used where
the criterion is a preference rather than a limit.

### Pass / fail counts — "25 passed, 1 failure"
25 individual checks returned yes; 1 returned no. Not a percentage: the one failure is
enough for NO-GO regardless of the 25.

---

## 2. Geometry and CAD

### STEP
The **exact** CAD file. It stores real surfaces — *this face is a cylinder of radius 21,
axis along Z*. You can ask a STEP file questions: "is there a Ø42 hole here?" Used for every
verification in this project.

### STL
A **mesh** — the shape approximated by triangles. This is what the printer reads. It cannot
tell you "this is a cylinder", only "here are 20 000 triangles". Used for collision checks
and printing.

### B-rep (boundary representation)
The way a STEP file describes a solid: as the set of surfaces that bound it. The reason a
STEP can be *interrogated* and an STL can only be *measured*.

### Boolean
A CAD operation that combines shapes: **union** (add), **cut** (subtract), **intersect**.

### Silent no-op
**A boolean that succeeds while doing nothing.** Subtract a Ø42 pocket from a region that is
already empty air and the operation succeeds and changes nothing — no error, no warning.
This produced three missing bearing pockets in ARM-450 while a parameter check reported
55/55 passing. It is the single most important failure mode in the project.

### Solid / body
One connected lump of material. **A part should be exactly one.** A `.union()` of two shapes
that do not touch also "succeeds" — it just returns two separate solids. That is how the
docking probe came to have three latch lugs floating in mid-air.

### Watertight
A mesh with no holes in its surface — a sealed shell. A non-watertight mesh has no
well-defined inside, so its volume is meaningless and a slicer may refuse it.

### Fillet
A **rounded** internal or external corner. Removes the stress concentration a sharp corner
creates.

### Chamfer
A **flat 45° cut** across an edge. Not decorative on a bearing pocket: it is the **lead-in**
that lets a Ø42 bearing start square in a Ø42 hole. ARM-450 had these specified everywhere
and cut nowhere until 2026-08-25.

---

## 3. Fits and fasteners

### Interference / press fit
The hole is **smaller** than the part going in, so it grips. A bearing in its pocket.

### Clearance / slip fit
The hole is **larger**, so the part slides in. The horn adapter on the shaft — Ø30.20 on a
Ø30 shaft — where a roll pin, not the fit, carries the torque.

### Fit coupon
**A small test print that measures what your machine does to a critical dimension.** ARM-450's
carries three bearing pockets at Ø41.95 / 42.00 / 42.05, three insert-hole sizes and a seam
sample. It prints in 0.9 h and **gates roughly 19 h of parts** — press a bearing into each
and whichever gives a firm thumb press is the right diameter for your printer.

### Preload
**Deliberate clamping force applied before any working load.** The M5 through-bolt squeezes
the two bearings of a pair against each other, removing internal play. Without it the joint
has slack even with perfect bearings.

### Heat-set insert
A **brass threaded sleeve pressed into plastic with a soldering iron.** Gives a real metal
thread in a printed part — a screw threaded straight into PLA strips after a few cycles.

### Grip length
**How much material a screw passes through before it reaches the thread.** Getting it wrong
is invisible: an M3 × 12 in ARM-450's seam looked fully seated and engaged nothing, because
it stopped 2.5 mm short of the insert.

### Roll pin / spring pin
A **slotted, slightly oversized pin** that compresses on the way in and springs outward.
Zero clearance, therefore zero backlash — which matters on an arm already backlash-limited.

---

## 4. Kinematics

### DOF (degrees of freedom)
Independent ways to move. ARM-450 has 6 — the minimum to place a tool at **any position and
any orientation** in space.

### Forward kinematics (FK)
**Joint angles → where the tool is.** Always has exactly one answer.

### Inverse kinematics (IK)
**Where you want the tool → what joint angles get it there.** The hard direction: there may
be several answers, or none.

### DH table (Denavit–Hartenberg)
**A standard four-numbers-per-joint recipe for describing a robot's geometry.** Compact and
unambiguous, so two engineers with the same table build the same maths. ARM-450 uses the
*modified* (Craig) convention.

### Jacobian
**The matrix relating joint speeds to tool speed.** *"If I turn J2 at 1 rad/s, how fast and
in what direction does the tool move?"* Used by IK to decide which way to step.

### Velocity propagation
A way of **building** the Jacobian by carrying velocity from one link to the next, rather
than by nudging each joint and measuring — which stays exact where numerical differencing
degrades.

### Singularity
**A pose where the arm loses a direction of movement.** In ARM-450 the J4 and J6 roll axes
line up when J5 → 0, and only their *sum* matters — the solver can then wander wildly.
Handled by locking J6 and damping the redundant pair.

### TCP (tool centre point)
**The point the robot is commanded to.** ARM-450's is 450 mm from the base, at the J6 face.

### Flange
The **mounting face** where a tool bolts on. Not the same as the working point: a gripper's
fingers reach **42 mm past** the flange — a 42 mm error if you confuse them, and this
project made exactly that mistake.

### Workspace
**Everywhere the tool can reach.** Quoted as maximum reach (450 mm bare) and swept volume
(188 litres).

### Waypoint
One point on a path. ARM-450's sine trace is 240 of them.

---

## 5. Structural analysis

### FEA (finite element analysis)
**Chop the part into small blocks, solve for the stress in each.** Handles shapes too
complicated for a formula.

### Stress (σ)
Force per unit area, in **MPa**. How hard the material is being pulled or pushed *internally*.

### Yield
The stress at which the material **stops springing back** and stays bent. PLA ≈ 55 MPa.

### Safety factor (SF)
**Yield ÷ working stress.** How many times harder you could push before it deforms. ARM-450's
worst is **23** — meaning strength was never the binding constraint.

### Compliance
**The opposite of stiffness — how far the tool moves under load.** ARM-450: 0.197 mm at full
extension with 300 g.

### Creep
**Slow permanent sag under a load held for a long time.** Plastics do this at room
temperature; metals mostly do not. ARM-450's is 0.097 %/year — hence *park the arm folded
when idle, not extended.*

### Fatigue
**Failure from many small load cycles**, each far below what would break it once.

### S-N curve
The **stress-vs-number-of-cycles** relationship. Metals often have an *endurance limit* — a
stress below which they last forever. **Polymers do not**, so ARM-450 uses a life target
(10⁸ cycles) instead.

### Backlash
**Lost motion — the slack you feel when you reverse a gear before anything moves.** In
ARM-450 it is **≈2.24 mm at the tool** and dominates everything: it beats structural
deflection by about 20×.

> **If you remember one term, make it this one.** The whole precision story of this project
> is that the arm is *servo-limited, not structure-limited* — the plastic is not the problem.

### rms (root mean square)
A **typical** error across many points, giving more weight to large ones. Reported alongside
*max* because a path can have a good average and one bad excursion.

---

## 6. ROS and simulation

### ROS 2
The robot software framework. Programs are **nodes**; they communicate by publishing and
subscribing to named **topics**.

### Node
One running program. ARM-450 has `sine_node` (plays the trajectory) and
`robot_state_publisher` (works out where every link is).

### Topic
A named channel. `/joint_states` carries the six joint angles.

### URDF
**The robot's description file** — links, joints, axes, limits, meshes, masses. Everything
that needs to know the robot's shape reads it.

### TF
ROS's system for tracking **where every frame is relative to every other**, continuously.

### RViz
The 3-D viewer. **It only draws what the URDF says** — which is why an arm can be verified
with a gripper fitted and still display without one, as ARM-450's did.

### Latched (TRANSIENT_LOCAL)
A message **kept for late subscribers.** A controller that starts after the trajectory was
published still receives it.

### Collision-free
**No two parts of the arm occupy the same space at any point on the path** — checked on the
real triangle meshes, not simplified boxes. ARM-450: 0 of 240 waypoints.

### Exact mesh collision
Checking with the **actual part geometry**. The cheap alternative is bounding boxes, which
would have missed the 1 mm clearance at the base pedestal.

---

## 7. Printing

### Nozzle
The hole the plastic comes out of. 0.4 or 0.6 mm.

### Layer height
How thick each printed layer is. **0.20 mm for ARM-450, and not a preference:** the seam
tongue is 1.6 mm and must be a whole number of layers, or the slicer rounds it up and the
tongue bottoms out before the two halves can close.

### Perimeter / wall line
The outlines the printer traces round each layer. **Four perimeters × 0.62 mm = 2.48 mm**,
which consumes ARM-450's 2.4 mm wall entirely — that is what makes it solid.

### Infill
The internal lattice filling hollow space. 40 % typical; **100 % where teeth are 2 mm**, or
they print as hollow shells and shear off.

### Shrinkage
Plastic **contracts as it cools**, so a printed hole comes out smaller than drawn. Plain PLA
0.40 %, carbon-filled 0.15 % — **0.17 mm on a Ø42 bore**, which is why the coupon exists.

### XY compensation
A slicer setting that **grows or shrinks every hole** to correct for shrinkage. Set from the
coupon result, not guessed.

### Support
Scaffolding printed under overhangs. **ARM-450 uses none on load paths** — support leaves a
rough weak surface, and you do not want that on a bearing seat.

### Brim
A flat skirt around the first layer for grip. Used on tall narrow parts.

---

## 8. This project's own vocabulary

### Interrogate the geometry
**Ask the exported STEP or STL what it contains, rather than trusting the numbers used to
draw it.** The central rule of the whole verification method.

### Parameter check vs geometry check
A parameter check reads `BEARING_POCKET_D == 42.0` — your *intention*. A geometry check
opens the file and looks for a Ø42 cylindrical face — the *result*. **They disagreed here,
and the parameter check was the one that passed.**

### Capture tolerance
**How far off you can be and still succeed.** ARM-450's docking cone accepts ±9.75 mm against
≈2.2 mm of arm error — docking made feasible by *geometry* rather than by accuracy.

### Bayonet
A **push-and-twist** coupling — like a camera lens. Used twice here: the tool quick-change,
and the docking latch that J6 turns.

### Payload vs arm mass
**Payload** is what the arm carries; **arm mass** is the arm itself. A fitted tool is payload
— which is why a 75 g gripper does not count against the 1450 g budget, but 244 g of bolts
does.

### Falsification condition
**What result would prove the prediction wrong**, written down *before* the test. "Backlash
1.9–2.2 mm; falsified if under 0.5 mm." Without it a prediction can be quietly retro-fitted
to whatever happens.
