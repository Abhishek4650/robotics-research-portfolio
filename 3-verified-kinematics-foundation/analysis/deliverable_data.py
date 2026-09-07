#!/usr/bin/env python3
"""
Shared content for the R_sine deliverables (PPT + detailed PDF + speaker guide).

One source of truth so all three documents stay consistent. Provides:
  * get_metrics()  — live-computed verification numbers.
  * DH_ROWS        — the identified modified-DH table.
  * SECTIONS       — ordered slide/section content (title, bullets, figure,
                     detailed prose, speaker script, anticipated questions).
"""
import os
import sys
import numpy as np

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
from Rsine import kinematics as K                        # noqa: E402
from Rsine.ik import DLSIKSolver                          # noqa: E402
from Rsine.sine_path import DrawingPlane, sine_waypoints  # noqa: E402

FIG = os.path.join(PKG, "figures")

DH_ROWS = [
    ("1", "0",   "0",       "0.13056", "+90"),
    ("2", "+90", "0",       "0",       "-90"),
    ("3", "0",   "-0.1104", "0",       "0"),
    ("4", "0",   "-0.096",  "0.06062", "-90"),
    ("5", "+90", "0",       "0.07318", "+90"),
    ("6", "-90", "0",       "0.0456",  "0"),
]


def get_metrics():
    rng = np.random.default_rng(0)
    fk_err = jac_err = 0.0
    for _ in range(600):
        q = rng.uniform(K.JOINT_LIMITS[:, 0], K.JOINT_LIMITS[:, 1])
        fk_err = max(fk_err, np.abs(K.forward_kinematics(q) - K.urdf_fk(q)).max())
        # finite-difference Jacobian
        J = np.zeros((6, 6)); T0 = K.forward_kinematics(q); p0, R0 = T0[:3, 3], T0[:3, :3]
        for i in range(6):
            dq = np.zeros(6); dq[i] = 1e-6; T1 = K.forward_kinematics(q + dq)
            J[:3, i] = (T1[:3, 3] - p0) / 1e-6
            dR = (T1[:3, :3] - R0) / 1e-6 @ R0.T
            J[3:, i] = [dR[2, 1], dR[0, 2], dR[1, 0]]
        jac_err = max(jac_err, np.abs(K.jacobian(q) - J).max())

    plane = DrawingPlane.vertical_board(x=0.14, z_center=0.21)
    positions, _ = sine_waypoints(plane, 0.05, 0.20, 2.0, 120)
    solver = DLSIKSolver(lam=0.04, max_iters=200, tol=1e-6)
    best = None
    for R_t in plane.candidate_orientations():
        Q, infos = solver.solve_trajectory(positions, R_t)
        w = max(i["pos_err"] for i in infos)
        if best is None or w < best[0]:
            best = (w, Q, infos)
    ik_track, Q, infos = best
    ik_iters = float(np.mean([i["iters"] for i in infos]))

    P = np.array([K.ee_position(rng.uniform(K.JOINT_LIMITS[:, 0], K.JOINT_LIMITS[:, 1]))
                  for _ in range(20000)])
    max_reach = float(np.linalg.norm(P, axis=1).max())
    max_horiz = float(np.linalg.norm(P[:, :2], axis=1).max())
    home = K.ee_position(np.zeros(6))

    return {
        "fk_err": fk_err, "jac_err": jac_err,
        "ik_track_mm": ik_track * 1000.0, "ik_iters": ik_iters,
        "max_reach": max_reach, "max_horiz": max_horiz,
        "home": home,
    }


# Section content. figure paths are basenames in figures/.
SECTIONS = [
    dict(
        title="R_sine — Drawing a Sine Wave with a myCobot 280",
        subtitle="Analytical kinematics + inverse kinematics on a 6-DOF arm (ROS 2)",
        bullets=[
            "Goal: make a 6-DOF myCobot 280 trace a sine wave on a surface",
            "Built from first principles: Modified-DH kinematics, velocity-propagation Jacobian, damped-least-squares IK",
            "Every result cross-checked against an independent library (ikpy)",
            "Runs in ROS 2 (Jazzy); visualized in RViz",
        ],
        figure="fig_home_pose.png",
        detail=(
            "This project drives a myCobot 280 — a small six-degree-of-freedom robotic "
            "arm — so that its end-effector traces a sine wave on a flat surface such as a "
            "vertical board. Rather than relying only on a black-box library, the kinematics "
            "are derived from first principles using the Modified (Craig) Denavit-Hartenberg "
            "convention, a velocity-propagation Jacobian, and a damped-least-squares inverse "
            "kinematics solver. Every stage is verified numerically, and an independent "
            "solver (ikpy) is used as a cross-check. The system runs as clean, separate "
            "ROS 2 nodes and is visualized in RViz."),
        say=(
            "Start here. 'My project makes a 6-DOF myCobot 280 arm draw a sine wave on a "
            "board. What makes it rigorous is that I derived the kinematics myself using the "
            "modified DH method from our course, built the Jacobian by velocity propagation, "
            "and solved the inverse kinematics with damped least squares — then I verified "
            "every piece against an independent library. It all runs in ROS 2.'"),
        questions=[
            ("Why a sine wave?",
             "It is a smooth, continuously-curving path that exercises all axes and makes "
             "tracking accuracy easy to see — a good test of the full FK/Jacobian/IK pipeline."),
        ],
    ),
    dict(
        title="Objective and Method",
        subtitle=None,
        bullets=[
            "Forward kinematics: where is the tool, given the joint angles?",
            "Jacobian: how tool velocity relates to joint velocities",
            "Inverse kinematics: which joint angles reach a desired tool pose?",
            "Pipeline: sine path -> IK per point -> joint trajectory -> RViz",
        ],
        figure=None,
        detail=(
            "The work is organized around the three classical kinematics problems. Forward "
            "kinematics maps joint angles to the tool pose. The Jacobian linearly relates "
            "joint velocities to tool velocity and is the engine of the IK. Inverse "
            "kinematics finds joint angles that achieve a desired tool pose. The pipeline "
            "generates the sine as a sequence of Cartesian targets, solves IK for each, and "
            "streams the resulting joint trajectory to the robot in RViz while tracing the "
            "actual drawn path for comparison."),
        say=(
            "'The project is built around the three standard problems: forward kinematics, "
            "the Jacobian, and inverse kinematics. I generate the sine as a list of 3D "
            "target points, solve inverse kinematics for each point to get joint angles, "
            "and play that back on the robot.'"),
        questions=[
            ("What is the difference between forward and inverse kinematics?",
             "Forward: joints -> pose (one unique answer). Inverse: pose -> joints "
             "(can have several answers or none if unreachable)."),
        ],
    ),
    dict(
        title="Modified (Craig) DH Convention",
        subtitle="Four parameters relate each pair of link frames",
        bullets=[
            "alpha_(i-1): link twist — angle between successive joint axes",
            "a_(i-1): link length — offset between successive joint axes",
            "d_i: link offset along the joint axis",
            "theta_i: joint angle (the variable for a revolute joint)",
            "Link transform: Rotx(alpha)·Transx(a)·Rotz(theta)·Transz(d)",
        ],
        figure=None,
        detail=(
            "The Modified DH convention attaches a frame to each link and describes the "
            "transform between consecutive frames with four parameters: the link twist "
            "alpha, link length a, link offset d, and joint angle theta. For a revolute "
            "joint theta is the variable and the others are fixed. Each link transform is "
            "Rotx(alpha_(i-1)) Transx(a_(i-1)) Rotz(theta_i) Transz(d_i); multiplying the "
            "six link transforms gives the full forward kinematics. This is exactly the "
            "matrix form used in the course reference notes."),
        say=(
            "'I use the modified DH convention from class. Each joint is described by four "
            "numbers — twist, length, offset, and angle — and each link transform is this "
            "product of a rotation and translation about x then z. Multiplying them gives "
            "the forward kinematics.'"),
        questions=[
            ("Why modified DH and not standard DH?",
             "Modified DH places the frame at the proximal joint, which many textbooks "
             "(Craig) use; it is the convention in my reference notes. Both are valid."),
        ],
    ),
    dict(
        title="Identified DH Table (verified to 1e-15 m)",
        subtitle="Fitted from the URDF, then checked against it",
        bullets=[
            "URDF link frames are not on DH axes -> table was identified numerically",
            "Fit (alpha, a, d, theta-offset) so DH-FK reproduces URDF-FK",
            "Residual is machine-zero: max |DH-FK - URDF-FK| ~ 1e-15 m",
            "Physically clean 'concentrated' convention chosen for readability",
        ],
        figure=None,
        table=DH_ROWS,
        detail=(
            "Because the URDF's link frames are not placed on DH axes, the DH parameters "
            "were identified: the four parameters per joint were fitted so that the DH "
            "forward kinematics reproduces the URDF forward kinematics, and the residual is "
            "at machine precision (about 1e-15 m over thousands of random configurations). "
            "Where parallel axes leave a gauge freedom in the offsets, the physically clean "
            "'concentrated' convention was chosen so the table reads naturally."),
        say=(
            "'This table is the heart of the model. The official URDF doesn't come with DH "
            "parameters, so I identified them by fitting the four numbers per joint until my "
            "DH forward kinematics matched the URDF exactly — the error is 1e-15 meters, "
            "essentially machine zero. So the table is provably correct, not guessed.'"),
        questions=[
            ("How do you know the table is right?",
             "Its forward kinematics matches the manufacturer URDF to ~1e-15 m over 5000 "
             "random joint configurations, and also matches ikpy to ~1e-16 m."),
        ],
    ),
    dict(
        title="Forward Kinematics",
        subtitle="Product of the six link transforms",
        bullets=[
            "T_EE = T1·T2·T3·T4·T5·T6",
            "Home pose (all joints 0): tool at (0.061, 0.046, 0.410) m",
            "Independently confirmed by ikpy's FK (~1e-16 m)",
        ],
        figure="fig_home_pose.png",
        detail=(
            "Multiplying the six modified-DH link transforms yields the base-to-tool "
            "transform. At the home configuration (all joint angles zero) the tool sits at "
            "(0.061, 0.046, 0.410) m. The figure shows the arm at home with a small "
            "coordinate triad drawn at every link frame; ikpy's forward kinematics confirms "
            "the same tool position to about 1e-16 m."),
        say=(
            "'Forward kinematics is just the product of the six link transforms. Here's the "
            "arm at its home pose with the frame at every joint. When I feed the same joints "
            "to ikpy, it lands on the identical position — so my FK is correct.'"),
        questions=[
            ("What frame is the tool?",
             "link6_flange — the output flange where a pen or gripper mounts."),
        ],
    ),
    dict(
        title="Jacobian — Velocity Propagation",
        subtitle="Built frame-by-frame, verified against finite differences",
        bullets=[
            "Propagate angular & linear velocity outward: base -> tool",
            "For revolute joints reduces to columns [ z_i x (p_e - p_i) ; z_i ]",
            "Maps joint rates to tool twist: [v; omega] = J(q) q_dot",
            "Verified vs numerical Jacobian to ~2.5e-7",
        ],
        figure=None,
        detail=(
            "The Jacobian is built with the velocity-propagation method: angular and linear "
            "velocities are carried outward from the base to the tool, frame by frame. For "
            "an all-revolute arm this collapses to the geometric Jacobian whose i-th column "
            "is [ z_i x (p_e - p_i) ; z_i ], where z_i and p_i are joint i's axis and origin "
            "and p_e is the tool position. It maps joint rates to the tool twist and is "
            "verified against a finite-difference Jacobian to about 2.5e-7."),
        say=(
            "'For the Jacobian I used the velocity-propagation method from the notes — you "
            "push the angular and linear velocity outward joint by joint. For revolute "
            "joints it simplifies to this cross-product form. I checked it against a "
            "numerical Jacobian and it agrees to seven decimal places.'"),
        questions=[
            ("What is the Jacobian used for here?",
             "It drives the inverse kinematics — each IK step multiplies a damped inverse of "
             "J by the pose error to update the joint angles."),
        ],
    ),
    dict(
        title="Reachable Workspace (verified)",
        subtitle="Where the tool can go — the sine sits well inside it",
        bullets=[
            "Sampled by FK over 40,000 valid joint configurations",
            "Max 3D reach 0.427 m; max horizontal reach 0.301 m",
            "Cross-checked against ikpy FK to ~1.7e-16 m",
            "The demo sine lies safely inside the reachable set",
        ],
        figure="fig_reachable_workspace.png",
        detail=(
            "The reachable workspace is the set of tool positions attainable with some "
            "joint configuration inside the URDF limits. Sampling forward kinematics over "
            "40,000 valid configurations gives the cloud shown from three views. The maximum "
            "3D reach is 0.427 m and the maximum horizontal reach 0.301 m, both within the "
            "geometric bound. The cloud is verified against ikpy's independent FK to about "
            "1.7e-16 m, and the demo sine (red) clearly lies inside the reachable region."),
        say=(
            "'This is the reachable workspace — every point the tool can reach — from three "
            "views. The red sine is my drawing target, and you can see it sits comfortably "
            "inside. This is also how I chose where to put the board: somewhere the arm can "
            "comfortably reach.'"),
        questions=[
            ("Why not draw further out or on the table?",
             "This is a small arm; keeping the pen perpendicular to a far or downward surface "
             "exceeds its reach. The vertical board close in is fully reachable."),
        ],
    ),
    dict(
        title="Inverse Kinematics — Damped Least Squares",
        subtitle="Jacobian-based, robust near singularities",
        bullets=[
            "q_(k+1) = q_k + J^T (J J^T + lambda^2 I)^(-1) e",
            "e = [position error ; orientation error] (6-vector)",
            "Damping lambda keeps updates stable near singularities",
            "~16 iterations per waypoint; solutions clamped to joint limits",
        ],
        figure=None,
        detail=(
            "Inverse kinematics is solved iteratively with damped least squares "
            "(Levenberg-Marquardt): each step adds J^T (J J^T + lambda^2 I)^(-1) e to the "
            "joint vector, where e stacks the position and orientation error between the "
            "current and target tool poses. The damping term lambda keeps the update "
            "well-conditioned near singularities at the cost of a tiny steady-state error. "
            "The solver converges in roughly 16 iterations per waypoint and clamps every "
            "solution to the URDF joint limits."),
        say=(
            "'I solve inverse kinematics with damped least squares. Each iteration nudges "
            "the joints by a damped inverse of the Jacobian times the pose error. The "
            "damping is what keeps it stable near singularities. It converges in about 16 "
            "steps per point and always respects the joint limits.'"),
        questions=[
            ("Why damped, not the plain Jacobian inverse?",
             "Near singularities the plain inverse blows up. Damping trades a negligible "
             "position error for numerical stability."),
        ],
    ),
    dict(
        title="Configurable Drawing Plane",
        subtitle="Same code, any surface",
        bullets=[
            "Plane defined by origin + two in-plane axes (advance u, wave v)",
            "Pen kept perpendicular to the surface (flange z along the normal)",
            "Solver auto-picks the reachable pen direction (+/- normal)",
            "Vertical board or flat plate — only the parameters change",
        ],
        figure=None,
        detail=(
            "The drawing surface is fully parameterized by an origin and two in-plane axes: "
            "the advance direction the pen travels along and the wave direction the sine "
            "oscillates along. The target orientation keeps the pen perpendicular to the "
            "surface. Because a small arm can only face a surface one way, the solver tries "
            "both perpendicular pen directions and keeps whichever is reachable. Switching "
            "from a vertical board to a flat plate is only a change of parameters, not code."),
        say=(
            "'The plane is general. I define it by an origin and two axes — one to advance "
            "along, one for the wave. The pen is kept perpendicular to the surface. So the "
            "same code draws on a vertical board or a flat plate; I just change the "
            "parameters at launch.'"),
        questions=[
            ("Could it draw a different shape?",
             "Yes — only the path generator changes; the FK/Jacobian/IK pipeline is the same."),
        ],
    ),
    dict(
        title="Result — Drawing the Sine",
        subtitle="Tool traces the target to sub-millimetre accuracy",
        bullets=[
            "20 cm wide, 10 cm tall, 2-cycle sine on a vertical board",
            "Pen held perpendicular to the board throughout",
            "Drawn path overlays the target essentially perfectly",
        ],
        figure="fig_draw_montage.png",
        detail=(
            "The demo draws a 20 cm wide, 10 cm tall, two-cycle sine on a vertical board at "
            "x = 0.14 m. The montage shows the arm progressing through the trajectory as the "
            "drawn line grows; the tool holds the pen perpendicular to the board the whole "
            "time and the drawn path overlays the target curve to sub-millimetre accuracy."),
        say=(
            "'And here's the result — the arm drawing the two-cycle sine, shown as a "
            "progression. The red line is what the tool actually traces. It matches the "
            "intended curve to under a millimetre, and the pen stays perpendicular to the "
            "board the entire time.'"),
        questions=[
            ("Is this simulation or the real robot?",
             "Simulation in RViz with the exact robot model; the design is hardware-ready "
             "since it uses the real URDF joint names and limits."),
        ],
    ),
    dict(
        title="Accuracy and Verification",
        subtitle="Numbers behind the result",
        bullets=[
            "DH-FK vs URDF-FK: ~1e-15 m",
            "Velocity-propagation Jacobian vs finite differences: ~2.5e-7",
            "Analytical IK tracking error: ~0.001 mm",
            "ikpy cross-check on every waypoint: ~0.001 mm",
        ],
        figure="fig_tracking_error.png",
        detail=(
            "Each stage is quantified. The DH forward kinematics matches the URDF to about "
            "1e-15 m; the velocity-propagation Jacobian matches finite differences to about "
            "2.5e-7; the analytical IK tracks the sine to about 0.001 mm; and an independent "
            "ikpy cross-check on every waypoint agrees to about 0.001 mm. The plotted error "
            "stays far below one millimetre across the whole trajectory."),
        say=(
            "'These are the numbers I'd stake the project on. Forward kinematics is exact to "
            "machine precision, the Jacobian to seven digits, and the drawn path tracks to "
            "about a micron. The error plot stays a thousand times under a millimetre.'"),
        questions=[
            ("0.001 mm seems too good — is it real?",
             "It is the numerical tracking error of the solver in simulation. On real "
             "hardware, encoder resolution and calibration would dominate; this bounds the "
             "algorithm's own contribution."),
        ],
    ),
    dict(
        title="Analytical vs ikpy",
        subtitle="Independent methods, same answer",
        bullets=[
            "My analytical FK/Jacobian/IK vs the ikpy library",
            "All agreement checks are far below 1 mm (log scale)",
            "Both solvers trace the same sine",
        ],
        figure="fig_method_comparison.png",
        detail=(
            "To defend the results, the analytical method is compared against ikpy, an "
            "independent kinematics library. The bar chart (log scale) shows every "
            "agreement check sitting far below one millimetre, the overlay shows both "
            "solvers tracing the same sine, and the per-waypoint error confirms both are "
            "sub-millimetre. The two entirely independent implementations agree, which is "
            "strong evidence the kinematics are correct."),
        say=(
            "'Finally, a sanity check against an independent library, ikpy. Every comparison "
            "— forward kinematics, Jacobian, inverse kinematics — agrees far below a "
            "millimetre, and both methods draw the same sine. Two independent "
            "implementations agreeing is my strongest evidence the math is right.'"),
        questions=[
            ("If ikpy exists, why write your own?",
             "To understand and control the math, handle orientation and limits my way, and "
             "have a solver that matches the course methodology — with ikpy only as a check."),
        ],
    ),
    dict(
        title="ROS 2 Architecture",
        subtitle="Clean, separate Python nodes",
        bullets=[
            "kinematics.py / ik.py / sine_path.py — pure math, no ROS",
            "sine_ik_node — precomputes IK, streams /joint_states at 30 Hz",
            "path_tracer_node — accumulates the real drawn line from TF",
            "One launch file -> robot_state_publisher + nodes + RViz",
        ],
        figure=None,
        detail=(
            "The system is organized as clean, single-responsibility ROS 2 (Jazzy) nodes in "
            "Python. The math lives in ROS-free modules (kinematics, ik, sine_path) so it "
            "can be tested and reused. The sine_ik_node precomputes the IK trajectory and "
            "streams joint states at 30 Hz; the path_tracer_node listens to TF and "
            "accumulates the real drawn line. A single launch file starts "
            "robot_state_publisher, both nodes, and RViz, with the sine fully parameterized."),
        say=(
            "'Software-wise, the math is in plain Python modules with no ROS, so I can unit-"
            "test them. Then two small ROS nodes: one streams the joint trajectory, the "
            "other records the drawn line. A single launch file brings up the robot and RViz."),
        questions=[
            ("Why precompute the trajectory instead of solving live?",
             "The sine is fixed, so solving once and replaying is smooth and cheap; live IK "
             "would just repeat the same work every loop."),
        ],
    ),
    dict(
        title="Conclusion and Future Work",
        subtitle=None,
        bullets=[
            "First-principles kinematics, fully verified, drawing a sine in ROS 2",
            "General plane + shape: extensible beyond a sine on a board",
            "Next: run on the physical myCobot 280; add a real pen and contact",
            "Next: Gazebo physics; trajectory timing / velocity limits",
        ],
        figure="fig_sine_3d.png",
        detail=(
            "The project delivers a fully verified, first-principles kinematics pipeline "
            "that draws a sine wave with a myCobot 280 in ROS 2, with an independent cross-"
            "check at every stage. The plane and path are general, so other surfaces and "
            "shapes follow with minimal change. Natural next steps are running on the "
            "physical robot with a real pen, adding Gazebo physics, and shaping the "
            "trajectory timing to respect joint velocity limits."),
        say=(
            "'To conclude: I built the kinematics from scratch, verified every stage, and "
            "used it to draw a sine on a board in ROS 2. It generalizes to other surfaces "
            "and shapes. Next I'd move to the physical arm with a real pen and add Gazebo "
            "physics. Thank you — happy to take questions.'"),
        questions=[
            ("What was the hardest part?",
             "Identifying a correct, physically clean DH table from a URDF that isn't in DH "
             "form — solved by fitting and verifying to machine precision."),
        ],
    ),
]
