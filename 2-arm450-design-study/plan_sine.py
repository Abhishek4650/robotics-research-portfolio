"""
Solve the sine trajectory offline and save it for the RViz node.

IK runs here, not in the node: a least-squares solve per waypoint would block
the ROS executor. The continuity term (w_cont) is essential — without it the
J4/J6 wrist singularity makes the solution jump up to 45 deg between adjacent
waypoints and the arm would appear to snap around in RViz.
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.expanduser("~/ros2_ws/src/Rsine"))
from Rsine.sine_path import DrawingPlane, sine_waypoints
from sine_check import ik, fk_full

# Chosen after sweeping board distance, height and continuity weight:
#   x = 300 mm gives 0.194 mm rms with 1.43 deg pen tilt and 1.94 deg max joint
#   step. Closer boards trace worse because the IK lands on a poorer branch.
# THERMAL: this pose runs J2 at 1.49 N.m, whose STEADY-STATE case temp is 105 C
#   -- but the servo time constant is Rth*Cth = 9 min, and a trace takes ~10 s.
#   After 3 minutes of back-to-back tracing the case is still 44 C. The thermal
#   limit applies to HOLDING a loaded pose, not to drawing.
BOARD_X, AMP, LEN, N = 0.30, 0.04, 0.14, 240
CYCLES = 2.0
# 2026-08-21: 0.20 -> 0.22. At 0.20 the exact-mesh check on the SOLVED PATH
# (not a random joint sweep) found the upper arm grazing the base pedestal on
# 30 % of waypoints -- 1.27 mm of penetration at the top rim. Raising the trace
# 20 mm clears every pose AND improves accuracy 14x, because the IK stops
# working near the J2 limit: 0.194 -> 0.014 mm rms.
Z_CENTER = 0.22
W_CONT = 0.01
# ORIENTATION WEIGHT. 0.05 is right for a bare flange, where position and
# orientation are almost independent. With a TOOL fitted they are not: the tip
# sits `tip` metres along the tool axis, so tilting the wrist moves the tip.
# At 0.05 the solver happily tilts 21 deg to shave position error, which on
# hardware means the gripper meets the board at an angle. Tools get W_ORI_TOOL.
W_ORI = 0.05
W_ORI_TOOL = 2.0


def _collides(q):
    """Exact mesh collision at one pose. Imported lazily so the pure-kinematics
    path stays fast for callers that do not need it."""
    import assemble as _A
    import interference as _I
    parts, _ = _A.build(*q)
    return bool(_I.check(parts))


# How far the working point sits past the J6 tool face, per fitted tool.
# Measured off assemble.build() by workspace._measure_tool_reach(); repeated
# here as metres so plan_sine has no import-time dependency on the assembly.
TOOL_TIP = {"none": 0.0, "pen": 0.030, "gripper": 0.042, "dock": 0.036}


def plan(board_x=BOARD_X, amp=AMP, length=LEN, n=N, w_cont=W_CONT,
         cycles=CYCLES, z_center=Z_CENTER, avoid_collision=False, tool="none",
         w_ori=W_ORI, tip_ik=False):
    """Solve the sine.

    `tool` is the fitted end effector. Its working point sits TOOL_TIP[tool]
    metres past the J6 flange, and that changes what "the board is at 300 mm"
    means:

      tip_ik=False (default) -- the IK drives the FLANGE onto `board_x`, and the
        tool tip therefore traces board_x + reach. This is the shipped
        trajectory, unchanged and already verified collision-free; fitting a
        gripper just means the board goes 42 mm further out. Accuracy at the
        TIP: 0.055 mm rms.

      tip_ik=True -- the IK drives the TOOL TIP onto `board_x`, putting the
        flange at board_x - reach. Use this when the board position is fixed and
        cannot move. It is a harder problem: the flange ends up 42 mm closer to
        the base, where the IK lands on a poorer branch, so it needs a higher
        trace (z_center 0.26) and a stronger orientation weight to match.

    Either way `P` -- the path published as the RViz marker -- is the TIP path
    when a tool is fitted. Drawing the flange path with a gripper on is what put
    the yellow curve 42 mm behind the fingers.
    """
    pl = DrawingPlane.vertical_board(x=board_x, z_center=z_center)
    pos, _ = sine_waypoints(pl, amplitude=amp, length=length, n_points=n,
                            cycles=cycles)
    pen = pl.n
    reach = TOOL_TIP.get(tool, 0.0)
    tip = reach if tip_ik else 0.0        # what the IK controls
    q = np.array([0., 0.9, -1.4, 0., -0.5, 0.])
    # COLLISION-AWARE ACQUISITION.
    # The IK is purely kinematic: it returns a pose that reaches the target and
    # says nothing about whether the arm can physically get there. At the shipped
    # placement it lands on a branch with J2 near 109 deg, which swings the upper
    # arm down into the base pedestal -- 30 % of the path collided on exact mesh
    # geometry while reporting 0.19 mm rms.
    # Fix the BRANCH at the first waypoint: try a bank of postures, keep only
    # those that reach AND clear, then track from there. Tracking keeps the rest
    # of the path on the same branch, so one good choice fixes all 240 poses.
    if avoid_collision:
        BANK = [np.array([0., 0.9, -1.4, 0., -0.5, 0.]),
                np.array([0., 0.5, -1.9, 0., 0.5, 0.]),
                np.array([0., 0.4, -1.6, 0., 0.9, 0.]),
                np.array([0., 0.7, -2.1, 0., 0.7, 0.]),
                np.array([0., 0.3, -1.2, 0., 1.0, 0.]),
                np.array([0., 1.1, -2.2, 0., 0.3, 0.])]
        best = None
        for s0 in BANK:
            qc, okc, ec = ik(np.asarray(pos[0]), pen, s0, w_cont=w_cont,
                             track=True, tip=tip, w_ori=w_ori)
            if ec > 2e-4:
                continue
            if _collides(qc):
                continue
            if best is None or ec < best[1]:
                best = (qc.copy(), ec)
        if best is not None:
            q = best[0]
    Q, P, E, TILT = [], [], [], []
    for k, p in enumerate(pos):
        # first waypoint: acquire with multiple seeds. after that: TRACK.
        q, ok, e = ik(np.asarray(p), pen, q, w_cont=w_cont,
                      track=(k > 0) or avoid_collision, tip=tip, w_ori=w_ori)
        pa, z = fk_full(q)
        # P is what gets drawn as the path marker in RViz, so it must be the
        # point that actually traces -- the tool tip when one is fitted, or the
        # flange when it is not. Publishing the flange path with a gripper on
        # is what made the marker float 42 mm behind the fingers.
        pa = pa + z * reach
        Q.append(q.copy()); P.append(pa.copy()); E.append(e * 1000)
        TILT.append(np.degrees(np.arccos(np.clip(np.dot(z, pen), -1, 1))))
    return np.array(Q), np.array(P), np.array(E), np.array(TILT), np.array(pos)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(
        description="Re-solve the ARM-450 sine trajectory. "
                    "Changing amplitude or cycles REQUIRES this step -- the "
                    "ROS node only replays a solved file, it does not do IK.")
    ap.add_argument("--amplitude", type=float, default=AMP,
                    help=f"sine amplitude in m (default {AMP})")
    ap.add_argument("--cycles", type=float, default=CYCLES,
                    help=f"number of full sine cycles (default {CYCLES})")
    ap.add_argument("--span", type=float, default=LEN,
                    help=f"trace length along the board in m (default {LEN})")
    ap.add_argument("--board-x", type=float, default=BOARD_X,
                    help=f"board distance from base in m (default {BOARD_X})")
    ap.add_argument("--points", type=int, default=N,
                    help=f"waypoints (default {N}); more = smoother, slower solve")
    ap.add_argument("--z-center", type=float, default=Z_CENTER,
                    help=f"trace centre height in m (default {Z_CENTER}). "
                         "THE lever for large amplitude: at 0.20 the shoulder J2 "
                         "sits on its +115 deg stop, so amplitude past 40 mm "
                         "degrades. 0.17 frees it -- 80 mm amplitude solves to "
                         "0.39 mm rms there vs 1.99 mm at 0.20.")
    ap.add_argument("--tool", default="none",
                    choices=sorted(TOOL_TIP),
                    help="fitted end effector. Its working point is past the J6 "
                         "flange (gripper 42 mm, dock 36, pen 30), so the board "
                         "the TIP traces is that much further out than --board-x "
                         "unless you also pass --tip-ik.")
    ap.add_argument("--tip-ik", action="store_true",
                    help="drive the TOOL TIP onto --board-x instead of the "
                         "flange. Use when the board cannot move. Harder solve "
                         "-- try --z-center 0.26 --w-ori 0.2 with a gripper.")
    ap.add_argument("--w-ori", type=float, default=W_ORI,
                    help=f"orientation weight (default {W_ORI}). Raise it with "
                         "--tip-ik: a tilt moves the tip, so at 0.05 the solver "
                         "trades 21 deg of tilt for position error.")
    ap.add_argument("--out", default="~/ros2_ws/src/arm450_sine/sine_traj.npz")
    A = ap.parse_args()
    BOARD_X, AMP, LEN, N = A.board_x, A.amplitude, A.span, A.points
    CYCLES = A.cycles
    Q, P, E, T, TGT = plan(board_x=A.board_x, amp=A.amplitude, length=A.span,
                           n=A.points, cycles=A.cycles, z_center=A.z_center,
                           tool=A.tool, tip_ik=A.tip_ik, w_ori=A.w_ori)
    reach = TOOL_TIP[A.tool]
    board_at = A.board_x if A.tip_ik else A.board_x + reach
    step = np.degrees(np.abs(np.diff(Q, axis=0)))
    out = os.path.expanduser(A.out)
    np.savez(out, Q=Q, P=P)
    print(f"SINE TRAJECTORY  board x={BOARD_X*1000:.0f} mm, "
          f"amp {AMP*1000:.0f} mm, span {LEN*1000:.0f} mm, "
          f"{CYCLES:g} cycles, z {A.z_center*1000:.0f} mm, {len(Q)} waypoints\n")
    print(f"  position error   rms {np.sqrt((E**2).mean()):6.3f} mm   "
          f"max {E.max():6.3f} mm")
    print(f"  pen tilt         rms {np.sqrt((T**2).mean()):6.3f} deg  "
          f"max {T.max():6.3f} deg")
    print(f"  largest joint step between waypoints: {step.max():.2f} deg "
          f"({'smooth' if step.max() < 5 else 'CHECK'})")
    print(f"  joint travel: " + "  ".join(
        f"J{i+1} {np.degrees(np.ptp(Q[:,i])):.0f}d" for i in range(6)))
    if reach:
        print(f"\n  TOOL: {A.tool}, working point {reach*1000:.0f} mm past the J6 flange")
        print(f"  {'IK drives the TOOL TIP' if A.tip_ik else 'IK drives the FLANGE'}")
        print(f"  *** PUT THE BOARD AT x = {board_at*1000:.0f} mm ***"
              f"  (flange sweeps x = {(board_at-reach)*1000:.0f} mm)")
    print(f"\n  wrote {out}")
