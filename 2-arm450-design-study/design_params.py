"""
ARM-450 : clean-sheet 6-DOF arm concept.

Design envelope requested by the user:
    - max overall length  <= 450 mm  (base mounting plane -> TCP, fully stretched)
    - max total mass      <= 1.2 kg
    - lightweight but stiff enough for the bending/torsion it actually sees
    - kinematic structure inspired by the myCobot 280 URDF, appearance deliberately different

Everything below is the single source of truth. Every figure, table and stress
number is computed from these values -- change one here and re-run to update all.
"""

import numpy as np

G = 9.80665

# ----------------------------------------------------------------------------
# 1. LENGTH BUDGET  (sums to exactly 450 mm)
# ----------------------------------------------------------------------------
# Serial chain, all values in mm.
# rev B: proportions revised after the user's paper sketch.
#   * 50 mm round base is the user's own dimension, kept exactly
#   * L2 == L3 so the reachable annulus has NO inner dead zone (|L2-L3| = 0).
#     This is the correction to the "450/6 = 75" equal-split idea: the split
#     that should be equal is upper-arm vs forearm, not all six segments.
L_BASE_HEIGHT = 50.0   # round base cylinder, user's sketch
L_SHOULDER_RISE = 40.0  # base top face -> J2 pitch axis (shoulder barrel centre)
L_BASE_TO_J2 = L_BASE_HEIGHT + L_SHOULDER_RISE

L_UPPER_ARM = 145.0    # J2 shoulder  -> J3 elbow
L_FOREARM = 145.0      # J3 elbow     -> J5 wrist centre  (J4 roll is inline in here)
L_WRIST_TCP = 70.0     # J5 wrist centre -> tool centre point (J6 roll flange + tool)

L_TOTAL = L_BASE_TO_J2 + L_UPPER_ARM + L_FOREARM + L_WRIST_TCP   # -> 450.0
REACH_HORIZ = L_UPPER_ARM + L_FOREARM + L_WRIST_TCP              # -> 355.0 from J1 axis

# ----------------------------------------------------------------------------
# 2. JOINTS  (structure copied from mycobot_280_arm.urdf, dimensions are ours)
# ----------------------------------------------------------------------------
# name, axis meaning, travel (deg), servo choice
JOINTS = [
    ("J1", "base yaw",     (-165, 165), "STS3215"),
    ("J2", "shoulder pitch", (-115, 115), "STS3250"),   # highest torque duty
    ("J3", "elbow pitch",  (-150, 150), "STS3215"),
    ("J4", "forearm roll", (-165, 165), "STS3215"),
    ("J5", "wrist pitch",  (-110, 110), "STS3215"),
    ("J6", "tool roll",    (-175, 175), "STS3215"),
]

# Waveshare serial-bus servos -- the family already on the user's bench.
SERVOS = {
    #            mass_kg, stall torque @12V (N*m), no-load speed (rad/s)
    "STS3215": dict(mass=0.060, stall=2.94, speed=4.6),
    "STS3250": dict(mass=0.080, stall=4.90, speed=3.7),
}

# ----------------------------------------------------------------------------
# 3. MASS BUDGET  (kg)
# ----------------------------------------------------------------------------
# Lumped masses placed along the chain, measured from the J2 shoulder axis
# in the fully-extended horizontal pose (worst case for J2).
#   (label, mass_kg, distance_from_J2_mm)
MASS_ITEMS_ARM = [
    ("upper-arm shell + spar",  0.085,  L_UPPER_ARM * 0.45),
    ("J3 elbow servo",          SERVOS["STS3215"]["mass"], L_UPPER_ARM),
    ("elbow yoke + bearings",   0.045,  L_UPPER_ARM + 15.0),
    ("forearm tube + shells",   0.070,  L_UPPER_ARM + L_FOREARM * 0.45),
    ("J4 roll servo",           SERVOS["STS3215"]["mass"], L_UPPER_ARM + L_FOREARM * 0.55),
    ("J5 wrist servo",          SERVOS["STS3215"]["mass"], L_UPPER_ARM + L_FOREARM),
    ("J6 servo + tool flange",  SERVOS["STS3215"]["mass"] + 0.030,
                                L_UPPER_ARM + L_FOREARM + L_WRIST_TCP * 0.55),
    ("wiring in arm",           0.035,  L_UPPER_ARM + L_FOREARM * 0.5),
]

# Below the shoulder -- carried by the base, never by J2.
MASS_ITEMS_BASE = [
    ("base plate + pedestal",   0.180),
    ("J1 servo + turret brg",   SERVOS["STS3215"]["mass"] + 0.040),
    ("shoulder yoke",           0.055),
    ("J2 shoulder servo",       SERVOS["STS3250"]["mass"]),
    ("driver board + cabling",  0.110),
    ("fasteners / inserts",     0.060),
]

PAYLOAD_RATED = 0.300   # kg at the TCP, arm fully extended horizontally

MASS_ARM = sum(m for _, m, _ in MASS_ITEMS_ARM)
MASS_BASE = sum(m for _, m in MASS_ITEMS_BASE)
MASS_TOTAL = MASS_ARM + MASS_BASE

# ----------------------------------------------------------------------------
# 4. STRUCTURE  (the "sturdy" half of lightweight-but-sturdy)
# ----------------------------------------------------------------------------
# Upper arm: twin side plates in double shear straddling the elbow servo.
# Forearm : single pultruded carbon square tube spar, printed end caps.
SECTIONS = {
    # rev B: cast-style closed shell (user's sketch language) rather than open
    # twin plates -- modelled as a hollow rectangular box section.
    "upper_arm": dict(kind="box shell", b=40.0, h=46.0, wall=3.0,
                      E=2.6e9, sigma_y=55e6, material="PA6-CF shell (printed)"),
    "forearm":   dict(kind="square tube", od=20.0, wall=1.5, n=1,
                      E=70e9, sigma_y=600e6, material="carbon fibre tube"),
}


def section_props(sec):
    """Return (I_mm4, Z_mm3, c_mm) about the bending axis."""
    if sec["kind"] == "box shell":
        b, h, t = sec["b"], sec["h"], sec["wall"]
        I = (b * h ** 3 - (b - 2 * t) * (h - 2 * t) ** 3) / 12.0
        c = h / 2.0
    else:  # square tube, bending about its own centroid
        a, t = sec["od"], sec["wall"]
        ai = a - 2 * t
        I = (a ** 4 - ai ** 4) / 12.0
        c = a / 2.0
    return I, I / c, c


def joint_torques():
    """Static gravity torque at J2 (shoulder) and J3 (elbow), arm horizontal."""
    tau_j2 = sum(m * G * (d * 1e-3) for _, m, d in MASS_ITEMS_ARM)
    tau_j2 += PAYLOAD_RATED * G * (REACH_HORIZ * 1e-3)

    # J3 only carries what is outboard of the elbow.
    tau_j3 = sum(m * G * ((d - L_UPPER_ARM) * 1e-3)
                 for _, m, d in MASS_ITEMS_ARM if d > L_UPPER_ARM)
    tau_j3 += PAYLOAD_RATED * G * ((L_FOREARM + L_WRIST_TCP) * 1e-3)
    return tau_j2, tau_j3


def beam_check():
    """Root bending stress + tip deflection for both arm segments."""
    out = {}

    # --- forearm: cantilever from elbow, carries wrist cluster + payload -----
    sec = SECTIONS["forearm"]
    I, Z, _ = section_props(sec)
    loads = [(m, d - L_UPPER_ARM) for _, m, d in MASS_ITEMS_ARM if d > L_UPPER_ARM]
    loads.append((PAYLOAD_RATED, L_FOREARM + L_WRIST_TCP))
    M = sum(m * G * (d * 1e-3) for m, d in loads)                 # N*m
    sigma = M * (sec["od"] / 2 * 1e-3) / (I * 1e-12)              # Pa
    defl = sum(m * G * (d * 1e-3) ** 3 / (3 * sec["E"] * I * 1e-12) for m, d in loads)
    out["forearm"] = dict(M=M, sigma=sigma / 1e6, sf=sec["sigma_y"] / sigma,
                          defl=defl * 1e3, I=I, Z=Z, material=sec["material"])

    # --- upper arm: cantilever from shoulder, carries everything ------------
    sec = SECTIONS["upper_arm"]
    I, Z, _ = section_props(sec)
    loads = [(m, d) for _, m, d in MASS_ITEMS_ARM]
    loads.append((PAYLOAD_RATED, REACH_HORIZ))
    M = sum(m * G * (d * 1e-3) for m, d in loads)
    sigma = M * (sec["h"] / 2 * 1e-3) / (I * 1e-12)
    # deflection of the 150 mm segment only, tip load equivalent
    P = sum(m for m, _ in loads) * G
    defl = P * (L_UPPER_ARM * 1e-3) ** 3 / (3 * sec["E"] * I * 1e-12)
    out["upper_arm"] = dict(M=M, sigma=sigma / 1e6, sf=sec["sigma_y"] / sigma,
                            defl=defl * 1e3, I=I, Z=Z, material=sec["material"])
    return out


if __name__ == "__main__":
    t2, t3 = joint_torques()
    print(f"overall length   {L_TOTAL:.0f} mm   (horizontal reach {REACH_HORIZ:.0f} mm)")
    print(f"mass  arm {MASS_ARM*1000:.0f} g + base {MASS_BASE*1000:.0f} g "
          f"= {MASS_TOTAL*1000:.0f} g   (budget 1200 g)")
    print(f"J2 torque {t2:.2f} N*m vs {SERVOS['STS3250']['stall']:.2f} stall "
          f"-> SF {SERVOS['STS3250']['stall']/t2:.2f}")
    print(f"J3 torque {t3:.2f} N*m vs {SERVOS['STS3250']['stall']:.2f} stall "
          f"-> SF {SERVOS['STS3250']['stall']/t3:.2f}")
    for k, v in beam_check().items():
        print(f"{k:10s} M={v['M']:.2f} Nm  sigma={v['sigma']:.1f} MPa  "
              f"SF={v['sf']:.1f}  tip defl={v['defl']:.2f} mm  [{v['material']}]")
