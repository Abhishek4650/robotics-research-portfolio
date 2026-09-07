"""
ARM-450 — stress and stiffness analysis of the clamshell link design.  rev 2

Closed-form classical mechanics. Every result states its formula.
Units: N, mm, MPa throughout.

--------------------------------------------------------------------------
rev 2 CORRECTIONS (2026-08-13) — three real errors in rev 1, found by an
adversarial review and then verified independently:

 1. BENDING AXIS WAS WRONG. rev 1 bent the link about the 29.0 mm dimension.
    The ST3215 is 24.7 mm thick and the link's split-normal cavity is
    29.0 - 2(2.0) = 25.0 mm, a deliberate 0.3 mm clearance fit. The servo
    output axis is normal to its 45.2 x 37.8 face, i.e. along its 24.7 mm
    thickness, so the JOINT AXIS lies along the 29.0 mm direction. That axis
    is horizontal, therefore 47.5 mm is the VERTICAL bending depth.
       I = 87,514 mm^4, not 39,899.  Droop was 2.19x too pessimistic.

 2. OPEN-SECTION BENDING PENALTY DOES NOT EXIST HERE. The split plane is
    parallel to the bending plane, so the two half-shells sit SIDE BY SIDE
    about the same neutral axis and their I simply adds: I_open = I_closed.
    rev 1 claimed a 4x bending penalty by wrongly applying a stacked-section
    rule. The seam matters for TORSION ONLY — but there it matters hugely.

 3. open_J DOUBLE-COUNTED THE WIDTH, and the torsion-to-TCP conversion used
    the wrong lever arm (the 70 mm wrist length instead of the tool's
    perpendicular offset, which is what a twist actually swings).

 Also: backlash now includes J1, which rev 1 omitted entirely.
--------------------------------------------------------------------------
"""

import numpy as np

G = 9.80665

# ============================================================================
# 1. GEOMETRY AND LOADS
# ============================================================================
L2 = L3 = 145.0          # link joint-to-joint, mm
L_WRIST = 70.0
REACH = L2 + L3 + L_WRIST                      # 360 mm from the J1 axis
D_J2, D_J3 = REACH, L3 + L_WRIST               # moment arms to the TCP

H_BEND = 50.0            # VERTICAL depth of the box -> sets I  (see rev 2 note 1)
                         # raised 47.5 -> 50.0 on 2026-08-17 so the end-boss arc is
                         # TANGENT to the straight edge (SEC_H/2 = BOSS_R = 25).
                         # The old 47.5 left a shallow 18.2 deg vertex that amplified
                         # the seam clearance into ~0.48 mm of visible misalignment.
W_AXIS = 29.0            # horizontal, along the joint axis; split normal
TOOL_OFFSET = 40.0       # perpendicular offset of the tool from the link axis

PAYLOAD = 0.300
M_WRIST, M_FOREARM, M_UPPER = 0.150, 0.130, 0.145

# ============================================================================
# 2. MATERIALS   (typical values, brand/print dependent)
# ============================================================================
MATS = {
    "PLA":       dict(E=3500., Sy_xy=50., Sy_z=25., G_ratio=0.36, Tg=60.),
    "PLA+CF":    dict(E=7000., Sy_xy=55., Sy_z=22., G_ratio=0.36, Tg=62.),
    "SLA resin": dict(E=2500., Sy_xy=55., Sy_z=55., G_ratio=0.37, Tg=65.),
}

# Derated allowable for a printed bracket carrying repeated shock loads.
# 25 MPa static interlayer x ~0.35 fatigue knockdown = ~9 MPa, then Kt.
ALLOW_FATIGUE = 9.0      # MPa, PLA, before stress concentration


# ============================================================================
# 3. SECTION PROPERTIES
# ============================================================================
def box_I(b, h, t):
    """Closed rectangular tube. b = width, h = depth (bending direction)."""
    return (b * h ** 3 - (b - 2 * t) * (h - 2 * t) ** 3) / 12.0


def box_J(b, h, t):
    """Bredt-Batho closed thin tube: J = 4 Am^2 t / perimeter."""
    Am = (b - t) * (h - t)
    return 4 * Am ** 2 * t / (2 * ((b - t) + (h - t)))


def open_J(b, h, t):
    """
    Seam open: two C-channels side by side. Each = one full-depth web (h) plus
    two half-width flanges (b/2 - t). St Venant J = sum(strip * t^3 / 3).
    """
    per_half = h + 2 * (b / 2.0 - t)
    return 2.0 * (per_half * t ** 3 / 3.0)


def first_moment_Q(b, h, t):
    """Q of the half-section above the neutral axis, for transverse shear."""
    hh = h / 2.0
    return b * t * (hh - t / 2.0) + 2 * (t * (hh - t)) * ((hh - t) / 2.0)


def area(b, h, t):
    return b * h - (b - 2 * t) * (h - 2 * t)


# ============================================================================
# 4. STRESS
# ============================================================================
def stress_report(t, mat="PLA"):
    m = MATS[mat]
    I, Jc = box_I(W_AXIS, H_BEND, t), box_J(W_AXIS, H_BEND, t)
    Q = first_moment_Q(W_AXIS, H_BEND, t)
    c = H_BEND / 2.0

    P_out = (M_FOREARM + M_WRIST + PAYLOAD) * G
    w_self = M_UPPER * G / L2
    M = P_out * L2 + w_self * L2 ** 2 / 2.0
    V = P_out + w_self * L2
    T = PAYLOAD * G * TOOL_OFFSET

    sigma_b = M * c / I
    tau_v = V * Q / (I * t * 2)
    Am = (W_AXIS - t) * (H_BEND - t)
    tau_t = T / (2 * Am * t)
    tau = tau_v + tau_t
    vm = np.sqrt(sigma_b ** 2 + 3 * tau ** 2)

    nu = 0.35
    sigma_cr = 4 * np.pi ** 2 * m["E"] / (12 * (1 - nu ** 2)) * (t / H_BEND) ** 2

    return dict(t=t, mat=mat, I=I, J=Jc, M=M, V=V, T=T,
                sigma_b=sigma_b, tau_v=tau_v, tau_t=tau_t, vm=vm,
                SF_xy=m["Sy_xy"] / vm, SF_z=m["Sy_z"] / vm,
                SF_fatigue=ALLOW_FATIGUE / vm,
                sigma_cr=sigma_cr, SF_buckle=sigma_cr / sigma_b)


# ============================================================================
# 5. STIFFNESS
# ============================================================================
def tip_compliance(t, mat):
    """One continuous cantilever a = L2+L3 with a rigid overhang b = L_WRIST."""
    E, I = MATS[mat]["E"], box_I(W_AXIS, H_BEND, t)
    a, b, EI = L2 + L3, L_WRIST, MATS[mat]["E"] * box_I(W_AXIS, H_BEND, t)
    d_a = a ** 3 / (3 * EI) + b * a ** 2 / (2 * EI)
    th_a = a ** 2 / (2 * EI) + b * a / EI
    return d_a + th_a * b


def tip_stiffness(t, mat):
    return 1.0 / tip_compliance(t, mat)


def self_weight_droop(t, mat):
    EI = MATS[mat]["E"] * box_I(W_AXIS, H_BEND, t)
    a, b = L2 + L3, L_WRIST
    w = (M_UPPER + M_FOREARM) * G / a
    return w * a ** 4 / (8 * EI) + (w * a ** 3 / (6 * EI)) * b


def structural_droop(t, mat):
    return (M_WRIST + PAYLOAD) * G * tip_compliance(t, mat) + self_weight_droop(t, mat)


def natural_freq(t, mat):
    k = tip_stiffness(t, mat) * 1000.0
    m_eff = (M_WRIST + PAYLOAD) + 0.24 * (M_UPPER + M_FOREARM)
    return np.sqrt(k / m_eff) / (2 * np.pi)


def torsion_tip(t, mat, closed=True):
    """TCP swing from shell twist. A twist swings the tool by its PERPENDICULAR
    offset from the link axis — not by the wrist length."""
    Gm = MATS[mat]["E"] * MATS[mat]["G_ratio"]
    J = box_J(W_AXIS, H_BEND, t) if closed else open_J(W_AXIS, H_BEND, t)
    T = PAYLOAD * G * TOOL_OFFSET
    return (T * (L2 + L3) / (Gm * J)) * TOOL_OFFSET


def backlash_tip(deg):
    """RSS over the three joints that swing the TCP: J1, J2 (both 360 mm) and J3."""
    r = np.radians(deg)
    return np.sqrt(2 * (REACH * np.tan(r)) ** 2 + (D_J3 * np.tan(r)) ** 2)


# ============================================================================
# 6. SEAM
# ============================================================================
def seam_analysis(t, pitches=(15, 20, 25, 30, 40, 60)):
    """
    The split plane is PARALLEL to the bending plane, so the seam lines sit at
    the extreme fibres where the bending shear flow Q is zero. The seam
    therefore carries essentially only TORSIONAL shear flow, which Bredt says
    is constant around the loop and does NOT halve between the two seam lines.
    """
    Am = (W_AXIS - t) * (H_BEND - t)
    T = PAYLOAD * G * TOOL_OFFSET
    q = T / (2 * Am)
    return q, [dict(pitch=p, F=q * p, bearing=q * p / (3.0 * t)) for p in pitches]


if __name__ == "__main__":
    print("=" * 84)
    print("ARM-450 rev 2 — STRESS (upper-arm root, worst station)")
    print("=" * 84)
    print(f"{'mat':10s} {'t':>4s} {'sig_b':>8s} {'vonMises':>9s} {'SF(Z)':>7s} "
          f"{'SF(fatigue)':>12s} {'buckleSF':>9s}")
    print("-" * 84)
    for mat in MATS:
        for t in (2.0, 2.4, 3.0):
            r = stress_report(t, mat)
            print(f"{mat:10s} {t:4.1f} {r['sigma_b']:8.3f} {r['vm']:9.3f} "
                  f"{r['SF_z']:7.0f} {r['SF_fatigue']:12.0f} {r['SF_buckle']:9.0f}")

    print("\n" + "=" * 84)
    print("SECTION — the seam penalty is TORSION ONLY")
    print("=" * 84)
    for t in (2.0, 2.4, 3.0):
        Ic = box_I(W_AXIS, H_BEND, t)
        Jc, Jo = box_J(W_AXIS, H_BEND, t), open_J(W_AXIS, H_BEND, t)
        print(f"t={t:.1f}  I={Ic:9,.0f} (unchanged by the seam)   "
              f"J closed {Jc:8,.0f}  open {Jo:6,.0f}  -> {Jc/Jo:5.0f}x penalty")

    print("\n" + "=" * 84)
    print("STIFFNESS, DROOP, FIRST MODE")
    print("=" * 84)
    print(f"{'mat':10s} {'t':>4s} {'k N/mm':>9s} {'droop mm':>9s} {'f1 Hz':>7s}")
    print("-" * 84)
    for mat in MATS:
        for t in (2.0, 2.4, 3.0):
            print(f"{mat:10s} {t:4.1f} {tip_stiffness(t,mat):9.1f} "
                  f"{structural_droop(t,mat):9.3f} {natural_freq(t,mat):7.1f}")

    print("\n" + "=" * 84)
    print("TCP ERROR BUDGET  (rev 2)")
    print("=" * 84)
    for d in (0.3, 0.5, 1.0):
        print(f"  servo backlash {d:.1f} deg, RSS over J1+J2+J3 : {backlash_tip(d):6.2f} mm")
    for t in (2.0, 2.4):
        print(f"  seam OPEN, torsion, PLA t={t:.1f}             : "
              f"{torsion_tip(t,'PLA',False):6.2f} mm")
        print(f"  seam bolted, torsion, PLA t={t:.1f}           : "
              f"{torsion_tip(t,'PLA',True):6.3f} mm")
    print(f"  elastic droop, PLA t=2.0                    : {structural_droop(2.0,'PLA'):6.3f} mm")
    print(f"  elastic droop, PLA+CF t=2.4                 : {structural_droop(2.4,'PLA+CF'):6.3f} mm")
