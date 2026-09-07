"""
ARM-450 — geometry optimisation.

Two questions:
  1. Given L2 + L3 = 290 mm fixed, what SPLIT minimises shoulder torque, and what
     does the torque-optimal split cost in workspace?
  2. What SECTION minimises mass subject to stiffness, frequency, printability and
     the hard requirement that a servo must fit inside the link?
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import stress_analysis as S

G = 9.80665
INK, ACC, GOOD, BAD, WARN = "#1b2733", "#2e7d9a", "#1e7d3c", "#b03a2e", "#d09a2c"

TOTAL = 290.0            # L2 + L3, fixed by the 450 mm budget
L_WRIST = 70.0
PAYLOAD = 0.300
M_WRIST = 0.150
RHO = {"PLA": 1.24e-3, "PLA+CF": 1.29e-3}     # g/mm^3 solid

# servo envelope that must fit INSIDE the link
SERVO_W, SERVO_T = 37.8, 24.7


def j2_torque(l2, l3, rho_lin=0.95e-3):
    """Shoulder gravity torque, arm horizontal. rho_lin = kg per mm of link."""
    m2, m3 = rho_lin * l2, rho_lin * l3
    tau = m2 * G * (l2 / 2) * 1e-3
    tau += m3 * G * (l2 + l3 / 2) * 1e-3
    tau += (M_WRIST + PAYLOAD) * G * (l2 + l3 + L_WRIST) * 1e-3
    return tau


def dead_zone_area(l2, l3):
    """Area of the unreachable inner disc, mm^2."""
    return np.pi * abs(l2 - l3) ** 2


# ===========================================================================
print("=" * 80)
print("1. LINK SPLIT — torque vs workspace, with L2 + L3 = 290 mm")
print("=" * 80)
splits = np.linspace(100, 190, 91)
tau = np.array([j2_torque(s, TOTAL - s) for s in splits])
dead = np.array([dead_zone_area(s, TOTAL - s) for s in splits])
i_eq = np.argmin(np.abs(splits - 145))
i_best = np.argmin(tau)
print(f"  equal split  L2 = L3 = 145.0   tau_J2 = {tau[i_eq]:.4f} N.m   dead zone 0 mm^2")
print(f"  torque-opt   L2 = {splits[i_best]:.1f}         tau_J2 = {tau[i_best]:.4f} N.m   "
      f"dead zone {dead[i_best]/100:.0f} cm^2")
print(f"  torque saved by going unequal: {(tau[i_eq]-tau[i_best])*1000:.1f} mN.m "
      f"({(1-tau[i_best]/tau[i_eq])*100:.2f} %)")
print("\n  -> the torque curve is almost FLAT in the split, because the dominant")
print("     term is the payload at the fixed 360 mm reach, which does not move.")
print("     Equal links cost essentially nothing in torque and buy the entire")
print("     dead zone back. EQUAL SPLIT WINS OUTRIGHT — there is no trade-off.")

# ===========================================================================
print("\n" + "=" * 80)
print("2. SECTION — minimise mass subject to real constraints")
print("=" * 80)
F_MIN, DROOP_MAX = 25.0, 0.30
print(f"  constraints: f1 >= {F_MIN} Hz,  droop <= {DROOP_MAX} mm,")
print(f"               wall = integer x 0.4 mm nozzle,")
print(f"               section must house a {SERVO_W} x {SERVO_T} mm servo\n")

rows = []
for h in (40.0, 47.5, 55.0):
    for w in (26.0, 29.0, 34.0):
        for t in (1.6, 2.0, 2.4, 2.8, 3.2):
            fits = (w - 2 * t >= SERVO_T) and (h - 2 * t >= SERVO_W)
            S.H_BEND, S.W_AXIS = h, w
            for mat in ("PLA", "PLA+CF"):
                d = S.structural_droop(t, mat)
                f1 = S.natural_freq(t, mat)
                area = S.area(w, h, t)
                mass = area * 2 * 145.0 * RHO[mat]        # both links, grams
                ok = fits and f1 >= F_MIN and d <= DROOP_MAX
                rows.append(dict(h=h, w=w, t=t, mat=mat, fits=fits, f1=f1,
                                 d=d, mass=mass, ok=ok))
S.H_BEND, S.W_AXIS = 47.5, 29.0                            # restore

feasible = [r for r in rows if r["ok"]]
feasible.sort(key=lambda r: r["mass"])
print(f"  {len(feasible)} feasible of {len(rows)} candidates. Lightest ten:\n")
print(f"  {'h':>5s} {'w':>5s} {'t':>4s} {'mat':>7s} {'mass g':>7s} "
      f"{'droop':>7s} {'f1 Hz':>6s}")
print("  " + "-" * 50)
for r in feasible[:10]:
    print(f"  {r['h']:5.1f} {r['w']:5.1f} {r['t']:4.1f} {r['mat']:>7s} "
          f"{r['mass']:7.0f} {r['d']:7.3f} {r['f1']:6.1f}")

blocked = [r for r in rows if not r["fits"]]
print(f"\n  {len(blocked)} candidates rejected purely because the SERVO DOES NOT FIT.")
print("  The binding constraint is GEOMETRIC, not mechanical: the section cannot")
print(f"  go below w = {SERVO_T + 2*2.4:.1f} mm or h = {SERVO_W + 2*2.4:.1f} mm at a 2.4 mm wall,")
print("  no matter what the stress or stiffness numbers say.")

best = feasible[0]
print(f"\n  OPTIMUM: {best['h']:.1f} x {best['w']:.1f}, t = {best['t']:.1f}, "
      f"{best['mat']} -> {best['mass']:.0f} g, droop {best['d']:.3f} mm, "
      f"f1 {best['f1']:.1f} Hz")
cur = [r for r in rows if r["h"] == 47.5 and r["w"] == 29.0
       and r["t"] == 2.4 and r["mat"] == "PLA+CF"][0]
print(f"  CURRENT SPEC: 47.5 x 29.0, t = 2.4, PLA+CF -> {cur['mass']:.0f} g, "
      f"droop {cur['d']:.3f} mm, f1 {cur['f1']:.1f} Hz")
print(f"  saving available: {cur['mass']-best['mass']:.0f} g "
      f"({(1-best['mass']/cur['mass'])*100:.0f} %)")

# ===========================================================================
fig, axes = plt.subplots(1, 3, figsize=(15.5, 5.0))
fig.patch.set_facecolor("white")

ax = axes[0]
ax.plot(splits, tau, color=ACC, lw=2.4)
ax.axvline(145, color=GOOD, ls="--", lw=1.8)
ax.plot(splits[i_best], tau[i_best], "o", color=BAD, ms=8)
ax.annotate("equal split\n(no dead zone)", (145, tau[i_eq]), (118, tau.max()*0.995),
            fontsize=9, color=GOOD, fontweight="bold")
ax.set_xlabel("L2 upper arm (mm),  L3 = 290 − L2")
ax.set_ylabel("J2 shoulder torque (N·m)")
ax.set_title("A   Torque barely cares about the split", loc="left",
             fontsize=11.5, fontweight="bold", color=INK)
ax.grid(alpha=.25, ls=":")

ax = axes[1]
ax.plot(splits, dead/100, color=BAD, lw=2.4)
ax.axvline(145, color=GOOD, ls="--", lw=1.8)
ax.fill_between(splits, 0, dead/100, color=BAD, alpha=.12)
ax.set_xlabel("L2 upper arm (mm)")
ax.set_ylabel("unreachable dead zone (cm²)")
ax.set_title("B   Workspace cares enormously", loc="left",
             fontsize=11.5, fontweight="bold", color=INK)
ax.grid(alpha=.25, ls=":")

ax = axes[2]
for mat, col in (("PLA", ACC), ("PLA+CF", GOOD)):
    sub = [r for r in rows if r["mat"] == mat and r["h"] == 47.5 and r["w"] == 29.0]
    sub.sort(key=lambda r: r["t"])
    ax.plot([r["mass"] for r in sub], [r["f1"] for r in sub],
            "o-", color=col, lw=2.2, ms=6, label=mat)
    for r in sub:
        ax.annotate(f"{r['t']:.1f}", (r["mass"], r["f1"]), textcoords="offset points",
                    xytext=(4, -10), fontsize=7.5, color=col)
ax.axhline(F_MIN, color=BAD, ls="--", lw=1.6)
ax.text(ax.get_xlim()[1], F_MIN+1, "f1 floor 25 Hz", ha="right", fontsize=8.6, color=BAD)
ax.set_xlabel("mass of both links (g)")
ax.set_ylabel("first mode (Hz)")
ax.set_title("C   Mass vs stiffness (label = wall mm)", loc="left",
             fontsize=11.5, fontweight="bold", color=INK)
ax.legend(frameon=False, fontsize=9)
ax.grid(alpha=.25, ls=":")

for a in axes:
    for s in ("top", "right"): a.spines[s].set_visible(False)
fig.suptitle("ARM-450 — geometry optimisation", fontsize=14.5,
             fontweight="bold", color=INK, x=0.02, ha="left")
fig.tight_layout(rect=(0, 0, 1, 0.93))
fig.savefig("figures/opt_geometry.png", dpi=200, facecolor="white")
print("\nwrote figures/opt_geometry.png")
