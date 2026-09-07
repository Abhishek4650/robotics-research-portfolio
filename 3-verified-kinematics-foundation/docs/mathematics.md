# R_sine — Mathematics

myCobot 280 tracing a sine wave. This document holds the math **separately** from
the code; every equation here is implemented in `Rsine/kinematics.py` and checked
numerically by `analysis/verify_kinematics.py`.

---

## 1. Robot and conventions

The myCobot 280 is a 6-DOF serial arm, all joints **revolute**, each rotating
about its local **z**-axis. We use the **Modified (Craig) Denavit–Hartenberg**
convention: frame *i* is rigidly attached to link *i*, and four parameters relate
frame *i−1* to frame *i*.

| symbol | name | meaning |
|--------|------|---------|
| αᵢ₋₁ | link twist  | angle from Zᵢ₋₁ to Zᵢ about Xᵢ₋₁ |
| aᵢ₋₁ | link length | distance from Zᵢ₋₁ to Zᵢ along Xᵢ₋₁ |
| dᵢ   | link offset | distance from Xᵢ₋₁ to Xᵢ along Zᵢ |
| θᵢ   | joint angle | angle from Xᵢ₋₁ to Xᵢ about Zᵢ (the joint variable) |

Joint limits (rad), from the URDF: J1–J5 ∈ [−2.879793, 2.879793], J6 ∈ [−3.05, 3.05].

---

## 2. Homogeneous transforms

A rigid transform is the 4×4 matrix

```
      | R   p |
T  =  | 0   1 |
```

with R a 3×3 rotation and p a translation. Elementary rotations Rₓ, R_y, R_z and a
translation Trans(x,y,z) are composed to build every frame. The URDF specifies each
joint's fixed origin as roll-pitch-yaw, applied in the URDF convention
`R = R_z(yaw)·R_y(pitch)·R_x(roll)`.

---

## 3. Modified-DH link transform

Each consecutive pair of frames is related by

```
 i-1
    T   =  Rotx(αᵢ₋₁) · Transx(aᵢ₋₁) · Rotz(θᵢ) · Transz(dᵢ)
 i
```

which multiplies out to

```
        | cθ            -sθ           0         aᵢ₋₁       |
i-1_T_i=| sθ·cα         cθ·cα        -sα       -sα·dᵢ      |
        | sθ·sα         cθ·sα         cα        cα·dᵢ      |
        | 0             0            0         1          |
```

(c = cos, s = sin, α = αᵢ₋₁, θ = θᵢ). This is exactly the matrix in the project
reference notes.

---

## 4. Identified DH table (myCobot 280)

The URDF link frames are **not** placed on DH axes, so the table was *identified*:
we fit (αᵢ₋₁, aᵢ₋₁, dᵢ, θ-offset) so that the DH forward kinematics reproduces the
URDF forward kinematics, then verified the residual is machine-zero.

| i | αᵢ₋₁ (°) | aᵢ₋₁ (m) | dᵢ (m) | θ offset (°) |
|---|----------|----------|--------|--------------|
| 1 | 0    | 0        | 0.13056 | +90 |
| 2 | +90  | 0        | 0       | −90 |
| 3 | 0    | −0.1104  | 0       | 0   |
| 4 | 0    | −0.096   | 0.06062 | −90 |
| 5 | +90  | 0        | 0.07318 | +90 |
| 6 | −90  | 0        | 0.0456  | 0   |

θᵢ = (θ offset)ᵢ + qᵢ, where qᵢ is the commanded joint angle.
**Verification:** `max |DH-FK − URDF-FK| ≈ 1.2e-15 m` over 5000 random configs.

> Note on offsets: joints 2–4 have parallel axes, which leaves a gauge freedom in
> how dᵢ is distributed. We use the *concentrated* convention (the 0.06062 offset
> sits at joint 4, where the URDF places it) for a clean physical table.

---

## 5. Forward kinematics

```
 0            0    1    2    3    4    5
   T      =    T ·  T ·  T ·  T ·  T ·  T
 EE           1    2    3    4    5    6
```

At the home configuration (all qᵢ = 0) the end-effector (link6_flange) sits at

```
p_home = (0.06062, 0.04560, 0.41014) m
```

---

## 6. Jacobian — velocity-propagation method

Craig's velocity propagation carries the angular and linear velocity outward, frame
by frame (revolute joint i, joint rate θ̇ᵢ):

```
 i+1                i+1  i             .      i+1 ^
    ω        =         R    ω     +    θ       Z
 i+1            i        i          i+1      i+1

 i+1                i+1 (  i          i     i      )      .       i+1 ^
    v        =         R ( v     +    ω  ×   P     )  +   d        Z
 i+1            i     (  i          i      i+1    )    i+1      i+1
```

For an all-revolute arm this closes to the compact **geometric Jacobian**, columns

```
J_v[:,i] = z_i × (p_e − p_i)      (linear velocity of the EE per unit θ̇ᵢ)
J_ω[:,i] = z_i                    (angular velocity of the EE per unit θ̇ᵢ)
```

where zᵢ and pᵢ are joint i's axis and origin in the base frame, and p_e is the
end-effector position. The 6×6 Jacobian maps joint rates to end-effector twist:
`[v; ω] = J(q) · q̇`.
**Verification:** `max |velprop-J − finite-difference-J| ≈ 2.5e-7`.

---

## 7. Inverse kinematics (next stage)

We drive the sine path with **damped least squares** (Levenberg–Marquardt) IK:

```
q_{k+1} = q_k + Jᵀ (J Jᵀ + λ² I)⁻¹ · e
```

with e the 6-vector pose error (position + orientation) between the current and
target end-effector poses, and λ a damping constant for stability near
singularities. The target orientation keeps the pen normal to the (configurable)
drawing plane. Each solved waypoint is cross-checked against an independent IK
(ikpy) within tolerance.
```
```
