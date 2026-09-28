# ARM-450 rev I.1 -- Modified (Craig) DH table

Derived from the CAD servo axes (`analysis/s1_dh_from_cad.py`); frame i on link i;
`^{i-1}_iT = Rot_x(alpha_{i-1}) Trans_x(a_{i-1}) Rot_z(theta_i) Trans_z(d_i)`.

| i | alpha_{i-1} (deg) | a_{i-1} (mm) | d_i (mm) | theta_i | theta from servo q | servo range | joint |
|---|---|---|---|---|---|---|---|
| 1 | 0 | 0.000 | 90.000 | th1 | th1 = -q1 | q1 +-90 deg | J1 base yaw |
| 2 | -90 | 0.000 | 0.000 | th2 (home -90 deg) | th2 = -q2 -90 deg | q2 +-54 deg | J2 shoulder |
| 3 | 0 | 119.000 | 0.000 | th3 (home +90 deg) | th3 = q3 +90 deg | q3 +-72 deg | J3 elbow |
| 4 | 90 | 0.000 | 221.369 | th4 | th4 = -q4 | q4 +-90 deg | J4 forearm roll |
| 5 | -90 | 0.000 | 0.000 | th5 | th5 = q5 | q5 +-46.5 deg | J5 wrist pitch |
| 6 | 90 | 0.000 | 0.000 | th6 | th6 = -q6 | q6 +-90 deg | J6 tool roll |

Tool: `^6_TT = Trans_z(d_T)`, d_T = 88.089 mm (wrist centre -> tool-flange face).
Base frame 0 = CAD world: origin on the J1 axis at the base's top face, Z up; the table top is z = -69.9.
Wrist centre W = O4 = O5 = O6 (J4, J5, J6 meet): spherical wrist -> closed-form IK.
Home (all servos at 0) = arm straight up; ^0_TT = Trans_z(518.458), tool frame parallel to the base frame.
