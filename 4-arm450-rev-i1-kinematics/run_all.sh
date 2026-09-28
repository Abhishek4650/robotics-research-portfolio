#!/bin/bash
# ARM-450 rev I.1 kinematics: every step, then the documents. Sequential.
cd "$(dirname "$0")/analysis"
for s in s1_dh_from_cad s2_verify_fk s3_ik s4_workspace s5_manipulability s6_ik_comparison s7_sine_demo s8_trajectory_timing; do
    echo "=== $s"; python3 -u $s.py 2>&1 | grep -v -i "warn\|SetCells" | tail -4
done
cd ../docs && python3 make_report.py && python3 make_slides.py && python3 make_speaker_guide.py
