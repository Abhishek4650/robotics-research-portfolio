#!/usr/bin/env python3
"""ARM450_KINEMATICS_how_to_present.pdf -- per slide: a thumbnail, what to
SAY, and answers IF ASKED (as the Rsine speaker guide). Numbers come from the
logs, so the guide never disagrees with the slides."""
import os
import re
import shutil
import subprocess
import tempfile
import textwrap

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                        # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages   # noqa: E402
import pymupdf as fitz                                 # noqa: E402

DOC = os.path.dirname(os.path.abspath(__file__))
KIN = os.path.dirname(DOC)
OUT = os.path.join(DOC, "ARM450_KINEMATICS_how_to_present.pdf")
W = lambda s, n=92: "\n".join(textwrap.fill(p, n) for p in s.split("\n"))


def txt(fn):
    p = os.path.join(KIN, fn)
    return open(p).read() if os.path.exists(p) else ""


def g(rx, s, d="?"):
    m = re.search(rx, s, flags=re.S)
    return m.group(1) if m else d


def main():
    fk, ik, cmp_, ws, man, sine, tim = (txt(f) for f in ("VERIFY_FK.log", "VERIFY_IK.log", "IK_COMPARISON.log",
                                                        "WORKSPACE.log", "MANIPULABILITY.log", "SINE_DEMO.log", "TIMING.log"))
    S = [
        ("Title", "This is the kinematics of the arm I designed and printed, done the way I did the myCobot thesis: Modified "
         "DH, velocity-propagation Jacobian, DLS checked with ikpy. Everything is measured on the CAD, and verified.",
         "Why not the myCobot? -- Same method, but on my own hardware: the numbers now drive a real design."),
        ("My convention", "Frame i is on link i. a and alpha are measured along and about X of the previous frame, d and "
         "theta along and about Z of this frame. The transform is Rot_x, Trans_x, Rot_z, Trans_z.",
         "Standard vs modified DH? -- Standard puts frame i at the far end of link i and uses a_i, alpha_i; modified (Craig) "
         "attaches it at the near end, which makes the velocity propagation clean."),
        ("DH table", "I did not type these numbers: the script builds the frames from the measured servo axes with my rules and "
         "reads the table back; every link transform has the exact Craig form. d4 is %.3f: elbow to wrist centre." % 221.369,
         "Why are there -90 and +90 offsets? -- So that theta is zero at the natural DH position while the servos read 0 with "
         "the arm straight up. Why minus signs on some servos? -- four servos are mounted turning about -z or -y."),
        ("Frames and link lengths", "The wrist centre is where J4, J5 and J6 meet -- that is what makes the IK closed-form.",
         "What is the tool point? -- The centre of the tool-flange face, %s mm beyond the wrist centre." % "88.089"),
        ("FK verified", "Three independent checks: the CAD itself moved by its own servo axes (no DH at all), the URDF in RViz "
         "through ikpy, and the Jacobian two ways. Agreement is %s mm." % g(r"product of exponentials\), position\s+max ([\d.e+-]+)", fk),
         "How do you know the CAD check is independent? -- It uses the product of exponentials of the measured screw axes; "
         "the DH table is never read."),
        ("Jacobian by velocity propagation", "Each column is the tip twist for a unit rate of one joint, carried from the base "
         "to the tip with the two propagation equations, then rotated into the base frame.",
         "Why not just differentiate FK? -- Finite differences are only approximate and break down near singularities; "
         "propagation is exact. They agree to %s (relative)." % g(r"central differences \(relative\)\s+max ([\d.e+-]+)", fk)),
        ("IK in closed form", "Because of the spherical wrist, position and orientation separate: back off the tool to the wrist "
         "centre, solve a 2-link planar arm, then read the wrist angles as Z-Y-Z Euler angles.",
         "How many solutions? -- Up to 8; the joint limits leave 1 or 2 (mean %s)." % g(r"mean ([\d.]+), min", ik)),
        ("The 8 branches", "Here are all eight for one pose; only the green one is inside every joint limit.",
         "Which do you use on the robot? -- The valid one nearest the current joints, so the arm never jumps."),
        ("IK solvers compared", "Like thesis experiment 2. The closed form is exact and fast. The iterative solvers are local: "
         "near the answer they are fine, from far away they fail often.",
         "Then why keep DLS? -- It is general (works without a spherical wrist) and is what follows a path; the closed form "
         "seeds it."),
        ("Reachable workspace", "Monte Carlo over the joint limits: reach %s mm from the J1 axis." % g(r"from the J1 axis: max ([\d.]+)", ws),
         "Why the hole above the base? -- The pitch joints are limited, so the arm cannot fold back on itself."),
        ("Task-constrained workspace", "The key finding: the pen can NEVER point straight down -- the pitch joints give 172.5 "
         "degrees, not 180. On a table the pen must be tilted, on a ring round the base; a vertical board works only with the "
         "pen pointing away.",
         "Can that be fixed? -- An angled pen holder (20-30 degrees), or larger joint ranges -- which the parts' clearances set."),
        ("Manipulability and singularities", "Home, straight up, is singular three ways: shoulder, elbow and wrist. That is why "
         "we start from the bent ready pose.",
         "Where is the arm most dexterous? -- Mid-reach with the elbow bent, like the ring in the myCobot thesis."),
        ("Sine tracing", "Like Rsine: placed with the task workspace, solved in closed form, re-traced by DLS and by ikpy. A pen "
         "is symmetric, so its roll is free: choosing it per point keeps the motion smooth past the wrist singularity.",
         "How smooth? -- Largest joint step %s deg on the board." % g(r"VERTICAL.*?largest joint step between points ([\d.]+)", sine)),
        ("Timing on the servos", "Timed against the ST3215 rating (270 deg/s): the board sine can run at about %s mm/s with the "
         "servos at half their rating. The encoder alone limits accuracy to about %s mm."
         % (g(r"VERTICAL.*?at half the rating: (\d+) mm/s", tim), g(r"VERTICAL.*?tool face [\d.]+ \.\. ([\d.]+) mm", tim)),
         "Why half the rating? -- The rating is no-load; carrying the arm the servos are slower."),
        ("The CAD arm drawing", "The real CAD posed by the IK, and the animations of both sines.",
         "Can I see it in RViz? -- Yes: ros2 launch sine_demo.launch.py which:=vertical."),
        ("Conclusions", "Read the bullets; end on the design lesson: the joint limits, not the link lengths, decide what the arm "
         "can draw.", "What next? -- An angled pen holder, then the same trajectory on the printed arm."),
    ]
    tmp = tempfile.mkdtemp()
    subprocess.run(["soffice", "--headless", "--convert-to", "pdf", "--outdir", tmp,
                    os.path.join(DOC, "ARM450_KINEMATICS_slides.pptx")], capture_output=True, timeout=300)
    thumbs = fitz.open(os.path.join(tmp, "ARM450_KINEMATICS_slides.pdf"))
    with PdfPages(OUT) as pdf:
        fig = plt.figure(figsize=(8.27, 11.69))
        fig.text(0.5, 0.92, "How to present: ARM-450 rev I.1 kinematics", ha="center", fontsize=18, fontweight="bold")
        fig.text(0.08, 0.86, W("One slide per idea; about 1 minute each, 15-18 minutes in all. For each slide: what to SAY, "
                               "and short answers IF ASKED.\n\nTips:\n1. Start from the question: what can this arm reach and "
                               "draw, and how do we compute it?\n2. Say every number with its check (\"FK matches the CAD to "
                               "1e-13 mm\").\n3. Linger on the task-workspace slide: 172.5 degrees is the result the guide will "
                               "remember.\n4. Show the GIF on the CAD-drawing slide in slideshow mode.\n5. If you get lost: "
                               "convention -> table -> FK -> IK -> workspace -> drawing.", 88), va="top", fontsize=11.5)
        pdf.savefig(fig); plt.close(fig)
        for k in range(0, len(S), 2):
            fig = plt.figure(figsize=(8.27, 11.69))
            for j, (title, say, ask) in enumerate(S[k:k + 2]):
                i = k + j
                y0 = 0.97 - 0.49 * j
                fig.text(0.06, y0, "Slide %d -- %s" % (i + 1, title), fontsize=13, fontweight="bold", va="top")
                if i < len(thumbs):
                    png = os.path.join(tmp, "t%d.png" % i)
                    thumbs[i].get_pixmap(dpi=60).save(png)
                    ax = fig.add_axes([0.06, y0 - 0.235, 0.40, 0.20]); ax.imshow(plt.imread(png)); ax.axis("off")
                fig.text(0.50, y0 - 0.04, "SAY\n" + W(say, 48), fontsize=9.6, va="top")
                fig.text(0.06, y0 - 0.26, "IF ASKED\n" + W(ask, 100), fontsize=9.6, va="top", color="#1a4a7a")
            pdf.savefig(fig); plt.close(fig)
    shutil.rmtree(tmp)
    print("written", OUT)


if __name__ == "__main__":
    main()
