#!/usr/bin/env python3
"""
Build the detailed explanation PDF (the full project write-up), from SECTIONS
plus live-computed verification metrics.

Run:  python3 analysis/make_report.py  ->  docs/R_sine_detailed_explanation.pdf
"""
import os
import sys
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Image,
                                Table, TableStyle, PageBreak)
from reportlab.lib.enums import TA_CENTER
from PIL import Image as PILImage

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG, "analysis"))
import deliverable_data as D   # noqa: E402

FIG = os.path.join(PKG, "figures")
OUT = os.path.join(PKG, "docs", "R_sine_detailed_explanation.pdf")
MAXW = 16 * cm

ss = getSampleStyleSheet()
H1 = ParagraphStyle("H1", parent=ss["Heading1"], fontSize=16, textColor=colors.HexColor("#1F3A5F"))
H2 = ParagraphStyle("H2", parent=ss["Heading2"], fontSize=12, textColor=colors.HexColor("#D62728"))
BODY = ParagraphStyle("Body", parent=ss["BodyText"], fontSize=10.5, leading=15)
BULLET = ParagraphStyle("Bullet", parent=BODY, leftIndent=14, bulletIndent=4)
TITLE = ParagraphStyle("Title", parent=ss["Title"], fontSize=24, textColor=colors.HexColor("#1F3A5F"))
CAP = ParagraphStyle("Cap", parent=BODY, fontSize=9, textColor=colors.grey, alignment=TA_CENTER)


def scaled_image(path, maxw=MAXW):
    w, h = PILImage.open(path).size
    ratio = maxw / w
    return Image(path, width=maxw, height=h * ratio)


def build():
    metrics = D.get_metrics()
    story = []

    # ---- title page ----
    story += [Spacer(1, 5 * cm),
              Paragraph("R_sine", TITLE),
              Spacer(1, 0.4 * cm),
              Paragraph("Drawing a Sine Wave with a myCobot 280 — Detailed Explanation",
                        ParagraphStyle("s", parent=H2, alignment=TA_CENTER, fontSize=14)),
              Spacer(1, 1 * cm),
              Paragraph("Modified-DH forward kinematics · velocity-propagation Jacobian · "
                        "damped-least-squares inverse kinematics · ROS 2 (Jazzy) · "
                        "independent verification against ikpy", BODY),
              PageBreak()]

    # ---- verification summary box ----
    story.append(Paragraph("Verification at a glance", H1))
    data = [
        ["Check", "Result"],
        ["Forward kinematics (DH) vs URDF", f"{metrics['fk_err']:.1e} m"],
        ["Velocity-propagation Jacobian vs finite diff.", f"{metrics['jac_err']:.1e}"],
        ["Analytical IK tracking of the sine", f"{metrics['ik_track_mm']:.3f} mm"],
        ["Mean IK iterations per waypoint", f"{metrics['ik_iters']:.0f}"],
        ["Max 3D reach / horizontal reach", f"{metrics['max_reach']:.3f} m / {metrics['max_horiz']:.3f} m"],
        ["Home tool position (all joints 0)",
         f"({metrics['home'][0]:.3f}, {metrics['home'][1]:.3f}, {metrics['home'][2]:.3f}) m"],
    ]
    t = Table(data, colWidths=[10 * cm, 6 * cm])
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1F3A5F")),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTSIZE", (0, 0), (-1, -1), 10),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.grey),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.whitesmoke, colors.white]),
    ]))
    story += [t, Spacer(1, 0.4 * cm),
              Paragraph("All numbers are reproduced by the scripts in <b>analysis/</b>.", CAP),
              PageBreak()]

    # ---- one section per SECTIONS entry ----
    for sec in D.SECTIONS:
        story.append(Paragraph(sec["title"], H1))
        if sec.get("subtitle"):
            story.append(Paragraph(sec["subtitle"], H2))
        story.append(Spacer(1, 0.15 * cm))
        story.append(Paragraph(sec["detail"], BODY))
        story.append(Spacer(1, 0.2 * cm))
        for b in sec["bullets"]:
            story.append(Paragraph(b, BULLET, bulletText="•"))
        if sec.get("table"):
            story.append(Spacer(1, 0.3 * cm))
            header = ["i", "alpha (deg)", "a (m)", "d (m)", "theta off (deg)"]
            tt = Table([header] + [list(r) for r in sec["table"]])
            tt.setStyle(TableStyle([
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1F3A5F")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
                ("GRID", (0, 0), (-1, -1), 0.5, colors.grey),
                ("FONTSIZE", (0, 0), (-1, -1), 9),
                ("ALIGN", (0, 0), (-1, -1), "CENTER"),
            ]))
            story.append(tt)
        if sec.get("figure"):
            path = os.path.join(FIG, sec["figure"])
            if os.path.exists(path):
                story += [Spacer(1, 0.3 * cm), scaled_image(path),
                          Paragraph("Figure: " + sec["figure"], CAP)]
        story.append(Spacer(1, 0.5 * cm))

    doc = SimpleDocTemplate(OUT, pagesize=A4, topMargin=1.6 * cm, bottomMargin=1.6 * cm,
                            leftMargin=2.5 * cm, rightMargin=2.5 * cm,
                            title="R_sine — Detailed Explanation")
    doc.build(story)
    print("wrote", OUT)


if __name__ == "__main__":
    build()
