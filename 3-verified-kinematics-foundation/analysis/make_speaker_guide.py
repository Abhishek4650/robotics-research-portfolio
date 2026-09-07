#!/usr/bin/env python3
"""
Build the 'How to present this to your guide' PDF — a slide-by-slide speaking
script with what to say, anticipated questions + answers, and general tips.

Run:  python3 analysis/make_speaker_guide.py -> docs/R_sine_how_to_present.pdf
"""
import os
import sys
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Table,
                                TableStyle, PageBreak)
from reportlab.lib.enums import TA_CENTER

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG, "analysis"))
import deliverable_data as D   # noqa: E402

OUT = os.path.join(PKG, "docs", "R_sine_how_to_present.pdf")

ss = getSampleStyleSheet()
TITLE = ParagraphStyle("T", parent=ss["Title"], fontSize=22, textColor=colors.HexColor("#1F3A5F"))
H1 = ParagraphStyle("H1", parent=ss["Heading2"], fontSize=13, textColor=colors.HexColor("#1F3A5F"))
SAY = ParagraphStyle("Say", parent=ss["BodyText"], fontSize=10.5, leading=15,
                     leftIndent=8, textColor=colors.HexColor("#222222"))
Q = ParagraphStyle("Q", parent=ss["BodyText"], fontSize=9.5, leading=13,
                   leftIndent=10, textColor=colors.HexColor("#8A4B00"))
BODY = ParagraphStyle("B", parent=ss["BodyText"], fontSize=10.5, leading=15)
CAP = ParagraphStyle("Cap", parent=BODY, fontSize=9, textColor=colors.grey)

TIPS = [
    "Total time: aim for 8–10 minutes of talking, then questions. About 40 seconds per slide.",
    "Open with the goal and the demo picture so the guide sees the result first, then explain how.",
    "Lead with your OWN work (the DH derivation, the Jacobian, the IK); mention ikpy only as a check.",
    "When a slide shows numbers, say the one-line takeaway ('matches to machine precision'), not every digit.",
    "Point at the figure while you talk — the drawn sine and the workspace are your strongest visuals.",
    "If you don't know an answer, say what you'd do to find out — that reads as maturity, not weakness.",
    "Keep the live RViz demo command ready as a backup: ros2 launch Rsine rsine_sine.launch.py",
]


def build():
    story = []
    story += [Spacer(1, 3.5 * cm),
              Paragraph("R_sine — How to Present to Your Guide", TITLE),
              Spacer(1, 0.4 * cm),
              Paragraph("A slide-by-slide speaking script, likely questions with answers, "
                        "and presentation tips. Read this once before you present; keep it "
                        "beside you during.", ParagraphStyle("s", parent=BODY, alignment=TA_CENTER)),
              PageBreak()]

    story += [Paragraph("Before you start — 7 tips", H1), Spacer(1, 0.2 * cm)]
    for t in TIPS:
        story.append(Paragraph(t, BODY, bulletText="•"))
        story.append(Spacer(1, 0.1 * cm))
    story.append(PageBreak())

    story += [Paragraph("Slide-by-slide script", H1), Spacer(1, 0.3 * cm)]
    for i, sec in enumerate(D.SECTIONS, start=1):
        head = Table([[f"Slide {i}", sec["title"]]], colWidths=[2.2 * cm, 13.5 * cm])
        head.setStyle(TableStyle([
            ("BACKGROUND", (0, 0), (0, 0), colors.HexColor("#1F3A5F")),
            ("TEXTCOLOR", (0, 0), (0, 0), colors.white),
            ("BACKGROUND", (1, 0), (1, 0), colors.HexColor("#E8EEF5")),
            ("FONTSIZE", (0, 0), (-1, -1), 11),
            ("FONTNAME", (0, 0), (-1, -1), "Helvetica-Bold"),
            ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
            ("LEFTPADDING", (0, 0), (-1, -1), 6),
            ("TOPPADDING", (0, 0), (-1, -1), 4), ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
        ]))
        story += [head, Spacer(1, 0.15 * cm),
                  Paragraph("<b>Say:</b> " + sec.get("say", ""), SAY),
                  Spacer(1, 0.15 * cm)]
        for q, a in sec.get("questions", []):
            story.append(Paragraph(f"<b>If asked:</b> {q}", Q))
            story.append(Paragraph(f"<b>Answer:</b> {a}", Q))
        story.append(Spacer(1, 0.45 * cm))

    doc = SimpleDocTemplate(OUT, pagesize=A4, topMargin=1.6 * cm, bottomMargin=1.6 * cm,
                            leftMargin=2.3 * cm, rightMargin=2.3 * cm,
                            title="R_sine — How to Present")
    doc.build(story)
    print("wrote", OUT)


if __name__ == "__main__":
    build()
