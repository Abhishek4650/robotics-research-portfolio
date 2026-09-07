#!/usr/bin/env python3
"""
Generate the 'How your ROS 2 system works' explainer:
  figures/fig_ros2_dataflow.png    — RViz vs Gazebo data-flow diagram
  docs/R_sine_how_ros2_works.pdf   — the written note (with the diagram)

Run:  python3 analysis/make_ros2_explainer.py
"""
import os
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                         # noqa: E402
from matplotlib.patches import FancyBboxPatch           # noqa: E402
from reportlab.lib.pagesizes import A4                  # noqa: E402
from reportlab.lib.units import cm                      # noqa: E402
from reportlab.lib import colors                        # noqa: E402
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle  # noqa: E402
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer,  # noqa: E402
                                Image, Preformatted)
from PIL import Image as PILImage                        # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG = os.path.join(PKG, "figures")
OUT = os.path.join(PKG, "docs", "R_sine_how_ros2_works.pdf")
DIAG = os.path.join(FIG, "fig_ros2_dataflow.png")

NAVY = "#1F3A5F"
BLUE = "#dbe6f3"
GREEN = "#dcefdc"
ORANGE = "#f6e3c8"


def _box(ax, x, y, w, h, text, fc):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                fc=fc, ec=NAVY, lw=1.5))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=9, wrap=True)


def _arrow(ax, x1, y1, x2, y2, label, color="#333333", rad=0.0, label_pos=None):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6,
                                connectionstyle=f"arc3,rad={rad}"))
    if label:
        if label_pos is None:
            label_pos = ((x1 + x2) / 2, max(y1, y2) + 0.18)
        ax.text(label_pos[0], label_pos[1], label, ha="center", va="center",
                fontsize=8, color=color, style="italic")


def make_diagram():
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 7.5))
    for ax in (ax1, ax2):
        ax.set_xlim(0, 12.2); ax.set_ylim(0, 4); ax.axis("off")
    BY, BH = 1.75, 0.95          # box y, height  -> boxes span 1.75..2.70
    AY = BY + BH / 2             # arrow height (mid-box)
    xs = [0.2, 3.5, 6.8, 9.9]    # box left edges, wide gaps for labels

    # ---- RViz (kinematic) pipeline ----
    ax1.set_title("RViz pipeline — kinematic (you command, it shows)", fontsize=12, color=NAVY)
    _box(ax1, xs[0], BY, 2.2, BH, "sine_ik_node\n(my IK, 30 Hz)", BLUE)
    _box(ax1, xs[1], BY, 2.5, BH, "robot_state_\npublisher (URDF)", BLUE)
    _box(ax1, xs[2], BY, 2.2, BH, "path_tracer_\nnode", BLUE)
    _box(ax1, xs[3], BY, 1.9, BH, "RViz", GREEN)
    _arrow(ax1, xs[0] + 2.2, AY, xs[1], AY, "/joint_states")
    _arrow(ax1, xs[1] + 2.5, AY, xs[2], AY, "/tf")
    _arrow(ax1, xs[2] + 2.2, AY, xs[3], AY, "/drawn_path")
    # target sine: arc ABOVE the chain, label in empty band on top
    _arrow(ax1, xs[0] + 1.0, BY + BH, xs[3] + 1.0, BY + BH, "", color="#2ca02c", rad=-0.32)
    ax1.text(6.1, 3.72, "/target_sine_path  (green goal, straight to RViz)",
             ha="center", fontsize=8, color="#2ca02c", style="italic")
    # tf/description: arc BELOW the chain, label in empty band below
    _arrow(ax1, xs[1] + 1.2, BY, xs[3] + 0.6, BY, "", color="#888888", rad=0.32)
    ax1.text(7.4, 0.72, "RViz also reads /tf + /robot_description",
             ha="center", fontsize=8, color="#888888", style="italic")

    # ---- Gazebo (physics) pipeline ----
    ax2.set_title("Gazebo pipeline — physics (you command, motors move, it reports back)",
                  fontsize=12, color=NAVY)
    _box(ax2, xs[0], BY, 2.2, BH, "sine_ik_node\n(commands)", BLUE)
    _box(ax2, xs[1], BY, 2.5, BH, "joint_trajectory_\ncontroller", ORANGE)
    _box(ax2, xs[2], BY, 2.2, BH, "Gazebo\n(physics)", ORANGE)
    _box(ax2, xs[3], BY, 2.3, BH, "joint_state_\nbroadcaster", ORANGE)
    _arrow(ax2, xs[0] + 2.2, AY, xs[1], AY, "command")
    _arrow(ax2, xs[1] + 2.5, AY, xs[2], AY, "drives motors")
    _arrow(ax2, xs[2] + 2.2, AY, xs[3], AY, "senses")
    # feedback: arc BELOW back to the left, label in empty band below
    _arrow(ax2, xs[3] + 1.1, BY, xs[0] + 1.1, BY, "", color="#888888", rad=0.34)
    ax2.text(6.1, 0.62, "/joint_states  ->  robot_state_publisher  ->  TF  ->  RViz / Gazebo view",
             ha="center", fontsize=8, color="#888888", style="italic")

    fig.tight_layout()
    fig.savefig(DIAG, dpi=150, bbox_inches="tight")
    plt.close(fig)


ss = getSampleStyleSheet()
H1 = ParagraphStyle("H1", parent=ss["Heading1"], fontSize=15, textColor=colors.HexColor(NAVY))
H2 = ParagraphStyle("H2", parent=ss["Heading2"], fontSize=12, textColor=colors.HexColor("#D62728"))
BODY = ParagraphStyle("Body", parent=ss["BodyText"], fontSize=10.5, leading=15)
BULL = ParagraphStyle("Bull", parent=BODY, leftIndent=14, bulletIndent=4)
MONO = ParagraphStyle("Mono", parent=ss["Code"], fontSize=8.5, leading=11,
                      backColor=colors.HexColor("#f4f4f4"))
TITLE = ParagraphStyle("T", parent=ss["Title"], fontSize=22, textColor=colors.HexColor(NAVY))

DIAGRAM_TXT = (
    " [sine_ik_node] --/joint_states--> [robot_state_publisher] --/tf--> [path_tracer_node]\n"
    "   (my IK: 6 angles              (has the URDF; converts        (reads base_link->\n"
    "    at 30 Hz)                      angles -> pose of every        link6_flange,\n"
    "        |                          link, as TF frames)            publishes /drawn_path)\n"
    "        |                                                                  |\n"
    "        +------ /target_sine_path (green goal) ------+                     |\n"
    "                                                     v                     v\n"
    "                                                  [ RViz ] <-- /tf, /robot_description,\n"
    "                                                                both path topics"
)


def scaled(path, maxw=16 * cm):
    w, h = PILImage.open(path).size
    return Image(path, width=maxw, height=h * maxw / w)


def bullets(items):
    return [Paragraph(t, BULL, bulletText="•") for t in items]


def build_pdf():
    story = [
        Paragraph("How Your ROS 2 System Works", TITLE),
        Spacer(1, 0.3 * cm),
        Paragraph("R_sine — a plain-language guide to the nodes, topics, TF, and the "
                  "difference between the RViz and Gazebo pipelines.", BODY),
        Spacer(1, 0.5 * cm),

        Paragraph("The 3 core ideas", H1),
        *bullets([
            "<b>Node</b> = one small program doing one job. This project runs three: "
            "sine_ik_node, robot_state_publisher, path_tracer_node.",
            "<b>Topic</b> = a named channel that nodes post to or read from (like a radio "
            "channel with a fixed name and a fixed message format).",
            "<b>Message type</b> = the shape of the data on a topic, e.g. /joint_states "
            "carries a sensor_msgs/JointState (joint names + angles).",
        ]),
        Paragraph("Nodes never call each other directly: a <b>publisher</b> posts to a topic "
                  "and any <b>subscriber</b> on that topic receives it. For /joint_states there "
                  "is 1 publisher (sine_ik_node) and 1 subscriber (robot_state_publisher).", BODY),
        Spacer(1, 0.4 * cm),

        Paragraph("The data flow (RViz pipeline)", H1),
        scaled(DIAG),
        Spacer(1, 0.2 * cm),
        Preformatted(DIAGRAM_TXT, MONO),
        Spacer(1, 0.3 * cm),

        Paragraph("Step by step", H2),
        *bullets([
            "<b>sine_ik_node</b> precomputes the inverse kinematics once, then publishes the "
            "6 joint angles to /joint_states 30 times per second. It also posts the green "
            "target sine to /target_sine_path. It only talks, never listens.",
            "<b>robot_state_publisher</b> is given the URDF (the robot geometry) once and "
            "subscribes to /joint_states. For each new set of angles it computes where every "
            "link ends up and publishes that as TF.",
            "<b>TF (/tf)</b> is the tree of coordinate frames — the position and orientation "
            "of base_link, link1 ... link6_flange relative to one another. It is a second, "
            "independent forward kinematics (done from the URDF), which cross-checks ours.",
            "<b>path_tracer_node</b> subscribes to /tf, asks 'where is link6_flange relative "
            "to base_link?', and appends that point to /drawn_path — the red drawn line.",
            "<b>RViz</b> subscribes to /robot_description (draws the mesh), /tf (poses it), and "
            "the two path topics (green target, red drawn).",
        ]),
        Paragraph("The launch file (rsine_sine.launch.py) simply starts these processes, hands "
                  "the URDF to robot_state_publisher, and passes the sine parameters to "
                  "sine_ik_node. One launch = the whole graph above.", BODY),
        Spacer(1, 0.4 * cm),

        Paragraph("Why Gazebo is different", H1),
        Paragraph("In the RViz pipeline WE choose the joint angles (our IK) and simply announce "
                  "them — nothing pushes back. It is kinematic: no gravity, motors, or contact. "
                  "Gazebo simulates physics, so you cannot teleport joints; you COMMAND them and "
                  "a controller drives the motors while physics responds. That is what "
                  "ros2_control provides:", BODY),
        *bullets([
            "<b>controller_manager</b> — the conductor; loads and runs controllers.",
            "<b>hardware interface</b> — what controllers talk to. In Gazebo it is "
            "gz_ros2_control (the simulated motors); on the real arm it is the serial driver. "
            "Same controllers, swap the hardware.",
            "<b>joint_trajectory_controller</b> — takes a timed trajectory and drives the "
            "joints to it smoothly.",
            "<b>joint_state_broadcaster</b> — reads the ACTUAL joint positions back from the "
            "sim and publishes /joint_states (the reverse direction from the RViz setup).",
        ]),
        Paragraph("So in Gazebo the flow flips: sine_ik_node -> controller command -> (physics) "
                  "-> joint_state_broadcaster -> /joint_states -> robot_state_publisher -> TF -> "
                  "RViz/Gazebo. Two things must be added: a controller config (YAML) and "
                  "ros2_control tags in the URDF marking which joints have motors.", BODY),
        Spacer(1, 0.3 * cm),
        Paragraph("Key mental model: RViz shows what you command; Gazebo shows what physically "
                  "happens when you command it.", H2),
    ]

    doc = SimpleDocTemplate(OUT, pagesize=A4, topMargin=1.6 * cm, bottomMargin=1.6 * cm,
                            leftMargin=2.3 * cm, rightMargin=2.3 * cm,
                            title="How Your ROS 2 System Works")
    doc.build(story)
    print("wrote", DIAG)
    print("wrote", OUT)


if __name__ == "__main__":
    make_diagram()
    build_pdf()
