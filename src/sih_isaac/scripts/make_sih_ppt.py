"""Build team QSLAM's SIH 2026 idea deck inside the official template.

    .demoenv/bin/python src/sih_isaac/scripts/make_sih_ppt.py

Content follows the team's registered idea (vision + map-anchored navigation
with no GNSS at any stage). Presentation is points, cards, stat tiles and
diagrams -- no paragraphs. The instructions slide is removed as the template
permits. Writes SIH26126_IDEA_Presentation.pptx.

Evidence used, and how it is labelled:
  * PoC-1 / PoC-2 figures come from the team's own proof-of-concept study and
    are labelled as such.
  * The Isaac Sim figures come from the recorded run in this workspace and are
    labelled as integration-testbed results for the navigation backbone.
"""
import pathlib
import shutil

from lxml import etree
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import PP_ALIGN
from pptx.util import Inches, Pt

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

WS = pathlib.Path("/home/qbotix-rover/sih_ws")
HOME = pathlib.Path("/home/qbotix-rover")
TEMPLATE = next((p for p in (WS / "SIH2026-IDEA-Presentation-Format.pptx",
                             HOME / "SIH2026-IDEA-Presentation-Format.pptx")
                 if p.exists()), WS / "SIH2026-IDEA-Presentation-Format.pptx")
OUT = WS / "SIH26126_IDEA_Presentation.pptx"
BUILD = WS / "mission_recordings/ppt_assets"
BUILD.mkdir(parents=True, exist_ok=True)

TEAM = "QSLAM"
NAVY, BLUE, GREEN, STAR = "1F4E79", "2E74B5", "2C5F2D", "B5651D"
INK, GREY = "1A1A1A", "4A4A4A"
TINT, TINT2, TINTG = "EEF3F9", "F7F3EC", "EAF1EA"
A_NS = "http://schemas.openxmlformats.org/drawingml/2006/main"


def a(tag):
    return f"{{{A_NS}}}{tag}"


# ----------------------------------------------------------- pipeline art --
def pipeline_diagram(out_png):
    fig, ax = plt.subplots(figsize=(5.6, 3.86))
    ax.set_xlim(0, 560)
    ax.set_ylim(54, 440)
    ax.axis("off")

    def box(x, y, w, h, title, sub, fc, ec, star=False, ts=8.4, ss=6.9):
        ax.add_patch(FancyBboxPatch(
            (x, y), w, h, boxstyle="round,pad=0,rounding_size=6",
            fc=fc, ec=ec, lw=1.2))
        ax.text(x + w / 2, y + (h * 0.63 if sub else h * 0.5),
                ("★  " if star else "") + title, ha="center",
                va="center", fontsize=ts, fontweight="bold",
                color="#" + (STAR if star else NAVY))
        if sub:
            ax.text(x + w / 2, y + h * 0.27, sub, ha="center", va="center",
                    fontsize=ss, color="#444444")

    def arrow(p1, p2, color="#8a8a8a", dashed=False, lw=1.3):
        ax.annotate("", xy=p2, xytext=p1, arrowprops=dict(
            arrowstyle="-|>", color=color, lw=lw,
            linestyle="--" if dashed else "-",
            shrinkA=0, shrinkB=0, mutation_scale=13))

    ax.text(14, 424, "ONE-TIME INITIALISATION", fontsize=7.4,
            fontweight="bold", color="#777777")
    box(14, 366, 248, 46, "Mission input",
        "Point B in UTM  +  offline satellite / DEM tile", "#FFFFFF", "#B9C9D9")
    box(298, 366, 248, 46, "Map-anchored localization",
        "BEV + 2.5D elevation  ↔  satellite / DEM", "#FBF3E8",
        "#D9B98C", star=True)
    arrow((264, 389), (296, 389))
    ax.text(280, 348, "start pose  ·  heading  ·  UTM ↔ map "
                      "fixed without GPS", ha="center", fontsize=7.2,
            style="italic", color="#666666")
    ax.plot([14, 546], [336, 336], color="#DDDDDD", lw=1)

    ax.text(14, 322, "AUTONOMY LOOP — repeats until Point B",
            fontsize=7.4, fontweight="bold", color="#777777")
    chain = [
        ("Sense", "ZED X stereo (RGB · depth · IMU) + wheel encoders",
         False),
        ("Localize", "cuVSLAM VIO + IMU + wheel odometry  →  EKF pose",
         False),
        ("Perceive", "semantic segmentation + GPU 2.5D elevation map", False),
        ("Score terrain", "traversability  +  ★ localizability cost layer",
         False),
        ("Plan & act", "Hybrid-A*  ·  MPPI  ·  Collision Monitor",
         False),
    ]
    ys = [258, 210, 162, 114, 66]
    for (title, sub, st), y in zip(chain, ys):
        fc = "#FBF3E8" if "localizability" in sub else "#F4F7FA"
        ec = "#D9B98C" if "localizability" in sub else "#B9C9D9"
        box(96, y, 300, 40, title, sub, fc, ec, star=st, ts=8.2, ss=6.5)
    for y1, y2 in zip(ys[:-1], ys[1:]):
        arrow((246, y1), (246, y2 + 40))

    arrow((96, 86), (62, 86), color="#9aa5b1")
    ax.plot([62, 62], [86, 278], color="#9aa5b1", lw=1.3)
    arrow((62, 278), (96, 278), color="#9aa5b1")
    ax.text(58, 182, "loop", rotation=90, ha="right", va="center",
            fontsize=6.8, color="#888888")

    box(412, 232, 134, 62, "Integrity\nmonitor",
        "VIO vs IMU + wheels\n→ degraded mode", "#FBF3E8", "#D9B98C",
        star=True, ts=7.6, ss=6.3)
    arrow((410, 230), (398, 222), color="#C0A176")
    ax.annotate("", xy=(479, 364), xytext=(479, 296),
                arrowprops=dict(arrowstyle="-|>", color="#C0A176", lw=1.3,
                                linestyle="--", mutation_scale=13))
    ax.text(484, 318, "re-anchor\n~200 m", fontsize=6.5, color="#9a7b4f",
            va="center")
    fig.tight_layout(pad=0.15)
    fig.savefig(out_png, dpi=200)
    plt.close(fig)
    return out_png


PIPE = pipeline_diagram(BUILD / "qslam_pipeline.png")

# ------------------------------------------------------------------ setup --
shutil.copy(TEMPLATE, OUT)
prs = Presentation(str(OUT))
sldIdLst = prs.slides._sldIdLst
last = sldIdLst[-1]
prs.part.drop_rel(last.rId)
sldIdLst.remove(last)
slides = list(prs.slides)


def clear_tf(tf):
    for p in list(tf.paragraphs):
        p._p.getparent().remove(p._p)


def para(tf, text, size, bold=False, color=INK, bullet=False, before=5,
         italic=False, align=None):
    p = tf.add_paragraph()
    r = p.add_run()
    r.text = text
    r.font.size, r.font.bold, r.font.italic = Pt(size), bold, italic
    r.font.name = "Arial"
    r.font.color.rgb = RGBColor.from_string(color)
    if align is not None:
        p.alignment = align
    pPr = p._p.get_or_add_pPr()
    if bullet:
        pPr.set("marL", "155575")
        pPr.set("indent", "-155575")
        etree.SubElement(pPr, a("buFont")).set("typeface", "Arial")
        etree.SubElement(pPr, a("buChar")).set("char", "•")
    else:
        etree.SubElement(pPr, a("buNone"))
    sp = etree.Element(a("spcBef"))
    etree.SubElement(sp, a("spcPts")).set("val", str(int(before * 100)))
    pPr.insert(0, sp)
    return p


def add_run(p, text, size, bold=False, color=INK):
    r = p.add_run()
    r.text = text
    r.font.size, r.font.bold, r.font.name = Pt(size), bold, "Arial"
    r.font.color.rgb = RGBColor.from_string(color)
    return r


def textbox(slide, x, y, w, h):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Inches(0.05)
    tf.margin_top = tf.margin_bottom = Inches(0.02)
    clear_tf(tf)
    return tf


def card(slide, x, y, w, h, fill=TINT, line=None):
    sh = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x),
                                Inches(y), Inches(w), Inches(h))
    sh.fill.solid()
    sh.fill.fore_color.rgb = RGBColor.from_string(fill)
    if line:
        sh.line.color.rgb = RGBColor.from_string(line)
        sh.line.width = Pt(0.75)
    else:
        sh.line.fill.background()
    sh.shadow.inherit = False
    try:
        sh.adjustments[0] = 0.07
    except Exception:
        pass
    return sh


def section(slide, x, y, w, text, size=13.5, color=NAVY):
    tf = textbox(slide, x, y, w, 0.32)
    para(tf, text, size, bold=True, color=color, before=0)


def multiline(tf, text, size, color=GREY, align=PP_ALIGN.CENTER, before=0,
              bold=False):
    """One paragraph per line -- a '\\n' inside a run does not take alignment."""
    for i, line in enumerate(text.split("\n")):
        para(tf, line, size, bold=bold, color=color, before=0 if i == 0
             else before, align=align)


def stat(slide, x, y, w, h, value, label, color=BLUE, fill=TINT, vsize=21,
         lsize=9):
    card(slide, x, y, w, h, fill)
    tf = textbox(slide, x + 0.05, y + 0.08, w - 0.10, h * 0.48)
    para(tf, value, vsize, bold=True, color=color, before=0,
         align=PP_ALIGN.CENTER)
    tf2 = textbox(slide, x + 0.05, y + h * 0.48, w - 0.10, h * 0.50)
    multiline(tf2, label, lsize, before=1)


def drop(slide, name):
    for sh in list(slide.shapes):
        if sh.name == name:
            sh._element.getparent().remove(sh._element)


def by_name(slide, name):
    for sh in slide.shapes:
        if sh.name == name:
            return sh
    raise KeyError(name)


def set_box(sh, x, y, w, h):
    sh.left, sh.top, sh.width, sh.height = (Inches(x), Inches(y), Inches(w),
                                            Inches(h))


# ------------------------------------------------------------- slide 1 -----
FIELDS = [
    ("Problem Statement ID", "SIH26126"),
    ("Problem Statement Title", "Vision Based Autonomous Navigation for "
                                "Unmanned Ground Vehicle for Outdoor "
                                "environment"),
    ("Theme", "Smart Automation"),
    ("PS Category", "Software"),
    ("Team ID", "14"),
    ("Team Name", TEAM),
]
box_ = by_name(slides[0], "TextBox 9")
set_box(box_, 0.36, 2.05, 6.55, 5.30)
tf = box_.text_frame
tf.word_wrap = True
clear_tf(tf)
for label, value in FIELDS:
    p = para(tf, f"{label} – ", 14.5, bold=True, bullet=True, before=11)
    add_run(p, value, 14.5, color=INK)

for s in slides[1:6]:
    for sh in s.shapes:
        if sh.has_text_frame and sh.text_frame.text.strip() == "Your Team Name":
            t = sh.text_frame
            p0 = t.paragraphs[0]
            p0.runs[0].text = TEAM
            p0.runs[0].font.size = Pt(15)
            p0.runs[0].font.bold = True
            for r in p0.runs[1:]:
                r._r.getparent().remove(r._r)
            for p in t.paragraphs[1:]:
                p._p.getparent().remove(p._p)

# ------------------------------------------------------------- slide 2 -----
s = slides[1]
drop(s, "TextBox 8")
t_sh = by_name(s, "Title 1")
set_box(t_sh, 1.85, 0.02, 8.75, 1.02)
tp = t_sh.text_frame.paragraphs[0]
tp.runs[0].text = "QSLAM — maps and vision replace GPS, end to end"
tp.runs[0].font.size = Pt(25)
for r in tp.runs[1:]:
    r._r.getparent().remove(r._r)
for p in t_sh.text_frame.paragraphs[1:]:
    p._p.getparent().remove(p._p)

section(s, 0.40, 1.12, 6.4, "Proposed solution")
tf = textbox(s, 0.40, 1.44, 6.45, 1.50)
for b in ("Operator gives one destination in UTM — nothing else",
          "Stereo vision, IMU and wheel odometry carry the UGV there",
          "Preloaded satellite / DEM maps replace the satellite fix",
          "No GNSS receiver at any stage — nothing to jam or spoof"):
    para(tf, b, 11.5, bullet=True, before=4)

section(s, 0.40, 3.02, 6.4, "How it addresses the problem")
tf = textbox(s, 0.40, 3.34, 6.45, 1.55)
for b in ("Start pose and heading from map matching, no compass needed",
          "Map re-anchoring every ~200 m keeps drift bounded on one-way runs",
          "Semantic + 2.5D elevation perception finds safe ground",
          "Hybrid-A* and MPPI drive it; Collision Monitor guards it"):
    para(tf, b, 11.5, bullet=True, before=4)

section(s, 7.10, 1.12, 5.8, "Innovation and uniqueness", color=STAR)
NOV = [
    ("Map-anchored global localization",
     "Passive BEV ↔ satellite / DEM matching fixes pose, heading and "
     "drift — the role GPS used to play."),
    ("Localizability-aware navigation",
     "A custom Nav2 cost layer scores feature density, so the route stays "
     "where vision keeps working."),
    ("Localization integrity monitor",
     "Cross-checks VIO against IMU and wheels every cycle — degraded "
     "mode instead of silent drift."),
]
cy = 1.44
for title, detail in NOV:
    card(s, 7.10, cy, 5.83, 1.06, TINT2)
    tfc = textbox(s, 7.24, cy + 0.09, 5.55, 0.92)
    p = tfc.add_paragraph()
    etree.SubElement(p._p.get_or_add_pPr(), a("buNone"))
    add_run(p, "★  ", 12, bold=True, color=STAR)
    add_run(p, title, 12, bold=True, color=NAVY)
    para(tfc, detail, 10, color=GREY, before=3)
    cy += 1.16

sw = (12.46 - 2 * 0.18) / 3
stat(s, 0.40, 5.20, sw, 1.28, "− 88 %",
     "position error after 3 km, no GNSS\n89.6 m → 10.6 m   (PoC)")
stat(s, 0.40 + sw + 0.18, 5.20, sw, 1.28, "− 59 %",
     "arrival error vs traversability-only\nplanner, for +18 % path   (PoC)")
stat(s, 0.40 + 2 * (sw + 0.18), 5.20, sw, 1.28, "0",
     "GNSS receivers, at any stage\nof any mission", color=GREEN, fill=TINTG)

# ------------------------------------------------------------- slide 3 -----
s = slides[2]
drop(s, "TextBox 8")
section(s, 0.40, 1.12, 6.4, "Technologies to be used")
STACK = [
    ("Sensing", "ZED X stereo camera (RGB · depth · IMU) · "
                "wheel encoders · no GNSS receiver"),
    ("Localization", "Isaac ROS cuVSLAM · robot_localization EKF · "
                     "cross-view BEV-to-satellite matcher · GeographicLib"),
    ("Perception", "SegFormer fine-tuned on RELLIS-3D + RUGD · TensorRT "
                   "· elevation_mapping_cupy (GPU)"),
    ("Planning & control", "ROS 2 Nav2 · Smac Hybrid-A* · MPPI "
                           "· Collision Monitor · custom C++ "
                           "costmap plugins"),
    ("Platform", "ROS 2 Jazzy · CUDA · C++17 / Python · Jetson "
                 "AGX Orin target · Isaac Sim 6 on RTX 4090"),
    ("Maps & data", "ISRO Bhuvan / Cartosat imagery + DEM, preloaded offline "
                    "· RELLIS-3D, RUGD for training"),
]
cy = 1.44
for title, detail in STACK:
    card(s, 0.40, cy, 6.45, 0.70)
    tfc = textbox(s, 0.53, cy + 0.04, 6.20, 0.62)
    p = tfc.add_paragraph()
    etree.SubElement(p._p.get_or_add_pPr(), a("buNone"))
    add_run(p, "●  ", 11, bold=True, color=BLUE)
    add_run(p, title, 11, bold=True, color=NAVY)
    para(tfc, detail, 9.5, color=GREY, before=2)
    cy += 0.76

card(s, 0.40, 6.02, 6.45, 0.62, TINTG)
tf = textbox(s, 0.50, 6.12, 6.25, 0.45)
para(tf, "Proven components reused · QSLAM adds the new modules", 10.5,
     bold=True, color=GREEN, before=0, align=PP_ALIGN.CENTER)

section(s, 7.10, 1.12, 5.8, "Methodology and process for implementation")
s.shapes.add_picture(str(PIPE), Inches(7.10), Inches(1.44), width=Inches(5.83),
                     height=Inches(5.83 * 3.86 / 5.6))
card(s, 7.10, 5.62, 5.83, 1.02, TINT2)
tfc = textbox(s, 7.24, 5.72, 5.55, 0.88)
p = tfc.add_paragraph()
etree.SubElement(p._p.get_or_add_pPr(), a("buNone"))
add_run(p, "★  ", 11, bold=True, color=STAR)
add_run(p, "marks QSLAM's own contribution", 11, bold=True, color=NAVY)
para(tfc, "Everything else is production-grade, open-source software we "
          "reuse rather than rebuild.", 9.5, color=GREY, before=3)

# ------------------------------------------------------------- slide 4 -----
s = slides[3]
drop(s, "TextBox 8")
section(s, 0.40, 1.10, 9.0,
        "Analysis of the feasibility — measured, not asserted")
cw = (12.46 - 2 * 0.18) / 3
FEAS = [
    ("− 59 %", "PoC-1  ·  localizability layer\narrival error vs "
                    "traversability-only planner (+18 % path)\n1 km synthetic "
                    "terrain · 300 Monte-Carlo runs", BLUE, TINT),
    ("− 88 %", "PoC-2  ·  map anchoring\nposition error after 3 km "
                    "with no GNSS (89.6 m → 10.6 m)\nmatch σ 5 m "
                    "· 75 % success · 300 runs", BLUE, TINT),
    ("189 m", "Integration testbed  ·  Isaac Sim 6\nGPS-free traverse to "
              "a UTM goal, 1.09 m drift\n7 recoveries handled autonomously",
     GREEN, TINTG),
]
for i, (v, lab, col, fill) in enumerate(FEAS):
    x = 0.40 + i * (cw + 0.18)
    card(s, x, 1.44, cw, 1.62, fill)
    tfc = textbox(s, x + 0.08, 1.52, cw - 0.16, 0.55)
    para(tfc, v, 23, bold=True, color=col, before=0, align=PP_ALIGN.CENTER)
    tfc2 = textbox(s, x + 0.08, 2.05, cw - 0.16, 0.95)
    multiline(tfc2, lab, 9, before=2)

section(s, 0.40, 3.22, 9.0,
        "Potential challenges and risks  →  strategies for overcoming them")
ROWS = [
    ("Start pose and heading unknown",
     "BEV-to-satellite / DEM match gives UTM pose and heading; surveyed "
     "launch point as fallback"),
    ("VIO drift on one-way runs",
     "Re-anchor to the map every ~200 m; wheel-odometry EKF and "
     "localizability-aware routing bound the error"),
    ("Goal lies beyond sensor range",
     "Rolling global costmap; unknown cells carry an optimistic cost; "
     "Hybrid-A* replans as the map fills"),
    ("Ditches, slopes and drops",
     "GPU 2.5D elevation map turns slope, step height and roughness into "
     "costs, fused with semantic classes"),
    ("Vision degrades — night, dust, low texture",
     "IMU and wheel dead-reckoning bridge outages; degraded mode slows, then "
     "safe-stops; thermal as an upgrade"),
    ("Onboard compute budget",
     "cuVSLAM, TensorRT and CuPy keep every heavy stage on GPU; per-node "
     "latency tracked for Jetson AGX Orin"),
]
tbl = s.shapes.add_table(len(ROWS) + 1, 2, Inches(0.40), Inches(3.56),
                         Inches(12.46), Inches(0.4)).table
tbl.first_row = True
tbl.horz_banding = False
tbl.columns[0].width = Inches(3.95)
tbl.columns[1].width = Inches(8.51)
tbl.rows[0].height = Inches(0.32)
for c, h in enumerate(("Challenge / risk", "Strategy for overcoming it")):
    cell = tbl.cell(0, c)
    cell.fill.solid()
    cell.fill.fore_color.rgb = RGBColor.from_string(NAVY)
    clear_tf(cell.text_frame)
    para(cell.text_frame, h, 10.5, bold=True, color="FFFFFF", before=0)
    cell.margin_left = cell.margin_right = Inches(0.08)
    cell.margin_top = cell.margin_bottom = Inches(0.02)
for r, (ch, ans) in enumerate(ROWS, start=1):
    tbl.rows[r].height = Inches(0.47)
    for c, txt in enumerate((ch, ans)):
        cell = tbl.cell(r, c)
        cell.fill.solid()
        cell.fill.fore_color.rgb = RGBColor.from_string(
            "FFFFFF" if r % 2 else TINT)
        ctf = cell.text_frame
        ctf.word_wrap = True
        clear_tf(ctf)
        para(ctf, txt, 10, bold=(c == 0), color=NAVY if c == 0 else INK,
             before=0)
        cell.margin_left = cell.margin_right = Inches(0.08)
        cell.margin_top = cell.margin_bottom = Inches(0.02)

# ------------------------------------------------------------- slide 5 -----
s = slides[4]
drop(s, "TextBox 8")
section(s, 0.40, 1.10, 8.0, "Potential impact on the target audience")
AUD = [
    ("Armed forces", "Navigate where GNSS is unavailable, jammed or spoofed"),
    ("BEL & defence industry",
     "Drop-in ROS 2 navigation module for existing UGV platforms"),
    ("Civil GNSS-denied users",
     "Disaster response, mines, tunnels and dense forest"),
]
aw = (12.46 - 2 * 0.18) / 3
for i, (t_, d_) in enumerate(AUD):
    x = 0.40 + i * (aw + 0.18)
    card(s, x, 1.42, aw, 0.90)
    tfc = textbox(s, x + 0.12, 1.48, aw - 0.24, 0.80)
    p = tfc.add_paragraph()
    etree.SubElement(p._p.get_or_add_pPr(), a("buNone"))
    add_run(p, "●  ", 11.5, bold=True, color=BLUE)
    add_run(p, t_, 11.5, bold=True, color=NAVY)
    para(tfc, d_, 10, color=GREY, before=3)

section(s, 0.40, 2.50, 8.0, "Mission applications")
APPS = ["Reconnaissance", "Border surveillance", "Last-mile resupply",
        "Hazardous / CBRN zones"]
pw = (12.46 - 3 * 0.16) / 4
for i, t_ in enumerate(APPS):
    x = 0.40 + i * (pw + 0.16)
    card(s, x, 2.82, pw, 0.50, TINT2)
    tfc = textbox(s, x + 0.06, 2.90, pw - 0.12, 0.36)
    para(tfc, t_, 11, bold=True, color=STAR, before=0, align=PP_ALIGN.CENTER)

section(s, 0.40, 3.48, 8.0, "Benefits of the solution")
BEN = [
    ("Strategic", "Immune to jamming and spoofing by design — no "
                  "satellite signal is used at any stage"),
    ("Social / safety", "The UGV takes the risky last mile; fewer soldiers "
                        "exposed in hazardous zones"),
    ("Economic", "COTS camera and open-source ROS 2 stack; retrofits with no "
                 "ground infrastructure"),
    ("Self-reliance", "Indigenous, auditable navigation software BEL can own, "
                      "adapt and certify"),
    ("Environmental", "Passive sensing, no RF emissions; suits electric UGVs "
                      "and cuts manned sorties"),
]
for i, (label, detail) in enumerate(BEN):
    x = 0.40 + (i % 2) * 6.30
    y = 3.82 + (i // 2) * 0.46
    tfc = textbox(s, x, y, 6.16, 0.44)
    p = tfc.add_paragraph()
    etree.SubElement(p._p.get_or_add_pPr(), a("buNone"))
    add_run(p, f"{label} — ", 10.5, bold=True, color=NAVY)
    add_run(p, detail, 10, color=INK)

section(s, 0.40, 5.34, 9.5,
        "Validation targets  ·  Isaac Sim first, then field trials")
TGT = [("≥ 90 %", "GPS-denied mission\nsuccess (A → B)"),
       ("≤ 2 %", "position error on\narrival, as % of distance"),
       ("≤ 5 m / 2°", "start alignment from\nmap matching"),
       ("≤ 100 ms", "sensing-to-command\nlatency"),
       ("≥ 15 FPS", "perception rate on\nJetson AGX Orin")]
tw = (12.46 - 4 * 0.14) / 5
for i, (v, lab) in enumerate(TGT):
    stat(s, 0.40 + i * (tw + 0.14), 5.66, tw, 1.00, v, lab, vsize=15,
         lsize=8.5)

# ------------------------------------------------------------- slide 6 -----
s = slides[5]
drop(s, "TextBox 8")
section(s, 0.40, 1.08, 9.0,
        "Details / links of the reference and research work")
GROUPS = [
    (0.40, 4.05, [
        ("Localization & SLAM", [
            "[1] Korovko et al., “cuVSLAM: CUDA Accelerated Visual "
            "Odometry and Mapping,” arXiv:2506.04359, 2025",
            "[2] NVIDIA Isaac ROS Visual SLAM — "
            "github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam",
            "[3] Cadena et al., “Past, Present and Future of SLAM,” "
            "IEEE T-RO, 2016",
            "[4] Moore & Stouch, “Generalized EKF for ROS” "
            "(robot_localization), IAS-13, 2014"]),
        ("Data & tools", [
            "[14] ISRO / NRSC Bhuvan geoportal — imagery and DEM "
            "— bhuvan.nrsc.gov.in",
            "[15] NVIDIA Isaac Sim · Stereolabs ZED X SDK · ROS 2 "
            "Jazzy documentation"])]),
    (4.61, 4.05, [
        ("Navigation & planning", [
            "[5] Macenski et al., “The Marathon 2: A Navigation "
            "System,” IROS 2020 — docs.nav2.org",
            "[6] Macenski et al., “Cost-Aware Kinematically Feasible "
            "Planning” (Smac), arXiv:2401.13078, 2024",
            "[7] Williams et al., “Information Theoretic MPC” "
            "(MPPI), ICRA 2017"]),
        ("Perception & traversability", [
            "[8] Miki et al., “Elevation Mapping using GPU,” IROS "
            "2022 — elevation_mapping_cupy",
            "[9] Xie et al., “SegFormer,” NeurIPS 2021",
            "[10] Jiang et al., RELLIS-3D, ICRA 2021; Wigness et al., RUGD, "
            "IROS 2019"])]),
    (8.82, 4.04, [
        ("Basis for QSLAM innovations", [
            "[11] Costante et al., “Perception-aware Path Planning,” "
            "arXiv:1605.04151 → localizability layer",
            "[12] Shi & Li, “Highly Accurate Vehicle Localization Using "
            "Satellite Image,” CVPR 2022 → map anchoring",
            "[13] Sarlin et al., “OrienterNet,” CVPR 2023"])]),
]
for x, w, groups in GROUPS:
    y = 1.40
    for gname, refs in groups:
        section(s, x, y, w, gname, size=11)
        y += 0.32
        tfc = textbox(s, x, y, w, 0.40 * len(refs) + 0.4)
        for rtext in refs:
            para(tfc, rtext, 9.2, color=GREY, before=5)
        y += 0.40 * len(refs) + 0.28
    if x > 8.0:   # the innovations column: call out why these papers matter
        card(s, x, y + 0.06, w, 1.34, TINT2)
        tfc = textbox(s, x + 0.12, y + 0.16, w - 0.24, 1.18)
        p = tfc.add_paragraph()
        etree.SubElement(p._p.get_or_add_pPr(), a("buNone"))
        add_run(p, "★  ", 11, bold=True, color=STAR)
        add_run(p, "Why these three matter", 11, bold=True, color=NAVY)
        para(tfc, "They are the published basis for QSLAM's own modules: "
                  "map anchoring, the localizability layer and the integrity "
                  "monitor.", 9.2, color=GREY, before=3)

BUILDS = [
    ("Reused, proven", "cuVSLAM · Nav2 (Smac, MPPI, Collision Monitor) "
                       "· SegFormer · GPU elevation mapping", TINT),
    ("QSLAM contribution", "Map-anchored localization · localizability "
                           "layer · integrity monitor · "
                           "degraded-mode policy", TINT2),
    ("Next research step", "Cross-view matcher fine-tuned on Indian terrain "
                           "(Bhuvan / Cartosat) · thermal for night",
     TINTG),
]
bw = (12.46 - 2 * 0.18) / 3
for i, (t_, d_, fill) in enumerate(BUILDS):
    x = 0.40 + i * (bw + 0.18)
    card(s, x, 5.42, bw, 1.16, fill)
    tfc = textbox(s, x + 0.12, 5.50, bw - 0.24, 1.00)
    para(tfc, t_, 11, bold=True, color=NAVY, before=0)
    para(tfc, d_, 9.5, color=GREY, before=3)

prs.save(str(OUT))
print(f"wrote {OUT}")
