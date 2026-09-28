"""Build the post-mission PDF report from a record_mission.sh run directory.

    .demoenv/bin/python src/sih_isaac/scripts/make_mission_report.py <run_dir> \
        [--ffmpeg PATH]

Reads mission_log.csv, goal.log and the recorded videos; writes
<run_dir>/SIH26126_Mission_Report.pdf and copies it to the workspace root.
"""
import argparse
import math
import pathlib
import re
import shutil
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image as PILImage

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (Image, PageBreak, Paragraph,
                                SimpleDocTemplate, Spacer, Table, TableStyle)

WS = pathlib.Path("/home/qbotix-rover/sih_ws")
GEN = WS / "src/sih_isaac/generated"

BLUE, ORANGE = "#2E74B5", "#E07B39"   # validated pair (CVD dE 21.8)
ACCENT = colors.HexColor("#1F4E79")
INK = colors.HexColor("#1a1a1a")
FAINT = colors.HexColor("#666666")

ap = argparse.ArgumentParser()
ap.add_argument("run_dir")
ap.add_argument("--ffmpeg", default="ffmpeg")
args = ap.parse_args()
RUN = pathlib.Path(args.run_dir).resolve()

# ------------------------------------------------------------ parse inputs --
goal_txt = (RUN / "goal.log").read_text() if (RUN / "goal.log").exists() else ""
m = re.search(r"goal E ([\d.]+) N ([\d.]+)\s+->\s+map \(([-\d.]+), ([-\d.]+)\)",
              goal_txt)
goal_e, goal_n, goal_x, goal_y = (map(float, m.groups()) if m
                                  else (0.0, 0.0, 0.0, 0.0))
recoveries = 0
for mm_ in re.finditer(r"recoveries (\d+)", goal_txt):
    recoveries = int(mm_.group(1))
status = re.search(r"finished with status (\d+)", goal_txt)
status = int(status.group(1)) if status else -1
reached = "GOAL REACHED" in goal_txt

rows = []
csv_path = RUN / "mission_log.csv"
if csv_path.exists():
    for ln in csv_path.read_text().splitlines()[1:]:
        p = ln.split(",")
        if len(p) == 8:
            rows.append([float(v) for v in p])
data = np.array(rows) if rows else np.zeros((0, 8))
t, gtx, gty = data[:, 0], data[:, 1], data[:, 2]
ekx, eky, cmdv = data[:, 4], data[:, 5], data[:, 6]

dist_goal = np.hypot(gtx - goal_x, gty - goal_y)
loc_err = np.hypot(gtx - ekx, gty - eky)
path_len = float(np.sum(np.hypot(np.diff(gtx), np.diff(gty))))
moving = np.abs(cmdv) > 0.05
move_time = float(np.sum(np.diff(t)[moving[:-1]])) if len(t) > 1 else 0.0
duration = float(t[-1] - t[0]) if len(t) else 0.0

def fmt(v, unit="", nd=2):
    return f"{v:.{nd}f}{unit}"

# ------------------------------------------------------------------ charts --
plt.rcParams.update({
    "font.size": 9, "axes.edgecolor": "#cccccc", "axes.linewidth": 0.8,
    "axes.grid": True, "grid.color": "#e6e6e6", "grid.linewidth": 0.6,
    "axes.axisbelow": True, "figure.facecolor": "white",
})

# trajectory over the prior map
fig, ax = plt.subplots(figsize=(7.2, 6.4))
try:
    yml = (GEN / "map.yaml").read_text()
    res = float(re.search(r"resolution: *([\d.]+)", yml).group(1))
    ox, oy = map(float, re.search(
        r"origin: *\[([-\d.]+), *([-\d.]+)", yml).groups())
    img = np.asarray(PILImage.open(GEN / "map.pgm"))
    h, w = img.shape[:2]
    ax.imshow(img, cmap="gray", vmin=0, vmax=255,
              extent=[ox, ox + w * res, oy, oy + h * res], alpha=0.75)
except Exception as e:
    print(f"[report] no map background: {e}")
ax.plot(gtx, gty, color=BLUE, lw=1.8, label="ground truth (sim state)")
ax.plot(ekx, eky, color=ORANGE, lw=1.6, ls="--", label="EKF estimate (GPS-free)")
ax.plot(gtx[0], gty[0], "o", color=BLUE, ms=8, mec="white")
ax.plot(goal_x, goal_y, "*", color="#1F4E79", ms=16, mec="white")
i3 = len(gtx) // 3
ax.annotate("ground truth", (gtx[i3], gty[i3]), textcoords="offset points",
            xytext=(8, 8), color=BLUE, fontsize=9, fontweight="bold")
ax.annotate("EKF estimate", (ekx[2 * i3], eky[2 * i3]),
            textcoords="offset points", xytext=(8, -14), color=ORANGE,
            fontsize=9, fontweight="bold")
ax.annotate("start", (gtx[0], gty[0]), textcoords="offset points",
            xytext=(6, -12), color="#444444", fontsize=8)
ax.annotate("goal", (goal_x, goal_y), textcoords="offset points",
            xytext=(8, 4), color="#1F4E79", fontsize=8)
ax.set_xlabel("map x (m, east)")
ax.set_ylabel("map y (m, north)")
ax.set_title("Mission trajectory over the GeoTIFF prior map", fontsize=11,
             color="#1F4E79", fontweight="bold")
ax.legend(loc="lower right", framealpha=0.9, fontsize=8)
ax.set_aspect("equal")
fig.tight_layout()
fig.savefig(RUN / "fig_trajectory.png", dpi=150)
plt.close(fig)

# telemetry: three stacked single-axis panels
fig, axs = plt.subplots(3, 1, figsize=(7.2, 6.0), sharex=True)
axs[0].plot(t - t[0], dist_goal, color=BLUE, lw=1.6)
axs[0].set_ylabel("distance to goal (m)")
axs[0].annotate(f"final {dist_goal[-1]:.2f} m", (t[-1] - t[0], dist_goal[-1]),
                textcoords="offset points", xytext=(-70, 10), color=BLUE,
                fontsize=9, fontweight="bold")
axs[1].plot(t - t[0], loc_err, color=ORANGE, lw=1.6)
axs[1].set_ylabel("localization error (m)")
imax = int(np.argmax(loc_err))
axs[1].annotate(f"max {loc_err[imax]:.2f} m", (t[imax] - t[0], loc_err[imax]),
                textcoords="offset points", xytext=(6, 4), color=ORANGE,
                fontsize=9, fontweight="bold")
axs[2].plot(t - t[0], cmdv, color=BLUE, lw=1.2)
axs[2].set_ylabel("commanded speed (m/s)")
axs[2].set_xlabel("mission time (s)")
for a in axs:
    a.margins(x=0.01)
fig.suptitle("Telemetry (5 Hz, sim time)", fontsize=11, color="#1F4E79",
             fontweight="bold")
fig.tight_layout()
fig.savefig(RUN / "fig_telemetry.png", dpi=150)
plt.close(fig)

# ------------------------------------------------------------- video stills --
def video_meta(p):
    """(duration seconds, 'WxH') for a video file."""
    try:
        out = subprocess.run([args.ffmpeg, "-i", str(p)], capture_output=True,
                             text=True).stderr
        d = re.search(r"Duration: (\d+):(\d+):([\d.]+)", out)
        sec = int(d.group(1)) * 3600 + int(d.group(2)) * 60 + float(d.group(3))
        r = re.search(r"Video:.*?,\s*(\d{2,5})x(\d{2,5})", out)
        res = f"{r.group(1)}×{r.group(2)}" if r else "?"
        return sec, res
    except Exception:
        return 0.0, "?"


stills, inventory = [], []
# Sample the follow cameras early in the run: mid-mission the rover is inside
# the tunnel, where a still is dark and shows nothing useful.
for label, fname, frac in (("behind the rover", "isaac_behind.mp4", 0.22),
                           ("side angle", "isaac_side.mp4", 0.30),
                           ("top-down", "isaac_top.mp4", 0.22),
                           ("RViz (the robot's belief)", "rviz.mp4", 0.55)):
    p = RUN / fname
    if not p.exists():
        continue
    sec, res = video_meta(p)
    inventory.append([fname, res, f"{sec:.0f} s",
                      f"{p.stat().st_size/1e6:.1f} MB"])
    still = RUN / f"still_{fname.replace('.mp4', '')}.jpg"
    subprocess.run([args.ffmpeg, "-y", "-loglevel", "error",
                    "-ss", str(max(sec * frac, 0.0)), "-i", str(p),
                    "-frames:v", "1", "-q:v", "2", str(still)], check=False)
    if still.exists():
        stills.append((label, still))

reel = RUN / "QSLAM_highlight.mp4"
if reel.exists():
    rsec, rres = video_meta(reel)
    inventory.append([reel.name + "  (highlight reel)", rres, f"{rsec:.0f} s",
                      f"{reel.stat().st_size/1e6:.1f} MB"])

# --------------------------------------------------------------------- pdf --
ss = getSampleStyleSheet()
body = ParagraphStyle("body", parent=ss["Normal"], fontName="Helvetica",
                      fontSize=10.5, leading=15, textColor=INK, spaceAfter=7)
h1 = ParagraphStyle("h1x", parent=ss["Heading1"], fontName="Helvetica-Bold",
                    fontSize=15, leading=19, textColor=ACCENT,
                    spaceBefore=14, spaceAfter=7)
small = ParagraphStyle("small", parent=body, fontSize=9, leading=12.5,
                       textColor=FAINT)

def table(rows_, widths, header=True):
    t_ = Table(rows_, colWidths=widths)
    st = [("FONTNAME", (0, 0), (-1, -1), "Helvetica"),
          ("FONTSIZE", (0, 0), (-1, -1), 9.5),
          ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#C9D4DE")),
          ("VALIGN", (0, 0), (-1, -1), "TOP"),
          ("TOPPADDING", (0, 0), (-1, -1), 4),
          ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
          ("LEFTPADDING", (0, 0), (-1, -1), 6)]
    if header:
        st += [("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#F2F6FA")),
               ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
               ("TEXTCOLOR", (0, 0), (-1, 0), ACCENT)]
    t_.setStyle(TableStyle(st))
    return t_


story = []
story.append(Paragraph("SIH26126 — Mission Report",
                       ParagraphStyle("t", parent=h1, fontSize=20, leading=25)))
story.append(Paragraph(
    f"Recorded run: {RUN.name} · goal UTM 44N E {goal_e:.2f} "
    f"N {goal_n:.2f} (map {goal_x:.2f}, {goal_y:.2f}) · no GNSS in the "
    f"loop", small))
story.append(Spacer(1, 6))
verdict = ("MISSION SUCCESSFUL — GOAL REACHED" if reached
           else f"MISSION ENDED — Nav2 status {status}")
story.append(Table([[verdict]], colWidths=[480], style=TableStyle([
    ("BACKGROUND", (0, 0), (-1, -1),
     colors.HexColor("#E8F1E8" if reached else "#F6E8E8")),
    ("TEXTCOLOR", (0, 0), (-1, -1),
     colors.HexColor("#2C5F2D" if reached else "#8B2020")),
    ("FONTNAME", (0, 0), (-1, -1), "Helvetica-Bold"),
    ("FONTSIZE", (0, 0), (-1, -1), 12),
    ("ALIGN", (0, 0), (-1, -1), "CENTER"),
    ("TOPPADDING", (0, 0), (-1, -1), 8),
    ("BOTTOMPADDING", (0, 0), (-1, -1), 8)])))
story.append(Spacer(1, 8))

story.append(Paragraph("1.  Result summary", h1))
story.append(table(
    [["Quantity", "Value", "Source"],
     ["Initial distance to goal", fmt(dist_goal[0], " m"),
      "ground truth at first log row"],
     ["Final distance to goal", fmt(dist_goal[-1], " m"), "ground truth"],
     ["Localization error at goal", fmt(loc_err[-1], " m"),
      "|ground truth − EKF| — the GPS-free claim"],
     ["Mean / max localization error", f"{loc_err.mean():.2f} / "
      f"{loc_err.max():.2f} m", "whole mission"],
     ["Path length driven", fmt(path_len, " m"), "integrated ground truth"],
     ["Mission duration", f"{duration:.0f} s sim time "
      f"({duration/60:.1f} min)", "telemetry log"],
     ["Time in motion", f"{move_time:.0f} s", "|cmd_v| > 0.05 m/s"],
     ["Average moving speed", fmt(path_len / max(move_time, 1e-6), " m/s"),
      "path / moving time"],
     ["Recovery manoeuvres", str(recoveries), "Nav2 feedback"],
     ["Nav2 final status", f"{status} "
      f"({'SUCCEEDED' if status == 4 else 'not succeeded'})",
      "NavigateToPose result"]],
    [150, 130, 200]))
story.append(Spacer(1, 4))
story.append(Paragraph(
    "Ground truth comes from the simulator state and is used for scoring "
    "only — nothing in the navigation stack subscribes to it. The EKF "
    "position is what the rover actually believed and navigated on, computed "
    "from body-frame velocities and IMU heading alone.", small))

story.append(PageBreak())
story.append(Paragraph("2.  Trajectory", h1))
story.append(Image(str(RUN / "fig_trajectory.png"), width=165 * mm,
                   height=146.6 * mm))
story.append(PageBreak())

story.append(Paragraph("3.  Telemetry", h1))
story.append(Image(str(RUN / "fig_telemetry.png"), width=165 * mm,
                   height=137.5 * mm))
story.append(Paragraph(
    "Flat stretches in distance-to-goal with nonzero speed are the tunnel "
    "and detours around discovered obstacles; drops to zero speed mark "
    "recovery manoeuvres.", small))
story.append(PageBreak())

story.append(Paragraph("4.  Camera perspectives (from the recordings)", h1))
cap_c = ParagraphStyle("capc", parent=small, alignment=1, spaceBefore=3,
                       spaceAfter=10)
for i, (label, still) in enumerate(stills):
    with PILImage.open(still) as im:
        iw, ih = im.size
    story.append(Image(str(still), width=150 * mm, height=150 * mm * ih / iw))
    story.append(Paragraph(f"{label} — {iw}×{ih}", cap_c))
    if i % 2 == 1 and i != len(stills) - 1:
        story.append(PageBreak())
        story.append(Paragraph("4.  Camera perspectives (continued)", h1))
story.append(PageBreak())

story.append(Paragraph("5.  Recording inventory", h1))
if inventory:
    story.append(table([["File", "Resolution", "Duration", "Size"]] +
                       inventory, [190, 100, 90, 90]))
story.append(Paragraph(
    f"All files are in {RUN}. The three isaac_*.mp4 views are rendered "
    "in-simulation at 15 fps of sim time (behind / side follow cameras and a "
    "heading-stabilised top-down view); rviz.mp4 is a screen capture of the "
    "navigation stack's live belief — costmaps, plans, lidar and TF.",
    small))

def footer(canvas, doc):
    canvas.saveState()
    canvas.setFont("Helvetica", 8)
    canvas.setFillColor(FAINT)
    canvas.drawString(50, 28, "SIH26126 — mission report")
    canvas.drawRightString(A4[0] - 50, 28, f"page {doc.page}")
    canvas.setStrokeColor(colors.HexColor("#DDDDDD"))
    canvas.line(50, 38, A4[0] - 50, 38)
    canvas.restoreState()


out = RUN / "SIH26126_Mission_Report.pdf"
doc = SimpleDocTemplate(str(out), pagesize=A4, leftMargin=50, rightMargin=50,
                        topMargin=46, bottomMargin=52,
                        title="SIH26126 Mission Report", author="RIG, KCT")
doc.build(story, onFirstPage=footer, onLaterPages=footer)
shutil.copy(out, WS / "SIH26126_Mission_Report.pdf")
print(f"wrote {out} (+ copy at {WS / 'SIH26126_Mission_Report.pdf'})")
