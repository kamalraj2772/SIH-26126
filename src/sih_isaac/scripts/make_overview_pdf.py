"""Build SIH26126_System_Overview.pdf -- the plain-language system document.

    .demoenv/bin/python src/sih_isaac/scripts/make_overview_pdf.py

Written for a reader who has never seen the project: what problem it solves,
how the pieces fit, what was measured, and how to run it. The numbers come
from the committed run (seed 42); update them if you re-measure.
"""
import pathlib

from reportlab.graphics.shapes import Drawing, Line, Polygon, Rect, String
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (Image, PageBreak, Paragraph, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

WS = pathlib.Path("/home/qbotix-rover/sih_ws")
OUT = WS / "SIH26126_System_Overview.pdf"
PREVIEW = WS / "src/sih_isaac/generated/scene_preview.png"

ACCENT = colors.HexColor("#1F4E79")
ACCENT2 = colors.HexColor("#2E74B5")
INK = colors.HexColor("#1a1a1a")
FAINT = colors.HexColor("#666666")
BOXBG = colors.HexColor("#F2F6FA")
OFFBG = colors.HexColor("#F5F5F0")

ss = getSampleStyleSheet()
body = ParagraphStyle("body", parent=ss["Normal"], fontName="Helvetica",
                      fontSize=10.5, leading=15, textColor=INK,
                      spaceAfter=7)
h1 = ParagraphStyle("h1x", parent=ss["Heading1"], fontName="Helvetica-Bold",
                    fontSize=16, leading=20, textColor=ACCENT,
                    spaceBefore=16, spaceAfter=8)
h2 = ParagraphStyle("h2x", parent=ss["Heading2"], fontName="Helvetica-Bold",
                    fontSize=12.5, leading=16, textColor=ACCENT2,
                    spaceBefore=12, spaceAfter=5)
small = ParagraphStyle("small", parent=body, fontSize=9, leading=12.5,
                       textColor=FAINT)
cap = ParagraphStyle("cap", parent=small, alignment=TA_CENTER, spaceBefore=4)
mono = ParagraphStyle("mono", parent=body, fontName="Courier", fontSize=9,
                      leading=12.5, backColor=colors.HexColor("#F4F4F4"),
                      borderPadding=6, leftIndent=6, spaceBefore=4,
                      spaceAfter=8)
bullet = ParagraphStyle("bul", parent=body, leftIndent=14, bulletIndent=4,
                        spaceAfter=4)


def B(text):
    return Paragraph(text, bullet, bulletText="–")


def table(rows, widths, header=True):
    t = Table(rows, colWidths=widths)
    style = [
        ("FONTNAME", (0, 0), (-1, -1), "Helvetica"),
        ("FONTSIZE", (0, 0), (-1, -1), 9),
        ("TEXTCOLOR", (0, 0), (-1, -1), INK),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#C9D4DE")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
        ("LEFTPADDING", (0, 0), (-1, -1), 6),
    ]
    if header:
        style += [("BACKGROUND", (0, 0), (-1, 0), BOXBG),
                  ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                  ("TEXTCOLOR", (0, 0), (-1, 0), ACCENT)]
    t.setStyle(TableStyle(style))
    return t


# ---------------------------------------------------------------- diagram ---
def dbox(d, x, y, w, h, title, lines, fill=colors.white, stroke=ACCENT2):
    d.add(Rect(x, y, w, h, fillColor=fill, strokeColor=stroke,
               strokeWidth=1.1, rx=3, ry=3))
    cx = x + w / 2.0
    ty = y + h - 13
    d.add(String(cx, ty, title, fontName="Helvetica-Bold", fontSize=8.3,
                 fillColor=ACCENT, textAnchor="middle"))
    for i, ln in enumerate(lines):
        d.add(String(cx, ty - 10.5 - i * 9.5, ln, fontName="Helvetica",
                     fontSize=7.4, fillColor=INK, textAnchor="middle"))


def arrow(d, pts, label=None, lx=0, ly=0, dashed=False, color=FAINT):
    for i in range(len(pts) - 1):
        (x1, y1), (x2, y2) = pts[i], pts[i + 1]
        ln = Line(x1, y1, x2, y2, strokeColor=color, strokeWidth=1.2)
        if dashed:
            ln.strokeDashArray = [3, 2.5]
        d.add(ln)
    (x1, y1), (x2, y2) = pts[-2], pts[-1]
    import math as m
    a = m.atan2(y2 - y1, x2 - x1)
    s = 5.5
    d.add(Polygon([x2, y2,
                   x2 - s * m.cos(a - 0.44), y2 - s * m.sin(a - 0.44),
                   x2 - s * m.cos(a + 0.44), y2 - s * m.sin(a + 0.44)],
                  fillColor=color, strokeColor=color))
    if label:
        d.add(String(lx, ly, label, fontName="Helvetica-Oblique",
                     fontSize=7.2, fillColor=FAINT, textAnchor="middle"))


def architecture_drawing():
    d = Drawing(495, 470)

    # offline strip
    d.add(Rect(10, 330, 475, 132, fillColor=OFFBG,
               strokeColor=colors.HexColor("#BBBBAF"), strokeWidth=0.8,
               rx=4, ry=4))
    d.add(String(20, 448, "BEFORE THE MISSION  (offline, run once)",
                 fontName="Helvetica-Bold", fontSize=7.5, fillColor=FAINT))
    dbox(d, 165, 398, 165, 44, "Terrain survey",
         ["GeoTIFF elevation map of the site", "(UTM zone 44N, 0.59 m/pixel)"])
    dbox(d, 20, 338, 145, 46, "3-D simulation world",
         ["Isaac Sim terrain mesh,", "tunnel, obstacles, humans"])
    dbox(d, 180, 338, 135, 46, "Survey anchor",
         ["one fixed UTM reference", "point (world.json)"])
    dbox(d, 330, 338, 145, 46, "Planner's prior map",
         ["slope steeper than 15°", "marked as blocked"])
    arrow(d, [(210, 398), (105, 384)])
    arrow(d, [(247, 398), (247, 384)])
    arrow(d, [(285, 398), (390, 384)])

    # runtime
    d.add(String(105, 316, "DURING THE MISSION  (live, closed loop)",
                 fontName="Helvetica-Bold", fontSize=7.5, fillColor=FAINT))
    dbox(d, 300, 252, 175, 52, "Operator",
         ["types a UTM goal (utm_goal.py);", "anchor is subtracted → goal",
          "in local map metres"], fill=BOXBG)
    dbox(d, 40, 190, 230, 82, "Nav2 — plans the route, drives the rover",
         ["Smac A*: full route on the prior map",
          "MPPI: steers around what the lidar",
          "sees right now, replans 20×/second"], fill=BOXBG)
    dbox(d, 40, 60, 230, 62, "Isaac Sim — the physical world",
         ["rover physics (4-wheel skid steer),",
          "RTX lidar, camera, IMU,", "walking humans, terrain"], fill=BOXBG)
    dbox(d, 320, 60, 155, 62, "EKF pose tracker",
         ["fuses body velocities +", "IMU heading into one",
          "position estimate"], fill=BOXBG)

    arrow(d, [(300, 272), (270, 258)], label="goal", lx=290, ly=276)
    arrow(d, [(390, 338), (390, 310), (155, 310), (155, 272)],
          label="prior map", lx=347, ly=314, dashed=True)
    arrow(d, [(92, 338), (92, 322), (25, 322), (25, 91), (40, 91)],
          dashed=True)
    arrow(d, [(120, 190), (120, 122)],
          label="/cmd_vel  (steer + speed)", lx=185, ly=152)
    arrow(d, [(210, 122), (210, 190)],
          label="/scan  (live lidar)", lx=218, ly=170)
    arrow(d, [(270, 91), (320, 91)], label="velocities + IMU", lx=295, ly=97)
    arrow(d, [(397, 122), (397, 230), (270, 230)],
          label="pose estimate (TF)", lx=345, ly=234)
    return d


# ---------------------------------------------------------------- content ---
story = []

story.append(Spacer(1, 30))
story.append(Paragraph(
    "SIH26126 — GPS-Denied Autonomous Navigation",
    ParagraphStyle("t", parent=h1, fontSize=22, leading=27,
                   alignment=TA_CENTER)))
story.append(Paragraph(
    "System overview and architecture — Isaac Sim 6.0 + ROS 2 stack",
    ParagraphStyle("st", parent=body, fontSize=12, alignment=TA_CENTER,
                   textColor=FAINT)))
story.append(Spacer(1, 14))
if PREVIEW.exists():
    story.append(Image(str(PREVIEW), width=165 * mm, height=92.8 * mm))
    story.append(Paragraph(
        "The simulated proving ground: 150 m × 150 m of real-shaped "
        "terrain with a berm, a lit tunnel bore, slopes, scattered obstacles "
        "and four humans — two standing, two walking patrol routes.", cap))
story.append(Spacer(1, 12))
story.append(Paragraph(
    "An operator types a map coordinate. A four-wheeled rover drives there "
    "on its own — across slopes, around obstacles and people, through a "
    "tunnel — and no satellite positioning of any kind is used or even "
    "present in the software. This document explains why that matters, how "
    "the system works, what was measured, and how to run it yourself. It is "
    "written to be readable without a robotics background; technical terms "
    "are collected in the glossary on the last page.", body))
story.append(PageBreak())

story.append(Paragraph("1.  The need: navigation where GPS cannot follow", h1))
story.append(Paragraph(
    "Almost every outdoor robot answers the question “where am I?” "
    "with a satellite fix. That answer disappears exactly where unmanned "
    "ground vehicles are most useful: inside tunnels and mines, under dense "
    "canopy, between tall structures, and anywhere a signal is jammed or "
    "spoofed — which in a defence context must be assumed, not feared. "
    "SIH problem statement 26126 asks for a UGV that still accepts a "
    "georeferenced task (“go to this UTM coordinate”) and completes "
    "it with onboard sensing alone.", body))
story.append(Paragraph(
    "The design rule we set ourselves is stricter than “works when GPS "
    "drops out”: there is no receiver in the loop at all. No NavSatFix "
    "message, no GPS topic, no hidden fallback — grep the source tree "
    "for “gps” and you find only documentation sentences like this "
    "one. If the system cheats, it fails visibly, because there is nothing "
    "to cheat with.", body))
story.append(Paragraph(
    "How can it accept a UTM coordinate without a receiver, then? The same "
    "way a surveyor does. Before deployment, the site is surveyed once: a "
    "georeferenced elevation map (GeoTIFF) and one fixed anchor point whose "
    "UTM coordinates are known. From then on, converting the operator's UTM "
    "goal to local map metres is a subtraction — pure arithmetic done "
    "offline-calibrated, nothing measured live. The rover's own position "
    "comes entirely from its wheels, inertial unit and lidar.", body))

story.append(Paragraph("2.  What the system does, start to finish", h1))
story.append(B("The site's GeoTIFF elevation map is turned into two things "
               "at once: the 3-D simulation world the rover drives in, and "
               "the route planner's prior map (any slope steeper than "
               "15° is marked impassable). Both come from the same "
               "file, so the world and the map agree by construction."))
story.append(B("The operator types a goal in UTM easting/northing. The "
               "survey anchor is subtracted and the goal lands in local map "
               "coordinates."))
story.append(B("A global planner (Smac A*) draws a full route on the prior "
               "map — through the tunnel if that is the short way, "
               "around the 36° mound it must not climb."))
story.append(B("A local controller (MPPI) follows that route while the "
               "lidar discovers what the prior map never knew: drums, logs, "
               "boulders — and people, who move. It re-evaluates "
               "hundreds of candidate trajectories twenty times a second."))
story.append(B("An EKF fuses body-frame velocities with the IMU's heading "
               "to keep track of where the rover is — dead reckoning, "
               "seeded at the surveyed start pose. Nothing corrects it from "
               "a satellite, so honesty about drift is part of the design."))
story.append(B("Progress is reported back in both frames — map metres "
               "and UTM — until the rover stands on the goal."))
story.append(Spacer(1, 4))
story.append(Paragraph(
    "On the committed configuration (world seed 42, goal E 403562.52 / "
    "N 1450024.12, 123.5 m away through the tunnel) the rover reached the "
    "goal with a final ground-truth error of 0.23 m and an accumulated "
    "localization error of 0.98 m, with one recovery manoeuvre, in about "
    "460 simulated seconds.", body))
story.append(PageBreak())

story.append(Paragraph("3.  Architecture", h1))
story.append(Paragraph(
    "One picture first, then each block in words. Everything above the "
    "line happens once, before the mission; everything below it is the "
    "live control loop.", body))
story.append(architecture_drawing())
story.append(PageBreak())

story.append(Paragraph("3.1  The GeoTIFF is the single source of truth", h2))
story.append(Paragraph(
    "world.py writes a float32 elevation raster (EPSG:32644, 0.59 m/pixel "
    "— it opens in QGIS like any survey product) and everything else "
    "derives from it: the Isaac Sim terrain mesh, the planner's occupancy "
    "map, and the anchor for UTM conversion. Because both the simulated "
    "ground and the planner's belief come from the same file, a whole class "
    "of “the map doesn't match the world” bugs cannot exist. "
    "Corridor obstacles and humans are deliberately left out of the prior "
    "map — the rover must find those with its lidar, as it would in "
    "the field.", body))

story.append(Paragraph("3.2  Simulation (Isaac Sim 6.0.1)", h2))
story.append(Paragraph(
    "The rover is a 4-wheel skid-steer articulation with a mast-top RTX "
    "lidar, an RGB-D camera and an IMU. PhysX reproduces the platform's "
    "real handling problem: a long-wheelbase skid-steer scrubs its wheels "
    "sideways in every turn, so commanded rotation is not achieved rotation "
    "(we measured only ~6% achieved with stock friction). Wheel friction, "
    "drive damping and a yaw gain of 6.5 were calibrated with a dedicated "
    "script until commands tracked. Two of the four humans walk fixed "
    "patrol routes that cross the rover's corridor. A chase camera follows "
    "the rover in the viewport so you can watch the mission in third "
    "person. The simulator publishes sensors and accepts wheel commands "
    "— and deliberately publishes no coordinate transforms and no "
    "positioning: it is the world, not part of the robot's brain.", body))

story.append(Paragraph("3.3  Localization — the hard part, done honestly", h2))
story.append(Paragraph(
    "With no satellites, position comes from an Extended Kalman Filter "
    "(robot_localization) fusing two streams: body-frame forward/lateral "
    "velocity, and the IMU's absolute heading plus turn rate. The filter "
    "is seeded with the surveyed deployment pose, so its map starts "
    "aligned with the world and only drift accumulates afterwards.", body))
story.append(Paragraph(
    "We did not arrive at that sensor set by convenience — wheel "
    "encoders were tried first and failed three separate, measured ways on "
    "this platform: heading computed from left/right wheel-speed difference "
    "is meaningless when wheels scrub; distance over-counts about 30% on "
    "slopes because wheels slip; and sideways crabbing on side-slopes is "
    "completely invisible to encoders. Missions ended 10–70 m from "
    "where the robot believed it was. The velocities now fused are exactly "
    "what a visual-inertial odometry system (cuVSLAM, already installed) "
    "and a magnetometer-grade AHRS measure on real hardware; in simulation "
    "they are synthesized from rigid-body state, and swapping in cuVSLAM "
    "is the designated next step. The failed wheel odometry is still "
    "published, so the comparison stays visible.", body))

story.append(Paragraph("3.4  Planning — a route from the prior, reflexes from the lidar", h2))
story.append(Paragraph(
    "Navigation is ROS 2's Nav2 stack. The global planner (Smac A*) plans "
    "once over the GeoTIFF-derived prior map; its cost settings matter "
    "more than they look — with travel cost multiplied above 1.0 the "
    "planner priced the tunnel's inflated-but-free cells as expensive and "
    "detoured around the mountain, so it stays at exactly 1.0. The local "
    "controller (MPPI) samples hundreds of short candidate trajectories "
    "every 50 ms, scores them against a rolling costmap built live from "
    "the lidar, and picks the best — that is what dodges drums, logs "
    "and walking humans the prior never contained. If the rover gets "
    "boxed in, Nav2's recovery behaviours (back up, spin, replan) take "
    "over; the committed run needed exactly one.", body))

story.append(Paragraph("3.5  Who is allowed to say where things are (TF ownership)", h2))
story.append(Paragraph(
    "In ROS, coordinate frames form a tree and multiple writers on one "
    "edge corrupt it silently — a prior attempt at this project lost "
    "days to exactly that. Here every edge has exactly one owner:", body))
story.append(table(
    [["Frame edge", "Sole owner"],
     ["map → odom", "static identity (EKF is seeded at the surveyed "
      "pose); slam_toolbox when slam:=true (experimental)"],
     ["odom → base_link", "the EKF, and nothing else"],
     ["base_link → lidar_link", "one static transform publisher"],
     ["base_link → wheels, camera, IMU", "robot_state_publisher, "
      "from the URDF model"]],
    [150, 330]))
story.append(Paragraph(
    "Isaac Sim publishes no transforms at all. Ground truth is published "
    "on its own topic for scoring only — nothing in the navigation "
    "stack subscribes to it.", small))
story.append(PageBreak())

story.append(Paragraph("4.  Measured results", h1))
story.append(table(
    [["Quantity", "Value", "How it was measured"],
     ["Mission", "goal reached, through the tunnel",
      "committed run, world seed 42"],
     ["Initial distance to goal", "123.5 m", "straight-line, map frame"],
     ["Final distance to goal", "0.23 m", "ground truth (sim state)"],
     ["Localization error at goal", "0.98 m",
      "EKF estimate vs ground truth after 123.5 m without GPS"],
     ["Recoveries", "1", "Nav2 behaviour log"],
     ["Mission time", "≈ 460 s simulated", "sim clock"],
     ["Skid-steer yaw calibration", "gain 6.5",
      "spin ratio 0.398 measured at gain 2.6, calibrate_yaw.py"]],
    [140, 130, 210]))
story.append(Spacer(1, 6))
story.append(Paragraph(
    "The 0.98 m localization error is the number that carries the claim: "
    "under one metre of drift over a 123.5 m GPS-free mission, achieved "
    "with dead reckoning of VIO-grade velocity and an absolute heading "
    "reference. Gyro-only heading integration was also tried and drifted "
    "~13° over 45 m — which is why an absolute heading source "
    "(magnetometer/AHRS class) is a hard requirement of the design.", body))

story.append(Paragraph("5.  Running it", h1))
story.append(Paragraph(
    "Three terminals, in this order. Wait for “[sim] running” "
    "before starting the second.", body))
story.append(Paragraph(
    "# terminal 1 — the world (add --headless for no window)<br/>"
    "cd ~/sih_ws &amp;&amp; ./run_isaac.sh<br/><br/>"
    "# terminal 2 — the robot's brain: EKF + Nav2 + RViz<br/>"
    "source /opt/ros/jazzy/setup.bash &amp;&amp; source "
    "~/sih_ws/install/setup.bash<br/>"
    "ros2 launch sih_isaac nav.launch.py<br/><br/>"
    "# terminal 3 — the mission<br/>"
    "ros2 run sih_isaac utm_goal.py -e 403562.52 -n 1450024.12", mono))
story.append(Paragraph("If something looks wrong", h2))
story.append(B("Goal command fails or the map frame is missing — "
               "almost always a stale earlier run still alive. Check with "
               "<font face='Courier' size='9'>ps -e | grep -E "
               "\"run_sim|nav.launch\"</font>; there must be exactly one "
               "of each. Kill leftovers, then restart terminal 2."))
story.append(B("EKF publishes nothing on /odometry/filtered for more than "
               "~15 s after launch — it saw a clock hiccup at startup "
               "and will not self-heal. Ctrl+C terminal 2 and relaunch; "
               "it costs ten seconds."))
story.append(B("Nav2 diagnostics stuck on “Managed nodes are "
               "unconfigured” — same cure: relaunch terminal 2."))
story.append(B("Full GUI + RViz on one machine can drop the simulator "
               "below real time and starve Nav2's transform timing. For "
               "demos, run the sim headless and keep RViz, or accept a "
               "slower mission."))

story.append(Paragraph("6.  Limits we know about, and what comes next", h1))
story.append(B("Localization velocities are synthesized at VIO grade from "
               "simulator state — idealized sensors, stated openly. "
               "The interfaces are already shaped for isaac_ros_visual_slam "
               "(cuVSLAM); wiring it in place of the synthesized topic is "
               "the next step."))
story.append(B("slam_toolbox (lidar SLAM for map→odom correction) is "
               "wired but experimental here: its corrections degraded the "
               "pose on this terrain after ~50 m. cuVSLAM is the intended "
               "replacement rather than further karto tuning."))
story.append(B("Dead reckoning drift grows with distance; 0.98 m over "
               "123.5 m is measured, kilometre-scale missions will need "
               "the visual loop closure above."))
story.append(B("Everything here is simulation — by design, per the "
               "problem statement's phase gates. The stack is standard "
               "ROS 2 Jazzy / Nav2, chosen so it transfers to hardware "
               "without rearchitecting."))
story.append(PageBreak())

story.append(Paragraph("7.  Glossary", h1))
story.append(table(
    [["Term", "Meaning here"],
     ["UTM", "a map grid that names any point on Earth in metres (easting/"
      "northing) instead of degrees; zone 44N covers the site"],
     ["GeoTIFF / DEM", "an image file whose pixels are ground elevations, "
      "tagged with real-world coordinates — the survey product "
      "everything derives from"],
     ["ROS 2", "the message-passing framework robot programs use to talk "
      "to each other; programs are “nodes”, named streams are "
      "“topics”"],
     ["Nav2", "ROS 2's standard navigation suite: planners, controller, "
      "recovery behaviours, lifecycle management"],
     ["Smac A*", "the global planner — searches the prior map for the "
      "cheapest drivable route to the goal"],
     ["MPPI", "the local controller — samples many candidate short "
      "trajectories per cycle and picks the best against live obstacles"],
     ["Costmap", "a grid scoring how dangerous each cell is; the global "
      "one comes from the prior, the local one from live lidar"],
     ["EKF", "Extended Kalman Filter — merges imperfect sensor "
      "streams into one best position estimate"],
     ["Odometry", "position tracked by integrating measured motion; "
      "drifts with distance, works with zero infrastructure"],
     ["IMU / AHRS", "inertial unit measuring turn rate and acceleration; "
      "an AHRS adds an absolute heading reference"],
     ["VIO", "visual-inertial odometry — camera + IMU motion "
      "tracking (cuVSLAM is the planned provider)"],
     ["TF", "ROS's live tree of coordinate-frame relationships — "
      "“where is the lidar relative to the map right now”"],
     ["Skid-steer", "steering by driving left and right wheels at "
      "different speeds, like a tank; wheels scrub sideways in turns"],
     ["Isaac Sim / USD", "NVIDIA's physics simulator and its scene file "
      "format; RTX lidar means raytraced, physically-modelled beams"],
     ["RViz", "ROS's visualization tool — the window showing what "
      "the robot believes, next to what is actually there"]],
    [95, 385]))
story.append(Spacer(1, 10))
story.append(Paragraph(
    "Repository: src/sih_isaac (Isaac Sim stack — this document), "
    "src/sih_demo (earlier MuJoCo demonstration of the same mission). A "
    "deeper engineering report with per-file detail is in "
    "SIH26126_Isaac_System_Report.docx.", small))


def footer(canvas, doc):
    canvas.saveState()
    canvas.setFont("Helvetica", 8)
    canvas.setFillColor(FAINT)
    canvas.drawString(50, 28, "SIH26126 — GPS-denied UGV navigation")
    canvas.drawRightString(A4[0] - 50, 28, f"page {doc.page}")
    canvas.setStrokeColor(colors.HexColor("#DDDDDD"))
    canvas.line(50, 38, A4[0] - 50, 38)
    canvas.restoreState()


doc = SimpleDocTemplate(str(OUT), pagesize=A4, leftMargin=50, rightMargin=50,
                        topMargin=48, bottomMargin=52,
                        title="SIH26126 System Overview",
                        author="RIG, KCT")
doc.build(story, onFirstPage=footer, onLaterPages=footer)
print(f"wrote {OUT}")
