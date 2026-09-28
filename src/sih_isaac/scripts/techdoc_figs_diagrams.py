"""Architecture diagrams and flowcharts for the SIH26126 technical PDF.

    cd src/sih_isaac/scripts && ../../../.demoenv/bin/python techdoc_figs_diagrams.py

Every node, topic, frame, rate and parameter drawn here is taken from the
repository (launch/nav.launch.py, config/*.yaml, scripts/run_sim.py,
sih_rover.urdf, utm_goal.py, record_mission.sh) and from the Nav2 Jazzy
default behaviour tree. QSLAM modules that are designed but not yet in the
repository are drawn with kind="design".
"""
import math

from matplotlib.patches import (Circle, Ellipse, FancyBboxPatch, Polygon,
                                Rectangle)

import techdoc_style as S

W = 6.8
MONO = "DejaVu Sans Mono"


# ------------------------------------------------------------------ helpers --
def diamond(ax, cx, cy, w, h, text, kind="white", fs=7.0):
    fc, ec, tc, _ = S.KINDS[kind]
    ax.add_patch(Polygon([(cx - w / 2, cy), (cx, cy + h / 2),
                          (cx + w / 2, cy), (cx, cy - h / 2)], closed=True,
                         fc=fc, ec=ec, lw=1.1, zorder=2))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fs, color=tc,
            fontweight="bold", zorder=3, linespacing=1.15)


def note(ax, x, y, text, fs=6.6, color=S.GREY, ha="left", style="italic",
         weight="normal", va="center", rotation=0):
    ax.text(x, y, text, fontsize=fs, color=color, ha=ha, va=va, style=style,
            fontweight=weight, zorder=6, linespacing=1.25, rotation=rotation)


def pill(ax, cx, cy, w, h, title, sub=None, kind="impl", ts=7.2, ss=6.5):
    """ROS node: rounded 'pill' like rqt_graph's node ellipses."""
    fc, ec, tc, dashed = S.KINDS[kind]
    ax.add_patch(FancyBboxPatch(
        (cx - w / 2, cy - h / 2), w, h,
        boxstyle=f"round,pad=0,rounding_size={h / 2 - 0.5}", fc=fc, ec=ec,
        lw=1.1, zorder=2, linestyle=(0, (4, 2.5)) if dashed else "-"))
    if sub:
        ax.text(cx, cy + h * 0.17, title, ha="center", va="center",
                fontsize=ts, fontweight="bold", color=tc, zorder=3)
        ax.text(cx, cy - h * 0.22, sub, ha="center", va="center",
                fontsize=ss, color="#333333", zorder=3, linespacing=1.15)
    else:
        ax.text(cx, cy, title, ha="center", va="center", fontsize=ts,
                fontweight="bold", color=tc, zorder=3, linespacing=1.15)


def topic(ax, cx, cy, w, h, text, fs=6.6, kind=None):
    ec = S.KINDS[kind][1] if kind else "#8C9BAB"
    fc = S.KINDS[kind][0] if kind else "#FFFFFF"
    ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h, fc=fc, ec=ec,
                           lw=0.9, zorder=2))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fs,
            family=MONO, color=S.INK, zorder=3, linespacing=1.2)


def line(ax, pts, color="#7F7F7F", lw=1.1, dashed=False, z=3):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=lw, zorder=z, solid_capstyle="butt",
            linestyle=(0, (4, 2.5)) if dashed else "-")


def forbid(ax, cx, cy, r, lw=2.0):
    ax.add_patch(Circle((cx, cy), r, fc="none", ec=S.RED, lw=lw, zorder=7))
    d = r * math.cos(math.pi / 4)
    ax.plot([cx - d, cx + d], [cy + d, cy - d], color=S.RED, lw=lw, zorder=7)


def check_mark(ax, cx, cy, s, color=S.GREEN, lw=2.4):
    ax.plot([cx - s, cx - s * 0.25, cx + s], [cy, cy - s * 0.75, cy + s * 0.8],
            color=color, lw=lw, solid_capstyle="round", zorder=6)


def cross_mark(ax, cx, cy, s, color=S.RED, lw=2.4):
    ax.plot([cx - s, cx + s], [cy - s, cy + s], color=color, lw=lw,
            solid_capstyle="round", zorder=6)
    ax.plot([cx - s, cx + s], [cy + s, cy - s], color=color, lw=lw,
            solid_capstyle="round", zorder=6)


def star_badge(ax, x, y, color=S.ORANGE, fs=9):
    ax.text(x, y, "★", fontsize=fs, color=color, ha="center",
            va="center", zorder=6)


# ------------------------------------------------------------------- icons ---
def ic_satellite(ax, cx, cy, s):
    ax.add_patch(Rectangle((cx - 0.28 * s, cy - 0.28 * s), 0.56 * s, 0.56 * s,
                           fc="#8A8F98", ec="#4A4F58", lw=0.8, zorder=5))
    for sgn in (-1, 1):
        x0 = cx + sgn * 0.33 * s - (0.72 * s if sgn < 0 else 0)
        ax.add_patch(Rectangle((x0, cy - 0.2 * s), 0.72 * s, 0.4 * s,
                               fc="#5B8BD0", ec="#2F5597", lw=0.8, zorder=5))
        for k in (1, 2):
            xx = x0 + k * 0.24 * s
            ax.plot([xx, xx], [cy - 0.2 * s, cy + 0.2 * s], color="#2F5597",
                    lw=0.5, zorder=6)
    ax.plot([cx, cx], [cy - 0.28 * s, cy - 0.55 * s], color="#4A4F58", lw=1,
            zorder=5)
    ax.add_patch(Ellipse((cx, cy - 0.62 * s), 0.45 * s, 0.16 * s,
                         fc="#C9CDD3", ec="#4A4F58", lw=0.7, zorder=5))


def ic_cloud(ax, cx, cy, s):
    for dx, dy, r in ((-0.45, -0.12, 0.34), (0.0, 0.12, 0.46),
                      (0.45, -0.1, 0.36)):
        ax.add_patch(Circle((cx + dx * s, cy + dy * s), r * s, fc="#DCE6F2",
                            ec="#7A93C2", lw=0.9, zorder=5))
    ax.add_patch(Rectangle((cx - 0.45 * s, cy - 0.46 * s), 0.9 * s, 0.36 * s,
                           fc="#DCE6F2", ec="none", zorder=5))
    ax.plot([cx - 0.62 * s, cx + 0.64 * s], [cy - 0.46 * s, cy - 0.46 * s],
            color="#7A93C2", lw=0.9, zorder=5)


def ic_tower(ax, cx, cy, s):
    c = "#555B66"
    ax.plot([cx - 0.35 * s, cx, cx + 0.35 * s], [cy - 0.7 * s, cy + 0.45 * s,
                                                 cy - 0.7 * s], color=c, lw=1.2,
            zorder=5)
    ax.plot([cx - 0.22 * s, cx + 0.22 * s], [cy - 0.25 * s, cy - 0.25 * s],
            color=c, lw=0.8, zorder=5)
    ax.plot([cx - 0.28 * s, cx + 0.12 * s], [cy - 0.48 * s, cy - 0.08 * s],
            color=c, lw=0.6, zorder=5)
    ax.add_patch(Circle((cx, cy + 0.45 * s), 0.08 * s, fc=c, ec=c, zorder=5))
    for r in (0.3, 0.5):
        for sgn in (-1, 1):
            a0 = 90 - 55 if sgn > 0 else 90 + 55
            th = [math.radians(a0 + sgn * 0 + k * (-sgn) * 0) for k in range(1)]
            del th
            ts = [math.radians(-40 + i * 8) for i in range(11)]
            if sgn < 0:
                ts = [math.pi - t for t in ts]
            ax.plot([cx + r * s * math.cos(t) for t in ts],
                    [cy + 0.45 * s + r * s * math.sin(t) for t in ts],
                    color=c, lw=0.9, zorder=5)


def ic_flag(ax, cx, cy, s, color=S.RED):
    ax.plot([cx - 0.35 * s, cx - 0.35 * s], [cy - 0.9 * s, cy + 0.9 * s],
            color="#444444", lw=1.8, zorder=5, solid_capstyle="round")
    ax.add_patch(Polygon([(cx - 0.35 * s, cy + 0.9 * s),
                          (cx + 0.75 * s, cy + 0.55 * s),
                          (cx - 0.35 * s, cy + 0.2 * s)], closed=True,
                         fc=color, ec=color, zorder=5))
    ax.add_patch(Ellipse((cx - 0.35 * s, cy - 0.9 * s), 0.9 * s, 0.22 * s,
                         fc="#BBBBBB", ec="none", zorder=4))


def ic_camera(ax, cx, cy, s):
    ax.add_patch(FancyBboxPatch((cx - s, cy - 0.55 * s), 2 * s, 1.1 * s,
                                boxstyle=f"round,pad=0,rounding_size={0.2 * s}",
                                fc="#3F4652", ec="#222831", lw=0.8, zorder=5))
    for dx in (-0.45, 0.45):
        ax.add_patch(Circle((cx + dx * s, cy), 0.32 * s, fc="#9FC3F0",
                            ec="#1F2A36", lw=1.0, zorder=6))
        ax.add_patch(Circle((cx + dx * s, cy), 0.13 * s, fc="#1F2A36",
                            ec="none", zorder=7))


def ic_gyro(ax, cx, cy, s):
    ax.add_patch(Circle((cx, cy), 0.8 * s, fc="none", ec="#6A1B9A", lw=1.3,
                        zorder=5))
    ax.add_patch(Ellipse((cx, cy), 1.6 * s, 0.55 * s, fc="none",
                         ec="#6A1B9A", lw=1.0, zorder=5))
    ax.add_patch(Ellipse((cx, cy), 0.55 * s, 1.6 * s, fc="none",
                         ec="#6A1B9A", lw=1.0, zorder=5))
    ax.add_patch(Circle((cx, cy), 0.16 * s, fc="#6A1B9A", ec="none",
                        zorder=6))


def ic_map(ax, cx, cy, s, route=True):
    xs = [cx - s, cx - s / 3, cx + s / 3, cx + s]
    tops = [cy + 0.72 * s, cy + 0.9 * s, cy + 0.72 * s, cy + 0.9 * s]
    bots = [cy - 0.9 * s, cy - 0.72 * s, cy - 0.9 * s, cy - 0.72 * s]
    shades = ["#DDEBCF", "#C9DDB5", "#DDEBCF"]
    for i in range(3):
        ax.add_patch(Polygon([(xs[i], bots[i]), (xs[i + 1], bots[i + 1]),
                              (xs[i + 1], tops[i + 1]), (xs[i], tops[i])],
                             closed=True, fc=shades[i], ec="#6E8B4E", lw=0.9,
                             zorder=5))
    if route:
        ax.plot([cx - 0.75 * s, cx - 0.3 * s, cx + 0.15 * s, cx + 0.6 * s],
                [cy - 0.5 * s, cy + 0.1 * s, cy - 0.25 * s, cy + 0.4 * s],
                color=S.RED, lw=1.3, linestyle=(0, (2.5, 1.8)), zorder=6)
        ax.add_patch(Circle((cx + 0.6 * s, cy + 0.4 * s), 0.12 * s,
                            fc=S.RED, ec="white", lw=0.6, zorder=7))


def ic_computer(ax, cx, cy, s):
    ax.add_patch(FancyBboxPatch((cx - 0.8 * s, cy - 0.6 * s), 1.6 * s, 1.2 * s,
                                boxstyle=f"round,pad=0,rounding_size={0.12 * s}",
                                fc="#2F3A48", ec="#1B2330", lw=0.8, zorder=5))
    for i in range(4):
        y = cy - 0.45 * s + i * 0.3 * s
        ax.plot([cx - 0.95 * s, cx - 0.8 * s], [y, y], color="#1B2330",
                lw=1, zorder=5)
        ax.plot([cx + 0.8 * s, cx + 0.95 * s], [y, y], color="#1B2330",
                lw=1, zorder=5)
    ax.add_patch(Rectangle((cx - 0.4 * s, cy - 0.3 * s), 0.8 * s, 0.6 * s,
                           fc="#76B900", ec="none", zorder=6))


def ic_lidar(ax, cx, cy, s):
    ax.add_patch(Rectangle((cx - 0.4 * s, cy - 0.7 * s), 0.8 * s, 0.9 * s,
                           fc="#555B66", ec="#333333", lw=0.8, zorder=5))
    ax.add_patch(Ellipse((cx, cy + 0.2 * s), 0.8 * s, 0.25 * s, fc="#888E99",
                         ec="#333333", lw=0.8, zorder=6))
    for ang in (-25, 0, 25):
        a = math.radians(ang)
        ax.plot([cx + 0.4 * s, cx + 0.4 * s + 0.8 * s * math.cos(a)],
                [cy - 0.1 * s, cy - 0.1 * s + 0.8 * s * math.sin(a)],
                color=S.RED, lw=0.9, zorder=5)


def ic_rover(ax, cx, cy, s):
    ax.add_patch(FancyBboxPatch((cx - 0.9 * s, cy - 0.25 * s), 1.8 * s,
                                0.65 * s, boxstyle="round,pad=0,rounding_size=3",
                                fc="#4A5160", ec="#2A2F38", lw=0.8, zorder=5))
    for dx in (-0.6, 0.6):
        ax.add_patch(Circle((cx + dx * s, cy - 0.35 * s), 0.3 * s,
                            fc="#1E1E1E", ec="#000000", zorder=6))
    ax.plot([cx + 0.2 * s, cx + 0.2 * s], [cy + 0.4 * s, cy + 0.85 * s],
            color="#4A5160", lw=1.6, zorder=5)
    ax.add_patch(Circle((cx + 0.2 * s, cy + 0.9 * s), 0.1 * s, fc=S.ORANGE,
                        ec="none", zorder=6))


def ic_person(ax, cx, cy, s, color="#6D4C41"):
    ax.add_patch(Circle((cx, cy + 0.6 * s), 0.22 * s, fc=color, ec="none",
                        zorder=5))
    ax.add_patch(FancyBboxPatch((cx - 0.22 * s, cy - 0.35 * s), 0.44 * s,
                                0.75 * s, boxstyle="round,pad=0,rounding_size=2",
                                fc=color, ec="none", zorder=5))
    ax.plot([cx - 0.1 * s, cx - 0.15 * s], [cy - 0.35 * s, cy - 0.9 * s],
            color=color, lw=2.0, zorder=5)
    ax.plot([cx + 0.1 * s, cx + 0.18 * s], [cy - 0.35 * s, cy - 0.9 * s],
            color=color, lw=2.0, zorder=5)


def ic_keyboard(ax, cx, cy, s):
    ax.add_patch(FancyBboxPatch((cx - s, cy - 0.5 * s), 2 * s, 1.0 * s,
                                boxstyle=f"round,pad=0,rounding_size={0.12 * s}",
                                fc="#FFFFFF", ec="#555B66", lw=1.0, zorder=5))
    ax.text(cx, cy + 0.12 * s, "E 403562", fontsize=6.5, family=MONO,
            ha="center", va="center", color=S.NAVY, zorder=6)
    ax.text(cx, cy - 0.25 * s, "N 1450024", fontsize=6.5, family=MONO,
            ha="center", va="center", color=S.NAVY, zorder=6)


def ic_drum(ax, cx, cy, s):
    ax.add_patch(Rectangle((cx - 0.3 * s, cy - 0.45 * s), 0.6 * s, 0.9 * s,
                           fc="#F2B233", ec="#9C6E10", lw=0.8, zorder=5))
    ax.add_patch(Ellipse((cx, cy + 0.45 * s), 0.6 * s, 0.18 * s, fc="#F7CD6B",
                         ec="#9C6E10", lw=0.8, zorder=6))


# ================================================================ figures ===
def fig_context():
    fig, ax = S.diagram(W, 3.6)
    S.band(ax, 6, 66, 268, 288, "BEFORE THE MISSION — offline, once",
           "offline")
    S.band(ax, 282, 66, 392, 288,
           "DURING THE MISSION — onboard, real time, no network",
           "neutral")

    # offline column
    S.box(ax, 16, 262, 114, 62, "Surveyed datum",
          "one lat/lon fixed at\nsurvey time:\n13.1147° N, 80.1098° E",
          kind="offline", ts=7.8, ss=6.6)
    S.box(ax, 150, 262, 116, 62, "world.json",
          "UTM anchor E₀, N₀\nstart pose, obstacle\n& human layout",
          kind="offline", ts=7.8, ss=6.6)
    S.arrow(ax, (130, 293), (150, 293))
    S.box(ax, 16, 176, 114, 62, "Site survey DEM",
          "float32 elevation\n257 × 257 px\n0.586 m / px",
          kind="offline", ts=7.8, ss=6.6)
    S.box(ax, 150, 176, 116, 62, "Prior GeoTIFF",
          "prior_dem_utm44n.tif\nEPSG:32644 (UTM 44N)\nopens in QGIS",
          kind="offline", ts=7.8, ss=6.6)
    S.arrow(ax, (130, 207), (150, 207))
    S.box(ax, 16, 82, 114, 62, "Isaac terrain",
          "same DEM → mesh\n(build_scene.py):\nworld = prior map",
          kind="offline", ts=7.8, ss=6.6)
    S.box(ax, 150, 82, 116, 62, "map.pgm / map.yaml",
          "slope > 15° → occupied\nNav2 static prior\n0.586 m / cell",
          kind="offline", ts=7.8, ss=6.6)
    S.arrow(ax, (208, 176), (208, 144))
    S.arrow(ax, (170, 176), (100, 144))

    # online column
    S.box(ax, 300, 268, 176, 52, "utm_goal.py",
          "x = E − E₀ ,  y = N − N₀\n(pure subtraction)",
          kind="impl", ts=8.0, ss=6.8)
    S.box(ax, 510, 268, 150, 52, "Operator",
          "types the goal as UTM\neasting / northing", kind="white",
          ts=8.0, ss=6.8)
    S.arrow(ax, (510, 294), (476, 294), label="E, N", lpos=(493, 305))
    S.box(ax, 300, 168, 118, 66, "Nav2",
          "Smac 2D A* route\nMPPI control 15 Hz\nrecovery behaviours",
          kind="impl", ts=8.0, ss=6.6)
    S.box(ax, 440, 168, 102, 66, "EKF",
          "vx, vy + IMU yaw\n→ odom→base_link\n30 Hz",
          kind="impl", ts=8.0, ss=6.6)
    S.box(ax, 564, 168, 96, 66, "Sensors",
          "RTX 2-D lidar\nRGB-D camera\nIMU", kind="impl", ts=8.0, ss=6.6)
    S.box(ax, 300, 84, 360, 48, "Skid-steer UGV  (Isaac Sim PhysX)",
          "/cmd_vel → four wheel drives; the motion is sensed again",
          kind="impl", ts=8.0, ss=6.8)
    S.arrow(ax, (330, 268), (330, 234))
    note(ax, 336, 251, "map-frame goal", fs=6.5)
    S.arrow(ax, (564, 201), (542, 201))
    S.arrow(ax, (440, 201), (418, 201))
    note(ax, 429, 214, "pose", fs=6.5, ha="center")
    note(ax, 553, 214, "data", fs=6.5, ha="center")
    S.arrow(ax, (359, 168), (359, 132), label="/cmd_vel", lpos=(378, 150),
            ha="left")
    S.arrow(ax, (612, 132), (612, 168), label="motion", lpos=(630, 150),
            ha="left")
    # scan straight to Nav2 costmaps
    line(ax, [(600, 234), (600, 246), (406, 246)], color="#9A9A9A", lw=0.9)
    S.arrow(ax, (406, 246), (406, 234), color="#9A9A9A", lw=0.9)
    note(ax, 505, 254, "/scan → costmaps", fs=6.5, ha="center")

    # offline -> online
    S.arrow(ax, (266, 300), (300, 300), dashed=True, color=S.NAVY)
    note(ax, 283, 312, "E₀, N₀", fs=6.5, ha="center", color=S.NAVY)
    line(ax, [(266, 113), (290, 113), (290, 192)], dashed=True,
         color=S.NAVY)
    S.arrow(ax, (290, 192), (300, 192), dashed=True, color=S.NAVY)
    note(ax, 283, 150, "prior map", fs=6.5, ha="center", color=S.NAVY,
         rotation=90)

    # NOT USED strip
    ax.add_patch(FancyBboxPatch((6, 6), 668, 52,
                                boxstyle="round,pad=0,rounding_size=6",
                                fc="#FFF6F6", ec="#E3B5B5", lw=0.8))
    note(ax, 20, 32, "NOT USED —\nanywhere in the system", fs=7.6,
         color=S.RED, style="normal", weight="bold")
    items = [(ic_satellite, "GNSS / GPS receiver"), (ic_cloud,
             "Internet / cloud"), (ic_tower, "Mobile network")]
    for i, (icon, label) in enumerate(items):
        cx = 205 + i * 175
        icon(ax, cx, 32, 17)
        forbid(ax, cx, 32, 20)
        note(ax, cx + 28, 32, label, fs=7.4, color=S.INK, style="normal",
             weight="bold")
    return S.save(fig, "context_offline_online")


def fig_layers():
    fig, ax = S.diagram(W, 4.8)
    layers = [
        ("Mission &\noperator", 404, 68, [
            ("utm_goal.py", "UTM goal → NavigateToPose;\nprogress in map + UTM",
             "impl"),
            ("RViz 2", "prior map, costmaps, plans,\ncamera, TF: the robot's view",
             "impl"),
            ("mission_logger.py", "ground truth vs EKF\n→ CSV at 5 Hz (scoring)",
             "eval")]),
        ("Navigation\n(Nav2, Jazzy)", 292, 104, [
            ("bt_navigator", "behaviour tree:\nNavigateToPose", "impl"),
            ("planner_server", "Smac 2D A* (GridBased)\nreplans at 1 Hz", "impl"),
            ("controller_server", "MPPI (FollowPath)\n15 Hz, 1800 samples", "impl"),
            ("behavior_server", "spin · back-up\nwait · drive on heading",
             "impl"),
            ("global + local costmap", "static + obstacle + inflation\n"
             "rolling local 14 × 14 m", "impl"),
            ("lifecycle_manager", "configures + activates\n5 Nav2 nodes (autostart)",
             "impl")]),
        ("Localization", 202, 82, [
            ("EKF", "robot_localization\nodom→base_link 30 Hz", "impl"),
            ("map→odom", "static identity;\nEKF seeded at start",
             "impl"),
            ("slam_toolbox", "lidar SLAM (sync)\nexperimental", "exp"),
            ("cuVSLAM", "Isaac ROS 4.6\ninstalled, not wired", "exp"),
            ("Map anchoring", "BEV ↔ satellite/DEM\nre-anchor ~200 m",
             "design")]),
        ("Perception\n& maps", 112, 82, [
            ("map_server", "/prior_map from\nGeoTIFF slope", "impl"),
            ("Lidar obstacles", "/scan ray-tracing\nmark 18 m, clear 20 m",
             "impl"),
            ("Semantics", "SegFormer + TensorRT\n(RELLIS-3D, RUGD)",
             "design"),
            ("GPU elevation", "2.5-D slope, step,\nroughness (cupy)", "design"),
            ("Localizability", "custom Nav2 layer:\nwhere VIO tracks",
             "design")]),
        ("Platform /\nsimulation", 22, 82, [
            ("Isaac Sim 6.0.1", "PhysX 60 Hz,\nrender 30 Hz", "impl"),
            ("ROS 2 bridge", "OmniGraph: /clock,\ncamera, /scan, joints",
             "impl"),
            ("Sensors", "RTX 2-D lidar, IMU\nRGB-D 640×360", "impl"),
            ("run_sim.py", "/cmd_vel → wheels;\nodometry + IMU", "impl"),
            ("Skid-steer UGV", "4 wheels, r 0.15 m\ntrack 0.70 m", "impl")]),
    ]
    x0, x1 = 104, 670
    for label, y, h, items in layers:
        ax.add_patch(FancyBboxPatch((6, y), 668, h,
                                    boxstyle="round,pad=0,rounding_size=6",
                                    fc="#FAFBFC", ec="#D5DDE6", lw=0.8,
                                    zorder=0))
        ax.text(52, y + h / 2, label, ha="center", va="center", fontsize=8.2,
                fontweight="bold", color=S.NAVY, linespacing=1.25)
        rows = 2 if len(items) > 5 else 1
        per = math.ceil(len(items) / rows)
        gap = 8
        bw = (x1 - x0 - gap * (per - 1)) / per
        bh = (h - 12 - (rows - 1) * 6) / rows
        for i, (t, sub, kind) in enumerate(items):
            r, c = divmod(i, per)
            bx = x0 + c * (bw + gap)
            by = y + h - 6 - (r + 1) * bh - r * 6
            five = per >= 5
            S.box(ax, bx, by, bw, bh, t, sub, kind=kind,
                  ts=7.2 if five else 7.5, ss=6.3 if five else 6.5)
    # dependency arrows between layers (left edge)
    for ya, yb in ((404, 396), (292, 284), (202, 194), (112, 104)):
        S.arrow(ax, (52, yb - 2), (52, ya + 2), color="#B0B8C2", lw=1.0)
    S.legend(ax, 12, 10, ["impl", "exp", "design", "eval"], size=6.6)
    return S.save(fig, "layered_architecture")


def fig_ros_graph():
    fig, ax = S.diagram(W, 5.8)
    ev = S.KINDS["eval"][1]
    # simulator node
    S.box(ax, 8, 118, 96, 360, "sih_isaac_sim",
          "run_sim.py inside\nIsaac Sim 6.0.1\n\nROS 2 bridge\n(OmniGraph)\n+ rclpy node\n\n"
          "publishes\nno TF", kind="impl", ts=7.6, ss=6.6)
    # sensor topics
    tx, tw = 176, 110
    rows = [("/clock", 455), ("/camera/image\n/camera/depth_image\n/camera/camera_info",
                              408), ("/scan", 350), ("/joint_states", 310),
            ("/imu/data", 270), ("/odom_wheel", 232), ("/cmd_vel", 185),
            ("/ground_truth", 140)]
    for name, y in rows:
        h = 44 if "\n" in name else 20
        topic(ax, tx, y, tw, h, name, kind="eval" if "ground" in name else None,
              fs=6.5)
        if name == "/cmd_vel":
            S.arrow(ax, (tx - tw / 2, y), (104, y))
        else:
            S.arrow(ax, (104, y), (tx - tw / 2, y),
                    color=ev if "ground" in name else "#7F7F7F")

    # middle nodes
    nx, nw = 318, 116
    pill(ax, nx, 545, nw, 28, "map_server", "GeoTIFF → map.pgm")
    pill(ax, nx, 500, nw, 28, "static_tf", "map → odom (identity)")
    pill(ax, nx, 460, nw, 28, "static_tf", "base_link → lidar_link")
    pill(ax, nx, 310, nw, 28, "robot_state_pub.", "URDF → /tf, /tf_static")
    pill(ax, nx, 251, nw, 34, "ekf_filter_node", "robot_localization")
    pill(ax, nx, 140, nw, 28, "mission_logger", "CSV @ 5 Hz", kind="eval")
    S.arrow(ax, (tx + tw / 2, 310), (nx - nw / 2, 310))
    S.arrow(ax, (tx + tw / 2, 270), (nx - nw / 2, 258))
    S.arrow(ax, (tx + tw / 2, 232), (nx - nw / 2, 244))
    S.arrow(ax, (tx + tw / 2, 140), (nx - nw / 2, 140), color=ev)
    S.arrow(ax, (205, 175), (nx - nw / 2 + 6, 150), color="#9A9A9A", lw=0.9)

    # second topic column
    t2, t2w = 442, 104
    topic(ax, t2, 545, t2w, 20, "/prior_map")
    S.arrow(ax, (nx + nw / 2, 545), (t2 - t2w / 2, 545))
    topic(ax, t2, 310, t2w, 44, "/tf\n/tf_static")
    topic(ax, t2, 238, t2w, 20, "/odometry/filtered")
    S.arrow(ax, (nx + nw / 2, 310), (t2 - t2w / 2, 310))
    S.arrow(ax, (nx + nw / 2, 262), (t2 - t2w / 2 + 12, 288))
    S.arrow(ax, (nx + nw / 2, 244), (t2 - t2w / 2, 240))
    # static TFs down into /tf_static
    line(ax, [(nx + nw / 2, 500), (452, 500), (452, 460)])
    line(ax, [(nx + nw / 2, 460), (452, 460)])
    S.arrow(ax, (452, 460), (452, 332))

    # Nav2 cluster
    cx0, cw = 522, 140
    ccx = cx0 + cw / 2
    ax.add_patch(FancyBboxPatch((cx0 - 8, 118), cw + 16, 402,
                                boxstyle="round,pad=0,rounding_size=7",
                                fc="#F7FAF7", ec="#C8DCC9", lw=0.8, zorder=0))
    note(ax, cx0 - 2, 509, "Nav2", fs=7.2, color="#1E5E23", style="normal",
         weight="bold")
    S.box(ax, cx0, 562 - 36, cw, 36, "utm_goal.py", "NavigateToPose action client",
          kind="impl", ts=7.4, ss=6.5)
    S.box(ax, cx0, 452, cw, 40, "bt_navigator", "behaviour tree; reads\n/odometry/filtered",
          kind="impl", ts=7.4, ss=6.5)
    S.box(ax, cx0, 378, cw, 46, "planner_server", "+ global_costmap\nSmac 2D A* → /plan",
          kind="impl", ts=7.4, ss=6.5)
    S.box(ax, cx0, 288, cw, 58, "controller_server",
          "+ local_costmap, MPPI →\n/cmd_vel, /local_plan,\n/trajectories",
          kind="impl", ts=7.4, ss=6.4)
    S.box(ax, cx0, 212, cw, 40, "behavior_server", "spin · backup · wait\n→ /cmd_vel",
          kind="impl", ts=7.4, ss=6.5)
    S.box(ax, cx0, 128, cw, 40, "lifecycle_manager", "configures + activates\nthe 5 Nav2 servers",
          kind="impl", ts=7.1, ss=6.5)
    S.arrow(ax, (ccx, 526), (ccx, 492))
    ax.text(ccx + 5, 509, "navigate_to_pose", fontsize=6.5, color=S.GREY,
            style="italic", va="center", ha="left")
    # action bus
    bx = cx0 + cw + 6
    line(ax, [(cx0 + cw, 462), (bx, 462), (bx, 232)], color=S.GREEN, lw=1.0)
    for y in (401, 319, 232):
        S.arrow(ax, (bx, y), (cx0 + cw, y), color=S.GREEN, lw=1.0)
    # lifecycle (dashed)
    S.arrow(ax, (cx0 + 20, 168), (cx0 + 20, 212), dashed=True, color="#8FB08F",
            lw=0.9)
    # /prior_map -> planner
    line(ax, [(t2 + t2w / 2, 545), (503, 545), (503, 412)])
    S.arrow(ax, (503, 412), (cx0, 412))
    # /scan channel -> planner + controller
    line(ax, [(tx + tw / 2, 350), (510, 350)])
    line(ax, [(510, 330), (510, 392)])
    S.arrow(ax, (510, 392), (cx0, 392))
    S.arrow(ax, (510, 330), (cx0, 330))
    # /odometry/filtered -> controller + logger
    line(ax, [(t2 + t2w / 2, 238), (498, 238), (498, 316)])
    S.arrow(ax, (498, 316), (cx0, 316))
    line(ax, [(t2 - 20, 228), (t2 - 20, 150)])
    S.arrow(ax, (t2 - 20, 150), (nx + nw / 2, 145))
    # /cmd_vel collector
    line(ax, [(cx0, 302), (514, 302), (514, 185)])
    line(ax, [(ccx, 212), (ccx, 185)])
    line(ax, [(ccx, 185), (tx + tw / 2, 185)])
    ax.plot([tx + tw / 2 + 4], [185], marker="<", color="#7F7F7F", ms=5,
            zorder=5)

    # rviz
    S.box(ax, 236, 30, 250, 62, "rviz2",
          "reads /camera/image, /scan, /prior_map, costmaps,\n/plan, /local_plan, "
          "/trajectories, /odometry/filtered, TF;\n'2D Goal Pose' → /goal_pose "
          "→ bt_navigator", kind="impl", ts=7.6, ss=6.5)
    note(ax, 8, 100, "Every node reads /clock\n(use_sim_time) and the TF tree.",
         fs=6.5)
    note(ax, 8, 64, "/ground_truth feeds only\nmission_logger (scoring);\n"
         "no navigation node\nsubscribes to it.", fs=6.5, color=S.KINDS["eval"][2])
    ax.add_patch(Rectangle((520, 80), 14, 10, fc="white", ec="#8C9BAB", lw=0.9))
    note(ax, 540, 85, "topic", fs=6.6, color=S.INK, style="normal")
    ax.add_patch(FancyBboxPatch((520, 60), 18, 10,
                                boxstyle="round,pad=0,rounding_size=4.5",
                                fc=S.KINDS["impl"][0], ec=S.KINDS["impl"][1]))
    note(ax, 544, 65, "node", fs=6.6, color=S.INK, style="normal")
    ax.plot([520, 538], [45, 45], color=S.GREEN, lw=1.0)
    note(ax, 544, 45, "action call", fs=6.6, color=S.INK, style="normal")
    ax.add_patch(Rectangle((520, 20), 14, 10, fc=S.KINDS["eval"][0],
                           ec=S.KINDS["eval"][1], lw=0.9))
    note(ax, 540, 25, "evaluation only", fs=6.6, color=S.INK, style="normal")
    return S.save(fig, "ros_graph")


def fig_tf_tree():
    fig, ax = S.diagram(W, 3.0)
    S.box(ax, 10, 158, 70, 40, "map", kind="neutral", ts=8.5)
    S.box(ax, 150, 158, 70, 40, "odom", kind="neutral", ts=8.5)
    S.box(ax, 300, 158, 84, 40, "base_link", kind="neutral", ts=8.5)
    S.arrow(ax, (80, 178), (150, 178))
    S.arrow(ax, (220, 178), (300, 178))
    note(ax, 115, 222, "static identity (default)\nor slam_toolbox\n(slam:=true)",
         fs=6.5, ha="center", color=S.NAVY, style="normal")
    note(ax, 260, 214, "robot_localization\nEKF, 30 Hz", fs=6.5, ha="center",
         color=S.NAVY, style="normal")
    kids = [
        ("wheel_fl / fr / rl / rr_link", "robot_state_publisher → /tf "
         "(continuous joints, from /joint_states)", 262),
        ("imu_link", "robot_state_publisher → /tf_static (z 0.105 m)", 222),
        ("camera_link", "robot_state_publisher → /tf_static (x 0.37, z 0.125 m)",
         182),
        ("base_link_gt", "robot_state_publisher → /tf_static (zero offset)", 142),
        ("lidar_link", "static_transform_publisher → /tf_static (z 0.45 m)", 102),
    ]
    kx, kw = 470, 150
    for name, owner, y in kids:
        S.box(ax, kx, y - 13, kw, 26, name, kind="white", ts=7.2)
        line(ax, [(384, 178), (420, 178), (420, y), (kx, y)], color="#8A8A8A",
             lw=1.0)
        ax.plot([kx - 3], [y], marker=">", color="#8A8A8A", ms=4.5, zorder=5)
    S.box(ax, 628, 170, 46, 24, "optical", kind="white", ts=6.6)
    S.arrow(ax, (620, 182), (628, 182))
    note(ax, 651, 161, "REP-103", fs=6.5, ha="center")
    owners_y = 78
    for i, (name, owner, _) in enumerate(kids):
        note(ax, 300, owners_y - i * 13, f"• {name.split(' ')[0]}: {owner}",
             fs=6.5, color=S.INK, style="normal")
    note(ax, 300, owners_y - 5 * 13, "• camera_optical_frame: robot_state_publisher "
         "(rpy −90°, 0, −90°)", fs=6.5, color=S.INK, style="normal")
    ax.add_patch(FancyBboxPatch((10, 12), 270, 104,
                                boxstyle="round,pad=0,rounding_size=6",
                                fc="#FFF8EC", ec="#E4C99A", lw=0.8))
    note(ax, 20, 64,
         "Exactly one publisher per edge.\n\n"
         "• Isaac Sim publishes no TF at all.\n"
         "• /ground_truth (map → base_link_gt) is a\n"
         "  topic for scoring, not part of /tf.\n"
         "• Nav2 publishes no TF.",
         fs=6.9, color=S.INK, style="normal")
    return S.save(fig, "tf_tree")


def fig_phases():
    fig, ax = S.diagram(W, 2.55)
    phases = [
        ("PHASE 0", "MuJoCo prototype", "proto", "DONE",
         "ORB + PnP-RANSAC\nstereo VSLAM\nA* on GeoTIFF\nDWA local planner\nUTM goal, pyproj"),
        ("PHASE 1", "Gazebo world", "proto", "DONE",
         "Gazebo Harmonic\nprocedural terrain\nrover + sensors\nros_gz bridge\nframe checks"),
        ("PHASE 2", "Isaac nav stack", "impl", "DONE (testbed)",
         "EKF localization\nNav2: Smac 2D A*\n+ MPPI + recoveries\nUTM goal → map\nmeasured missions"),
        ("PHASE 2b", "cuVSLAM wiring", "exp", "NEXT",
         "Isaac ROS cuVSLAM\ninstalled; replace\nsynthesized VIO-\ngrade odometry"),
        ("PHASE 3", "Perception", "design", "DESIGNED",
         "semantic seg.\n(SegFormer)\nGPU 2.5-D elevation\nlocalizability\ncost layer"),
        ("PHASE 4", "Robustness", "design", "DESIGNED",
         "map anchoring\n(BEV ↔ satellite)\nintegrity monitor\ndegraded mode\nsafe stop"),
    ]
    n = len(phases)
    gap = 8
    cw = (668 - gap * (n - 1)) / n
    y0, h = 50, 196
    for i, (ph, title, kind, status, body) in enumerate(phases):
        x = 6 + i * (cw + gap)
        fc, ec, tc, dashed = S.KINDS[kind]
        ax.add_patch(FancyBboxPatch((x, y0), cw, h,
                                    boxstyle="round,pad=0,rounding_size=6",
                                    fc=fc, ec=ec, lw=1.1, zorder=2,
                                    linestyle=(0, (4, 2.5)) if dashed else "-"))
        ax.add_patch(FancyBboxPatch((x, y0 + h - 30), cw, 30,
                                    boxstyle="round,pad=0,rounding_size=6",
                                    fc=ec, ec=ec, lw=0, zorder=3))
        ax.add_patch(Rectangle((x, y0 + h - 30), cw, 10, fc=ec, ec="none",
                               zorder=3))
        ax.text(x + cw / 2, y0 + h - 15, ph, ha="center", va="center",
                fontsize=8.2, fontweight="bold", color="white", zorder=4)
        ax.text(x + cw / 2, y0 + h - 44, title, ha="center", va="center",
                fontsize=7.1, fontweight="bold", color=tc, zorder=4)
        ax.text(x + cw / 2, y0 + h - 58 - 40, body, ha="center", va="center",
                fontsize=6.6, color="#333333", zorder=4, linespacing=1.3)
        ax.add_patch(FancyBboxPatch((x + 10, y0 + 8), cw - 20, 18,
                                    boxstyle="round,pad=0,rounding_size=8",
                                    fc="white", ec=ec, lw=1.0, zorder=4))
        ax.text(x + cw / 2, y0 + 17, status, ha="center", va="center",
                fontsize=6.6, fontweight="bold", color=tc, zorder=5)
        if i < n - 1:
            S.arrow(ax, (x + cw + 1, y0 + h / 2), (x + cw + gap - 1, y0 + h / 2),
                    lw=1.0)
    S.legend(ax, 10, 30, ["proto", "impl", "exp", "design"], size=6.3,
             dx=172)
    note(ax, 10, 11, "Phases 0–1 and the Gazebo phase gates were run first; "
         "Phase 2 was then built on Isaac Sim 6.0.1 as the integration testbed.",
         fs=6.5)
    return S.save(fig, "phases_roadmap")


def fig_mission_flow():
    fig, ax = S.diagram(W, 8.2)
    # 1 offline
    S.band(ax, 6, 712, 668, 102,
           "1   OFFLINE PREPARATION — once per site, no vehicle, no network",
           "offline")
    S.box(ax, 16, 724, 126, 62, "world.py",
          "seeded DEM (257 × 257)\nberm, tunnel, plateau\nobstacle + human layout",
          kind="offline", ts=7.8, ss=6.6)
    S.box(ax, 170, 724, 172, 62, "Generated files",
          "prior_dem_utm44n.tif · dem.npy\nmap.pgm + map.yaml (slope > 15°)\n"
          "world.json (anchor, start, layout)", kind="offline", ts=7.8, ss=6.6)
    S.box(ax, 370, 724, 136, 62, "build_scene.py",
          "DEM → terrain mesh\ntunnel roof, obstacles,\nhumans, lights",
          kind="offline", ts=7.8, ss=6.6)
    S.box(ax, 534, 724, 130, 62, "sih26126_world.usd",
          "the Isaac Sim world:\nsame DEM as the\nplanner's prior", kind="offline",
          ts=7.6, ss=6.6)
    S.arrow(ax, (142, 755), (170, 755))
    S.arrow(ax, (342, 755), (370, 755))
    S.arrow(ax, (506, 755), (534, 755))
    # 2 bring-up
    S.band(ax, 6, 600, 668, 104, "2   BRING-UP — two processes, sim time on /clock",
           "neutral")
    S.box(ax, 16, 612, 300, 64, "run_isaac.sh → run_sim.py  (Isaac Sim)",
          "loads the USD; ROS 2 bridge publishes /clock, camera,\n/scan, /imu/data, "
          "/odom_wheel, /joint_states;\nsubscribes /cmd_vel → wheel drives",
          kind="impl", ts=7.8, ss=6.6)
    S.box(ax, 350, 612, 314, 64, "ros2 launch sih_isaac nav.launch.py",
          "EKF seeded at the surveyed pose (−62, 16, 8°)\nstatic map→odom "
          "· map_server → /prior_map\nNav2 lifecycle autostart · RViz",
          kind="impl", ts=7.8, ss=6.6)
    S.arrow(ax, (316, 644), (350, 644))
    note(ax, 333, 656, "“[sim] running”", fs=6.5, ha="center")
    S.arrow(ax, (300, 712), (300, 676), color="#B0A890")
    S.arrow(ax, (505, 724), (505, 676), color="#B0A890")
    # 3 mission
    S.band(ax, 6, 8, 668, 584, "3   MISSION — closed loop until the goal",
           "white")
    cx, bw = 232, 264
    bx = cx - bw / 2
    S.box(ax, bx, 522, bw, 44, "Operator enters the UTM goal",
          "ros2 run sih_isaac utm_goal.py -e E -n N", kind="white", ts=7.8,
          ss=6.7)
    S.box(ax, bx, 458, bw, 44, "Convert to the map frame",
          "x = E − E₀ ,  y = N − N₀   (anchor from world.json)",
          kind="impl", ts=7.8, ss=6.7)
    diamond(ax, cx, 408, 150, 52, "Inside the\n150 m site?", fs=7.0)
    S.box(ax, 470, 390, 190, 36, "Reject: exit, no goal sent", kind="eval",
          ts=7.4)
    S.box(ax, bx, 322, bw, 44, "NavigateToPose action  (frame: map)",
          "bt_navigator starts the behaviour tree", kind="impl", ts=7.8, ss=6.7)
    S.box(ax, bx, 262, bw, 44, "ComputePathToPose — Smac 2D A*",
          "global costmap: GeoTIFF prior + lidar obstacles", kind="impl",
          ts=7.8, ss=6.7)
    S.box(ax, bx, 200, bw, 44, "FollowPath — MPPI at 15 Hz",
          "local costmap 8 Hz · EKF pose 30 Hz · /cmd_vel", kind="impl",
          ts=7.8, ss=6.7)
    diamond(ax, cx, 152, 150, 50, "Within 0.7 m\nof the goal?", fs=7.0)
    diamond(ax, cx, 78, 170, 56, "Progress ≥ 0.5 m\nin 20 s, no error?",
            fs=6.9)
    S.box(ax, 470, 134, 190, 40, "GOAL REACHED",
          "SUCCEEDED (status 4): final pose\nprinted in map + UTM", kind="impl",
          ts=7.8, ss=6.5)
    S.box(ax, 440, 26, 222, 84, "Recovery (round robin)",
          "clear local + global costmaps → spin\n1.57 rad → wait 5 s → back up\n"
          "0.30 m @ 0.15 m/s; up to 6 retries,\nthen ABORTED", kind="neutral",
          ts=7.8, ss=6.6)
    S.box(ax, 440, 206, 222, 58, "Feedback every cycle",
          "pose in map + UTM, distance\nremaining, number of recoveries\n(utm_goal.py prints it)",
          kind="white", ts=7.6, ss=6.5)
    # main flow arrows
    S.arrow(ax, (cx, 522), (cx, 502))
    S.arrow(ax, (cx, 458), (cx, 434))
    S.arrow(ax, (cx, 382), (cx, 366))
    note(ax, cx + 6, 374, "yes", fs=6.6, style="normal", color=S.GREEN,
         weight="bold")
    S.arrow(ax, (cx + 75, 408), (470, 408))
    note(ax, cx + 90, 418, "no", fs=6.6, style="normal", color=S.RED,
         weight="bold")
    S.arrow(ax, (cx, 322), (cx, 306))
    S.arrow(ax, (cx, 262), (cx, 244))
    S.arrow(ax, (cx, 200), (cx, 177))
    S.arrow(ax, (cx + 75, 152), (470, 152))
    note(ax, cx + 90, 162, "yes", fs=6.6, style="normal", color=S.GREEN,
         weight="bold")
    S.arrow(ax, (cx, 127), (cx, 106))
    note(ax, cx + 6, 116, "no", fs=6.6, style="normal", color=S.RED,
         weight="bold")
    # progress ok -> loop back to FollowPath
    line(ax, [(cx - 85, 78), (60, 78), (60, 222)], color=S.GREEN)
    S.arrow(ax, (60, 222), (bx, 222), color=S.GREEN)
    note(ax, 64, 150, "yes: keep\nfollowing", fs=6.5, color=S.GREEN)
    # replan loop at 1 Hz
    S.arrow(ax, (bx + 20, 244), (bx + 20, 262), color=S.NAVY, dashed=True)
    note(ax, bx + 26, 253, "replan 1 Hz", fs=6.5,
         color=S.NAVY)
    # progress not ok -> recovery
    S.arrow(ax, (cx + 85, 78), (440, 78), color=S.RED)
    note(ax, cx + 100, 88, "no", fs=6.6, style="normal", color=S.RED,
         weight="bold")
    # recovery -> replan
    line(ax, [(662, 68), (670, 68), (670, 290)], color=S.NAVY)
    S.arrow(ax, (670, 290), (bx + bw, 290), color=S.NAVY)
    note(ax, 500, 298, "retry: compute a new path", fs=6.5, color=S.NAVY)
    # feedback dashed
    S.arrow(ax, (bx + bw, 222), (440, 232), dashed=True, color="#9A9A9A")
    note(ax, 20, 560, "legend", fs=6.5) if False else None
    S.legend(ax, 20, 18, ["impl", "eval"], size=6.5)
    return S.save(fig, "mission_flowchart")


def fig_bt():
    fig, ax = S.diagram(W, 3.6)

    def tnode(x, y, w, h, text, kind, fs=6.7):
        S.box(ax, x, y - h / 2, w, h, text, kind=kind, ts=fs)
        return (x, y, w)

    def edge(parent, child_x, child_y):
        px, py, pw = parent
        xm = (px + pw + child_x) / 2
        line(ax, [(px + pw, py), (xm, py), (xm, child_y), (child_x, child_y)],
             color="#8A8A8A", lw=0.9, z=1)

    ctl, cond, act = "neutral", "white", "impl"
    root = tnode(4, 190, 94, 46, "RecoveryNode\nNavigateRecovery\n(6 retries)",
                 ctl, 6.2)
    pipe = tnode(106, 288, 114, 36, "PipelineSequence\nNavigateWithReplanning",
                 ctl, 6.2)
    rseq = tnode(104, 108, 110, 36, "Sequence\n(recovery branch)", ctl, 6.4)
    edge(root, 104, 288)
    edge(root, 104, 108)
    rate = tnode(230, 300, 100, 30, "RateController\n1 Hz", ctl, 6.4)
    fprec = tnode(230, 232, 100, 34, "RecoveryNode\nFollowPath (1 retry)", ctl,
                  6.2)
    fb = tnode(230, 170, 100, 26, "Fallback", ctl, 6.4)
    rfb = tnode(230, 92, 100, 34, "ReactiveFallback\nRecoveryFallback", ctl,
                6.2)
    edge(pipe, 230, 300)
    edge(pipe, 230, 232)
    edge(rseq, 230, 170)
    edge(rseq, 230, 92)
    cprec = tnode(348, 300, 106, 34, "RecoveryNode\nComputePath (1 retry)",
                  ctl, 6.2)
    rr = tnode(348, 62, 106, 34, "RoundRobin\nRecoveryActions", ctl, 6.4)
    edge(rate, 348, 300)
    edge(rfb, 348, 62)

    lx, lw = 474, 200
    leaves = [
        (pipe, 340, "select FollowPath / GridBased", act),
        (cprec, 312, "ComputePathToPose → Smac 2D A*", act),
        (cprec, 285, "recoverable? clear global costmap", cond),
        (fprec, 246, "FollowPath → MPPI controller", act),
        (fprec, 219, "recoverable? clear local costmap", cond),
        (fb, 181, "would a recovery help?", cond),
        (rfb, 150, "GoalUpdated (new goal pre-empts)", cond),
        (rr, 118, "clear local + global costmaps", act),
        (rr, 91, "Spin 1.57 rad", act),
        (rr, 64, "Wait 5.0 s", act),
        (rr, 37, "BackUp 0.30 m at 0.15 m/s", act),
    ]
    for parent, y, text, kind in leaves:
        if parent is pipe:
            px, py, pw = pipe
            line(ax, [(px + pw, py + 8), (220, py + 8), (220, y), (lx, y)],
                 color="#8A8A8A", lw=0.9, z=1)
        elif parent in (fprec, fb):
            px, py, pw = parent
            xm = 462
            line(ax, [(px + pw, py), (xm, py), (xm, y), (lx, y)],
                 color="#8A8A8A", lw=0.9, z=1)
        elif parent is rfb:
            px, py, pw = parent
            line(ax, [(px + pw, py + 6), (340, py + 6), (340, y), (lx, y)],
                 color="#8A8A8A", lw=0.9, z=1)
        else:
            edge(parent, lx, y)
        S.box(ax, lx, y - 11, lw, 22, text, kind=kind, ts=6.5)
    note(ax, 6, 338, "normal navigation:\nplan at 1 Hz, follow continuously",
         fs=6.5, color=S.NAVY)
    note(ax, 104, 40, "runs only after a failure;\none recovery per retry,\nin rotation",
         fs=6.5, color=S.NAVY)
    ax.add_patch(FancyBboxPatch((226, 14), 14, 10, boxstyle="round,pad=0,rounding_size=2",
                                fc=S.KINDS[ctl][0], ec=S.KINDS[ctl][1]))
    note(ax, 244, 19, "control", fs=6.5, color=S.INK, style="normal")
    ax.add_patch(FancyBboxPatch((290, 14), 14, 10, boxstyle="round,pad=0,rounding_size=2",
                                fc=S.KINDS[cond][0], ec=S.KINDS[cond][1]))
    note(ax, 308, 19, "condition", fs=6.5, color=S.INK, style="normal")
    ax.add_patch(FancyBboxPatch((360, 14), 14, 10, boxstyle="round,pad=0,rounding_size=2",
                                fc=S.KINDS[act][0], ec=S.KINDS[act][1]))
    note(ax, 378, 19, "action", fs=6.5, color=S.INK, style="normal")
    return S.save(fig, "bt_navigator")


def fig_ekf_loop():
    fig, ax = S.diagram(W, 3.2)
    # inputs (left)
    S.box(ax, 8, 234, 176, 70, "Initial state (surveyed)",
          "x = −62 m,  y = 16 m,  ψ = 8°\nodom = map at t₀",
          kind="offline", ts=7.6, ss=6.5)
    S.box(ax, 8, 128, 176, 78, "/odom_wheel   (30 Hz)",
          "body velocity vₓ, vᵧ (VIO-grade,\nfrom the rigid-body state)\n"
          "variance 0.02 (m/s)²", kind="impl", ts=7.6, ss=6.5)
    S.box(ax, 8, 16, 176, 88, "/imu/data   (30 Hz)",
          "absolute yaw ψ (var 0.05 rad²)\nyaw rate ω (var 0.005)\n"
          "forward accel. aₓ (var 0.04)", kind="impl", ts=7.6, ss=6.5)
    # the filter
    S.box(ax, 226, 216, 238, 88, "PREDICT   (every 1/30 s)",
          "$\\hat{x}^{-} = f(\\hat{x},\\,\\Delta t)$\n"
          "$P^{-} = F\\,P\\,F^{T} + Q\\,\\Delta t$\n"
          "omnidirectional kinematic model", kind="impl", ts=7.6, ss=6.9)
    S.box(ax, 226, 16, 238, 100, "UPDATE   (per measurement)",
          "$K = P^{-} H^{T} (H P^{-} H^{T} + R)^{-1}$\n"
          "$\\hat{x} = \\hat{x}^{-} + K\\,(z - H\\hat{x}^{-})$\n"
          "$P = (I-KH)P^{-}(I-KH)^{T} + KRK^{T}$", kind="impl", ts=7.6,
          ss=6.9)
    S.box(ax, 300, 140, 90, 52, "State", "x̂ (15-D), P\ntwo_d_mode",
          kind="neutral", ts=7.4, ss=6.4)
    S.arrow(ax, (430, 216), (430, 116), color=S.NAVY, lw=1.4)
    note(ax, 436, 166, "prior\nx̂⁻, P⁻", fs=6.6, color=S.NAVY)
    S.arrow(ax, (260, 116), (260, 216), color=S.NAVY, lw=1.4)
    note(ax, 254, 166, "posterior\nx̂, P", fs=6.6, color=S.NAVY, ha="right")
    # inputs into the filter
    S.arrow(ax, (184, 269), (226, 269), dashed=True, color="#B0A890")
    note(ax, 205, 278, "x₀", fs=6.8, ha="center", color="#8A7F5E")
    S.arrow(ax, (184, 150), (226, 92), color="#7F7F7F")
    S.arrow(ax, (184, 60), (226, 60), color="#7F7F7F")
    note(ax, 205, 112, "z", fs=7.2, ha="center", color=S.INK)
    # outputs (right)
    S.box(ax, 510, 196, 162, 80, "/odometry/filtered",
          "pose + twist, frame odom\n→ controller_server,\nbt_navigator, logger",
          kind="impl", ts=7.4, ss=6.5)
    S.box(ax, 510, 92, 162, 64, "TF  odom → base_link",
          "sole owner of this edge\n(publish_tf: true)", kind="impl", ts=7.4,
          ss=6.5)
    S.arrow(ax, (464, 90), (510, 226))
    S.arrow(ax, (464, 70), (510, 118))
    note(ax, 510, 50, "No GNSS input exists; nothing\nresets the filter from a "
         "satellite.", fs=6.5)
    return S.save(fig, "ekf_loop")


def fig_vslam():
    fig, ax = S.diagram(W, 3.2)
    S.band(ax, 6, 172, 668, 142,
           "FRONT-END (tracking) — implemented in the MuJoCo prototype, vslam.py",
           "white")
    steps = [
        ("Stereo / RGB-D", "left image +\ndepth map"),
        ("ORB features", "FAST corners +\nrBRIEF, 1500/frame"),
        ("Match", "Hamming distance,\nLowe ratio 0.78"),
        ("Back-project", "pinhole + depth\n→ 3-D landmarks"),
        ("PnP-RANSAC", "2.5 px, 220 iter.\n+ LM refinement"),
        ("Keyframe?", "moved 0.45 m, 11°\nor < 40 inliers"),
    ]
    gap = 14
    bw = (648 - gap * 5) / 6
    for i, (t, sub) in enumerate(steps):
        x = 16 + i * (bw + gap)
        S.box(ax, x, 206, bw, 64, t, sub, kind="proto", ts=7.6, ss=6.5)
        if i < 5:
            S.arrow(ax, (x + bw, 238), (x + bw + gap, 238))
    xk = 16 + 5 * (bw + gap)
    x3 = 16 + 2 * (bw + gap)
    line(ax, [(xk + bw / 2, 270), (xk + bw / 2, 288), (x3 + bw / 2, 288)],
         color=S.NAVY, dashed=True)
    S.arrow(ax, (x3 + bw / 2, 288), (x3 + bw / 2, 270), color=S.NAVY,
            dashed=True)
    note(ax, (x3 + xk + bw) / 2, 296, "new keyframe: later frames match against it",
         fs=6.5, ha="center", color=S.NAVY)

    S.band(ax, 6, 8, 668, 152,
           "BACK-END (mapping) — provided by cuVSLAM / RTAB-Map; not in the prototype",
           "exp")
    bw2 = 116
    back = [("Local map", "sparse landmarks in\nthe world frame", "proto"),
            ("Pose graph / BA", "optimise keyframe poses\n+ landmarks together", "exp"),
            ("Loop closure", "bag-of-words place\nrecognition", "exp"),
            ("Global map", "drift-corrected\nkeyframe graph", "exp")]
    xs = [16, 150, 284, 418]
    for (t, sub, kind), x in zip(back, xs):
        S.box(ax, x, 50, bw2, 64, t, sub, kind=kind, ts=7.6, ss=6.5)
    for a, b in zip(xs[:-1], xs[1:]):
        S.arrow(ax, (a + bw2, 82), (b, 82))
    S.arrow(ax, (284 + 58, 50), (150 + 58, 50), rad=-0.45, color="#8A8A8A",
            dashed=True)
    note(ax, 276, 24, "loop constraint", fs=6.5, ha="center")
    S.box(ax, 552, 50, 112, 64, "Odometry → EKF",
          "camera pose as a\nmotion measurement", kind="impl", ts=7.6, ss=6.5)
    x5 = 16 + 4 * (bw + gap)
    line(ax, [(x5 + bw / 2, 206), (x5 + bw / 2, 186), (608, 186)],
         color=S.NAVY)
    S.arrow(ax, (608, 186), (608, 114), color=S.NAVY)
    line(ax, [(xk + bw / 2, 206), (xk + bw / 2, 196), (74, 196)],
         color="#8A8A8A")
    S.arrow(ax, (74, 196), (74, 114), color="#8A8A8A")
    S.legend(ax, 470, 150, ["proto", "exp"], size=6.5, dx=100) if False else None
    return S.save(fig, "vslam_pipeline")


def fig_mppi_cycle():
    fig, ax = S.diagram(W, 2.6)
    steps = [
        ("Shift", "reuse last\nsequence U"),
        ("Sample", "K = 1800 samples\nσᵥ 0.2, σω 0.4"),
        ("Roll out", "DiffDrive model\n56 × 0.067 s"),
        ("Score", "8 critics\n→ cost Sₖ"),
        ("Weight", "wₖ ∝ exp(−Sₖ/λ)\nλ = 0.3"),
        ("Update", "U ← U + Σ wₖ εₖ\n+ SG smoothing"),
        ("Publish", "first command\n→ /cmd_vel"),
    ]
    gap = 9
    bw = (664 - gap * 6) / 7
    y = 112
    for i, (t, sub) in enumerate(steps):
        x = 8 + i * (bw + gap)
        S.box(ax, x, y, bw, 60, t, sub, kind="impl", ts=7.6, ss=6.2)
        if i < 6:
            S.arrow(ax, (x + bw, y + 30), (x + bw + gap, y + 30))
    xs = 8 + 3 * (bw + gap)
    for j, lab in enumerate(("local costmap", "/plan", "/odometry/filtered")):
        xx = xs - 108 + j * 108 + bw / 2
        topic(ax, xx, 232, 102, 18, lab, fs=6.5)
        S.arrow(ax, (xx, 223), (xs + bw / 2 + (j - 1) * 22, y + 60),
                color="#9A9A9A", lw=0.9)
    xl = 8 + 6 * (bw + gap)
    line(ax, [(xl + bw / 2, y), (xl + bw / 2, 84), (8 + bw / 2, 84)],
         color=S.NAVY)
    S.arrow(ax, (8 + bw / 2, 84), (8 + bw / 2, y), color=S.NAVY)
    note(ax, 340, 93, "next cycle after 1/15 s (controller_frequency 15 Hz)",
         fs=6.6, ha="center", color=S.NAVY)
    note(ax, 340, 52,
         "Critic weights:  Constraint 4.0 · Cost 3.81 · Goal 5.0 · "
         "GoalAngle 3.0 · PathAlign 14.0 · PathFollow 5.0 · "
         "PathAngle 2.0 · PreferForward 5.0", fs=6.6, ha="center",
         color=S.INK, style="normal")
    note(ax, 340, 32,
         "Limits: vx ∈ [−0.25, 0.90] m/s, |wz| ≤ 1.0 rad/s · "
         "γ = 0.015 · motion model DiffDrive · visualize: true "
         "(/trajectories)", fs=6.6, ha="center", color=S.INK, style="normal")
    return S.save(fig, "mppi_cycle")


def fig_innovations():
    fig, ax = S.diagram(W, 4.4)
    cols = [8, 234, 460]
    cw = 212
    heads = ["① Map-anchored localization", "② Localizability cost layer",
             "③ Integrity monitor"]
    for x, h in zip(cols, heads):
        ax.add_patch(FancyBboxPatch((x, 396), cw, 34,
                                    boxstyle="round,pad=0,rounding_size=6",
                                    fc=S.KINDS["design"][1], ec="none"))
        ax.text(x + cw / 2, 413, h, ha="center", va="center", fontsize=8.2,
                fontweight="bold", color="white")

    def col(x, items, y_top=384, gap=10):
        y = y_top
        centers = []
        for t, sub, h in items:
            y -= h
            S.box(ax, x, y, cw, h, t, sub, kind="design", ts=7.4, ss=6.5)
            centers.append((x + cw / 2, y, y + h))
            y -= gap
        for a, b in zip(centers[:-1], centers[1:]):
            S.arrow(ax, (a[0], a[1]), (b[0], b[2]), color="#C0A176")
        return centers

    c1 = col(cols[0], [
        ("Camera + 2.5-D elevation", "what the rover sees right now", 40),
        ("Bird's-eye view (BEV)", "project onto the ground plane", 40),
        ("Cross-view match", "vs preloaded satellite / DEM tile,\nsearched around the "
         "predicted pose", 50),
        ("Pose + heading + covariance", "an absolute fix without GNSS", 40),
        ("χ² innovation gate", "reject implausible matches", 40)])
    c2 = col(cols[1], [
        ("Satellite / DEM prior", "texture, edges, feature density", 40),
        ("Localizability score", "per cell: how well VIO\nwill keep tracking here", 50),
        ("Nav2 costmap plugin", "custom C++ layer beside\nstatic / obstacle / inflation",
         50),
        ("Planner trade-off", "slightly longer route through\nfeature-rich ground", 50)])
    c3 = col(cols[2], [
        ("VIO  vs  IMU + wheel odometry", "two independent motion sources", 40),
        ("Consistency test", "normalised innovation squared\n(NIS) against a χ² bound",
         50),
        ("Mode", "NORMAL → DEGRADED → SAFE STOP", 40),
        ("Actions", "limit speed · request re-anchor\n· stop and hold", 50)])
    # implemented anchors
    S.box(ax, cols[0], 30, cw, 50, "robot_localization EKF",
          "absolute pose update (every ~200 m)\non top of VIO dead reckoning",
          kind="impl", ts=7.6, ss=6.5)
    S.box(ax, cols[1], 30, cw, 50, "Nav2 global costmap + Smac",
          "existing layered costmap and\nplanner, unchanged", kind="impl",
          ts=7.6, ss=6.5)
    S.box(ax, cols[2], 30, cw, 50, "MPPI controller + EKF",
          "speed limit / stop command;\nmeasurement weighting", kind="impl",
          ts=7.6, ss=6.5)
    for c in (c1, c2, c3):
        last = c[-1]
        S.arrow(ax, (last[0], last[1]), (last[0], 80), color=S.GREEN)
    # re-anchor request: col3 -> col1
    y_req = c3[-1][1] + 25
    line(ax, [(cols[2], y_req), (cols[1] + cw + 6, y_req), (cols[1] + cw + 6, 90),
              (cols[0] + cw + 6, 90)], color=S.ORANGE, dashed=True, lw=1.0)
    S.arrow(ax, (cols[0] + cw + 6, 90), (cols[0] + cw + 6, c1[-2][1] + 20),
            color=S.ORANGE, dashed=True, lw=1.0)
    note(ax, (cols[0] + cw + cols[1] + cw / 2) / 2 + 4, 98, "re-anchor request",
         fs=6.5, ha="center", color=S.ORANGE)
    S.legend(ax, 12, 12, ["impl", "design"], size=6.6)
    return S.save(fig, "qslam_innovations")


def fig_integrity_fsm():
    fig, ax = S.diagram(W, 2.7)
    by = 128
    states = [(20, "NORMAL", "full speed;\nVIO + EKF as usual"),
              (262, "DEGRADED", "reduced speed; IMU +\nwheel dead reckoning;\nrequest re-anchor"),
              (504, "SAFE STOP", "stop, hold position,\nreport to operator")]
    for x, t, sub in states:
        S.box(ax, x, by, 156, 66, t, sub, kind="design", ts=8.2, ss=6.6)
    top, bot = by + 66, by
    S.arrow(ax, (150, top), (288, top), rad=-0.35, color=S.ORANGE)
    note(ax, 219, top + 46, "NIS > χ² gate for N cycles,\nor VIO tracking lost",
         fs=6.6, ha="center", color=S.INK, style="normal")
    S.arrow(ax, (392, top), (530, top), rad=-0.35, color=S.RED)
    note(ax, 461, top + 46, "degraded longer than T s,\nor error bound > limit",
         fs=6.6, ha="center", color=S.INK, style="normal")
    S.arrow(ax, (288, bot), (150, bot), rad=-0.35, color=S.GREEN)
    note(ax, 252, bot - 40, "consistent again, or\nre-anchor succeeded", fs=6.6,
         ha="center", color=S.INK, style="normal")
    S.arrow(ax, (560, bot), (100, bot), rad=-0.4, color="#8A8A8A", dashed=True)
    note(ax, 520, 44, "re-anchor + operator\nclearance", fs=6.6, ha="center",
         color=S.GREY)
    note(ax, 6, 10, "QSLAM design (Phase 4) — thresholds N, T and limits are "
         "to be tuned; this monitor is not in the repository yet.", fs=6.5)
    return S.save(fig, "integrity_fsm")


def fig_recording():
    fig, ax = S.diagram(W, 2.4)
    S.box(ax, 6, 70, 108, 100, "record_mission.sh",
          "orchestrates one\nrecorded mission;\ncleans up on exit", kind="impl",
          ts=6.9, ss=6.5)
    procs = [
        ("run_sim.py --headless --record", "3 follow cameras → mp4", 206),
        ("nav.launch.py rviz:=true", "lifecycle retry × 3", 168),
        ("ffmpeg x11grab", "RViz screen, 15 fps", 130),
        ("mission_logger.py", "GT vs EKF vs cmd, 5 Hz", 92),
        ("utm_goal.py -e E -n N", "runs the mission", 54),
    ]
    arts = ["3 × isaac_*.mp4", "nav.log", "rviz.mp4",
            "mission_log.csv", "goal.log"]
    for (t, _sub, y), art in zip(procs, arts):
        S.box(ax, 138, y - 16, 212, 32, t, None, kind="impl", ts=7.0)
        line(ax, [(114, 120), (124, 120), (124, y), (138, y)], color="#8A8A8A",
             lw=0.9)
        topic(ax, 440, y, 136, 20, art, fs=6.5)
        S.arrow(ax, (350, y), (372, y), lw=0.9)
    S.box(ax, 540, 152, 134, 56, "make_highlight.py",
          "→ QSLAM_highlight.mp4\n(PIL text overlays)", kind="impl", ts=7.0,
          ss=6.5)
    S.box(ax, 540, 44, 134, 56, "make_mission_report.py",
          "→ Mission_Report.pdf\nfigures + stills", kind="impl", ts=6.5,
          ss=6.5)
    for y in (206, 130):
        S.arrow(ax, (508, y), (540, 180), lw=0.9, color="#9A9A9A")
    for y in (92, 54, 130):
        S.arrow(ax, (508, y), (540, 72), lw=0.9, color="#9A9A9A")
    note(ax, 6, 14, "Each run lands in mission_recordings/run_<timestamp>/ with "
         "sim.log and nav.log alongside.", fs=6.5)
    return S.save(fig, "recording_pipeline")


def fig_plain_how():
    fig, ax = S.diagram(W, 3.0)
    panels = [
        ("1", "A known starting point", "Its start position is\nsurveyed once, like\na marked pillar.",
         "white", lambda cx, cy: ic_flag(ax, cx, cy, 24)),
        ("2", "Feels & sees its motion", "Cameras and a motion\nsensor track every\nturn and metre.",
         "white", None),
        ("3", "Carries its own map", "An elevation map of\nthe site is loaded\nbefore it sets off.",
         "white", lambda cx, cy: ic_map(ax, cx, cy, 26)),
        ("4", "Checks map vs. view", "Matches what it sees\nto satellite maps to\nremove drift. (planned)",
         "design", None),
    ]
    pw, gap = 156, 12
    for i, (num, title, body, kind, draw) in enumerate(panels):
        x = 10 + i * (pw + gap)
        fc, ec, tc, _ = S.KINDS[kind]
        ax.add_patch(FancyBboxPatch((x, 62), pw, 226,
                                    boxstyle="round,pad=0,rounding_size=8",
                                    fc=fc, ec=ec, lw=1.2))
        ax.add_patch(Circle((x + 20, 268), 12, fc=S.NAVY if kind != "design"
                            else S.ORANGE, ec="none"))
        ax.text(x + 20, 268, num, ha="center", va="center", fontsize=9,
                fontweight="bold", color="white")
        cx, cy = x + pw / 2, 212
        if draw:
            draw(cx, cy)
        elif num == "2":
            ic_camera(ax, cx - 28, cy + 4, 20)
            ic_gyro(ax, cx + 36, cy + 4, 18)
        else:
            ax.add_patch(Rectangle((cx - 58, cy - 26), 48, 52, fc="#DDEBCF",
                                   ec="#6E8B4E", lw=1))
            ax.plot([cx - 52, cx - 36, cx - 22, cx - 14], [cy - 16, cy + 6,
                    cy - 2, cy + 18], color="#6E8B4E", lw=1.2)
            ax.add_patch(Rectangle((cx + 10, cy - 26), 48, 52, fc="#E8E1D5",
                                   ec="#8A7B60", lw=1))
            ax.plot([cx + 16, cx + 30, cx + 44, cx + 52], [cy - 16, cy + 6,
                    cy - 2, cy + 18], color="#8A7B60", lw=1.2)
            S.arrow(ax, (cx - 8, cy), (cx + 8, cy), color=S.ORANGE,
                    style="<|-|>")
            check_mark(ax, cx, cy - 40, 8, color=S.GREEN)
        ax.text(x + pw / 2, 150, title, ha="center", va="center", fontsize=8.6,
                fontweight="bold", color=tc if kind == "design" else S.NAVY)
        ax.text(x + pw / 2, 106, body, ha="center", va="center", fontsize=7.4,
                color="#333333", linespacing=1.35)
    ic_satellite(ax, 250, 30, 18)
    forbid(ax, 250, 30, 22)
    ax.text(282, 30, "No GPS or other satellite signal is used — at any stage.",
            ha="left", va="center", fontsize=8.4, fontweight="bold", color=S.RED)
    return S.save(fig, "plain_how_it_knows")


def fig_plain_story():
    fig, ax = S.diagram(W, 2.4)
    steps = [("Load the map", "before the mission", lambda cx, cy: ic_map(ax, cx, cy, 22)),
             ("Type the destination", "as UTM numbers", lambda cx, cy: ic_keyboard(ax, cx, cy, 26)),
             ("Plan the route", "around steep ground", None),
             ("Drive & dodge", "obstacles and people", None),
             ("Arrive & report", "position in map + UTM", lambda cx, cy: ic_flag(ax, cx, cy, 22, S.GREEN))]
    pw, gap = 118, 19
    for i, (t, sub, draw) in enumerate(steps):
        x = 10 + i * (pw + gap)
        ax.add_patch(FancyBboxPatch((x, 30), pw, 196,
                                    boxstyle="round,pad=0,rounding_size=8",
                                    fc=S.LIGHT, ec="#B9C9D9", lw=1.1))
        ax.add_patch(Circle((x + 18, 208), 11, fc=S.NAVY, ec="none"))
        ax.text(x + 18, 208, str(i + 1), ha="center", va="center",
                fontsize=8.6, fontweight="bold", color="white")
        cx, cy = x + pw / 2, 146
        if draw:
            draw(cx, cy)
        elif i == 2:
            ic_map(ax, cx, cy, 24, route=False)
            ax.plot([cx - 18, cx - 6, cx + 6, cx + 18], [cy - 16, cy - 2, cy + 4,
                    cy + 16], color=S.BLUE, lw=2.0, zorder=7)
            ax.add_patch(Circle((cx + 4, cy - 12), 6, fc="#8B6B3E", ec="none",
                                zorder=6))
        else:
            ic_rover(ax, cx - 22, cy - 4, 18)
            ic_person(ax, cx + 30, cy, 20)
            ic_drum(ax, cx + 6, cy + 18, 14)
            S.arrow(ax, (cx - 40, cy - 30), (cx + 36, cy - 30), color=S.BLUE,
                    rad=-0.3)
        ax.text(cx, 84, t, ha="center", va="center", fontsize=8.2,
                fontweight="bold", color=S.NAVY)
        ax.text(cx, 62, sub, ha="center", va="center", fontsize=7.2,
                color="#444444")
        if i < 4:
            S.arrow(ax, (x + pw + 2, 128), (x + pw + gap - 2, 128), lw=1.4,
                    color=S.NAVY)
    ax.text(10, 12, "Step 1 happens once, before the vehicle moves; steps "
            "2–5 run on the vehicle, live, with no network connection.",
            fontsize=7.0, color=S.GREY, style="italic", va="center")
    return S.save(fig, "plain_mission_story")


def fig_plain_needs():
    fig, ax = S.diagram(W, 2.8)
    needs = ["An onboard computer (GPU)", "Cameras (stereo / RGB-D)",
             "A motion sensor (IMU)", "A laser scanner (lidar)",
             "Map files loaded before the mission", "One surveyed start point"]
    nots = ["GPS / GNSS or any satellite signal", "Internet connection",
            "Cloud server", "Mobile / cellular network",
            "A remote driver with a joystick"]
    for x, title, items, good in ((10, "What it needs", needs, True),
                                  (350, "What it does NOT need", nots, False)):
        kind = "impl" if good else "eval"
        fc, ec, tc, _ = S.KINDS[kind]
        ax.add_patch(FancyBboxPatch((x, 10), 320, 262,
                                    boxstyle="round,pad=0,rounding_size=8",
                                    fc=fc, ec=ec, lw=1.2))
        ax.text(x + 160, 250, title, ha="center", va="center", fontsize=9.5,
                fontweight="bold", color=tc)
        for i, it in enumerate(items):
            y = 214 - i * 34
            if good:
                check_mark(ax, x + 28, y, 8)
            else:
                cross_mark(ax, x + 28, y, 7)
            ax.text(x + 50, y, it, ha="left", va="center", fontsize=8.2,
                    color=S.INK)
    return S.save(fig, "plain_needs")


BUILDERS = [fig_context, fig_layers, fig_ros_graph, fig_tf_tree, fig_phases,
            fig_mission_flow, fig_bt, fig_ekf_loop, fig_vslam, fig_mppi_cycle,
            fig_innovations, fig_integrity_fsm, fig_recording, fig_plain_how,
            fig_plain_story, fig_plain_needs]


def build():
    out = {}
    for fn in BUILDERS:
        p = fn()
        out[p.stem] = p
    return out


if __name__ == "__main__":
    import sys
    only = set(sys.argv[1:])
    for fn in BUILDERS:
        if only and fn.__name__ not in only:
            continue
        print(fn())
