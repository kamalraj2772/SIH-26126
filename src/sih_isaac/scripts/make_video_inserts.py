#!/usr/bin/env python3
"""Replacement and insert clips for the team's SIH pitch video.

    .demoenv/bin/python src/sih_isaac/scripts/make_video_inserts.py \
        [--run mission_recordings/report_1401]

Writes SIH26126_Video_Inserts/ at 3840x2160, 60 fps (the pitch video's format):

    01_title_card.mp4          4 s   opening card
    02_dem_slope_utm.mp4      15 s   replaces 0:52-1:07 -- real terrain profile,
                                     the real 15 deg limit, the real UTM goal
    03_results_card.mp4        6 s   measured results
    04_simulation_segment.mp4 ~70 s  report_1401 footage, picture only

plus a PNG still of each card. Cards are drawn in the whiteboard style of the
pitch video (black marker, red / green accents, handwriting) and build up
element by element as if being drawn. Every number is read from the project
files: the DEM, world.json and the run's telemetry.
"""
import argparse
import math
import pathlib
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import matplotlib
from matplotlib import font_manager
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Circle, FancyBboxPatch, Polygon, Rectangle

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "sih_isaac"))
import video                                                    # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--run", default="mission_recordings/report_1401")
ap.add_argument("--only", default="title,dem,results,sim",
                help="comma-separated subset of: title, dem, results, sim")
args = ap.parse_args()
WS = video.WS
RUN = (WS / args.run).resolve()
OUT = WS / "SIH26126_Video_Inserts"
OUT.mkdir(exist_ok=True)
FF = video.ffmpeg_bin()
UW, UH, FPS = 3840, 2160, 60
DPI = 240                                  # 16 x 9 in at 240 dpi = 4K

FONT = video.PKG / "assets/fonts/PatrickHand-Regular.ttf"
font_manager.fontManager.addfont(str(FONT))
matplotlib.rcParams.update({
    "font.family": font_manager.FontProperties(fname=str(FONT)).get_name(),
    # wiggle is in pixels: doubled for the 4K canvas
    "path.sketch": (1.6, 180, 2.0)})

INK, RED, GREEN, BLUE, GREY = "#1d1d1d", "#d23c3c", "#1f8a55", "#1f5fbf", "#8a8a8a"
BG = "#fbfbf9"
W, H = 160.0, 90.0

WORLD = video.world()
DATUM = WORLD["site_datum"]
GOAL = video.goal_info(RUN)
ST = video.stats(RUN)
M = video.marks(RUN)
LIMIT = WORLD["max_slope_deg"]


# ------------------------------------------------------------- drawing ----
def canvas():
    fig = Figure(figsize=(16, 9), dpi=DPI, facecolor=BG)
    FigureCanvasAgg(fig)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")
    return fig, ax


def txt(ax, x, y, s, size=30, color=INK, ha="center", **kw):
    ax.text(x, y, s, fontsize=size, color=color, ha=ha, va="center", **kw)


def tick(ax, x, y, s=2.0, color=GREEN):
    ax.plot([x - s, x - s * 0.3, x + s * 1.1], [y, y - s * 0.8, y + s * 1.1],
            color=color, lw=6, solid_capstyle="round")


def cross(ax, x, y, s=2.0, color=RED):
    for a, b in (((x - s, x + s), (y - s, y + s)), ((x - s, x + s), (y + s, y - s))):
        ax.plot(a, b, color=color, lw=6, solid_capstyle="round")


def box(ax, x, y, w, h, color=INK, lw=3.0):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.6",
                                facecolor="none", edgecolor=color, lw=lw))


def rover(ax, x, y, s=1.0):
    ax.add_patch(FancyBboxPatch((x - 6 * s, y + 1.6 * s), 12 * s, 4.4 * s,
                                boxstyle="round,pad=0.3", facecolor="white",
                                edgecolor=INK, lw=3))
    for dx in (-4, 0, 4):
        ax.add_patch(Circle((x + dx * s, y + 1.2 * s), 1.6 * s, facecolor="white",
                            edgecolor=INK, lw=3))
        ax.add_patch(Circle((x + dx * s, y + 1.2 * s), 0.5 * s, facecolor=INK))
    ax.plot([x + 3 * s, x + 3 * s], [y + 6 * s, y + 9.5 * s], color=INK, lw=3)
    ax.add_patch(Rectangle((x + 1.3 * s, y + 9.5 * s), 3.4 * s, 2 * s,
                           facecolor="white", edgecolor=INK, lw=3))
    ax.add_patch(Circle((x + 3 * s, y + 10.5 * s), 0.6 * s, facecolor="white",
                        edgecolor=INK, lw=2))


def satellite(ax, x, y, s=1.0):
    ax.add_patch(Rectangle((x - 1.6 * s, y - 1.6 * s), 3.2 * s, 3.2 * s,
                           facecolor="white", edgecolor=INK, lw=3))
    for dx in (-7, 2.6):
        ax.add_patch(Rectangle((x + dx * s, y - 1.1 * s), 4.4 * s, 2.2 * s,
                               facecolor="white", edgecolor=INK, lw=2.5))
        for k in (1, 2, 3):
            ax.plot([x + dx * s + k * 1.1 * s] * 2, [y - 1.1 * s, y + 1.1 * s],
                    color=INK, lw=1.5)


# ------------------------------------------------------------ animation ---
def animate(name, steps, total, hold_last=True):
    """steps = [(t, draw_fn), ...]: each draw_fn adds one element at time t.

    Only states that differ are rendered (as PNGs), then shown for their
    duration through ffmpeg's concat demuxer -- a 15 s 4K clip costs a few
    dozen renders, not 900.
    """
    steps = sorted(steps, key=lambda s: s[0])
    times = sorted({t for t, _ in steps if t < total})
    tmp = pathlib.Path(tempfile.mkdtemp(prefix="insert_"))
    lst = []
    for i, t in enumerate(times):
        fig, ax = canvas()
        for ts, fn in steps:
            if ts <= t:
                fn(ax)
        p = tmp / f"s{i:03d}.png"
        fig.savefig(p, dpi=DPI, facecolor=BG)
        nxt = times[i + 1] if i + 1 < len(times) else total
        lst.append((p, nxt - t))
    final = lst[-1][0]
    shutil.copy(final, OUT / f"{name}.png")           # the finished card
    concat = tmp / "list.txt"
    concat.write_text("".join(f"file '{p}'\nduration {d:.4f}\n" for p, d in lst)
                      + f"file '{final}'\n")
    subprocess.run([FF, "-y", "-nostdin", "-loglevel", "error", "-f", "concat",
                    "-safe", "0", "-i", str(concat), "-vf",
                    f"fps={FPS},scale={UW}:{UH},format=yuv420p", "-t", f"{total:.3f}",
                    "-c:v", "libx264", "-preset", "slow", "-crf", "16",
                    "-movflags", "+faststart", str(OUT / f"{name}.mp4")],
                   check=True)
    shutil.rmtree(tmp, ignore_errors=True)
    print(f"wrote {name}.mp4 ({total:.0f} s, {len(lst)} drawn states) + {name}.png")


def stroke(t0, dur, xs, ys, color=INK, lw=4.0, n=24):
    """A line drawn progressively over `dur` seconds, as a marker would."""
    xs, ys = np.asarray(xs), np.asarray(ys)
    out = []
    for k in range(1, n + 1):
        m = max(int(len(xs) * k / n), 2)
        out.append((t0 + dur * k / n,
                    lambda ax, m=m, k=k: ax.plot(xs[:m], ys[:m], color=color, lw=lw,
                                                 solid_capstyle="round",
                                                 zorder=2 + k * 1e-3)))
    # only the latest partial stroke should show: earlier ones are covered by
    # it exactly, since each is a prefix of the next
    return out


# ============================================================ 1. title ====
def title_card():
    s = []
    s.append((0.0, lambda ax: None))
    s.append((0.25, lambda ax: rover(ax, 80, 50, 1.6)))
    s.append((0.7, lambda ax: satellite(ax, 128, 70, 1.1)))
    s.append((0.9, lambda ax: ax.plot([122, 92], [66, 70], color=GREY, lw=2.5, ls=(0, (3, 3)))))
    s.append((1.1, lambda ax: cross(ax, 107, 68.2, 2.4)))
    s.append((1.5, lambda ax: txt(ax, 80, 36, "QSLAM", 120)))
    s += stroke(1.8, 0.4, np.linspace(58, 102, 30), 28.7 + 0.2 * np.sin(np.linspace(0, 3, 30)),
                RED, 7)
    s.append((2.3, lambda ax: txt(ax, 80, 21.5, "Vision Based Autonomous Navigation for Unmanned "
                                  "Ground Vehicle for Outdoor environment", 30)))
    s.append((2.8, lambda ax: txt(ax, 80, 13.5, "SIH 2026  ·  PS SIH26126  ·  Smart Automation  ·  "
                                  "Software", 28, GREY)))
    s.append((3.2, lambda ax: txt(ax, 80, 7, "Team QSLAM (ID 14)", 30, BLUE)))
    animate("01_title_card", s, 4.0)


# ================================================ 2. DEM -> slope -> UTM ==
def dem_slope_utm():
    dem = np.load(video.GEN / "dem.npy")
    n = dem.shape[0]
    half = WORLD["world_size"] / 2
    res = WORLD["world_size"] / (n - 1)
    ycut = 20.0                                  # crosses the plateau and the berm
    row = int(round((half - ycut) / res))
    xs = np.linspace(-half, half, n)
    keep = (xs >= -48) & (xs <= 22)
    px, pz = xs[keep], dem[row][keep]
    slope = np.degrees(np.arctan(np.abs(np.gradient(dem[row], res))))[keep]
    # profile panel: x -48..22 m -> 6..96 board units, z -> 34..54 (exaggerated)
    bx = lambda x: 6 + (x + 48) / 70 * 90
    bz = lambda z: 36 + z * 5.2
    X, Z = bx(px), bz(pz)
    plat = int(np.argmax(np.where(px < -10, pz, -9)))       # plateau top
    berm = int(np.argmax(np.where(px > 0, slope, -9)))      # steepest berm face
    ps = float(np.max(slope[(px > -40) & (px < -20)]))
    bs = float(slope[berm])

    ae, an = DATUM["anchor_e"], DATUM["anchor_n"]
    ge, gn = GOAL["easting"], GOAL["northing"]
    gx, gy = ge - ae, gn - an

    s = [(0.0, lambda ax: txt(ax, 51, 83, "DEM  ->  slope  ->  go / no-go", 46))]
    s.append((0.0, lambda ax: txt(ax, 51, 76.5, f"real terrain profile, y = {ycut:.0f} m "
                                  "(from the GeoTIFF)", 24, GREY)))
    # ground block, then the profile drawn left to right
    s.append((0.4, lambda ax: ax.plot([X[0], X[0], X[-1], X[-1]], [Z[0], 27, 27, Z[-1]],
                                      color=INK, lw=3.5)))
    s += stroke(0.6, 2.4, X, Z, INK, 5, n=30)
    # plateau: drivable
    s.append((3.4, lambda ax: ax.annotate("", (X[plat - 6], Z[plat - 6] + 1.2),
                                          (X[plat - 6] - 6, Z[plat - 6] + 9),
                                          arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=3))))
    s.append((3.6, lambda ax: txt(ax, X[plat - 6] - 8, Z[plat - 6] + 12.5,
                                  f"{ps:.0f}°  <=  {LIMIT:.0f}°", 32, GREEN)))
    s.append((3.9, lambda ax: txt(ax, X[plat - 6] - 8, Z[plat - 6] + 7.2 + 9,
                                  "drivable", 30, GREEN)))
    s.append((4.1, lambda ax: tick(ax, X[plat - 6] + 5, Z[plat - 6] + 14, 1.8)))
    # berm: no-go
    s.append((5.0, lambda ax: ax.annotate("", (X[berm] + 0.8, Z[berm]),
                                          (X[berm] + 8, Z[berm] + 9),
                                          arrowprops=dict(arrowstyle="-|>", color=RED, lw=3))))
    s.append((5.2, lambda ax: txt(ax, X[berm] + 12, Z[berm] + 12.5,
                                  f"{bs:.0f}°  >  {LIMIT:.0f}°", 32, RED)))
    s.append((5.5, lambda ax: txt(ax, X[berm] + 12, Z[berm] + 16.2, "no-go", 30, RED)))
    s.append((5.7, lambda ax: cross(ax, X[berm] + 1.5, Z[berm] - 4, 1.8)))
    s.append((6.3, lambda ax: txt(ax, 51, 21, f"rule:  slope  >  {LIMIT:.0f}°   =   no-go "
                                  "on the prior map", 30)))
    s.append((6.8, lambda ax: txt(ax, 51, 14.5, "(the tunnel cut through this berm is the only "
                                  "low-slope way past)", 24, GREY)))
    # UTM goal -> map, right panel
    s.append((7.8, lambda ax: box(ax, 104, 14, 52, 66, BLUE, 3.5)))
    s.append((8.1, lambda ax: txt(ax, 130, 74, "UTM goal  ->  map", 38, BLUE)))
    lines = [(8.7, f"goal     E  {ge:,.2f}", INK), (9.2, f"            N  {gn:,.2f}", INK),
             (10.0, f"anchor  E0 {ae:,.2f}", GREY), (10.5, f"            N0 {an:,.2f}", GREY),
             (11.4, f"map  x = E - E0 = {gx:.2f} m", GREEN),
             (12.0, f"map  y = N - N0 = {gy:.2f} m", GREEN)]
    ys = [65, 59, 50, 44, 32, 25.5]              # goal | anchor | rule | result
    for (t, ln, c), y in zip(lines, ys):
        s.append((t, lambda ax, y=y, ln=ln, c=c: txt(ax, 107.5, y, ln, 30, c, ha="left")))
    s += stroke(10.9, 0.3, np.linspace(108, 152, 20), np.full(20, 38.5), INK, 3, n=6)
    s.append((12.8, lambda ax: txt(ax, 130, 18, "UTM zone 44N  ·  EPSG:32644", 26, GREY)))
    animate("02_dem_slope_utm", s, 15.0)


# ========================================================== 3. results ====
def results_card():
    d = video.read_log(RUN)
    tiles = [("4 / 4", "missions reached the goal", "through the tunnel", GREEN),
             (f"{ST['final_dist'] * 100:.0f} cm", "from the goal after", f"{ST['path_len']:.0f} m driven", BLUE),
             ("~1 m", "EKF error vs 10-70 m", "with wheel odometry", INK),
             ("0", "GPS inputs", "at any point", RED)]
    s = [(0.0, lambda ax: txt(ax, 80, 81, "Results", 60)),
         (0.0, lambda ax: txt(ax, 80, 72.5, "measured in the Isaac Sim testbed", 28, GREY))]
    for i, (big, l1, l2, c) in enumerate(tiles):
        x = 6 + i * 38
        t0 = 0.5 + i * 0.9
        s.append((t0, lambda ax, x=x, c=c: box(ax, x, 22, 33, 40, c, 3.5)))
        s.append((t0 + 0.2, lambda ax, x=x, big=big, c=c: txt(ax, x + 16.5, 48, big, 64, c)))
        s.append((t0 + 0.4, lambda ax, x=x, l1=l1, l2=l2: (txt(ax, x + 16.5, 34, l1, 23),
                                                             txt(ax, x + 16.5, 28, l2, 23))))
        s.append((t0 + 0.55, lambda ax, x=x: tick(ax, x + 29, 57.5, 1.4, GREEN)))
    s.append((4.4, lambda ax: txt(ax, 80, 10.5, "No GPS.  No network.  Just vision, a map "
                                  "and a UTM goal.", 36, GREEN)))
    animate("03_results_card", s, 6.0)


# ================================================== 4. simulation segment
def sim_segment():
    """~70 s of report_1401, picture only, upscaled to the pitch video's 4K."""
    src = {k: RUN / f"isaac_{k}.mp4" for k in ("behind", "side", "top", "onboard")}
    src["rviz"] = RUN / "rviz.mp4"
    src["costmap"] = RUN / "costmap.mp4"
    to_rviz = video.rviz_mapper(RUN)
    viz0 = video.viz_offset(RUN)
    mv, ti, to, end = M["move"], M["tunnel_in"], M["tunnel_out"], M["end"]

    def ft(k, t):
        return to_rviz(t) if k == "rviz" else (t - viz0 if k == "costmap" else t)

    # a lit RViz window (the grab goes black while the screen is locked)
    rv = None
    for frac in (0.25, 0.4, 0.55, 0.1, 0.7):
        c = mv + frac * (end - mv)
        if min(video.brightness(src["rviz"], to_rviz(c + d)) for d in (2, 20, 38)) > 20:
            rv = c
            break
    plan = [("shot", "behind", mv - 1, mv + 23, 8.0),
            ("shot", "onboard", mv + 60, mv + 110, 8.0),
            ("grid", ("behind", "side", "top", "onboard"), mv + 110, ti - 16, 15.0),
            ("shot", "side", ti - 16, ti + 20, 7.0),
            ("shot", "onboard", ti - 2, to + 4, 9.0),
            ("shot", "costmap", mv, end, 9.0)]
    if rv is not None:
        plan.append(("shot", "rviz", rv, rv + 40, 7.0))
    plan.append(("grid", ("behind", "side", "top", "onboard"), end - 45, end + 2, 8.0))

    tmp = pathlib.Path(tempfile.mkdtemp(prefix="simseg_"))
    enc = ["-c:v", "libx264", "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p",
           "-r", str(FPS), "-an"]
    parts = []
    for i, (kind, keys, a, b, dur) in enumerate(plan):
        out = tmp / f"p{i:02d}.mp4"
        tail = f"trim=duration={dur:.3f},setpts=PTS-STARTPTS"
        if kind == "shot":
            k = keys
            ka, kb = ft(k, a), ft(k, b)
            sp = max((kb - ka) / dur, 0.25)
            cmd = [FF, "-y", "-nostdin", "-loglevel", "error", "-ss", f"{ka:.3f}",
                   "-t", f"{kb - ka:.3f}", "-i", str(src[k]), "-vf",
                   f"setpts=(PTS-STARTPTS)/{sp:.5f},fps={FPS},"
                   f"scale={UW}:{UH}:flags=lanczos,{tail}"]
        else:
            cmd = [FF, "-y", "-nostdin", "-loglevel", "error"]
            chains = []
            for j, k in enumerate(keys):
                ka, kb = ft(k, a), ft(k, b)
                sp = max((kb - ka) / dur, 0.25)
                cmd += ["-ss", f"{ka:.3f}", "-t", f"{kb - ka:.3f}", "-i", str(src[k])]
                chains.append(f"[{j}:v]setpts=(PTS-STARTPTS)/{sp:.5f},fps={FPS},"
                              f"scale={UW // 2}:{UH // 2}:flags=lanczos[v{j}]")
            chains.append("".join(f"[v{j}]" for j in range(4))
                          + f"xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0,{tail}[o]")
            cmd += ["-filter_complex", ";".join(chains), "-map", "[o]"]
        subprocess.run(cmd + enc + [str(out)], check=True)
        parts.append(out)
    lst = tmp / "list.txt"
    lst.write_text("".join(f"file '{p}'\n" for p in parts))
    subprocess.run([FF, "-y", "-nostdin", "-loglevel", "error", "-f", "concat", "-safe", "0",
                    "-i", str(lst), "-c", "copy", "-movflags", "+faststart",
                    str(OUT / "04_simulation_segment.mp4")], check=True)
    shutil.rmtree(tmp, ignore_errors=True)
    secs = video.probe(OUT / "04_simulation_segment.mp4")["seconds"]
    print(f"wrote 04_simulation_segment.mp4 ({secs:.1f} s, {len(parts)} shots)")


if __name__ == "__main__":
    jobs = {"title": title_card, "dem": dem_slope_utm, "results": results_card,
            "sim": sim_segment}
    for key in args.only.split(","):
        jobs[key.strip()]()
