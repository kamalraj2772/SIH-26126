#!/usr/bin/env python3
"""Professional whiteboard-animation inserts for the SIH pitch video.

    .demoenv/bin/python src/sih_isaac/scripts/make_wb_inserts.py \
        [--only title,dem,results,sim] [--run mission_recordings/report_1401]

3840x2160, 60 fps, into SIH26126_Video_Inserts/. A real hand (the one from
the team's own video) draws every line and writes every word, in clean black
marker with red / green / blue accents -- no wobble, no pop-ins. Numbers come
from the project files (DEM, world.json, the run's telemetry).
"""
import argparse
import math
import pathlib
import shutil
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "sih_isaac"))
import video                                                    # noqa: E402
import whiteboard as wb                                         # noqa: E402
from whiteboard import BLUE, GREEN, GREY, INK, RED, Scene       # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--run", default="mission_recordings/report_1401")
ap.add_argument("--only", default="title,dem,results,sim")
args = ap.parse_args()
RUN = (video.WS / args.run).resolve()
OUT = video.WS / "SIH26126_Video_Inserts"
OUT.mkdir(exist_ok=True)
FF = video.ffmpeg_bin()
WORLD = video.world()
DATUM = WORLD["site_datum"]
GOAL = video.goal_info(RUN)
ST = video.stats(RUN)
M = video.marks(RUN)
LIMIT = WORLD["max_slope_deg"]


def save_still(frame, name):
    Image.fromarray(frame).save(OUT / f"{name}.png")


# --------------------------------------------------------------- drawings --
def rover(sc, cx, base, s=1.0, color=INK):
    """Clean side-view rover line art (body, panel, mast, camera, 3 wheels)."""
    S = lambda x, y: (cx + x * s, base + y * s)
    w = 14
    sc.line(S(-520, 0), S(520, 0), color=GREY, width=8)                  # ground
    sc.rect(*S(-300, -360), *S(300, -176), r=28 * s, color=color, width=w)
    sc.rect(*S(-240, -320), *S(90, -218), r=10 * s, color=color, width=10)
    for x in (-160, -80, 0):                                             # vents
        sc.line(S(x, -305), S(x, -233), color=color, width=7, speed=4200)
    sc.line(S(170, -360), S(170, -490), color=color, width=w)            # mast
    sc.rect(*S(100, -565), *S(245, -490), r=10 * s, color=color, width=w)
    sc.circle(S(172, -527), 20 * s, color=color, width=10)               # lens
    sc.line(S(-240, -360), S(-240, -455), color=color, width=10)         # antenna
    sc.circle(S(-240, -470), 14 * s, color=color, width=9)
    for x in (-205, 0, 205):                                             # wheels
        sc.circle(S(x, -88), 86 * s, color=color, width=w)
        sc.circle(S(x, -88), 34 * s, color=color, width=10)


def satellite(sc, cx, cy, s=1.0, color=INK):
    S = lambda x, y: (cx + x * s, cy + y * s)
    sc.rect(*S(-45, -45), *S(45, 45), color=color, width=11)
    for x0, x1 in ((-205, -60), (60, 205)):
        sc.rect(*S(x0, -35), *S(x1, 35), color=color, width=10)
        for k in (1, 2):
            x = x0 + (x1 - x0) * k / 3
            sc.line(S(x, -35), S(x, 35), color=color, width=6, speed=5000)


# ================================================================ title ====
def title_card():
    sc = Scene(speed=4600, text_speed=3600, travel_speed=7000)
    rover(sc, 1560, 1110, 1.3)
    satellite(sc, 2900, 380, 1.1)
    sc.stroke([(2700, 460), (2540, 540), (2380, 620), (2230, 700)], color=GREY, width=9)
    sc.cross((2460, 580), s=52)
    sc.text("QSLAM", (1920, 1400), size=300)
    sc.stroke([(1540, 1562), (1920, 1572), (2300, 1560)], color=RED, width=16)
    sc.text("Vision Based Autonomous Navigation for Unmanned Ground Vehicle "
            "for Outdoor environment", (1920, 1730), size=86, text_speed=8000)
    sc.text("SIH 2026  ·  SIH26126  ·  Smart Automation  ·  Software",
            (1920, 1878), size=72, color=GREY, text_speed=8000)
    sc.text("Team QSLAM (ID 14)", (1920, 2010), size=82, color=BLUE, text_speed=4500)
    last = sc.render(OUT / "01_title_card.mp4", FF, hold=0.8)
    save_still(last, "01_title_card")
    print(f"wrote 01_title_card.mp4 ({sc.t + 1.25:.1f} s)")


def fit(build, target, hold):
    """Build a scene whose total length (draw + exit + hold) is `target` s,
    by scaling every drawing speed; travel minimums keep it non-linear, so
    iterate."""
    f = 1.0
    for _ in range(6):
        sc = build(f)
        total = sc.t + 0.45 + hold
        if abs(total - target) < 0.04:
            break
        f *= (total - 1.2) / max(target - 1.2, 0.5)
    return sc


def hdr_arrow(sc, x, y, color=INK, n=90):
    """A short drawn arrow used between words (the font has no arrow)."""
    sc.arrow((x - n / 2, y), (x + n / 2, y), head=30, color=color, width=11)


# ===================================================== DEM -> slope -> UTM
def dem_slope_utm():
    dem = np.load(video.GEN / "dem.npy")
    n = dem.shape[0]
    half = WORLD["world_size"] / 2
    res = WORLD["world_size"] / (n - 1)
    ycut = 20.0
    row = int(round((half - ycut) / res))
    xs = np.linspace(-half, half, n)
    keep = (xs >= -48) & (xs <= 22)
    px, pz = xs[keep], dem[row][keep]
    slope = np.degrees(np.arctan(np.abs(np.gradient(dem[row], res))))[keep]
    X = 230 + (px + 48) / 70 * 2080                 # 230 .. 2310 px
    Z = 1230 - pz * 150                             # 3.2 m berm -> 480 px
    profile = np.column_stack([X, Z])
    plat = int(np.argmax(np.where(px < -10, slope, -1)))      # steepest plateau side
    berm = int(np.argmax(np.where(px > 0, slope, -1)))        # steepest berm face
    ps, bs = float(slope[plat]), float(slope[berm])
    ae, an = DATUM["anchor_e"], DATUM["anchor_n"]
    ge, gn = GOAL["easting"], GOAL["northing"]
    gx, gy = ge - ae, gn - an
    dx, dy = 80, -62                                 # block depth

    def build(f):
        sc = Scene(speed=3600 * f, text_speed=3800 * f, travel_speed=7000 * f)
        # heading
        sc.text("DEM", (520, 200), size=150, anchor="lm")
        hdr_arrow(sc, 930, 205)
        sc.text("SLOPE", (1030, 200), size=150, anchor="lm")
        hdr_arrow(sc, 1560, 205)
        sc.text("GO / NO-GO", (1660, 200), size=150, anchor="lm")
        sc.text("real terrain profile, read from the GeoTIFF", (1270, 330),
                size=68, color=GREY)
        # terrain block: front face, then depth
        sc.stroke(profile, width=13)
        sc.poly([(X[-1], Z[-1]), (X[-1], 1420), (X[0], 1420), (X[0], Z[0])], width=13)
        # back edge, with the parts hidden behind the front face left out
        back = profile + (dx, dy)
        front_top = np.interp(back[:, 0], X, Z, right=np.inf)
        vis = (back[:, 1] < front_top - 6) | (back[:, 0] > X[-1])
        runs, cur = [], []
        for pt, v in zip(back, vis):
            if v:
                cur.append(pt)
            elif cur:
                runs.append(cur); cur = []
        if cur:
            runs.append(cur)
        for r in runs:
            if len(r) > 3:
                sc.stroke(np.array(r), color=GREY, width=7, speed=5200 * f)
        sc.line((X[0], Z[0]), (X[0] + dx, Z[0] + dy), color=GREY, width=7)
        sc.line((X[-1], Z[-1]), (X[-1] + dx, Z[-1] + dy), color=GREY, width=7)
        sc.line((X[-1], 1420), (X[-1] + dx, 1420 + dy), color=GREY, width=7)
        sc.line((X[-1] + dx, 1420 + dy), (X[-1] + dx, Z[-1] + dy), color=GREY, width=7)
        # plateau: drivable
        pxp, pzp = X[plat], Z[plat]
        sc.arrow((pxp - 170, pzp - 300), (pxp - 8, pzp - 16), head=34, color=GREEN, width=11)
        sc.text(f"{ps:.0f}° ≤ {LIMIT:.0f}°", (pxp - 190, pzp - 440), size=104,
                color=GREEN)
        sc.text("DRIVABLE", (pxp - 190, pzp - 350), size=78, color=GREEN)
        sc.tick((pxp + 130, pzp - 430), s=46)
        # berm: no-go
        bxp, bzp = X[berm], Z[berm]
        sc.arrow((bxp + 260, bzp - 330), (bxp + 12, bzp - 12), head=34, color=RED, width=11)
        sc.text(f"{bs:.0f}° > {LIMIT:.0f}°", (bxp + 330, bzp - 470), size=104, color=RED)
        sc.text("NO-GO", (bxp + 330, bzp - 380), size=78, color=RED)
        sc.cross((bxp - 55, bzp + 80), s=40)
        # the rule
        sc.text(f"rule:  slope  >  {LIMIT:.0f}°   =   no-go on the prior map",
                (1270, 1620), size=92)
        sc.text("the tunnel through this berm is the only low-slope way past",
                (1270, 1750), size=66, color=GREY, text_speed=6500 * f)
        # UTM -> map panel
        sc.rect(2600, 330, 3700, 1890, r=34, color=BLUE, width=12)
        sc.text("UTM GOAL", (2880, 470), size=104, color=BLUE)
        hdr_arrow(sc, 3200, 475, BLUE, 110)
        sc.text("MAP", (3440, 470), size=104, color=BLUE)
        L = 2690
        sc.text(f"goal     E  {ge:,.2f}", (L, 680), size=88, anchor="lm")
        sc.text(f"           N  {gn:,.2f}", (L, 800), size=88, anchor="lm")
        sc.text(f"anchor  E0  {ae:,.2f}", (L, 980), size=80, color=GREY, anchor="lm")
        sc.text(f"             N0  {an:,.2f}", (L, 1095), size=80, color=GREY, anchor="lm")
        sc.line((2690, 1200), (3610, 1200), width=10)
        sc.text(f"x = E − E0 = {gx:.2f} m", (L, 1330), size=92, color=GREEN, anchor="lm")
        sc.text(f"y = N − N0 = {gy:.2f} m", (L, 1460), size=92, color=GREEN, anchor="lm")
        sc.text("UTM zone 44N  ·  EPSG:32644", (3150, 1760), size=62, color=GREY)
        return sc

    sc = fit(build, 15.0, hold=0.7)
    last = sc.render(OUT / "02_dem_slope_utm.mp4", FF, hold=0.7)
    save_still(last, "02_dem_slope_utm")
    print(f"wrote 02_dem_slope_utm.mp4 ({sc.t + 1.15:.2f} s)")


# ================================================== simulation segment ====
FX0, FY0, FW, FH = 336, 190, 3168, 1782          # footage window (16:9)


class Footage:
    """Raw frames of one shot, decoded by ffmpeg at the window size."""

    def __init__(self, cmd, n):
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL)
        self.n, self.i, self.last = n, 0, None

    def next(self):
        if self.i < self.n:
            buf = self.proc.stdout.read(FW * FH * 3)
            if len(buf) == FW * FH * 3:
                self.last = np.frombuffer(buf, np.uint8).reshape(FH, FW, 3)
            self.i += 1
        return self.last

    def close(self):
        self.proc.stdout.close()
        self.proc.kill()


def sim_segment():
    src = {k: RUN / f"isaac_{k}.mp4" for k in ("behind", "side", "top", "onboard")}
    src["rviz"] = RUN / "rviz.mp4"
    src["costmap"] = RUN / "costmap.mp4"
    to_rviz = video.rviz_mapper(RUN)
    viz0 = video.viz_offset(RUN)
    mv, ti, to, end = M["move"], M["tunnel_in"], M["tunnel_out"], M["end"]

    def ft(k, t):
        return to_rviz(t) if k == "rviz" else (t - viz0 if k == "costmap" else t)

    rv = None                                   # a lit RViz window
    for frac in (0.25, 0.4, 0.55, 0.1, 0.7):
        c = mv + frac * (end - mv)
        if min(video.brightness(src["rviz"], to_rviz(c + d)) for d in (2, 20, 38)) > 20:
            rv = c
            break
    G4 = ("behind", "side", "top", "onboard")
    plan = [("behind", mv - 1, mv + 23, 8.0), ("onboard", mv + 60, mv + 110, 8.0),
            (G4, mv + 110, ti - 16, 15.0), ("side", ti - 16, ti + 20, 7.0),
            ("onboard", ti - 2, to + 4, 9.0), ("costmap", mv, end, 9.0)]
    if rv is not None:
        plan.append(("rviz", rv, rv + 40, 7.0))
    plan.append((G4, end - 45, end + 2, 8.0))

    TRANS = 0.5
    out = subprocess.Popen(
        [FF, "-y", "-nostdin", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{wb.W}x{wb.H}", "-framerate", str(wb.FPS), "-i", "-",
         "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p",
         "-movflags", "+faststart", str(OUT / "04_simulation_segment.mp4")],
        stdin=subprocess.PIPE)
    blank = np.empty((wb.H, wb.W, 3), np.uint8)
    blank[:] = wb.BOARD
    prev = None
    nframes = 0
    for idx, (keys, a, b, D) in enumerate(plan):
        # the hand draws the frame (and grid lines for the 4-camera view)
        sc = Scene(speed=11000, travel_speed=9000)
        sc.rect(FX0 - 16, FY0 - 16, FX0 + FW + 16, FY0 + FH + 16, r=26, width=13)
        grid = isinstance(keys, tuple)
        if grid:
            sc.line((FX0 + FW / 2, FY0 - 16), (FX0 + FW / 2, FY0 + FH + 16), width=11)
            sc.line((FX0 - 16, FY0 + FH / 2), (FX0 + FW + 16, FY0 + FH / 2), width=11)
        trans = TRANS if prev is not None else 0.0
        scene_len = D - trans
        t_play = sc.t + 0.05
        play = scene_len - t_play
        n_play = int(round(play * wb.FPS)) + 2
        # footage for the window, sped to fit its play time
        base = [FF, "-nostdin", "-loglevel", "error"]
        if not grid:
            ka, kb = ft(keys, a), ft(keys, b)
            sp = max((kb - ka) / play, 0.25)
            cmd = base + ["-ss", f"{ka:.3f}", "-t", f"{kb - ka:.3f}", "-i", str(src[keys]),
                          "-vf", f"setpts=(PTS-STARTPTS)/{sp:.5f},fps={wb.FPS},"
                                 f"scale={FW}:{FH}:flags=lanczos",
                          "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
        else:
            cmd, chains = list(base), []
            hw, hh = FW // 2, FH // 2
            for j, k in enumerate(keys):
                ka, kb = ft(k, a), ft(k, b)
                sp = max((kb - ka) / play, 0.25)
                cmd += ["-ss", f"{ka:.3f}", "-t", f"{kb - ka:.3f}", "-i", str(src[k])]
                chains.append(f"[{j}:v]setpts=(PTS-STARTPTS)/{sp:.5f},fps={wb.FPS},"
                              f"scale={hw}:{hh}:flags=lanczos[v{j}]")
            chains.append("[v0][v1][v2][v3]xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0[o]")
            cmd += ["-filter_complex", ";".join(chains), "-map", "[o]",
                    "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
        foot = Footage(cmd, n_play)
        FADE = 0.35

        def extra(canvas, t, foot=foot, t_play=t_play, grid=grid):
            if t < t_play:
                return
            fr = foot.next()
            if fr is None:
                return
            k = min((t - t_play) / FADE, 1.0)
            reg = canvas[FY0:FY0 + FH, FX0:FX0 + FW]
            reg *= (1 - k)
            reg += k * (fr.astype(np.float32) / 255)
            if grid:                           # keep the drawn dividers on top
                c = np.array(wb.INK, np.float32) / 255
                x = FX0 + FW // 2
                y = FY0 + FH // 2
                canvas[FY0:FY0 + FH, x - 6:x + 6] = c
                canvas[y - 6:y + 6, FX0:FX0 + FW] = c

        # slide the previous board out to the left
        if prev is not None:
            nt = int(round(trans * wb.FPS))
            for i in range(nt):
                u = wb.ease((i + 1) / nt)
                sh = int(round(u * wb.W))
                fr = np.empty_like(prev)
                fr[:, :wb.W - sh] = prev[:, sh:]
                fr[:, wb.W - sh:] = blank[:, :sh]
                out.stdin.write(fr.tobytes())
                nframes += 1
        n_scene = int(round(scene_len * wb.FPS))
        last = None
        for i, fr in enumerate(sc.frames(hold=100.0, extra=extra)):
            if i >= n_scene:
                break
            last = (np.clip(fr, 0, 1) * 255).astype(np.uint8)
            out.stdin.write(last.tobytes())
            nframes += 1
        foot.close()
        prev = last
        print(f"[sim] shot {idx + 1}/{len(plan)}: {keys if not grid else '4 cameras'} "
              f"{D:.1f} s", flush=True)
    out.stdin.close()
    out.wait()
    print(f"wrote 04_simulation_segment.mp4 ({nframes / wb.FPS:.1f} s)")


if __name__ == "__main__":
    jobs = {"title": title_card, "dem": dem_slope_utm, "sim": sim_segment}
    for k in args.only.split(","):
        k = k.strip()
        if k in jobs:
            jobs[k]()
