"""Cut a short 1080p highlight reel from a recorded mission run.

    .demoenv/bin/python src/sih_isaac/scripts/make_highlight.py <run_dir> \
        [--ffmpeg PATH] [--out FILE]

Segment boundaries are derived from the run's own telemetry (first motion,
berm/tunnel crossing, arrival), so the cuts land on real mission events rather
than fixed timestamps. Output: 1920x1080, 30 fps, ~90 s, silent.
"""
import argparse
import json
import pathlib
import subprocess
import tempfile

import numpy as np
from PIL import Image, ImageDraw, ImageFont

WS = pathlib.Path("/home/qbotix-rover/sih_ws")
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_R = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
NAVY = (16, 38, 62)
WHITE = (255, 255, 255)
SKY = (120, 176, 230)
GOLD = (232, 176, 96)
GREENC = (140, 200, 150)

ap = argparse.ArgumentParser()
ap.add_argument("run_dir")
ap.add_argument("--ffmpeg", default="ffmpeg")
ap.add_argument("--out", default=None)
args = ap.parse_args()
RUN = pathlib.Path(args.run_dir).resolve()
OUT = pathlib.Path(args.out) if args.out else RUN / "QSLAM_highlight.mp4"
FF = args.ffmpeg
W, H, FPS = 1920, 1080, 30

VIEWS = {k: RUN / f"isaac_{k}.mp4" for k in ("behind", "side", "top")}
VIEWS["rviz"] = RUN / "rviz.mp4"
for k, v in VIEWS.items():
    if not v.exists():
        raise SystemExit(f"missing {v}")

# ----------------------------------------------------- timings from telemetry
rows = [[float(x) for x in l.split(",")]
        for l in (RUN / "mission_log.csv").read_text().splitlines()[1:]
        if len(l.split(",")) == 8]
d = np.array(rows)
t, gx, gy, cv = d[:, 0], d[:, 1], d[:, 2], d[:, 6]
t_move = float(t[np.abs(cv) > 0.05][0])
berm = json.loads((WS / "src/sih_isaac/generated/world.json").read_text())["berm"]
ent = np.where(gx >= berm["x0"])[0]
ext = np.where(gx >= berm["x1"])[0]
t_tun_in = float(t[ent[0]]) if len(ent) else t_move + 300
t_tun_out = float(t[ext[0]]) if len(ext) else t_tun_in + 200
t_end = float(t[-1])

dist0 = float(np.hypot(gx[0] - 56.0, gy[0] - 34.0))
path_len = float(np.sum(np.hypot(np.diff(gx), np.diff(gy))))
loc_err = float(np.hypot(gx[-1] - d[-1, 4], gy[-1] - d[-1, 5]))
print(f"[reel] move {t_move:.0f}s  tunnel {t_tun_in:.0f}-{t_tun_out:.0f}s  "
      f"end {t_end:.0f}s")

tmp = pathlib.Path(tempfile.mkdtemp(prefix="reel_"))


# ------------------------------------------------------------------- cards --
def card(path, lines, bg=NAVY):
    img = Image.new("RGB", (W, H), bg)
    dr = ImageDraw.Draw(img)
    total = sum(sz + gap for _, sz, _, gap in lines)
    y = (H - total) // 2
    for text, size, col, gap in lines:
        f = ImageFont.truetype(FONT_B if size >= 54 else FONT_R, size)
        w = dr.textbbox((0, 0), text, font=f)[2]
        dr.text(((W - w) // 2, y), text, font=f, fill=col)
        y += size + gap
    img.save(path)
    return path


title = card(tmp / "title.png", [
    ("QSLAM", 132, WHITE, 26),
    ("GPS-denied autonomous ground navigation", 54, SKY, 54),
    ("Isaac Sim integration testbed  ·  navigation backbone", 38,
     (170, 190, 210), 16),
    ("SIH 2026  ·  PS SIH26126  ·  Team QSLAM (ID 14)", 34,
     (150, 170, 195), 0),
])
endc = card(tmp / "end.png", [
    ("MISSION SUCCESSFUL", 92, GREENC, 40),
    (f"{path_len:.0f} m driven   ·   {loc_err:.2f} m localization drift",
     58, WHITE, 22),
    ("no GNSS receiver used at any point", 44, GOLD, 52),
    ("Isaac Sim 6  ·  ROS 2 Jazzy  ·  Nav2 (Smac A* + MPPI)  "
     "·  EKF", 34, (170, 190, 210), 0),
])


PANEL_ORIGINS = [(0, 0), (W // 2, 0), (0, H // 2), (W // 2, H // 2)]


def overlay_png(name, caption, panels=None):
    """Caption bar (+ optional quadrant labels) as one RGBA layer.

    This build of ffmpeg has no drawtext filter (FFmpeg 7 needs harfbuzz),
    so text is rendered here and composited with the overlay filter.
    """
    img = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    dr = ImageDraw.Draw(img)

    def label(xy, text, size, pad, anchor_centre=False):
        f = ImageFont.truetype(FONT_B, size)
        l, t_, r, b = dr.textbbox((0, 0), text, font=f)
        tw, th = r - l, b - t_
        x, y = xy
        if anchor_centre:
            x -= tw // 2
        dr.rounded_rectangle([x - pad, y - pad, x + tw + pad, y + th + pad * 2],
                             radius=10, fill=(13, 31, 51, 196))
        dr.text((x - l, y - t_ + pad // 2), text, font=f, fill=WHITE)

    if panels:
        for (ox, oy), text in zip(PANEL_ORIGINS, panels):
            label((ox + 26, oy + 22), text, 28, 11)
    label((W // 2, H - 152), caption, 46, 20, anchor_centre=True)
    p = tmp / f"{name}_ovl.png"
    img.save(p)
    return p


ENC = ["-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
       "-pix_fmt", "yuv420p", "-r", str(FPS), "-an"]
segs = []


def run(cmd, out):
    r = subprocess.run(cmd + ENC + [str(out)], stdout=subprocess.DEVNULL,
                       stderr=subprocess.PIPE, text=True)
    if r.returncode:
        raise SystemExit(f"ffmpeg failed for {out.name}:\n{r.stderr[-1500:]}")
    segs.append(out)


def still(png, dur, out):
    run([FF, "-y", "-loglevel", "error", "-loop", "1", "-t", str(dur),
         "-i", str(png), "-vf", f"scale={W}:{H},fps={FPS},format=yuv420p"], out)


def single(view, a, b, speed, caption, out):
    dur = max(b - a, 0.5)
    ovl = overlay_png(out.stem, caption)
    run([FF, "-y", "-loglevel", "error", "-ss", f"{a:.2f}", "-t", f"{dur:.2f}",
         "-i", str(VIEWS[view]), "-loop", "1", "-i", str(ovl),
         "-filter_complex",
         f"[0:v]setpts=PTS/{speed},scale={W}:{H},fps={FPS}[b];"
         f"[b][1:v]overlay=0:0:shortest=1,format=yuv420p[out]",
         "-map", "[out]"], out)


def quad(a, b, speed, caption, out):
    dur = max(b - a, 0.5)
    order = [("behind", "behind the rover"), ("side", "side angle"),
             ("top", "top-down"),
             ("rviz", "RViz — what the rover believes")]
    ovl = overlay_png(out.stem, caption, panels=[lab for _, lab in order])
    cmd = [FF, "-y", "-loglevel", "error"]
    for key, _ in order:
        cmd += ["-ss", f"{a:.2f}", "-t", f"{dur:.2f}", "-i", str(VIEWS[key])]
    cmd += ["-loop", "1", "-i", str(ovl)]
    parts = [f"[{i}:v]setpts=PTS/{speed},scale={W//2}:{H//2}[v{i}]"
             for i in range(4)]
    parts.append("[v0][v1][v2][v3]xstack=inputs=4:"
                 "layout=0_0|w0_0|0_h0|w0_h0[g]")
    parts.append(f"[g]fps={FPS}[gg];"
                 f"[gg][4:v]overlay=0:0:shortest=1,format=yuv420p[out]")
    cmd += ["-filter_complex", ";".join(parts), "-map", "[out]"]
    run(cmd, out)


still(title, 3.5, tmp / "s0.mp4")
single("behind", t_move + 1, t_move + 21, 2.0,
       "Mission start — one UTM grid coordinate is the only input",
       tmp / "s1.mp4")
quad(t_move + 26, t_tun_in - 8, 30.0,
     "Open traverse — planned route, live lidar avoidance, GPS-free pose",
     tmp / "s2.mp4")
single("side", t_tun_in - 18, t_tun_in + 52, 5.0,
       "Reaching the berm — the short way through is the tunnel",
       tmp / "s3.mp4")
single("behind", t_tun_in + 52, t_tun_out, 20.0,
       "Through the tunnel — no satellite signal, lidar and inertial only",
       tmp / "s4.mp4")
quad(t_end - 72, t_end, 4.0,
     "Final approach — goal reached", tmp / "s5.mp4")
still(endc, 5.0, tmp / "s6.mp4")

lst = tmp / "list.txt"
lst.write_text("".join(f"file '{p}'\n" for p in segs))
subprocess.run([FF, "-y", "-loglevel", "error", "-f", "concat", "-safe", "0",
                "-i", str(lst), "-c", "copy", str(OUT)], check=True)
print(f"wrote {OUT}")
