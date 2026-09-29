"""Shared look, plumbing and mission bookkeeping for every SIH26126 video.

The videos carry NO words: no titles, captions, labels, legends or text cards
-- the operator asked for pure picture. Drawn videos (costmaps, EKF, SLAM and
graph, the GeoTIFF->DEM->UTM sequence) are matplotlib figures passed through
strip_words(), which hides every text element and keeps only the pictures and
the numeric tick marks that give them scale. The camera and RViz captures are
left exactly as recorded.

Figures are pyplot-free Agg figures built once and updated per frame (47 ms at
1080p, measured), so a live recorder costs well under half a core.

Timebase, which every consumer depends on:

    isaac_*.mp4                                  video t == sim t
    costmap.mp4, ekf.mp4, slam_graph.mp4         video t == sim t - viz_start
    rviz.mp4                                     wall-clock paced

run_sim.py captures a frame every second 30 Hz control tick and writes at
15 fps, so its videos run on sim time. The live recorder is paced the same
way on purpose. Only the X11 screen grab is wall-clock, so viz_recorder logs a
sim<->wall table (timebase.csv) and `rviz_mapper()` uses it.
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import re
import subprocess

import numpy as np

WS = pathlib.Path(os.environ.get("SIH_WS", "/home/qbotix-rover/sih_ws"))
PKG = WS / "src/sih_isaac"
GEN = PKG / "generated"

# One palette for the whole deliverable (dark, projector-safe, CVD-checked
# against the report's BLUE/ORANGE pair).
NAVY = "#0C1D31"
PANEL = "#14293F"
EDGE = "#2A4D70"
WHITE = "#FFFFFF"
MUTED = "#9FB4C9"
SKY = "#78B0E6"
GOLD = "#E8B060"
GREEN = "#7FC894"
RED = "#E2645A"
ORANGE = "#E07B39"
BLUE = "#2E74B5"
VIOLET = "#B08CD8"
FREE_GREY = (188, 187, 190)     # how sih.rviz renders the prior map's free cells

NO_GNSS = "no GNSS receiver is read at any point"      # VIDEOS.md only

VIDEO_W, VIDEO_H = 1920, 1080
LIVE_FPS = 8                     # frames per second of SIM time


# --------------------------------------------------------------- ffmpeg ----
def ffmpeg_bin() -> str:
    """The static ffmpeg shipped in .demoenv (this box has no system one)."""
    d = WS / ".demoenv/lib/python3.12/site-packages/imageio_ffmpeg/binaries"
    for cand in sorted(d.glob("ffmpeg-linux-*")):
        if os.access(cand, os.X_OK):
            return str(cand)
    import shutil
    return shutil.which("ffmpeg") or "ffmpeg"


class FramePipe:
    """RGB frames in, H.264 mp4 out.

    Fragmented on purpose: finish_mission.sh may have to SIGKILL a recorder,
    and a fragmented mp4 carries its index in every fragment, so the file stays
    playable however the process dies.
    """

    def __init__(self, path, size=(VIDEO_W, VIDEO_H), fps=LIVE_FPS,
                 ffmpeg=None, crf=21, fragmented=True):
        self.path = pathlib.Path(path)
        self.size = size
        self.frames = 0
        cmd = [ffmpeg or ffmpeg_bin(), "-y", "-nostdin", "-loglevel", "error",
               "-f", "rawvideo", "-pix_fmt", "rgb24",
               "-s", f"{size[0]}x{size[1]}", "-framerate", str(fps),
               "-i", "-",
               "-c:v", "libx264", "-preset", "veryfast", "-crf", str(crf),
               "-pix_fmt", "yuv420p", "-g", str(max(2 * fps, 2))]
        if fragmented:
            cmd += ["-movflags", "+frag_keyframe+empty_moov"]
        cmd += [str(self.path)]
        # own session: a terminal Ctrl-C never reaches the encoder; it ends
        # at EOF when the writer closes the pipe or dies
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                     start_new_session=True)

    def write(self, rgb: np.ndarray) -> None:
        buf = np.ascontiguousarray(rgb, dtype=np.uint8)
        self.proc.stdin.write(buf.tobytes())
        self.frames += 1

    def close(self) -> None:
        try:
            self.proc.stdin.close()
        except Exception:
            pass
        try:
            self.proc.wait(timeout=120)
        except Exception:
            self.proc.kill()


# ----------------------------------------------------------- matplotlib ----
RC = {
    "figure.facecolor": NAVY, "savefig.facecolor": NAVY,
    "axes.facecolor": PANEL, "axes.edgecolor": EDGE, "axes.linewidth": 1.0,
    "axes.labelcolor": MUTED, "axes.titlecolor": WHITE,
    "text.color": WHITE, "xtick.color": MUTED, "ytick.color": MUTED,
    "grid.color": "#22405E", "grid.linewidth": 0.7, "axes.grid": True,
    "font.family": "DejaVu Sans", "font.size": 13,
    "legend.facecolor": PANEL, "legend.edgecolor": EDGE,
    "legend.framealpha": 0.92, "legend.labelcolor": WHITE,
}


def new_figure(w=VIDEO_W, h=VIDEO_H, dpi=100):
    """A pyplot-free Agg figure (safe to drive from a worker thread)."""
    import matplotlib
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    matplotlib.rcParams.update(RC)
    fig = Figure(figsize=(w / dpi, h / dpi), dpi=dpi, facecolor=NAVY)
    FigureCanvasAgg(fig)
    return fig


def fig_rgb(fig) -> np.ndarray:
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba())[..., :3]


def strip_words(fig) -> None:
    """Hide every word the figure would draw; keep pictures and tick numbers.

    Titles, axis labels, legends, figure and axes text, annotations and the
    labels of secondary (e.g. UTM) axes. Call it after a figure is built;
    hidden artists stay hidden when their text is later updated.
    """
    for ax in fig.axes:
        # secondary axes (e.g. the UTM numbers) are child axes with a
        # narrower API: no title, one label, no legend
        for a in [ax] + list(getattr(ax, "child_axes", [])):
            if hasattr(a, "set_title"):
                for loc in ("center", "left", "right"):
                    a.set_title("", loc=loc)
            for setter in ("set_xlabel", "set_ylabel"):
                if hasattr(a, setter):
                    getattr(a, setter)("")
            leg = a.get_legend() if hasattr(a, "get_legend") else None
            if leg is not None:
                leg.remove()
            for t in getattr(a, "texts", []):
                t.set_visible(False)
    for t in fig.texts:
        t.set_visible(False)


# ------------------------------------------------------------- mission -----
def world() -> dict:
    return json.loads((GEN / "world.json").read_text())


def read_log(run) -> np.ndarray:
    """mission_log.csv as an (n, 8) array: t gt_x gt_y gt_yaw ekf_x ekf_y v w."""
    p = pathlib.Path(run) / "mission_log.csv"
    if not p.exists():
        return np.zeros((0, 8))
    rows = []
    for line in p.read_text().splitlines()[1:]:
        parts = line.split(",")
        if len(parts) != 8:
            continue
        try:
            rows.append([float(v) for v in parts])
        except ValueError:
            continue
    return np.array(rows) if rows else np.zeros((0, 8))


def goal_info(run) -> dict:
    """What the operator asked for, parsed back out of goal.log."""
    p = pathlib.Path(run) / "goal.log"
    txt = p.read_text(errors="replace") if p.exists() else ""
    out = dict(easting=None, northing=None, lat=None, lon=None,
               map_x=None, map_y=None, reached="GOAL REACHED" in txt,
               status=None, recoveries=0, text="")
    m = re.search(r"E ([\d.]+) N ([\d.]+)\s*->\s*map \(([-\d.]+), ([-\d.]+)\)",
                  txt)
    if m:
        out["easting"], out["northing"] = float(m.group(1)), float(m.group(2))
        out["map_x"], out["map_y"] = float(m.group(3)), float(m.group(4))
    m = re.search(r"map goal \(([-\d.]+), ([-\d.]+)\) = UTM E ([\d.]+) "
                  r"N ([\d.]+)", txt)
    if m and out["easting"] is None:
        out["map_x"], out["map_y"] = float(m.group(1)), float(m.group(2))
        out["easting"], out["northing"] = float(m.group(3)), float(m.group(4))
    m = re.search(r"lat ([-\d.]+) lon ([-\d.]+)", txt)
    if m:
        out["lat"], out["lon"] = float(m.group(1)), float(m.group(2))
    m = re.search(r"finished with status (\d+)", txt)
    if m:
        out["status"] = int(m.group(1))
    for mm in re.finditer(r"recoveries (\d+)", txt):
        out["recoveries"] = int(mm.group(1))
    m = re.search(r"\[utm_goal\] ((?:WGS84|UTM(?: \d+[NS])?|map) goal .*)",
                  txt)
    out["text"] = m.group(1) if m else ""
    if out["easting"] is None:
        # a goal clicked in RViz leaves no goal.log; viz_recorder notes where
        # the global plan ended, which is that goal
        w = world()
        try:
            gx, gy = map(float, (pathlib.Path(run) / "goal_seen.txt")
                         .read_text().split()[:2])
            out["map_x"], out["map_y"] = gx, gy
            out["easting"] = gx + w["site_datum"]["anchor_e"]
            out["northing"] = gy + w["site_datum"]["anchor_n"]
            out["text"] = (f"goal set in RViz -> map ({gx:.2f}, {gy:.2f}) "
                           f"= UTM E {out['easting']:.2f} "
                           f"N {out['northing']:.2f}")
            return out
        except (OSError, ValueError):
            pass
        g = w["goal_default"]
        out["easting"] = g["utm"]["easting"]
        out["northing"] = g["utm"]["northing"]
        out["map_x"], out["map_y"] = g["x"], g["y"]
    return out


def marks(run) -> dict:
    """Mission-time landmarks, in sim seconds, from the run's own telemetry."""
    d = read_log(run)
    w = world()
    berm = w["berm"]
    if len(d) == 0:
        return dict(start=0.0, move=0.0, tunnel_in=0.0, tunnel_out=0.0,
                    end=0.0, empty=True)
    t, gx, cv = d[:, 0], d[:, 1], d[:, 6]
    moving = np.abs(cv) > 0.05
    t_move = float(t[moving][0]) if moving.any() else float(t[0])
    ent = np.where(gx >= berm["x0"])[0]
    ext = np.where(gx >= berm["x1"])[0]
    t_in = float(t[ent[0]]) if len(ent) else None
    t_out = float(t[ext[0]]) if len(ext) else None
    return dict(start=float(t[0]), move=t_move, tunnel_in=t_in,
                tunnel_out=t_out, end=float(t[-1]), empty=False)


def stats(run) -> dict:
    """Headline numbers for the end card and the caption lines."""
    d = read_log(run)
    g = goal_info(run)
    if len(d) == 0:
        return dict(path_len=0.0, loc_err=0.0, duration=0.0, max_err=0.0,
                    final_dist=0.0, goal=g)
    gx, gy, ex, ey = d[:, 1], d[:, 2], d[:, 4], d[:, 5]
    ok = np.isfinite(ex) & np.isfinite(ey)
    err = np.hypot(gx[ok] - ex[ok], gy[ok] - ey[ok]) if ok.any() else np.zeros(1)
    return dict(
        path_len=float(np.sum(np.hypot(np.diff(gx), np.diff(gy)))),
        loc_err=float(err[-1]), max_err=float(np.max(err)),
        rms_err=float(np.sqrt(np.mean(err ** 2))),
        duration=float(d[-1, 0] - d[0, 0]),
        final_dist=float(np.hypot(gx[-1] - g["map_x"], gy[-1] - g["map_y"])),
        goal=g)


# ------------------------------------------------------------- timebase ----
def rviz_mapper(run):
    """sim seconds -> seconds into rviz.mp4 (which is wall-clock paced).

    viz_recorder.py logs a sim<->wall table while it records and nav.launch.py
    stamps the moment the screen grab started, so the two clocks can be tied
    together. Falls back to the identity when either file is missing, which is
    what the pipeline did before the table existed.
    """
    run = pathlib.Path(run)
    tb, t0 = run / "timebase.csv", run / "rviz_start.txt"
    try:
        wall0 = float(t0.read_text().strip())
        rows = [l.split(",") for l in tb.read_text().splitlines()[1:]]
        sim = np.array([float(r[0]) for r in rows])
        wall = np.array([float(r[1]) for r in rows])
        if len(sim) < 2:
            raise ValueError("too few samples")
    except (OSError, ValueError, IndexError):
        return lambda t: t
    order = np.argsort(sim)
    sim, wall = sim[order], wall[order]

    def to_rviz(t):
        return float(max(np.interp(t, sim, wall) - wall0, 0.0))
    return to_rviz


_PROBE: dict = {}


def probe(path) -> dict:
    """{width, height, seconds} for a video, cached.

    There is no ffprobe in the imageio-ffmpeg build, and a fragmented mp4's
    header duration is unreliable, so the length is taken from a stream copy
    to null -- exact, and cheap because nothing is decoded.
    """
    path = pathlib.Path(path)
    key = str(path)
    if key in _PROBE:
        return _PROBE[key]
    out = dict(width=0, height=0, seconds=0.0, exists=path.exists())
    if path.exists():
        r = subprocess.run([ffmpeg_bin(), "-nostdin", "-i", str(path),
                            "-map", "0:v:0", "-c", "copy", "-f", "null", "-"],
                           capture_output=True, text=True)
        m = re.search(r"Video:.*?, (\d{2,5})x(\d{2,5})", r.stderr)
        if m:
            out["width"], out["height"] = int(m.group(1)), int(m.group(2))
        last = None
        for last in re.finditer(r"time=(\d+):(\d+):([\d.]+)", r.stderr):
            pass
        if last:
            h, mi, sec = last.groups()
            out["seconds"] = int(h) * 3600 + int(mi) * 60 + float(sec)
    _PROBE[key] = out
    return out


def brightness(path, t) -> float:
    """Mean luma (0-255) of one frame at t seconds; -1 if it can't be read.

    A screen grab goes black when the desktop locks or the display sleeps, and
    a highlight must not cut to that.
    """
    r = subprocess.run([ffmpeg_bin(), "-nostdin", "-loglevel", "error",
                        "-ss", f"{max(t, 0.0):.2f}", "-i", str(path),
                        "-frames:v", "1", "-vf", "scale=64:36",
                        "-f", "rawvideo", "-pix_fmt", "gray", "-"],
                       capture_output=True)
    return (sum(r.stdout) / len(r.stdout)) if r.stdout else -1.0


def frame_rgb(path, t, size=None):
    """One decoded frame as an (h, w, 3) uint8 array, or None."""
    info = probe(path)
    w, h = size or (info["width"], info["height"])
    if not (w and h):
        return None
    r = subprocess.run([ffmpeg_bin(), "-nostdin", "-loglevel", "error",
                        "-ss", f"{max(t, 0.0):.2f}", "-i", str(path),
                        "-frames:v", "1", "-vf", f"scale={w}:{h}",
                        "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                       capture_output=True)
    if len(r.stdout) != w * h * 3:
        return None
    return np.frombuffer(r.stdout, np.uint8).reshape(h, w, 3)


def rviz_viewport(path, samples=9):
    """(x, y, w, h) of RViz's 3D view inside a full-screen grab, or None.

    The screen grab also holds RViz's panels, menus, the desktop bar and the
    terminals -- all text. The 3D view is the one large region dominated by
    the prior map's free-space grey, so: per-pixel median over several lit
    frames, mask that grey, and take the span of rows and columns where it
    dominates. Returns None if nothing plausible (under a quarter of the
    screen) is found.
    """
    info = probe(path)
    if not info["seconds"]:
        return None
    frames = []
    for k in range(samples):
        f = frame_rgb(path, info["seconds"] * (k + 1) / (samples + 1))
        if f is not None and f.mean() > 20:           # skip blanked screens
            frames.append(f)
    if len(frames) < 3:
        return None
    med = np.median(np.stack(frames), axis=0)
    grey = np.array(FREE_GREY, dtype=float)
    mask = np.abs(med - grey).sum(-1) < 24
    h, w = mask.shape

    # the longest run of indices, bridging gaps where map overlays (costmap
    # strips, the berm) interrupt the grey
    def longest(idx, gap):
        if not len(idx):
            return None
        br = np.where(np.diff(idx) > gap)[0]
        seg = max(np.split(idx, br + 1), key=len)
        return int(seg[0]), int(seg[-1])
    r = longest(np.where(mask.mean(axis=1) > 0.35)[0], 40)
    if r is None:
        return None
    y0, y1 = r
    # strict coverage keeps the side panels out (theirs is ~0.15); wide
    # bridging spans the map overlays, whose columns can read zero
    c = longest(np.where(mask[y0:y1 + 1].mean(axis=0) > 0.6)[0], 150)
    if c is None:
        return None
    x0, x1 = c
    x0, y0, x1, y1 = x0 + 4, y0 + 4, x1 - 4, y1 - 4    # clear the panel edges
    if (x1 - x0) * (y1 - y0) < 0.25 * w * h:
        return None
    return x0, y0, x1 - x0, y1 - y0


def viz_offset(run) -> float:
    """Sim time at t = 0 of costmap.mp4 / ekf.mp4 / slam_graph.mp4."""
    try:
        return float((pathlib.Path(run) / "viz_start.txt").read_text().strip())
    except (OSError, ValueError):
        return 0.0


def duration(path) -> float:
    return probe(path)["seconds"]


def yaw_of(qz, qw) -> float:
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
