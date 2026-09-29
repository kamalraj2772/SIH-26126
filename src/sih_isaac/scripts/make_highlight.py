"""Cut the final highlight reel (at most 2 minutes) from a recorded mission.

    .demoenv/bin/python src/sih_isaac/scripts/make_highlight.py <run_dir> \
        [--ffmpeg PATH] [--out FILE] [--max-seconds 120]

Pure picture -- no title cards, captions or labels -- 1920x1080 at 30 fps,
silent, in mission order:

    the input      GeoTIFF, slope, UTM <-> map and goal, from geo_pipeline.mp4
    the drive      chase camera at the start, the rover's own camera, a 2x2
                   camera grid over the traverse and, when the route crosses
                   the berm, the berm and the tunnel from the rover's camera
    the stack      RViz, then the costmap, EKF and SLAM/graph videos
    arrival        the camera grid on the final approach

Cuts land on real mission events (first motion, berm entry and exit, arrival)
read from mission_log.csv. Every source is optional: a missing video drops
its shot. The total is budgeted and enforced -- never more than --max-seconds.
Camera and RViz shots come from raw/ when a run still has one (runs captioned
by an earlier version of this pipeline).
"""
import argparse
import concurrent.futures as cf
import json
import pathlib
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "sih_isaac"))
import video                                                    # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run_dir")
ap.add_argument("--ffmpeg", default=None)
ap.add_argument("--out", default=None)
ap.add_argument("--max-seconds", type=float, default=120.0)
args = ap.parse_args()

RUN = pathlib.Path(args.run_dir).resolve()
OUT = pathlib.Path(args.out) if args.out else RUN / "QSLAM_highlight.mp4"
FF = args.ffmpeg or video.ffmpeg_bin()
W, H, FPS = video.VIDEO_W, video.VIDEO_H, 30
LIMIT = args.max_seconds
TMP = pathlib.Path(tempfile.mkdtemp(prefix="reel_"))


def src(name):
    raw = RUN / "raw" / name
    return raw if raw.exists() else RUN / name


SRC = {k: src(f"isaac_{k}.mp4") for k in ("behind", "side", "top", "onboard")}
SRC["rviz"] = src("rviz.mp4")
for k in ("costmap", "ekf", "slam_graph", "geo_pipeline"):
    SRC[k] = RUN / f"{k}.mp4"
HAVE = {k: v.exists() and video.probe(v)["seconds"] > 1.0
        for k, v in SRC.items()}
LEN = {k: video.probe(v)["seconds"] if HAVE[k] else 0.0
       for k, v in SRC.items()}
print("[reel] sources: " + ", ".join(
    f"{k} {LEN[k]:.0f}s" if HAVE[k] else f"{k} -" for k in SRC), flush=True)

M = video.marks(RUN)
if M["empty"]:
    raise SystemExit("[reel] mission_log.csv is empty; nothing to cut")
TUNNEL = M["tunnel_in"] is not None and M["tunnel_out"] is not None
to_rviz = video.rviz_mapper(RUN)
viz0 = video.viz_offset(RUN)
print(f"[reel] move {M['move']:.0f}s  tunnel "
      f"{M['tunnel_in'] or 0:.0f}-{M['tunnel_out'] or 0:.0f}s  "
      f"end {M['end']:.0f}s", flush=True)


def file_time(key, t_sim):
    """Mission (sim) time -> seconds into that source file."""
    if key == "rviz":
        return to_rviz(t_sim)
    if key in ("costmap", "ekf", "slam_graph"):
        return t_sim - viz0
    return t_sim


# ------------------------------------------------------------- the plan ---
PLAN = []


def shot(key, t0, t1, dur, sim=True):
    """One source over [t0, t1] (mission time, or file time when sim=False)."""
    if not HAVE[key]:
        return False
    a = file_time(key, t0) if sim else t0
    b = file_time(key, t1) if sim else t1
    a, b = max(a, 0.0), min(b, LEN[key] - 0.2)
    if b - a < 1.0:
        return False
    PLAN.append(dict(kind="shot", key=key, a=a, b=b, dur=dur))
    return True


def grid(keys, t0, t1, dur):
    """2x2 camera grid; with fewer than three views, one shot instead."""
    keys = [k for k in keys if HAVE[k]]
    if len(keys) < 3:
        return bool(keys) and shot(keys[0], t0, t1, dur)
    b = min([t1] + [LEN[k] - 0.2 for k in keys if k != "rviz"])
    if b - t0 < 1.0:
        return False
    PLAN.append(dict(kind="grid", keys=keys, a=max(t0, 0.0), b=b, dur=dur))
    return True


GRID4 = ["behind", "side", "top", "onboard" if HAVE["onboard"] else "rviz"]
t_mv, t_end = M["move"], M["end"]

# the input
geo_ch = {}
gj = RUN / "geo_chapters.json"
if gj.exists():
    geo_ch = {c.get("key"): c for c in json.loads(gj.read_text())}
used = False
for key, dur in (("geotiff", 4.0), ("slope", 5.0), ("utm_map", 4.0),
                 ("goal", 6.0)):
    c = geo_ch.get(key)
    if c:
        used |= shot("geo_pipeline", c["t0"] + 0.2, c["t1"] - 0.1, dur,
                     sim=False)
if not used and HAVE["geo_pipeline"]:
    shot("geo_pipeline", 0.0, LEN["geo_pipeline"], 18.0, sim=False)

# the drive
shot("behind", t_mv - 1.0, t_mv + 19.0, 8.0)
shot("onboard", t_mv + 19.0, t_mv + 45.0, 8.0)
trav_end = (M["tunnel_in"] - 14.0) if TUNNEL else (t_end - 50.0)
grid(GRID4, t_mv + 45.0, trav_end, 16.0 if TUNNEL else 26.0)
if TUNNEL:
    shot("side", M["tunnel_in"] - 14.0, M["tunnel_in"] + 16.0, 6.0)
    shot("onboard", M["tunnel_in"], M["tunnel_out"] + 4.0, 8.0)

# the stack -- an RViz window with the screen actually lit: the grab goes
# black while the desktop is locked or the display sleeps
mid0 = None
if HAVE["rviz"]:
    for frac in (0.25, 0.4, 0.55, 0.1, 0.7, 0.85):
        c = t_mv + frac * (t_end - t_mv)
        probes = [video.brightness(SRC["rviz"], to_rviz(c + d))
                  for d in (2.0, 15.0, 28.0, 38.0)]
        if min(probes) > 20.0:
            mid0 = c
            break
    if mid0 is None:
        print("[reel] RViz capture is blank throughout the mission "
              "(screen locked or asleep?) -- leaving it out", flush=True)
if mid0 is not None:
    shot("rviz", mid0, mid0 + 40.0, 9.0)
span = (t_mv - 5.0, t_end + 3.0)
shot("costmap", *span, 11.0)
shot("ekf", *span, 10.0)
shot("slam_graph", *span, 9.0)

# arrival
grid(GRID4, t_end - 45.0, t_end + 2.0, 9.0)

# ------------------------------------------------------------ the budget ---
total = sum(s["dur"] for s in PLAN)
budget = LIMIT - 1.0                              # headroom for rounding
if total > budget:
    k = budget / total
    for s in PLAN:
        s["dur"] *= k
    print(f"[reel] trimmed shots by {100 * (1 - k):.0f}% to stay under "
          f"{LIMIT:.0f}s", flush=True)
print(f"[reel] {len(PLAN)} shots, {sum(s['dur'] for s in PLAN):.1f}s planned "
      f"(limit {LIMIT:.0f}s)", flush=True)

# ---------------------------------------------------------------- render ---
ENC = ["-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
       "-pix_fmt", "yuv420p", "-r", str(FPS), "-an"]


def ffrun(cmd, out):
    r = subprocess.run(cmd + ENC + [str(out)], stdout=subprocess.DEVNULL,
                       stderr=subprocess.PIPE, text=True)
    if r.returncode:
        raise RuntimeError(f"ffmpeg failed for {out.name}:\n{r.stderr[-1500:]}")
    return out


def render(i, s):
    out = TMP / f"s{i:02d}.mp4"
    base = [FF, "-y", "-nostdin", "-loglevel", "error"]
    tail = f"trim=duration={s['dur']:.3f},setpts=PTS-STARTPTS"
    if s["kind"] == "shot":
        sp = max((s["b"] - s["a"]) / s["dur"], 0.25)
        return ffrun(base + [
            "-ss", f"{s['a']:.3f}", "-t", f"{s['b'] - s['a']:.3f}",
            "-i", str(SRC[s["key"]]), "-vf",
            f"setpts=(PTS-STARTPTS)/{sp:.5f},scale={W}:{H}:flags=lanczos,"
            f"fps={FPS},{tail},format=yuv420p"], out)
    # grid: each input gets its own window and speed -- RViz is wall-clock,
    # so the same mission span covers a different length of its file
    keys, cmd, parts = s["keys"], list(base), []
    for j, k in enumerate(keys):
        ka, kb = file_time(k, s["a"]), min(file_time(k, s["b"]), LEN[k] - 0.2)
        sp = max((kb - ka) / s["dur"], 0.25)
        cmd += ["-ss", f"{ka:.3f}", "-t", f"{kb - ka:.3f}", "-i", str(SRC[k])]
        parts.append(f"[{j}:v]setpts=(PTS-STARTPTS)/{sp:.5f},"
                     f"scale={W // 2}:{H // 2},fps={FPS}[v{j}]")
    n = len(keys)                     # 3 or 4: the bounding box is W x H
    layout = "|".join(["0_0", "w0_0", "0_h0", "w0_h0"][:n])
    parts.append("".join(f"[v{j}]" for j in range(n))
                 + f"xstack=inputs={n}:layout={layout}:fill=#0C1D31,"
                 f"{tail},format=yuv420p[out]")
    return ffrun(cmd + ["-filter_complex", ";".join(parts),
                        "-map", "[out]"], out)


with cf.ThreadPoolExecutor(max_workers=6) as pool:
    futs = [pool.submit(render, i, s) for i, s in enumerate(PLAN)]
    segs = []
    for i, f in enumerate(futs):
        try:
            segs.append(f.result())
        except Exception as exc:                  # drop the shot, keep the reel
            print(f"[reel] shot {i} ({PLAN[i]['kind']} "
                  f"{PLAN[i].get('key', '')}) dropped: {exc}", flush=True)

lst = TMP / "list.txt"
lst.write_text("".join(f"file '{p}'\n" for p in segs))
subprocess.run([FF, "-y", "-nostdin", "-loglevel", "error", "-f", "concat",
                "-safe", "0", "-i", str(lst), "-c", "copy",
                "-movflags", "+faststart", str(OUT)], check=True)
video._PROBE.pop(str(OUT), None)
got = video.probe(OUT)["seconds"]
shutil.rmtree(TMP, ignore_errors=True)
print(f"[reel] wrote {OUT}  ({got:.1f}s, limit {LIMIT:.0f}s)", flush=True)
if got > LIMIT + 0.5:
    raise SystemExit(f"[reel] ERROR: reel is {got:.1f}s, over the {LIMIT:.0f}s cap")
