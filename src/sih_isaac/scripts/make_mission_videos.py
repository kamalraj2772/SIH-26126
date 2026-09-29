#!/usr/bin/env python3
"""Build every post-mission video for a run and index them in VIDEOS.md.

    .demoenv/bin/python src/sih_isaac/scripts/make_mission_videos.py <run_dir> \
        [--ffmpeg PATH] [--max-seconds 120] [--index-only]

    0. rviz.mp4              cropped to RViz's 3D view (the full screen grab,
                             panels and terminals included, is kept in
                             with_text_originals/)
    1. geo_pipeline.mp4      GeoTIFF -> DEM -> slope -> UTM -> this run's goal
    2. QSLAM_highlight.mp4   the final reel, at most --max-seconds (default 120)
    3. VIDEOS.md             every video in the run: length, what it shows

No video carries words: the camera captures stay exactly as recorded, RViz is
cut down to its 3D view, and the drawn videos are pictures and tick numbers
only. A run
captioned by an earlier version of this pipeline gets its caption-free
originals back from raw/ first.

The live videos (costmap.mp4, ekf.mp4, slam_graph.mp4) are written during the
mission by viz_recorder.py and the camera views by run_sim.py; this script
only finishes the set. Each stage runs even if another one failed, and the
exit status reports whether all of them succeeded.
"""
import argparse
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "sih_isaac"))
import video                                                    # noqa: E402

CATALOGUE = [
    ("QSLAM_highlight.mp4", "Final highlight reel",
     "the whole mission in at most two minutes: the input, the drive, "
     "inside the stack, arrival"),
    ("isaac_onboard.mp4", "Rover camera",
     "the mast-mounted ZED camera, 1080p, rendered in Isaac Sim"),
    ("isaac_behind.mp4", "Chase camera",
     "third-person follow view from behind the rover"),
    ("isaac_side.mp4", "Side camera",
     "follow view from the rover's right flank"),
    ("isaac_top.mp4", "Top-down camera",
     "bird's-eye follow view, 16 m above the rover"),
    ("rviz.mp4", "RViz 3D view",
     "the operator's live view -- prior map, costmaps, plans, lidar, EKF "
     "pose -- cropped from the screen capture (full grab, with its panels "
     "and terminals, in with_text_originals/)"),
    ("costmap.mp4", "Costmaps and planning",
     "global costmap (GeoTIFF slope prior + lidar + inflation), Smac A* "
     "route, rolling local costmap with MPPI candidates"),
    ("ekf.mp4", "EKF state estimation",
     "EKF estimate vs ground truth vs wheel-only dead reckoning, with the "
     "error and heading histories"),
    ("slam_graph.mp4", "Mapping and the live ROS 2 graph",
     "map built from lidar with the pose graph, beside the node/topic graph "
     "whose links light up while their topics carry traffic"),
    ("geo_pipeline.mp4", "GeoTIFF -> DEM -> UTM sequence",
     "the GeoTIFF on its UTM grid, the DEM, slope, occupancy grid, UTM and "
     "map frames, and this run's goal and route"),
]


def restore_raw(run):
    """Undo captions burned in by an earlier version of this pipeline: put the
    caption-free originals from raw/ back in place."""
    raw = run / "raw"
    if not raw.is_dir():
        return
    for f in sorted(raw.glob("*.mp4")):
        f.replace(run / f.name)
        print(f"[videos] restored caption-free {f.name}", flush=True)
    try:
        raw.rmdir()
    except OSError:
        pass
    (run / ".annotated.json").unlink(missing_ok=True)


def rviz_view(run, ff):
    """Make rviz.mp4 word-free: crop the screen grab to RViz's 3D view.

    The grab also shows RViz's panels and menus, the desktop bar and the
    terminals -- all text. The full grab is kept in with_text_originals/;
    rviz.mp4 becomes the 3D view alone (16:9, 1080p, same timeline). When the
    3D view can't be found (other windows over it, a different layout) there
    is no word-free rviz.mp4 for that run, and the reel leaves RViz out.
    """
    keep = run / "with_text_originals"
    full = keep / "rviz.mp4"
    if full.exists() and (run / "rviz.mp4").exists():
        return True                                  # already done
    if not full.exists():
        if not (run / "rviz.mp4").exists():
            return True                              # no screen grab this run
        keep.mkdir(exist_ok=True)
        (run / "rviz.mp4").replace(full)
    box = video.rviz_viewport(full)
    if box is None:
        print("[videos] RViz 3D view not found in the screen grab (windows "
              "over it?) -- no word-free rviz.mp4 for this run; the full grab "
              "is in with_text_originals/", flush=True)
        return True
    x, y, w, h = box
    cw = w - w % 2
    ch = int(round(cw * 9 / 16)) // 2 * 2
    if ch > h:
        ch = h - h % 2
        cw = int(round(ch * 16 / 9)) // 2 * 2
    cx, cy = x + (w - cw) // 2, y + (h - ch) // 2
    print(f"[videos] rviz.mp4 -> 3D view only ({cw}x{ch} at {cx},{cy})",
          flush=True)
    r = subprocess.run([ff, "-y", "-nostdin", "-loglevel", "error",
                        "-i", str(full), "-vf",
                        f"crop={cw}:{ch}:{cx}:{cy},scale=1920:1080:"
                        "flags=lanczos,format=yuv420p",
                        "-c:v", "libx264", "-preset", "veryfast", "-crf", "21",
                        "-movflags", "+faststart", str(run / "rviz.mp4")],
                       capture_output=True, text=True)
    if r.returncode:
        print(f"[videos] rviz crop failed:\n{r.stderr[-800:]}", flush=True)
        (run / "rviz.mp4").unlink(missing_ok=True)
        return False
    return True


def write_index(run):
    """VIDEOS.md: every video in the run, its length, what it shows."""
    lines = [f"# Mission videos - {run.name}", "",
             "No video carries on-screen words: pictures only.", "",
             "| file | length | what it shows |",
             "|---|---|---|"]
    for fname, title, what in CATALOGUE:
        p = run / fname
        if not p.exists():
            lines.append(f"| `{fname}` | not recorded | {title}: {what} |")
            continue
        video._PROBE.pop(str(p), None)
        info = video.probe(p)
        secs = info["seconds"]
        lines.append(f"| `{fname}` | {int(secs // 60)}:{secs % 60:04.1f} | "
                     f"**{title}** - {what} |")
    st = video.stats(run)
    g = st["goal"]
    lines += ["", "## Mission", "",
              f"- commanded goal: {g['text'] or 'default goal'}",
              f"- outcome: {'GOAL REACHED' if g['reached'] else 'status ' + str(g['status'])}",
              f"- driven {st['path_len']:.1f} m, finished "
              f"{st['final_dist']:.2f} m from the goal",
              f"- EKF error at arrival {st['loc_err']:.2f} m "
              f"(RMS {st.get('rms_err', 0.0):.2f} m)",
              f"- {video.NO_GNSS}", ""]
    (run / "VIDEOS.md").write_text("\n".join(lines))
    print(f"[videos] index -> {run / 'VIDEOS.md'}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--ffmpeg", default=None)
    ap.add_argument("--max-seconds", type=float, default=120.0)
    ap.add_argument("--index-only", action="store_true",
                    help="only rewrite VIDEOS.md")
    args = ap.parse_args()
    run = pathlib.Path(args.run_dir).resolve()
    if args.index_only:
        write_index(run)
        return
    ff = args.ffmpeg or video.ffmpeg_bin()
    py = sys.executable
    failed = []

    restore_raw(run)
    if not rviz_view(run, ff):
        failed.append("rviz_view")
    if subprocess.call([py, str(HERE / "make_geo_video.py"), str(run),
                        "--ffmpeg", ff]) != 0:
        failed.append("geo_pipeline")
    if subprocess.call([py, str(HERE / "make_highlight.py"), str(run),
                        "--ffmpeg", ff,
                        "--max-seconds", str(args.max_seconds)]) != 0:
        failed.append("highlight")

    write_index(run)
    if failed:
        print(f"[videos] FAILED stages: {', '.join(failed)}", flush=True)
        sys.exit(1)
    print("[videos] all mission videos built", flush=True)


if __name__ == "__main__":
    main()
