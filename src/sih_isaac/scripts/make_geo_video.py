#!/usr/bin/env python3
"""Animate the georeferencing chain that turns one grid reference into a route.

    .demoenv/bin/python src/sih_isaac/scripts/make_geo_video.py <run_dir> \
        [--ffmpeg PATH] [--out FILE]

Writes <run_dir>/geo_pipeline.mp4 (1920x1080, 30 fps, about 55 s, silent,
no words -- pictures and numeric tick marks only):

    1 GeoTIFF      the raster drawn on its own UTM 44N grid, revealed north
                   to south
    2 DEM          elevation, a sweep line and its live height profile
    3 slope        slope magnitude beside the blocked cells as the threshold
                   sweeps down to the platform limit
    4 occupancy    map.pgm, the Nav2 static layer
    5 UTM <-> map  one picture, map-frame numbers on two sides and UTM numbers
                   on the other two; the surveyed anchor appears at (0, 0)
    6 goal         start, this run's commanded goal and the route driven

Every picture is computed from the artefacts the mission actually used (the
GeoTIFF's own transform, world.json's datum, this run's goal and telemetry).
Chapter spans go to geo_chapters.json for make_highlight.py. Needs rasterio,
so it runs under .demoenv.
"""
import argparse
import json
import math
import pathlib
import sys

import numpy as np
import rasterio

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "sih_isaac"))
import video                                                    # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run_dir")
ap.add_argument("--ffmpeg", default=None)
ap.add_argument("--out", default=None)
ap.add_argument("--fps", type=int, default=30)
args = ap.parse_args()

RUN = pathlib.Path(args.run_dir).resolve()
OUT = pathlib.Path(args.out) if args.out else RUN / "geo_pipeline.mp4"
FPS = args.fps
GEN = video.GEN

W = video.world()
DATUM = W["site_datum"]
HALF = W["world_size"] / 2.0
MAXSLOPE = W["max_slope_deg"]
GOAL = video.goal_info(RUN)
LOG = video.read_log(RUN)

tif = rasterio.open(GEN / "prior_dem_utm44n.tif")
DEM = tif.read(1)                       # north-up: row 0 is the north edge
B = tif.bounds
RES = W["world_size"] / (DEM.shape[0] - 1)
MAP_EXT = (-HALF, HALF, -HALF, HALF)    # with origin="upper": row 0 at +y
UTM_EXT = (B.left, B.right, B.bottom, B.top)


def hillshade(z, res, az=315.0, alt=45.0):
    dy, dx = np.gradient(z.astype(np.float64), res)
    slope = np.arctan(np.hypot(dx, dy))
    aspect = np.arctan2(-dx, dy)
    a, h = math.radians(az), math.radians(alt)
    return np.clip(math.sin(h) * np.cos(slope)
                   + math.cos(h) * np.sin(slope) * np.cos(a - aspect), 0, 1)


def slope_deg(z, res):
    dy, dx = np.gradient(z.astype(np.float64), res)
    return np.degrees(np.arctan(np.hypot(dx, dy)))


HS = hillshade(DEM, RES)
SL = slope_deg(DEM, RES)
OCC = SL > MAXSLOPE
ALONG = (np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(LOG[:, 1]),
                                                   np.diff(LOG[:, 2])))])
         if len(LOG) > 1 else np.zeros(len(LOG)))

pipe = video.FramePipe(OUT, fps=FPS, ffmpeg=args.ffmpeg or video.ffmpeg_bin(),
                       crf=19, fragmented=False)
print(f"[geo] writing {OUT}", flush=True)
CHAPTERS = []


def ease(p):
    return 0.5 - 0.5 * math.cos(math.pi * min(max(p, 0.0), 1.0))


def scene(key, seconds, setup, update):
    """Build the figure once, update it per frame; repeat unchanged frames.

    `update` returns a key describing what it drew; an unchanged key reuses
    the previous frame instead of re-rasterising a static 1080p figure.
    """
    t0 = pipe.frames / FPS
    fig = video.new_figure()
    st = setup(fig)
    video.strip_words(fig)
    total = int(seconds * FPS)
    last, frame = object(), None
    for i in range(total):
        k = update(st, ease(i / max(total - 1, 1)))
        if frame is None or k != last:
            frame, last = video.fig_rgb(fig), k
        pipe.write(frame)
    CHAPTERS.append(dict(key=key, t0=round(t0, 3),
                         t1=round(pipe.frames / FPS, 3)))
    print(f"[geo] {key}: {seconds:.0f} s", flush=True)


def square(fig, cx=0.5, size=0.90):
    """A centred axes square on screen (1080 px tall frame)."""
    h = size
    w = h * video.VIDEO_H / video.VIDEO_W
    return fig.add_axes([cx - w / 2, (1 - h) / 2, w, h])


# ------------------------------------------------------------- 1. GeoTIFF --
def s_tif(fig):
    ax = square(fig)
    ax.imshow(HS, cmap="gray", origin="upper", extent=UTM_EXT,
              interpolation="bilinear")
    ax.imshow(DEM, cmap="terrain", origin="upper", extent=UTM_EXT, alpha=0.55,
              interpolation="bilinear")
    ax.ticklabel_format(style="plain", useOffset=False)
    ax.tick_params(labelsize=11)
    # a curtain lifted north to south reveals the raster
    from matplotlib.colors import ListedColormap
    curtain = ax.imshow(np.zeros((2, 2)), cmap=ListedColormap([video.NAVY]),
                        extent=UTM_EXT, zorder=5)
    ax.set_xlim(B.left, B.right)
    ax.set_ylim(B.bottom, B.top)
    return dict(ax=ax, curtain=curtain)


def u_tif(st, p):
    top = B.top - (B.top - B.bottom) * min(p / 0.8, 1.0)
    st["curtain"].set_extent((B.left, B.right, B.bottom, max(top, B.bottom)))
    st["curtain"].set_visible(top > B.bottom + 0.01)
    return round(top, 1)


scene("geotiff", 8.0, s_tif, u_tif)


# ----------------------------------------------------------------- 2. DEM --
def s_dem(fig):
    ax = fig.add_axes([0.03, 0.30, 0.44, 0.66])
    ax.imshow(DEM, cmap="terrain", origin="upper", extent=MAP_EXT,
              interpolation="bilinear")
    ax.set_aspect("equal")
    sweep, = ax.plot([], [], color=video.GOLD, lw=2.4)
    ax2 = fig.add_axes([0.53, 0.30, 0.44, 0.66])
    ax2.imshow(HS, cmap="gray", origin="upper", extent=MAP_EXT,
               interpolation="bilinear")
    ax2.set_aspect("equal")
    sweep2, = ax2.plot([], [], color=video.GOLD, lw=2.4)
    px = fig.add_axes([0.06, 0.05, 0.88, 0.19])
    px.set_xlim(-HALF, HALF)
    px.set_ylim(float(DEM.min()) - 0.4, float(DEM.max()) + 0.4)
    prof, = px.plot([], [], color=video.SKY, lw=2.2)
    fill = [px.fill_between([0, 0], [0, 0], color=video.SKY, alpha=0.25)]
    return dict(sweep=sweep, sweep2=sweep2, prof=prof, px=px, fill=fill)


XS = np.linspace(-HALF, HALF, DEM.shape[1])


def u_dem(st, p):
    row = int(p * (DEM.shape[0] - 1))
    y = HALF - row * RES
    st["sweep"].set_data([-HALF, HALF], [y, y])
    st["sweep2"].set_data([-HALF, HALF], [y, y])
    st["prof"].set_data(XS, DEM[row])
    st["fill"][0].remove()
    st["fill"][0] = st["px"].fill_between(XS, DEM.min() - 0.4, DEM[row],
                                          color=video.SKY, alpha=0.25)
    return row


scene("dem", 10.0, s_dem, u_dem)


# --------------------------------------------------------------- 3. slope --
def s_slope(fig):
    ax = fig.add_axes([0.03, 0.05, 0.45, 0.90])
    ax.imshow(SL, cmap="inferno", origin="upper", extent=MAP_EXT, vmin=0,
              vmax=40, interpolation="bilinear")
    ax.set_aspect("equal")
    bx = fig.add_axes([0.52, 0.05, 0.45, 0.90])
    blocked = bx.imshow(np.zeros_like(SL), cmap="Reds", origin="upper",
                        extent=MAP_EXT, vmin=0, vmax=1,
                        interpolation="nearest")
    bx.set_aspect("equal")
    return dict(blocked=blocked)


def u_slope(st, p):
    thr = round(40.0 - (40.0 - MAXSLOPE) * min(p / 0.75, 1.0), 1)
    st["blocked"].set_data((SL > thr).astype(float))
    return thr


scene("slope", 10.0, s_slope, u_slope)


# ----------------------------------------------------------- 4. occupancy --
def s_occ(fig):
    ax = square(fig)
    rgb = np.empty(OCC.shape + (3,), dtype=np.uint8)
    rgb[...] = (36, 62, 90)
    rgb[OCC] = (226, 236, 246)
    ax.imshow(HS, cmap="gray", origin="upper", extent=MAP_EXT,
              interpolation="bilinear")
    over = ax.imshow(rgb, origin="upper", extent=MAP_EXT,
                     interpolation="nearest", alpha=0.0)
    ax.set_aspect("equal")
    return dict(over=over)


def u_occ(st, p):
    a = round(min(p / 0.5, 1.0), 2)                 # cross-fade terrain -> grid
    st["over"].set_alpha(a)
    return a


scene("occupancy", 7.0, s_occ, u_occ)


# ----------------------------------------------------------------- 5. UTM --
def s_utm(fig):
    ax = fig.add_axes([0.25, 0.08, 0.50, 0.84])
    ax.imshow(HS, cmap="gray", origin="upper", extent=MAP_EXT, alpha=0.8,
              interpolation="bilinear")
    ax.set_aspect("equal")
    from matplotlib.ticker import FormatStrFormatter
    top = ax.secondary_xaxis("top", functions=(
        lambda v: v + DATUM["anchor_e"], lambda v: v - DATUM["anchor_e"]))
    right = ax.secondary_yaxis("right", functions=(
        lambda v: v + DATUM["anchor_n"], lambda v: v - DATUM["anchor_n"]))
    for sec, axis in ((top, "xaxis"), (right, "yaxis")):
        sec.tick_params(colors=video.GOLD, labelsize=10)
        getattr(sec, axis).set_major_formatter(FormatStrFormatter("%.0f"))
    anchor, = ax.plot([], [], marker="P", ms=24, color=video.GOLD,
                      mec=video.NAVY, mew=1.6, ls="none", zorder=6)
    ring, = ax.plot([], [], color=video.GOLD, lw=2.0, zorder=5)
    hline, = ax.plot([], [], color=video.GOLD, lw=1.2, ls="--", alpha=0.8)
    vline, = ax.plot([], [], color=video.GOLD, lw=1.2, ls="--", alpha=0.8)
    return dict(anchor=anchor, ring=ring, hline=hline, vline=vline)


def u_utm(st, p):
    q = min(p / 0.6, 1.0)
    st["hline"].set_data([-HALF, -HALF + 2 * HALF * q], [0, 0])
    st["vline"].set_data([0, 0], [-HALF, -HALF + 2 * HALF * q])
    shown = p > 0.6
    if shown:
        st["anchor"].set_data([0.0], [0.0])
        r = 3.0 + 6.0 * ((p - 0.6) / 0.4)
        th = np.linspace(0, 2 * np.pi, 80)
        st["ring"].set_data(r * np.cos(th), r * np.sin(th))
    return round(q, 2), shown, round(p, 2) if shown else 0


scene("utm_map", 8.0, s_utm, u_utm)


# ---------------------------------------------------------------- 6. goal --
def s_goal(fig):
    ax = square(fig)
    ax.imshow(HS, cmap="gray", origin="upper", extent=MAP_EXT, alpha=0.7,
              interpolation="bilinear")
    ax.imshow(np.where(OCC, 1.0, np.nan), cmap="autumn", origin="upper",
              extent=MAP_EXT, alpha=0.55, interpolation="nearest")
    ax.set_aspect("equal")
    ax.plot([W["start"]["x"]], [W["start"]["y"]], marker="o", ms=15,
            color=video.SKY, mec=video.NAVY, mew=1.5, ls="none", zorder=6)
    goal, = ax.plot([], [], marker="*", ms=30, color=video.VIOLET,
                    mec=video.NAVY, mew=1.4, ls="none", zorder=6)
    driven, = ax.plot([], [], color=video.GREEN, lw=2.6, zorder=5)
    return dict(goal=goal, driven=driven)


def u_goal(st, p):
    shown = p > 0.15
    if shown:
        st["goal"].set_data([GOAL["map_x"]], [GOAL["map_y"]])
    n = 0
    if p > 0.3 and len(LOG) > 1 and ALONG[-1] > 0:
        q = min((p - 0.3) / 0.65, 1.0)
        n = max(int(np.searchsorted(ALONG, q * ALONG[-1], side="right")), 2)
        st["driven"].set_data(LOG[:n, 1], LOG[:n, 2])
    return shown, n


scene("goal", 10.0, s_goal, u_goal)

pipe.close()
(OUT.parent / "geo_chapters.json").write_text(json.dumps(CHAPTERS, indent=1))
print(f"[geo] wrote {OUT} ({pipe.frames} frames, {pipe.frames / FPS:.0f} s) "
      "and geo_chapters.json", flush=True)
