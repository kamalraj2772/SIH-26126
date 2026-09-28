"""Data and analysis figures for the SIH26126 technical description PDF.

    cd src/sih_isaac/scripts && ../../../.demoenv/bin/python techdoc_figs_data.py

Every number drawn here is read from the repository: the generated DEM,
GeoTIFF and occupancy map, the Nav2/EKF configs, and the recorded mission
telemetry. Illustrations and reconstructions say so inside the figure.
Also writes build/tech_doc_figs/data_stats.json for the PDF text.
"""
import csv
import heapq
import json
import math
import sys
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.colors import (LightSource, LinearSegmentedColormap,
                               ListedColormap, Normalize)
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle
from matplotlib.ticker import FuncFormatter

import techdoc_style as S

sys.path[:0] = [str(S.WS / "src/sih_sim"), str(S.WS / "src/sih_demo"),
                str(S.WS / "src/sih_isaac")]
from sih_sim import terrain  # noqa: E402
from sih_isaac import world as W  # noqa: E402

WORLD = json.loads((S.GEN / "world.json").read_text())
DEM = np.load(S.GEN / "dem.npy").astype(np.float64)       # north-up
N_DEM = DEM.shape[0]
SIZE = float(WORLD["world_size"])
HALF = SIZE / 2.0
RES = SIZE / (N_DEM - 1)
E0 = WORLD["site_datum"]["anchor_e"]
N0 = WORLD["site_datum"]["anchor_n"]
START = (WORLD["start"]["x"], WORLD["start"]["y"])
GOAL = (WORLD["goal_default"]["x"], WORLD["goal_default"]["y"])
EXT = [-HALF - RES / 2, HALF + RES / 2, -HALF - RES / 2, HALF + RES / 2]

LS = LightSource(azdeg=315, altdeg=40)
HS = LS.hillshade(DEM, vert_exag=2.5, dx=RES, dy=RES)
_gr, _gc = np.gradient(DEM, RES)
SLOPE = np.degrees(np.arctan(np.hypot(_gr, _gc)))          # north-up

ELEV_CMAP = LinearSegmentedColormap.from_list(
    "elev", ["#35604f", "#6f9a5b", "#c9c38a", "#c09a62", "#8a5a36", "#5e3a22"])

# Nav2 costmap constants (nav2_costmap_2d) and our configured values
LETHAL, INSCRIBED = 254, 253
ROBOT_R = 0.55          # robot_radius, nav2_params.yaml
INFL_R = 0.95           # inflation_radius
K_SCALE = 3.0           # cost_scaling_factor
GRES = 0.293            # global_costmap resolution (map res / 2)

NOTE = dict(fontsize=6.5, style="italic", color=S.GREY)
STATS: dict = {}


def fmt_m(v, _=None):
    return f"{v:,.0f}".replace(",", " ")


def dem_at(x, y, arr=DEM):
    """Bilinear sample of a north-up map-frame raster at map (x, y)."""
    c = (np.asarray(x, float) + HALF) / RES
    r = (HALF - np.asarray(y, float)) / RES
    c0 = np.clip(np.floor(c).astype(int), 0, N_DEM - 2)
    r0 = np.clip(np.floor(r).astype(int), 0, N_DEM - 2)
    fc, fr = np.clip(c - c0, 0, 1), np.clip(r - r0, 0, 1)
    top = arr[r0, c0] * (1 - fc) + arr[r0, c0 + 1] * fc
    bot = arr[r0 + 1, c0] * (1 - fc) + arr[r0 + 1, c0 + 1] * fc
    return top * (1 - fr) + bot * fr


def load_run(name):
    with open(S.REC / name / "mission_log.csv") as f:
        rows = list(csv.DictReader(f))
    a = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    ok = ~np.isnan(a["ekf_x"])
    a = {k: v[ok] for k, v in a.items()}
    a["err"] = np.hypot(a["gt_x"] - a["ekf_x"], a["gt_y"] - a["ekf_y"])
    a["dist"] = np.concatenate([[0.0], np.cumsum(np.hypot(
        np.diff(a["gt_x"]), np.diff(a["gt_y"])))])
    a["name"] = name
    return a


RUNS = {"run_final": load_run("run_final"), "run_1080p": load_run("run_1080p")}
RUN_STYLE = {"run_final": dict(color=S.NAVY, label="run_final (headless)"),
             "run_1080p": dict(color=S.ORANGE, label="run_1080p (GUI recording)")}
RECOVERIES = {"run_final": 7, "run_1080p": 1}   # from each run's goal.log


# ------------------------------------------------------------ costmap model
def inflation_cost(d):
    """Nav2 InflationLayer::computeCost for our configured values."""
    d = np.asarray(d, float)
    c = np.where(d <= ROBOT_R, INSCRIBED,
                 np.floor(252.0 * np.exp(-K_SCALE * (d - ROBOT_R))))
    c = np.where(d > INFL_R, 0, c)
    return np.where(d <= 1e-9, LETHAL, c).astype(np.int32)


def global_costmap():
    """Static layer (map.pgm resampled to 0.293 m) + inflation, north-up."""
    pgm = cv2.imread(str(S.GEN / "map.pgm"), cv2.IMREAD_UNCHANGED)
    occ = np.kron((pgm == 0).astype(np.uint8), np.ones((2, 2), np.uint8))
    res = RES / 2.0
    dist = cv2.distanceTransform((1 - occ).astype(np.uint8), cv2.DIST_L2,
                                 cv2.DIST_MASK_PRECISE) * res
    cost = inflation_cost(dist)
    cost[occ > 0] = LETHAL
    return cost, res, pgm


COST, CRES, PGM = global_costmap()
C_N = COST.shape[0]
C_ORIGIN = -HALF - RES / 2.0           # map.yaml origin (-75.293)
C_TOP = C_ORIGIN + C_N * CRES


def to_cell(x, y):
    return (int((C_TOP - y) / CRES), int((x - C_ORIGIN) / CRES))


def to_xy(r, c):
    return (C_ORIGIN + (c + 0.5) * CRES, C_TOP - (r + 0.5) * CRES)


def rviz_cost_cmap():
    cols = np.zeros((256, 4))
    cols[0] = (0.93, 0.93, 0.93, 1)
    ramp = plt.get_cmap("coolwarm")(np.linspace(0.05, 1.0, 252))
    cols[1:253] = ramp
    cols[253] = (0.0, 0.85, 0.95, 1)        # inscribed: cyan
    cols[254] = (0.45, 0.05, 0.45, 1)       # lethal: purple
    cols[255] = (0.44, 0.54, 0.53, 1)
    return ListedColormap(cols)


# ================================================================== figures
def fig_site_map():
    fig = plt.figure(figsize=(6.8, 4.4))
    ax = fig.add_axes([0.115, 0.13, 0.50, 0.83])
    ext = [EXT[0] + E0, EXT[1] + E0, EXT[2] + N0, EXT[3] + N0]
    ax.imshow(HS, cmap="gray", extent=ext, origin="upper", alpha=0.95,
              vmin=0.15, vmax=1.0)
    ax.add_patch(Rectangle((E0 - HALF, N0 - HALF), SIZE, SIZE, fill=False,
                           ec=S.NAVY, lw=1.3, ls="--"))
    ax.text(E0 - HALF + 3, N0 + HALF - 5, "150 m site / GeoTIFF footprint",
            fontsize=6.6, color=S.NAVY, va="top",
            bbox=dict(fc="white", ec="none", alpha=0.8, pad=1))
    # map-frame axes at the anchor
    for dx, dy, lab in ((28, 0, "x (east)"), (0, 28, "y (north)")):
        ax.annotate("", xy=(E0 + dx, N0 + dy), xytext=(E0, N0),
                    arrowprops=dict(arrowstyle="-|>", color=S.RED, lw=1.6,
                                    mutation_scale=10))
        ax.text(E0 + dx + (2 if dx else 1.5), N0 + dy + (0 if dx else 2),
                lab, color=S.RED, fontsize=7, fontweight="bold",
                va="center" if dx else "bottom", ha="left" if dx else "center")
    ax.plot(E0, N0, marker="*", ms=13, color=S.RED, mec="white", mew=0.8,
            zorder=5)
    ax.text(E0 + 2, N0 - 5, "map origin =\nsurveyed datum", fontsize=6.6,
            color=S.RED, va="top", fontweight="bold",
            bbox=dict(fc="white", ec="none", alpha=0.8, pad=1))
    sx, sy = START
    gx, gy = GOAL
    ax.plot(E0 + sx, N0 + sy, marker="^", ms=9, color=S.GREEN, mec="white",
            zorder=5)
    ax.text(E0 + sx + 3, N0 + sy - 3, f"START\nE {E0 + sx:,.2f}\nN {N0 + sy:,.2f}"
            .replace(",", " "), fontsize=6.6, va="top", color=S.GREEN,
            fontweight="bold", bbox=dict(fc="white", ec="none", alpha=0.85, pad=1))
    ax.plot(E0 + gx, N0 + gy, marker="*", ms=12, color=S.ORANGE, mec="white",
            zorder=5)
    ax.text(E0 + gx - 2, N0 + gy + 5, f"GOAL\nE {E0 + gx:,.2f}\nN {N0 + gy:,.2f}"
            .replace(",", " "), fontsize=6.6, va="bottom", ha="right",
            color=S.ORANGE, fontweight="bold",
            bbox=dict(fc="white", ec="none", alpha=0.85, pad=1))
    ax.xaxis.set_major_formatter(FuncFormatter(fmt_m))
    ax.yaxis.set_major_formatter(FuncFormatter(fmt_m))
    ax.set_xticks(np.arange(403450, 403600, 50))
    ax.set_yticks(np.arange(1449925, 1450070, 50))
    ax.tick_params(axis="x", labelsize=6.8)
    ax.tick_params(axis="y", labelsize=6.8)
    ax.set_xlabel("UTM easting E (m)  —  EPSG:32644, zone 44N", fontsize=7.5)
    ax.set_ylabel("UTM northing N (m)", fontsize=7.5)
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])

    # explanatory side panel
    from sih_demo import geo
    from pyproj import Proj
    conv = Proj("EPSG:32644").get_factors(geo.SITE_LON, geo.SITE_LAT)
    STATS["utm"] = dict(
        site_lat=geo.SITE_LAT, site_lon=geo.SITE_LON, epsg=32644,
        anchor_e=round(E0, 3), anchor_n=round(N0, 3),
        pyproj_anchor_e=round(geo.ANCHOR_E, 3),
        pyproj_anchor_n=round(geo.ANCHOR_N, 3),
        utm_zone=geo.utm_zone_for_lon(geo.SITE_LON),
        meridian_convergence_deg=round(conv.meridian_convergence, 4),
        scale_factor=round(conv.meridional_scale, 6),
        start_e=round(E0 + sx, 3), start_n=round(N0 + sy, 3),
        goal_e=round(E0 + gx, 3), goal_n=round(N0 + gy, 3),
        start_goal_dist_m=round(math.hypot(gx - sx, gy - sy), 2))
    tx = 0.655
    lines = [
        ("Surveyed site datum (WGS 84)", True),
        (f"lat {geo.SITE_LAT:.4f}° N,  lon {geo.SITE_LON:.4f}° E", False),
        ("", False),
        ("pyproj, offline, once  →  UTM zone "
         f"{geo.utm_zone_for_lon(geo.SITE_LON)}N", True),
        (f"E₀ = {E0:,.3f} m".replace(",", " "), False),
        (f"N₀ = {N0:,.3f} m".replace(",", " "), False),
        ("", False),
        ("Online (pure arithmetic, no receiver)", True),
        ("map x = E − E₀", False),
        ("map y = N − N₀", False),
        ("", False),
        ("Local grid geometry at the datum", True),
        (f"meridian convergence {conv.meridian_convergence:+.3f}°", False),
        (f"UTM point scale factor {conv.meridional_scale:.5f}", False),
        ("→ map frame = grid north; over 150 m", False),
        ("   a pure translation is exact enough", False),
    ]
    y = 0.93
    for txt, bold in lines:
        fig.text(tx, y, txt, fontsize=7.3 if bold else 7.0,
                 fontweight="bold" if bold else "normal",
                 color=S.NAVY if bold else S.INK)
        y -= 0.052 if txt else 0.028
    return S.save(fig, "site_map")


def fig_dem_features():
    fig = plt.figure(figsize=(6.8, 5.0))
    ax = fig.add_axes([0.075, 0.075, 0.62, 0.9])
    vmin, vmax = -1.6, 4.0
    rgb = LS.shade(DEM, cmap=ELEV_CMAP, blend_mode="soft", vert_exag=2.5,
                   dx=RES, dy=RES, vmin=vmin, vmax=vmax)
    ax.imshow(rgb, extent=EXT, origin="upper")
    lab = dict(fontsize=6.7, color=S.INK, fontweight="bold",
               bbox=dict(fc="white", ec="#BBBBBB", lw=0.5, alpha=0.9,
                         pad=1.5, boxstyle="round,pad=0.25"))
    arr = dict(arrowstyle="-", color="#333333", lw=0.8)

    def note(txt, xy, xyt, **kw):
        ax.annotate(txt, xy=xy, xytext=xyt, arrowprops=arr, ha="center",
                    va="center", **{**lab, **kw})

    ax.text(6, -52, "Berm 3.2 m — full-width wall", rotation=90, ha="center",
            va="center", **lab)
    note("Tunnel bore  y = 25 m\n5.6 m wide, roofed + lit", (6, 25), (-3, 58))
    note("Plateau 8°, 1.1 m", (-30, 20), (-45, 2))
    note("Mound 35°, 4 m\n(impassable)", (35, 30), (50, 56))
    note("Ridge, 12° faces", (-22, -35), (-48, -62))
    note("Ditch 1.5 m deep", (-7, 5), (-30, -12))
    ax.add_patch(Rectangle((10, -40), 30, 30, fill=False, ec="#222222",
                           lw=0.9, ls=(0, (4, 3))))
    ax.text(25, -25, "Low-texture\npatch", ha="center", va="center",
            fontsize=6.7, color="#222222", style="italic")
    trees = np.array([[t["x"], t["y"]] for t in WORLD["trees"]])
    rocks = np.array([[r["x"], r["y"]] for r in WORLD["rocks"]])
    ax.scatter(trees[:, 0], trees[:, 1], s=7, c="#1b5e20", ec="white",
               lw=0.3, zorder=4)
    ax.scatter(rocks[:, 0], rocks[:, 1], s=5, c="#555555", ec="white",
               lw=0.3, zorder=4)
    note(f"Tree grove ({len(trees)})", (-45, 45), (-45, 66))
    note(f"Rock field ({len(rocks)})", (55, -55), (26, -66))
    kinds = {"drum": ("o", "#F2A900"), "crate": ("s", "#8D6E63"),
             "log": ("v", "#5D4037"), "boulder": ("D", "#9E9E9E")}
    for k, (m, c) in kinds.items():
        pts = [(o["x"], o["y"]) for o in WORLD["obstacles"] if o["kind"] == k]
        if pts:
            p = np.array(pts)
            ax.scatter(p[:, 0], p[:, 1], marker=m, s=16, c=c, ec="black",
                       lw=0.5, zorder=6, label=f"{k} ({len(p)})")
    for h in WORLD["humans"]:
        if h["kind"] == "static":
            ax.scatter(h["x"], h["y"], marker="P", s=40, c=S.RED,
                       ec="white", lw=0.6, zorder=7)
        else:
            wp = np.array(h["waypoints"])
            ax.plot(wp[:, 0], wp[:, 1], color=S.RED, lw=1.6, zorder=7)
            ax.annotate("", xy=wp[1], xytext=wp[0], arrowprops=dict(
                arrowstyle="<|-|>", color=S.RED, lw=1.2, mutation_scale=7),
                zorder=7)
    ax.plot(*START, marker="^", ms=10, color=S.GREEN, mec="white", zorder=8)
    ax.plot(*GOAL, marker="*", ms=13, color=S.ORANGE, mec="black", mew=0.5,
            zorder=8)
    ax.plot([START[0], GOAL[0]], [START[1], GOAL[1]], color="#777777", lw=0.9,
            ls=(0, (3, 2)), zorder=5)
    ax.set_xlim(-75, 75)
    ax.set_ylim(-75, 75)
    ax.set_xlabel("map x (m, east)")
    ax.set_ylabel("map y (m, north)")

    cax = fig.add_axes([0.745, 0.075, 0.022, 0.30])
    sm = plt.cm.ScalarMappable(norm=Normalize(vmin, vmax), cmap=ELEV_CMAP)
    cb = fig.colorbar(sm, cax=cax)
    cb.set_label("elevation (m)", fontsize=7.5)
    cb.ax.tick_params(labelsize=7)

    straight = math.hypot(GOAL[0] - START[0], GOAL[1] - START[1])
    handles = [
        Line2D([], [], marker="^", ls="", color=S.GREEN, mec="white", ms=8,
               label="start (−62, 16)"),
        Line2D([], [], marker="*", ls="", color=S.ORANGE, mec="black", ms=10,
               label="goal (56, 34)"),
        Line2D([], [], color="#777777", lw=0.9, ls=(0, (3, 2)),
               label=f"straight line, {straight:.1f} m")]
    for k, (m, c) in kinds.items():
        n = sum(o["kind"] == k for o in WORLD["obstacles"])
        handles.append(Line2D([], [], marker=m, ls="", color=c, mec="black",
                              mew=0.5, ms=5, label=f"{k} ×{n} (not in prior)"))
    handles += [
        Line2D([], [], marker="P", ls="", color=S.RED, mec="white", ms=7,
               label="standing human ×2"),
        Line2D([], [], color=S.RED, lw=1.6, label="patrol route ×2 (0.8, 0.7 m/s)"),
        Line2D([], [], marker="o", ls="", color="#1b5e20", ms=4, label="tree"),
        Line2D([], [], marker="o", ls="", color="#555555", ms=4, label="rock")]
    leg = fig.legend(handles=handles, loc="upper left",
                     bbox_to_anchor=(0.725, 0.975), fontsize=6.8,
                     handlelength=1.6, labelspacing=0.55, frameon=True,
                     facecolor="#FAFAFA", edgecolor="#CCCCCC")
    leg.get_frame().set_linewidth(0.6)
    fig.text(0.725, 0.47, "DEM 257 × 257 px, 0.586 m/px\n"
             f"elevation {DEM.min():.2f} … {DEM.max():.2f} m\n"
             "hillshade: sun az 315°, alt 40°", fontsize=6.7, color=S.GREY,
             va="top")
    STATS["dem"] = dict(size_px=N_DEM, res_m=round(RES, 6),
                        min_m=round(float(DEM.min()), 3),
                        max_m=round(float(DEM.max()), 3),
                        n_trees=len(trees), n_rocks=len(rocks),
                        n_obstacles=len(WORLD["obstacles"]),
                        n_humans=len(WORLD["humans"]))
    return S.save(fig, "dem_features")


def fig_dem_oblique():
    step = 2
    z = DEM[::step, ::step]
    n = z.shape[0]
    xs = np.linspace(-HALF, HALF, n)
    ys = np.linspace(HALF, -HALF, n)             # north-up rows
    X, Y = np.meshgrid(xs, ys)
    alpha = np.radians(58.0)                     # view direction (from SW)
    el = np.radians(24.0)
    vex = 3.5

    def proj(x, y, h):
        u = x * np.sin(alpha) - y * np.cos(alpha)
        depth = x * np.cos(alpha) + y * np.sin(alpha)
        v = depth * np.sin(el) + h * vex * np.cos(el)
        return u, v, depth

    U, V, D = proj(X, Y, z)
    rgb = LightSource(azdeg=300, altdeg=35).shade(
        z, cmap=ELEV_CMAP, blend_mode="soft", vert_exag=2.5, dx=RES * step,
        dy=RES * step, vmin=-1.6, vmax=4.0)
    quads, cols, depths = [], [], []
    for i in range(n - 1):
        for j in range(n - 1):
            quads.append([(U[i, j], V[i, j]), (U[i, j + 1], V[i, j + 1]),
                          (U[i + 1, j + 1], V[i + 1, j + 1]),
                          (U[i + 1, j], V[i + 1, j])])
            cols.append(rgb[i, j, :3])
            depths.append(D[i:i + 2, j:j + 2].mean())
    order = np.argsort(depths)[::-1]
    quads = [quads[k] for k in order]
    cols = np.array(cols)[order]
    fig = plt.figure(figsize=(6.8, 3.0))
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
    ax.add_collection(PolyCollection(quads, facecolors=cols, edgecolors=cols,
                                     linewidths=0.25))
    ax.set_xlim(U.min() - 2, U.max() + 2)
    ax.set_ylim(V.min() - 4, V.max() + 6)
    ax.set_aspect("equal")
    ax.axis("off")

    def pin(x, y, txt, dy=10, color=S.INK, marker=None):
        u, v, _ = proj(x, y, W.height_map_frame(x, y))
        ax.plot([u, u], [v, v + dy], color=color, lw=0.8)
        if marker:
            ax.plot(u, v, marker=marker, color=color, ms=6, mec="white")
        ax.text(u, v + dy + 1, txt, ha="center", va="bottom", fontsize=6.8,
                fontweight="bold", color=color,
                bbox=dict(fc="white", ec="none", alpha=0.85, pad=0.8))

    pin(6, 25, "tunnel notch\n(roofed in simulator)", dy=16)
    pin(35, 30, "mound 35°", dy=12)
    pin(-30, 20, "plateau 8°", dy=13)
    pin(6, -40, "berm 3.2 m", dy=12)
    pin(-22, -35, "ridge 12°", dy=11)
    pin(*START, "START", dy=9, color=S.GREEN, marker="^")
    pin(*GOAL, "GOAL", dy=9, color=S.ORANGE, marker="*")
    ax.text(U.min() + 4, V.min() - 1, "oblique view from the south-west, "
            "vertical exaggeration ×3.5 — rendered from dem.npy (the "
            "GeoTIFF's raster)", **NOTE, va="bottom")
    return S.save(fig, "dem_oblique")


def fig_terrain_synthesis():
    fig, axs = plt.subplots(1, 3, figsize=(6.8, 2.8),
                            gridspec_kw=dict(width_ratios=[1.15, 1, 1.15]))
    fig.subplots_adjust(left=0.085, right=0.99, bottom=0.17, top=0.87,
                        wspace=0.38)
    ax = axs[0]
    x = np.linspace(0, 30, 1500)
    y = np.full_like(x, 12.3)
    amp, cell, total = 0.06, 6.0, np.zeros_like(x)
    cols = ["#9ecae1", "#6baed6", "#3182bd", "#08519c"]
    for k in range(4):
        o = amp * terrain._value_noise_octave(x, y, 42 + k * 97, cell)
        total += o
        ax.plot(x, o * 100, color=cols[k], lw=0.8,
                label=f"{cell:.2f} m, ±{amp * 100:.1f} cm")
        amp *= 0.5
        cell *= 0.45
    ax.plot(x, total * 100, color=S.RED, lw=1.3, label="sum")
    ax.set_xlabel("distance along a line (m)")
    ax.set_ylabel("height (cm)")
    ax.set_ylim(-7.5, 13)
    ax.legend(fontsize=6.5, loc="upper left", ncol=2, borderaxespad=0.2,
              labelspacing=0.2, handlelength=1.1, columnspacing=0.6,
              title="octave cell, amplitude", title_fontsize=6.5)
    ax.text(0.0, 1.05, "(a) Value-noise octaves", transform=ax.transAxes,
            fontsize=9, fontweight="bold", color=S.NAVY)

    ax = axs[1]
    v = np.linspace(-4, 16, 600)
    berm = W._box_profile(v, 2.0, 10.0, 3.0)
    ax.plot(v, berm, color=S.NAVY, lw=1.4)
    ax.set_xlabel("x (m) — berm profile B(x)", color=S.NAVY)
    ax.tick_params(axis="x", colors=S.NAVY)
    ax2 = ax.twiny()
    t = np.linspace(10, 40, 600)
    carve = W._box_profile(t, 25 - 2.8, 25 + 2.8, 1.2)
    ax2.plot(t, carve, color=S.ORANGE, lw=1.3, ls=(0, (4, 2)))
    ax2.set_xlabel("y (m) — tunnel carve C(y)", fontsize=7, color=S.ORANGE,
                   labelpad=2)
    ax2.tick_params(axis="x", labelsize=6.8, colors=S.ORANGE)
    ax.set_ylabel("weight (0 … 1)")
    ax.set_ylim(-0.05, 1.5)
    ax.text(0.5, 0.86, "smoothstep s(t) = 3t² − 2t³", transform=ax.transAxes,
            fontsize=6.6, ha="center", va="top", color=S.GREY)
    ax.text(0.5, 0.96, "h = 3.2·B(x)·B(y)·(1 − C(y))", transform=ax.transAxes,
            fontsize=6.6, ha="center", va="top", color=S.INK)
    ax.text(0.03, 0.75, "(b) Box\nprofiles", transform=ax.transAxes,
            fontsize=8.5, fontweight="bold", color=S.NAVY, va="top")
    ax.text(0.0, 1.30, "(b) Smooth box profiles", transform=ax.transAxes,
            fontsize=9, fontweight="bold", color=S.NAVY)

    ax = axs[2]
    xx = np.linspace(-15, 25, 800)
    h25 = W.height_map_frame(xx, np.full_like(xx, 25.0))
    h35 = W.height_map_frame(xx, np.full_like(xx, 35.0))
    ax.fill_between(xx, -0.5, h35, color="#e8dcc8", zorder=1)
    ax.plot(xx, h35, color="#8a5a36", lw=1.4, label="y = 35 m (berm wall)")
    ax.plot(xx, h25, color=S.NAVY, lw=1.4, label="y = 25 m (tunnel floor)")
    top_z = float(W.height_map_frame(6.0, 35.0))
    roof_x0 = 6.0 - (10 - 2 + 3.0) / 2
    ax.add_patch(Rectangle((roof_x0, top_z + 0.05 - 0.3), 11.0, 0.6,
                           fc="#9e9186", ec="#5e5046", lw=0.6, zorder=3,
                           hatch="////"))
    ax.text(12.3, top_z - 0.1, "roof slab\n(Isaac scene\nonly)", ha="left",
            va="center", fontsize=6.5, color="#5e5046")
    ax.set_ylim(-0.5, 5.6)
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("height (m)")
    ax.legend(fontsize=6.5, loc="upper left", borderaxespad=0.3)
    ax.text(0.0, 1.05, "(c) Berm cross-sections",
            transform=ax.transAxes, fontsize=9, fontweight="bold",
            color=S.NAVY)
    return S.save(fig, "terrain_synthesis")


def fig_geotiff_structure():
    import rasterio
    path = S.GEN / "prior_dem_utm44n.tif"
    with rasterio.open(path) as src:
        prof = src.profile
        tr = src.transform
        crs = src.crs
        bounds = src.bounds
        tags = src.tags()
        desc = src.descriptions[0]
        dtype = src.dtypes[0]
        comp = src.compression.value if src.compression else "none"
        blocks = src.block_shapes[0]
        data = src.read(1)
    same = bool(np.allclose(data, DEM.astype(np.float32)))
    STATS["geotiff"] = dict(
        driver=prof["driver"], width=prof["width"], height=prof["height"],
        dtype=dtype, crs=crs.to_string(), a=tr.a, e=tr.e, c=round(tr.c, 3),
        f=round(tr.f, 3), bounds=[round(b, 3) for b in bounds],
        compression=comp, block=list(blocks), tags=tags, band_desc=desc,
        identical_to_dem_npy=same)

    fig = plt.figure(figsize=(6.8, 3.2))
    ax = fig.add_axes([0.02, 0.03, 0.44, 0.94])
    ax.set_aspect("equal")
    ax.axis("off")
    npx = 4
    for i in range(npx + 1):
        ax.plot([0, npx], [-i, -i], color="#999999", lw=0.8)
        ax.plot([i, i], [0, -npx], color="#999999", lw=0.8)
    for r in range(npx):
        for c in range(npx):
            ax.add_patch(Rectangle((c, -r - 1), 1, 1, fc=ELEV_CMAP(
                (data[r, c] + 1.6) / 5.6), ec="none", alpha=0.35))
            ax.plot(c + 0.5, -r - 0.5, "o", ms=3, color=S.NAVY)
    ax.plot(0, 0, "s", ms=6, color=S.RED, zorder=5)
    ce = f"{tr.c:,.3f}".replace(",", " ")
    cf = f"{tr.f:,.3f}".replace(",", " ")
    ax.annotate(f"upper-left corner (c, f)\nE {ce}\nN {cf}", xy=(0, 0),
                xytext=(-1.2, 0.75), fontsize=6.8, color=S.RED,
                fontweight="bold", va="bottom",
                arrowprops=dict(arrowstyle="-|>", color=S.RED, lw=0.9,
                                mutation_scale=8))
    ax.annotate("pixel (row 0, col 0) centre\n= (E₀ − 75 m, N₀ + 75 m)",
                xy=(0.5, -0.5), xytext=(1.75, 0.75), fontsize=6.8,
                color=S.NAVY, va="bottom",
                arrowprops=dict(arrowstyle="-|>", color=S.NAVY, lw=0.9,
                                mutation_scale=8))
    ax.annotate("col → (+E)", xy=(npx, -npx - 0.35), xytext=(0, -npx - 0.35),
                fontsize=7, va="center", ha="left", color=S.INK,
                arrowprops=dict(arrowstyle="-|>", color=S.INK, lw=0.9))
    ax.annotate("", xy=(-0.35, -npx), xytext=(-0.35, 0),
                arrowprops=dict(arrowstyle="-|>", color=S.INK, lw=0.9))
    ax.text(-0.5, -npx / 2, "row ↓ (−N)", rotation=90, fontsize=7,
            ha="right", va="center")
    ax.annotate("", xy=(npx + 0.25, -npx), xytext=(npx + 0.25, -npx + 1),
                arrowprops=dict(arrowstyle="<->", color=S.GREY, lw=0.8))
    ax.text(npx + 0.35, -npx + 0.5, f"{tr.a:.4f} m", fontsize=6.8,
            color=S.GREY, va="center")
    eq = ("E = c + a·(col + ½),   a = +{:.7f}\n"
          "N = f + e·(row + ½),   e = {:.7f}").format(tr.a, tr.e)
    ax.text(-0.5, -npx - 0.9, eq, fontsize=6.9, va="top", family="DejaVu Sans Mono",
            color=S.INK)
    ax.set_xlim(-1.3, npx + 1.6)
    ax.set_ylim(-npx - 2.0, 2.75)
    ax.text(-1.25, 2.72, "(a) Raster ↔ UTM georeferencing", fontsize=9,
            fontweight="bold", color=S.NAVY, va="top")

    ax = fig.add_axes([0.49, 0.03, 0.5, 0.94])
    ax.axis("off")
    ax.text(0.0, 0.985, "(b) Read back from prior_dem_utm44n.tif",
            fontsize=9, fontweight="bold", color=S.NAVY, va="top",
            transform=ax.transAxes)
    rows = [
        ("driver", f"{prof['driver']}  ({prof['count']} band)"),
        ("size", f"{prof['width']} × {prof['height']} px"),
        ("dtype", f"{dtype} — elevation (m)"),
        ("CRS", crs.to_string()),
        ("", "WGS 84 / UTM zone 44N"),
        ("transform", f"a={tr.a:.7f}  b={tr.b:.0f}"),
        ("", f"c={tr.c:,.3f}".replace(",", " ")),
        ("", f"d={tr.d:.0f}  e={tr.e:.7f}"),
        ("", f"f={tr.f:,.3f}".replace(",", " ")),
        ("bounds E", f"{bounds.left:,.1f} … {bounds.right:,.1f}".replace(",", " ")),
        ("bounds N", f"{bounds.bottom:,.1f} … {bounds.top:,.1f}".replace(",", " ")),
        ("storage", f"{comp}, tiled {blocks[1]}×{blocks[0]}"),
        ("band", desc),
        ("SITE_LAT/LON", f"{tags.get('SITE_LAT')} / {tags.get('SITE_LON')}"),
        ("ANCHOR_E", tags.get("ANCHOR_E")),
        ("ANCHOR_N", tags.get("ANCHOR_N")),
        ("SOURCE", tags.get("SOURCE")),
        ("pixels", "identical to dem.npy" if same else "differ from dem.npy"),
    ]
    y = 0.90
    for k, v in rows:
        ax.text(0.0, y, k, fontsize=6.8, family="DejaVu Sans Mono",
                color=S.BLUE, transform=ax.transAxes, va="top")
        ax.text(0.27, y, v, fontsize=6.8, family="DejaVu Sans Mono",
                color=S.INK, transform=ax.transAxes, va="top")
        y -= 0.0495
    ax.add_patch(Rectangle((-0.02, y + 0.02), 1.02, 0.91 - y, fill=False,
                           ec="#CCCCCC", lw=0.6, transform=ax.transAxes))
    return S.save(fig, "geotiff_structure")


def fig_slope_occupancy():
    fig = plt.figure(figsize=(6.8, 2.7))
    w = 0.26
    xs = [0.06, 0.385, 0.71]
    axs = [fig.add_axes([x, 0.25, w, 0.645]) for x in xs]
    for ax in axs:
        ax.set_aspect("equal")
        ax.tick_params(labelsize=6.5)
    ax = axs[0]
    im = ax.imshow(SLOPE, extent=EXT, origin="upper", cmap="magma_r",
                   vmin=0, vmax=40)
    xs_ = np.linspace(-HALF, HALF, N_DEM)
    ax.contour(xs_, xs_[::-1], SLOPE, levels=[15.0], colors=[S.BLUE],
               linewidths=0.7)
    ax.set_title("(a) Slope from the DEM", loc="left", fontsize=8.5)
    cax = fig.add_axes([xs[0], 0.125, w, 0.028])
    cb = fig.colorbar(im, cax=cax, orientation="horizontal")
    cb.ax.tick_params(labelsize=6.5)
    cb.set_label("slope (°); blue contour = 15° limit", fontsize=6.6,
                 labelpad=1)
    ax = axs[1]
    ax.imshow(PGM, extent=EXT, origin="upper", cmap="gray", vmin=0, vmax=254)
    ax.set_title("(b) map.pgm (static layer)", loc="left", fontsize=8.5)
    blocked = float((PGM == 0).mean())
    ax.text(0.5, -0.13, f"black = occupied (slope > 15°): {blocked * 100:.1f} %"
            " of cells\nwhite = free; drums, humans etc. are absent",
            transform=ax.transAxes, ha="center", va="top", fontsize=6.5,
            color=S.GREY)
    ax = axs[2]
    ext_c = [C_ORIGIN, C_ORIGIN + C_N * CRES, C_ORIGIN, C_TOP]
    ax.imshow(COST, extent=ext_c, origin="upper", cmap=rviz_cost_cmap(),
              vmin=0, vmax=255, interpolation="nearest")
    ax.set_xlim(-10, 22)
    ax.set_ylim(9, 41)
    ax.set_title("(c) Costmap at the tunnel", loc="left", fontsize=8.5)
    ax.text(0.5, -0.13, "reconstruction: static + inflation (0.293 m);\n"
            "cyan = inscribed 253, purple = lethal 254", transform=ax.transAxes,
            ha="center", va="top", fontsize=6.5, color=S.GREY, style="italic")
    for ax in axs:
        ax.set_xlabel("")
    STATS["occupancy"] = dict(
        blocked_fraction=round(blocked, 4),
        slope_p50=round(float(np.median(SLOPE)), 2),
        slope_max=round(float(SLOPE.max()), 1),
        costmap_cells=int(C_N), costmap_res=round(CRES, 6),
        lethal_fraction=round(float((COST == LETHAL).mean()), 4),
        inscribed_fraction=round(float((COST == INSCRIBED).mean()), 4))
    return S.save(fig, "slope_occupancy")


def fig_inflation_curve():
    fig, ax = plt.subplots(figsize=(6.8, 2.3))
    fig.subplots_adjust(left=0.085, right=0.98, bottom=0.2, top=0.95)
    d = np.linspace(0.0, 1.3, 1300)
    c = inflation_cost(d)
    ax.axvspan(0, ROBOT_R, color="#E0F7FA", zorder=0)
    ax.axvspan(ROBOT_R, INFL_R, color="#FFF3E0", zorder=0)
    ax.axvspan(INFL_R, 1.3, color="#F1F8E9", zorder=0)
    ax.plot(d, c, color=S.NAVY, lw=1.8)
    ax.plot([0], [LETHAL], "o", color="purple", ms=5)
    ax.text(0.02, 262, "254 lethal (obstacle cell)", fontsize=6.8,
            color="purple", va="bottom")
    ax.text(ROBOT_R / 2, 200, "253 inscribed\nrobot centre here ⇒\nfootprint touches "
            "obstacle", ha="center", fontsize=6.8, color="#006064")
    ax.text((ROBOT_R + INFL_R) / 2, 200,
            "252·exp(−k·(d − r_ins))\nk = cost_scaling_factor 3.0",
            ha="center", fontsize=6.8, color="#E65100")
    ax.text((INFL_R + 1.3) / 2, 200, "free (0)\nbeyond inflation_radius",
            ha="center", fontsize=6.8, color="#33691E")
    c95 = 252 * math.exp(-K_SCALE * (INFL_R - ROBOT_R))
    ax.annotate(f"{c95:.0f} at 0.95 m", xy=(INFL_R, c95),
                xytext=(INFL_R + 0.1, c95 + 35), fontsize=6.8,
                arrowprops=dict(arrowstyle="-|>", color=S.GREY, lw=0.8,
                                mutation_scale=8))
    ax.set_xlim(0, 1.3)
    ax.set_ylim(0, 290)
    ax.set_xticks([0, 0.25, ROBOT_R, 0.75, INFL_R, 1.1, 1.3])
    ax.set_xticklabels(["0", "0.25", "0.55\nrobot_radius", "0.75",
                        "0.95\ninflation_radius", "1.1", "1.3"])
    ax.set_xlabel("distance from nearest lethal cell d (m)", labelpad=1)
    ax.set_ylabel("costmap cost")
    STATS["inflation"] = dict(robot_radius=ROBOT_R, inflation_radius=INFL_R,
                              cost_scaling_factor=K_SCALE,
                              cost_at_inflation_radius=round(c95, 1))
    return S.save(fig, "inflation_curve")


def astar(cost, start, goal, mult, keep_g=False):
    """8-connected A*, Smac-2D-style traversal cost and Euclidean heuristic."""
    n = cost.shape[0]
    flat = cost.ravel().tolist()
    INF = float("inf")
    g = [INF] * (n * n)
    parent = [-1] * (n * n)
    closed = bytearray(n * n)
    s = start[0] * n + start[1]
    gl = goal[0] * n + goal[1]
    gr, gc = goal
    g[s] = 0.0
    pq = [(math.hypot(start[0] - gr, start[1] - gc), s)]
    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 2 ** 0.5), (-1, 1, 2 ** 0.5), (1, -1, 2 ** 0.5),
            (1, 1, 2 ** 0.5)]
    expanded = 0
    t0 = time.perf_counter()
    while pq:
        _, cur = heapq.heappop(pq)
        if closed[cur]:
            continue
        closed[cur] = 1
        expanded += 1
        if cur == gl:
            break
        r, c = divmod(cur, n)
        gcur = g[cur]
        for dr, dc, step in nbrs:
            rr, cc = r + dr, c + dc
            if rr < 0 or cc < 0 or rr >= n or cc >= n:
                continue
            k = rr * n + cc
            if closed[k]:
                continue
            ck = flat[k]
            if ck >= INSCRIBED:
                continue
            ng = gcur + step * (1.0 + mult * ck / 252.0)
            if ng < g[k]:
                g[k] = ng
                parent[k] = cur
                heapq.heappush(pq, (ng + math.hypot(rr - gr, cc - gc), k))
    dt = time.perf_counter() - t0
    path = [gl]
    while path[-1] != s:
        path.append(parent[path[-1]])
    path.reverse()
    pr = np.array([divmod(p, n) for p in path])
    xy = np.array([to_xy(r, c) for r, c in pr])
    length = float(np.hypot(*np.diff(xy, axis=0).T).sum())
    out = dict(path_xy=xy, expanded=expanded, length=length, time=dt,
               closed=np.frombuffer(bytes(closed), np.uint8).reshape(n, n),
               cost_to_goal=g[gl] * CRES)
    if keep_g:
        out["g"] = np.array(g).reshape(n, n) * CRES
    return out


def fig_astar_search():
    s, gcell = to_cell(*START), to_cell(*GOAL)
    runs = {}
    for m in (1.0, 2.0, 4.0):
        runs[m] = astar(COST, s, gcell, m, keep_g=(m == 1.0))
    base = runs[1.0]
    ty = [p[1] for p in base["path_xy"] if 2.0 <= p[0] <= 10.0]
    detour = {m: bool(not any(2.0 <= p[0] <= 10.0 and abs(p[1] - 25) < 4
                              for p in r["path_xy"])) for m, r in runs.items()}
    STATS["astar"] = {
        f"mult_{m:g}": dict(expanded=r["expanded"],
                            path_length_m=round(r["length"], 2),
                            runtime_s=round(r["time"], 2),
                            path_cost_m=round(r["cost_to_goal"], 2),
                            through_tunnel=not detour[m])
        for m, r in runs.items()}
    STATS["astar"]["tunnel_y_range"] = [round(min(ty), 2), round(max(ty), 2)]
    STATS["astar"]["grid"] = f"{C_N}x{C_N} @ {CRES:.4f} m"
    STATS["astar"]["straight_line_m"] = round(math.hypot(
        GOAL[0] - START[0], GOAL[1] - START[1]), 2)

    fig = plt.figure(figsize=(6.8, 4.0))
    ext_c = [C_ORIGIN, C_ORIGIN + C_N * CRES, C_ORIGIN, C_TOP]
    xl, yl = (-67, 62), (4, 42)
    ah = 0.355
    aw = ah * 4.0 * (xl[1] - xl[0]) / (yl[1] - yl[0]) / 6.8
    a1 = fig.add_axes([0.075, 0.6, aw, ah])
    a2 = fig.add_axes([0.075, 0.18, aw, ah])
    for ax in (a1, a2):
        ax.set_aspect("equal")
        ax.set_xlim(*xl)
        ax.set_ylim(*yl)
        ax.tick_params(labelsize=6.5)
    a1.tick_params(labelbottom=False)
    a1.imshow(COST, extent=ext_c, origin="upper", cmap=rviz_cost_cmap(),
              vmin=0, vmax=255, interpolation="nearest", alpha=0.55)
    cl = np.ma.masked_where(base["closed"] == 0, base["closed"])
    a1.imshow(cl, extent=ext_c, origin="upper", cmap=ListedColormap(["#FFC107"]),
              alpha=0.45, interpolation="nearest")
    p = base["path_xy"]
    a1.plot(p[:, 0], p[:, 1], color=S.NAVY, lw=1.8, label="path, m = 1.0")
    p4 = runs[4.0]["path_xy"]
    a1.plot(p4[:, 0], p4[:, 1], color=S.RED, lw=1.0, ls=(0, (3, 2)),
            label="path, m = 4.0")
    a1.plot(*START, "^", color=S.GREEN, ms=8, mec="white")
    a1.plot(*GOAL, "*", color=S.ORANGE, ms=11, mec="black", mew=0.5)
    a1.legend(handles=[
        Line2D([], [], color=S.NAVY, lw=1.8,
               label=f"m = 1.0: path {base['length']:.1f} m, "
                     + f"{base['expanded']:,} expansions".replace(",", " ")),
        Line2D([], [], color=S.RED, lw=1.0, ls=(0, (3, 2)),
               label=f"m = 4.0: path {runs[4.0]['length']:.1f} m, "
                     + f"{runs[4.0]['expanded']:,} expansions".replace(",", " ")),
        Rectangle((0, 0), 1, 1, fc="#FFC107", alpha=0.45,
                  label="closed set (m = 1.0)")],
        loc="upper left", fontsize=6.5, frameon=True, facecolor="white",
        framealpha=0.92, edgecolor="#CCCCCC", borderaxespad=0.3)
    a1.set_title("(a) A* expansions and path on the global costmap "
                 "(cost multiplier m)", loc="left", fontsize=8.5, pad=3)
    a1.set_ylabel("map y (m)", fontsize=7)
    a2.set_ylabel("map y (m)", fontsize=7)

    if detour[4.0] or detour[2.0]:
        m_d = 4.0 if detour[4.0] else 2.0
        r = runs[m_d]
        a2.imshow(COST, extent=ext_c, origin="upper", cmap=rviz_cost_cmap(),
                  vmin=0, vmax=255, interpolation="nearest", alpha=0.55)
        a2.plot(r["path_xy"][:, 0], r["path_xy"][:, 1], color=S.RED, lw=1.6)
        a2.set_title(f"(b) m = {m_d:g} reproduces the detour", loc="left",
                     fontsize=8.5)
    else:
        gfield = np.where(np.isfinite(base["g"]), base["g"], np.nan)
        im = a2.imshow(gfield, extent=ext_c, origin="upper", cmap="viridis",
                       interpolation="nearest")
        a2.contour(np.linspace(C_ORIGIN + CRES / 2, C_ORIGIN + C_N * CRES - CRES / 2, C_N),
                   np.linspace(C_TOP - CRES / 2, C_ORIGIN + CRES / 2, C_N),
                   np.nan_to_num(gfield, nan=0), levels=np.arange(20, 200, 20),
                   colors="white", linewidths=0.4, alpha=0.8)
        a2.imshow(np.ma.masked_where(COST < LETHAL, COST), extent=ext_c,
                  origin="upper", cmap=ListedColormap(["#B0A6B8"]),
                  interpolation="nearest")
        a2.plot(p[:, 0], p[:, 1], color="white", lw=1.8)
        a2.plot(p[:, 0], p[:, 1], color=S.RED, lw=0.9)
        x0 = 0.075 + aw + 0.015
        cax = fig.add_axes([x0, 0.18, 0.018, ah])
        cb = fig.colorbar(im, cax=cax)
        cb.ax.tick_params(labelsize=6.5)
        cb.set_label("g(n): cost-weighted m from START", fontsize=6.6,
                     labelpad=2)
        a2.set_title("(b) Cost-to-come field g(n) of the same search, "
                     "m = 1.0 (grey = lethal)", loc="left", fontsize=8.5,
                     pad=3)
    a2.plot(*START, "^", color=S.GREEN, ms=8, mec="white")
    a2.plot(*GOAL, "*", color=S.ORANGE, ms=11, mec="black", mew=0.5)
    a2.set_xlabel("map x (m)", fontsize=7, labelpad=1)
    fig.text(0.075, 0.008, "Textbook re-computation on the reconstructed "
             "global costmap (514 × 514, 0.293 m): 8-connected, step·(1 + "
             "m·cost/252),\nEuclidean heuristic, cells ≥ 253 impassable — "
             "mirrors Smac 2D's cost model; not a log of the running planner."
             "\nThe berm spans the whole site, so every m yields the same "
             "tunnel route in this reconstruction.", **NOTE)
    return S.save(fig, "astar_search")


# ------------------------------------------------------------------- MPPI
class MPPIDemo:
    K, T, DT = 1800, 56, 0.067
    VX_STD, WZ_STD = 0.2, 0.4
    VX_MIN, VX_MAX, WZ_MAX = -0.25, 0.9, 1.0
    LAMBDA, GAMMA = 0.3, 0.015
    DRUM = np.array([2.2, 0.45])
    DRUM_R = 0.30
    PATH_END = 4.5

    def __init__(self, seed=3):
        self.rng = np.random.default_rng(seed)
        self.U = np.tile([0.5, 0.0], (self.T, 1))

    def cost_at(self, x, y):
        d = np.hypot(x - self.DRUM[0], y - self.DRUM[1]) - self.DRUM_R
        return inflation_cost(np.maximum(d, 0.0)), d

    def rollout(self, state, V):
        x = np.full(V.shape[0], state[0])
        y = np.full(V.shape[0], state[1])
        th = np.full(V.shape[0], state[2])
        X = np.empty(V.shape[:2])
        Y = np.empty(V.shape[:2])
        for t in range(self.T):
            th = th + V[:, t, 1] * self.DT
            x = x + V[:, t, 0] * np.cos(th) * self.DT
            y = y + V[:, t, 0] * np.sin(th) * self.DT
            X[:, t], Y[:, t] = x, y
        return X, Y, th

    def step(self, state):
        eps = self.rng.normal(size=(self.K, self.T, 2)) * [self.VX_STD,
                                                          self.WZ_STD]
        V = self.U[None] + eps
        V[..., 0] = np.clip(V[..., 0], self.VX_MIN, self.VX_MAX)
        V[..., 1] = np.clip(V[..., 1], -self.WZ_MAX, self.WZ_MAX)
        eps = V - self.U[None]
        X, Y, th = self.rollout(state, V)
        c, d = self.cost_at(X, Y)
        collide = (c >= INSCRIBED).any(axis=1)
        cost_crit = 3.81 * (c / 254.0).mean(axis=1) + 1e6 * collide
        # path is the x-axis from x=0 to PATH_END; PathAlign switches itself
        # off when > 5 % of the path is blocked (max_path_occupancy_ratio)
        px = np.linspace(0, self.PATH_END, 90)
        pc, _ = self.cost_at(px, np.zeros_like(px))
        occ_ratio = float((pc >= INSCRIBED).mean())
        align = 0.0 if occ_ratio > 0.05 else 14.0 * np.abs(Y).mean(axis=1)
        follow = 5.0 * np.hypot(X[:, -1] - self.PATH_END, Y[:, -1])
        ang = np.abs(np.arctan2(-Y[:, -1], self.PATH_END - X[:, -1]) - th)
        ang = np.minimum(ang, 2 * np.pi - ang)
        path_angle = 2.0 * np.where(ang > 1.0, ang, 0.0)
        prefer_fwd = 5.0 * np.maximum(-V[..., 0], 0).sum(axis=1) * self.DT
        ctrl = self.GAMMA * ((self.U[None] / np.array([self.VX_STD ** 2,
                                                         self.WZ_STD ** 2]))
                             * eps).sum(axis=(1, 2))
        S_ = cost_crit + align + follow + path_angle + prefer_fwd + ctrl
        w = np.exp(-(S_ - S_.min()) / self.LAMBDA)
        w /= w.sum()
        self.U = self.U + (w[:, None, None] * eps).sum(axis=0)
        self.U[:, 0] = np.clip(self.U[:, 0], self.VX_MIN, self.VX_MAX)
        self.U[:, 1] = np.clip(self.U[:, 1], -self.WZ_MAX, self.WZ_MAX)
        Xo, Yo, _ = self.rollout(state, self.U[None])
        return dict(X=X, Y=Y, S=S_, w=w, collide=collide, Xo=Xo[0], Yo=Yo[0],
                    occ_ratio=occ_ratio, u0=self.U[0].copy())

    def shift(self):
        self.U = np.vstack([self.U[1:], self.U[-1:]])


def fig_mppi_rollouts():
    mp = MPPIDemo(seed=11)
    st = np.array([0.0, 0.0, 0.0])
    executed = [st[:2].copy()]
    snap, snap_state, snap_k = None, None, None
    min_clear, dt = 9.0, 1 / 15.0
    for k in range(200):
        out = mp.step(st)
        if snap is None:
            snap, snap_state, snap_k = out, st.copy(), k
        v, w = out["u0"]
        st = st + [v * math.cos(st[2] + w * dt) * dt,
                   v * math.sin(st[2] + w * dt) * dt, w * dt]
        executed.append(st[:2].copy())
        min_clear = min(min_clear, float(np.hypot(*(st[:2] - mp.DRUM)))
                        - mp.DRUM_R - ROBOT_R)
        mp.shift()
        if st[0] > mp.PATH_END - 0.4:
            break
    executed = np.array(executed)
    first = snap

    fig = plt.figure(figsize=(6.8, 3.4))
    a = fig.add_axes([0.065, 0.27, 0.56, 0.65])
    b = fig.add_axes([0.715, 0.27, 0.215, 0.65])
    rng = np.random.default_rng(0)
    idx = rng.choice(mp.K, 300, replace=False)
    ok = [i for i in idx if not first["collide"][i]]
    bad = [i for i in idx if first["collide"][i]]
    for i in bad:
        a.plot(first["X"][i], first["Y"][i], color="#F28B82", lw=0.35,
               alpha=0.7, zorder=1)
    Sok = first["S"][ok]
    norm = Normalize(vmin=np.percentile(Sok, 2), vmax=np.percentile(Sok, 95))
    order = np.argsort(Sok)[::-1]
    segs = [np.column_stack([first["X"][ok[j]], first["Y"][ok[j]]])
            for j in order]
    lc = LineCollection(segs, cmap="viridis", norm=norm, lw=0.55,
                        alpha=0.85, zorder=2)
    lc.set_array(Sok[order])
    a.add_collection(lc)
    for r_, fc, al in ((mp.DRUM_R + INFL_R, "#FFE0B2", 0.75),
                       (mp.DRUM_R + ROBOT_R, "#B2EBF2", 0.85),
                       (mp.DRUM_R, "#6A1B9A", 1.0)):
        a.add_patch(Circle(mp.DRUM, r_, fc=fc, ec="none", alpha=al, zorder=3))
    a.annotate("drum r = 0.30 m, with inscribed (cyan)\nand inflation "
               "(orange) rings", xy=(mp.DRUM[0] + 0.3, mp.DRUM[1] + 0.85),
               xytext=(2.75, 1.62), fontsize=6.5, color="#4A148C", zorder=6,
               arrowprops=dict(arrowstyle="-", color="#4A148C", lw=0.6))
    a.plot([0, mp.PATH_END], [0, 0], color="#555555", lw=1.0,
           ls=(0, (5, 3)), zorder=4, label="global path (through inscribed zone)")
    a.plot(first["Xo"], first["Yo"], color=S.RED, lw=2.0, zorder=5,
           label="optimal plan (weighted average of rollouts)")
    a.plot(executed[:, 0], executed[:, 1], color="black", lw=1.2,
           ls=(0, (1.5, 1.2)), zorder=5,
           label=f"executed path at 15 Hz; min footprint clearance "
                 f"{min_clear:.2f} m")
    a.plot(*snap_state[:2], marker=(3, 0, -90 + math.degrees(snap_state[2])),
           ms=9, color=S.GREEN, zorder=6)
    a.set_aspect("equal")
    a.set_xlim(-0.2, 4.6)
    a.set_ylim(-1.55, 1.95)
    a.set_xlabel("x (m)", fontsize=7, labelpad=1)
    a.set_ylabel("y (m)", fontsize=7)
    a.legend(loc="lower left", fontsize=6.5, frameon=True, facecolor="white",
             framealpha=0.92, edgecolor="#CCCCCC", borderaxespad=0.3)
    a.set_title("(a) 300 of 1 800 rollouts, first cycle", loc="left",
                fontsize=8.5)
    cax = fig.add_axes([0.065, 0.1, 0.25, 0.025])
    cb = fig.colorbar(lc, cax=cax, orientation="horizontal")
    cb.ax.tick_params(labelsize=6.5)
    cb.set_label("rollout cost S (collision-free); red = collides",
                 fontsize=6.5, labelpad=1)

    w = np.sort(first["w"])[::-1]
    ess = 1.0 / np.sum(first["w"] ** 2)
    n90 = int(np.searchsorted(np.cumsum(w), 0.9) + 1)
    rank = np.arange(1, len(w) + 1)
    b.loglog(rank, np.maximum(w, 1e-12), color=S.NAVY, lw=1.2)
    b.set_ylim(1e-8, 0.2)
    b.set_xlim(1, mp.K)
    b.set_xlabel("sample rank (cheapest first)", fontsize=7, labelpad=1)
    b.set_ylabel("softmax weight wₖ", fontsize=7, color=S.NAVY)
    b.tick_params(labelsize=6.5)
    b2 = b.twinx()
    b2.semilogx(rank, np.cumsum(w), color=S.ORANGE, lw=1.2)
    b2.set_ylim(0, 1.05)
    b2.set_ylabel("cumulative weight", fontsize=7, color=S.ORANGE)
    b2.tick_params(axis="y", colors=S.ORANGE, labelsize=6.5)
    b.axvline(n90, color=S.GREY, lw=0.8, ls=":")
    b.text(1.4, 3e-8, f"{n90} of {mp.K} samples\ncarry 90 % of the\n"
           f"weight; ESS = {ess:.0f}", fontsize=6.6, color=S.INK,
           va="bottom", bbox=dict(fc="white", ec="none", alpha=0.85, pad=1))
    b.set_title("(b) Weights, λ = 0.3", loc="left", fontsize=8.5)
    fig.text(0.37, 0.085, "Numerical illustration with the configured MPPI "
             "values (DiffDrive, 56 × 0.067 s,\n1 800 samples, σv 0.2, σω 0.4,"
             " λ 0.3, γ 0.015, vx −0.25…0.9, |ω| ≤ 1.0) and\nsimplified "
             "critics (Cost, PathFollow, PathAngle, PreferForward); PathAlign"
             "\nauto-disables: "
             f"{first['occ_ratio'] * 100:.0f} % of the path is blocked "
             "(threshold 5 %).", **NOTE, va="center")
    STATS["mppi_demo"] = dict(
        batch=mp.K, horizon_s=round(mp.T * mp.DT, 3), snapshot_cycle=snap_k,
        ess=round(float(ess), 1), n90=n90,
        collided_fraction=round(float(first["collide"].mean()), 3),
        closed_loop_min_footprint_clearance_m=round(min_clear, 3),
        path_occupancy_ratio=round(first["occ_ratio"], 3),
        drum_xy=mp.DRUM.tolist())
    return S.save(fig, "mppi_rollouts")


# -------------------------------------------------------------------- EKF
def _hs_background(ax, alpha=0.35):
    ax.imshow(HS, cmap="gray", extent=EXT, origin="upper", alpha=alpha,
              vmin=0.1, vmax=1.0, zorder=0)
    ax.add_patch(Rectangle((2, -75), 8, 150, fc="#C8A77E", ec="none",
                           alpha=0.25, zorder=0))
    ax.add_patch(Rectangle((2, 22.2), 8, 5.6, fc="white", ec="#6D4C41",
                           lw=0.8, hatch="////", zorder=1, alpha=0.9))


def _nav_events():
    """Recovery-related events of run_final, nav.log wall clock -> sim time.

    Two behaviour-server actions have unmistakable /cmd_vel signatures in the
    telemetry (Spin: constant 0.8 rad/s from sim 436.2 s; Wait: zero command
    from 954.4 s); they anchor a linear wall->sim mapping for every event.
    """
    import re
    log = (S.REC / "run_final" / "nav.log").read_text().splitlines()
    pats = [("no-progress abort", r"Failed to make progress"),
            ("spin", r"behavior_server\]: Running spin"),
            ("wait", r"behavior_server\]: Running wait"),
            ("patience abort", r"Controller patience exceeded"),
            ("abort", r"\[follow_path\] \[ActionServer\] Aborting handle"),
            ("clear local", r"clear entirely the local_costmap"),
            ("clear global", r"clear entirely the global_costmap"),
            ("goal succeeded", r"bt_navigator\]: Goal succeeded")]
    ev = []
    for line in log:
        m = re.search(r"\[(\d{10}\.\d+)\]", line)
        if not m:
            continue
        for name, pat in pats:
            if re.search(pat, line):
                ev.append((float(m.group(1)), name))
                break
    w_spin = next(w for w, n in ev if n == "spin")
    w_wait = next(w for w, n in ev if n == "wait")
    s_spin, s_wait = 436.2, 954.4
    rate = (s_wait - s_spin) / (w_wait - w_spin)
    out = [(s_spin + (w - w_spin) * rate, n) for w, n in ev]
    return out, rate


def fig_ekf_trajectory():
    fig = plt.figure(figsize=(6.8, 4.6))
    STATS.setdefault("ekf", {})
    for k, (name, top) in enumerate((("run_final", 0.53), ("run_1080p", 0.05))):
        r = RUNS[name]
        ax = fig.add_axes([0.07, top, 0.91, 0.42])
        _hs_background(ax)
        ax.plot(r["gt_x"], r["gt_y"], color="black", lw=1.4, zorder=3,
                label="ground truth (simulator, scoring only)")
        ax.plot(r["ekf_x"], r["ekf_y"], color=RUN_STYLE[name]["color"], lw=1.1,
                ls=(0, (4, 2)), zorder=4,
                label="EKF estimate  /odometry/filtered")
        ax.plot(*START, "^", color=S.GREEN, ms=8, mec="white", zorder=5)
        ax.plot(*GOAL, "*", color=S.ORANGE, ms=11, mec="black", mew=0.5,
                zorder=5)
        ax.text(6, 29.5, "tunnel", ha="center", fontsize=6.6, color="#4E342E",
                zorder=6, bbox=dict(fc="white", ec="none", alpha=0.8, pad=0.5))
        ax.set_xlim(-68, 66)
        ax.set_ylim(6, 44)
        ax.set_aspect("equal")
        ax.tick_params(labelsize=6.5)
        ax.set_ylabel("map y (m)", fontsize=7)
        if k == 1:
            ax.set_xlabel("map x (m)", fontsize=7)
        fe = float(r["err"][-1])
        nrec = RECOVERIES[name]
        ax.set_title(f"({'ab'[k]}) {name}: {r['dist'][-1]:.0f} m driven, "
                     f"{r['t'][-1] - r['t'][0]:.0f} s sim, {nrec} "
                     f"recover{'y' if nrec == 1 else 'ies'} — final EKF "
                     f"error {fe:.2f} m", loc="left", fontsize=8.3)
        ax.legend(loc="lower right", fontsize=6.5, frameon=True,
                  facecolor="white", framealpha=0.9, edgecolor="#CCCCCC",
                  borderaxespad=0.3)
        ins = ax.inset_axes([0.79, 0.42, 0.2, 0.5])
        ins.plot(r["gt_x"], r["gt_y"], color="black", lw=1.3)
        ins.plot(r["ekf_x"], r["ekf_y"], color=RUN_STYLE[name]["color"],
                 lw=1.1, ls=(0, (4, 2)))
        ins.add_patch(Circle(GOAL, 0.7, fill=False, ec=S.ORANGE, lw=0.8,
                             ls=(0, (2, 2))))
        ins.plot(*GOAL, "*", color=S.ORANGE, ms=9, mec="black", mew=0.4)
        g_end = (r["gt_x"][-1], r["gt_y"][-1])
        e_end = (r["ekf_x"][-1], r["ekf_y"][-1])
        ins.plot(*g_end, "o", color="black", ms=4)
        ins.plot(*e_end, "o", color=RUN_STYLE[name]["color"], ms=4)
        ins.plot([g_end[0], e_end[0]], [g_end[1], e_end[1]], color=S.RED,
                 lw=1.0)
        ins.text(e_end[0] - 0.15, e_end[1] + 0.15, f"{fe:.2f} m",
                 color=S.RED, fontsize=6.6, ha="right", va="bottom",
                 fontweight="bold")
        ins.plot([GOAL[0] - 1.9, GOAL[0] - 0.9], [GOAL[1] + 0.7] * 2,
                 color=S.INK, lw=1.2)
        ins.text(GOAL[0] - 1.4, GOAL[1] + 0.78, "1 m", fontsize=6.5,
                 ha="center", va="bottom")
        ins.set_xlim(GOAL[0] - 2.2, GOAL[0] + 1.6)
        ins.set_ylim(GOAL[1] - 2.2, GOAL[1] + 1.4)
        ins.set_aspect("equal")
        ins.set_xticks([])
        ins.set_yticks([])
        ax.text(0.89, 0.93, "arrival (goal tolerance 0.7 m)", fontsize=6.5,
                ha="center", va="bottom", transform=ax.transAxes,
                color=S.GREY)
        ax.indicate_inset_zoom(ins, edgecolor="#555555", lw=0.6)
    return S.save(fig, "ekf_trajectory")


def _err_stats(r):
    e = r["err"]
    return dict(samples=int(len(e)),
                t_start_s=round(float(r["t"][0]), 2),
                t_end_s=round(float(r["t"][-1]), 2),
                distance_m=round(float(r["dist"][-1]), 1),
                duration_s=round(float(r["t"][-1] - r["t"][0]), 1),
                mean_m=round(float(e.mean()), 3),
                median_m=round(float(np.median(e)), 3),
                p95_m=round(float(np.percentile(e, 95)), 3),
                max_m=round(float(e.max()), 3),
                final_m=round(float(e[-1]), 3),
                final_pct_of_distance=round(float(e[-1] / r["dist"][-1] * 100), 3),
                gt_final_to_goal_m=round(float(math.hypot(
                    r["gt_x"][-1] - GOAL[0], r["gt_y"][-1] - GOAL[1])), 3),
                ekf_final_to_goal_m=round(float(math.hypot(
                    r["ekf_x"][-1] - GOAL[0], r["ekf_y"][-1] - GOAL[1])), 3),
                recoveries=RECOVERIES[r["name"]])


def fig_ekf_error():
    fig, axs = plt.subplots(2, 2, figsize=(6.8, 4.6))
    fig.subplots_adjust(left=0.08, right=0.975, bottom=0.1, top=0.94,
                        wspace=0.28, hspace=0.5)
    for name, r in RUNS.items():
        STATS.setdefault("ekf", {})[name] = _err_stats(r)
        st = RUN_STYLE[name]
        axs[0, 0].plot(r["t"], r["err"], color=st["color"], lw=1.0,
                       label=st["label"])
        axs[0, 1].plot(r["dist"], r["err"], color=st["color"], lw=1.0,
                       label=st["label"])
    ax = axs[0, 0]
    ax.set_xlabel("sim time (s)", fontsize=7)
    ax.set_ylabel("‖EKF − ground truth‖ (m)", fontsize=7)
    ax.set_title("(a) Position error vs time", loc="left", fontsize=8.5)
    ax.legend(fontsize=6.5, loc="upper left")
    ax.set_ylim(0, 1.6)
    ax = axs[0, 1]
    dmax = max(r["dist"][-1] for r in RUNS.values())
    dd = np.linspace(0, dmax, 50)
    ax.plot(dd, 0.005 * dd, color=S.GREY, lw=0.8, ls=(0, (4, 2)))
    ax.plot(dd, 0.01 * dd, color=S.GREY, lw=0.8, ls=(0, (1, 1.5)))
    ax.annotate("1 % of distance", xy=(125, 1.25), xytext=(12, 1.38),
                fontsize=6.5, color=S.GREY, va="center",
                arrowprops=dict(arrowstyle="-", color=S.GREY, lw=0.6))
    ax.annotate("0.5 % of distance", xy=(240, 1.2), xytext=(185, 0.45),
                fontsize=6.5, color=S.GREY, va="center",
                arrowprops=dict(arrowstyle="-", color=S.GREY, lw=0.6))
    ax.set_xlabel("distance driven (m, from ground truth)", fontsize=7)
    ax.set_ylabel("error (m)", fontsize=7)
    ax.set_ylim(0, 1.6)
    ax.set_xlim(0, dmax)
    ax.set_title("(b) Position error vs distance", loc="left", fontsize=8.5)
    ax = axs[1, 0]
    r = RUNS["run_final"]
    tt = r["t"]
    ax.plot(tt, r["gt_x"], color="black", lw=1.2, label="x ground truth")
    ax.plot(tt, r["ekf_x"], color=S.NAVY, lw=1.0, ls=(0, (4, 2)),
            label="x EKF")
    ax.plot(tt, r["gt_y"], color="#6D4C41", lw=1.2, label="y ground truth")
    ax.plot(tt, r["ekf_y"], color=S.ORANGE, lw=1.0, ls=(0, (4, 2)),
            label="y EKF")
    ax.set_xlabel("sim time (s)", fontsize=7)
    ax.set_ylabel("map coordinate (m)", fontsize=7)
    ax.set_title("(c) run_final: x(t), y(t)", loc="left", fontsize=8.5)
    ax.legend(fontsize=6.5, ncol=2, loc="lower right", columnspacing=0.8,
              handlelength=1.6)
    ax = axs[1, 1]
    y0 = 0.98
    for name, r in RUNS.items():
        e = np.sort(r["err"])
        ax.plot(e, np.arange(1, len(e) + 1) / len(e),
                color=RUN_STYLE[name]["color"], lw=1.3)
        s_ = STATS["ekf"][name]
        ax.text(1.04, y0, f"{name}\nmean {s_['mean_m']:.2f}  median "
                f"{s_['median_m']:.2f}\n95th {s_['p95_m']:.2f}  max "
                f"{s_['max_m']:.2f}\nfinal {s_['final_m']:.2f} m "
                f"({s_['final_pct_of_distance']:.2f} %\nof {s_['distance_m']:.0f} m)",
                fontsize=6.5, color=RUN_STYLE[name]["color"], va="top",
                transform=ax.transAxes)
        y0 -= 0.52
    ax.set_xlabel("position error (m)", fontsize=7)
    ax.set_ylabel("fraction of samples ≤ error", fontsize=7)
    ax.set_xlim(0, 1.5)
    ax.set_ylim(0, 1.02)
    ax.set_title("(d) Error CDF (5 Hz samples)", loc="left", fontsize=8.5)
    pos = ax.get_position()
    ax.set_position([pos.x0, pos.y0, pos.width * 0.6, pos.height])
    return S.save(fig, "ekf_error")


def _intervals(mask, t):
    out, on = [], None
    for i, m in enumerate(mask):
        if m and on is None:
            on = t[i]
        elif not m and on is not None:
            out.append((on, t[i]))
            on = None
    if on is not None:
        out.append((on, t[-1]))
    return out


def fig_ekf_commands():
    r = RUNS["run_final"]
    t = r["t"]
    ev, rate = _nav_events()
    fig, (a, b) = plt.subplots(2, 1, figsize=(6.8, 2.4), sharex=True)
    fig.subplots_adjust(left=0.085, right=0.985, bottom=0.27, top=0.8,
                        hspace=0.1)
    rev = _intervals(r["cmd_v"] < -0.01, t)
    spin = [s for s, n in ev if n == "spin"][0]
    wait = [s for s, n in ev if n == "wait"][0]
    spin_end = t[(t > spin) & (np.abs(r["cmd_w"] - 0.8) > 0.03)][0]
    stuck = [s for s, n in ev if n == "no-progress abort"]
    pat = [s for s, n in ev if n == "patience abort"]
    other_abort = [s for s, n in ev if n == "abort"
                   and not any(abs(s - q) < 1.0 for q in stuck + pat)]
    goal_t = [s for s, n in ev if n == "goal succeeded"][0]
    for ax in (a, b):
        for t0, t1 in rev:
            ax.axvspan(t0, t1, color="#FFE0B2", lw=0, zorder=0)
        ax.axvspan(min(stuck) - 20, spin, color="#E3E8EC", lw=0, zorder=0)
        ax.axvspan(spin, spin_end, color="#CE93D8", lw=0, zorder=1)
        ax.axvspan(wait, wait + 5.0, color="#90A4AE", lw=0, zorder=1)
        for s_ in stuck + pat + other_abort:
            ax.axvline(s_, color=S.RED, lw=0.8, zorder=2)
        ax.axvline(goal_t, color=S.GREEN, lw=1.2, zorder=2)
    a.plot(t, r["cmd_v"], color=S.NAVY, lw=0.7, zorder=3)
    a.axhline(0, color="#999999", lw=0.5)
    a.set_ylabel("v (m/s)", fontsize=7)
    a.set_ylim(-0.35, 1.0)
    b.plot(t, r["cmd_w"], color=S.TEAL, lw=0.6, zorder=3)
    b.axhline(0, color="#999999", lw=0.5)
    b.set_ylabel("ω (rad/s)", fontsize=7)
    b.set_ylim(-1.1, 1.1)
    b.set_xlabel("sim time (s)", fontsize=7, labelpad=1)
    b.set_xlim(t[0], t[-1] + 4)
    marks = [(np.mean(stuck), "1"), (spin + 3, "2"),
             (other_abort[0] if other_abort else 746.5, "3"),
             (pat[0], "4"), (goal_t - 6, "5")]
    for x_, lab_ in marks:
        a.text(x_, 1.03, lab_, transform=a.get_xaxis_transform(),
               ha="center", va="bottom", fontsize=6.8, fontweight="bold",
               color="white", bbox=dict(boxstyle="circle,pad=0.18",
                                        fc=S.NAVY, ec="none"))
    fig.text(0.085, 0.985,
             f"1  stuck 1 m from a crate: {len(stuck)} no-progress aborts "
             f"({stuck[0]:.0f}–{stuck[-1]:.0f} s)   2  Spin 1.57 rad "
             f"({spin:.0f} s)   3  controller abort ({marks[2][0]:.0f} s)\n"
             f"4  'controller patience exceeded' at {pat[0]:.0f} s and "
             f"{pat[-1]:.0f} s   5  Wait 5 s, then goal succeeded "
             f"({goal_t:.0f} s).   BT recovery count: {RECOVERIES['run_final']}",
             fontsize=6.6, va="top", color=S.INK)
    fig.legend(handles=[
        Rectangle((0, 0), 1, 1, fc="#FFE0B2",
                  label=f"MPPI output v < 0 ({len(rev)}×; vx_min −0.25)"),
        Rectangle((0, 0), 1, 1, fc="#E3E8EC", label="no-progress window"),
        Rectangle((0, 0), 1, 1, fc="#CE93D8", label="Spin"),
        Rectangle((0, 0), 1, 1, fc="#90A4AE", label="Wait"),
        Line2D([], [], color=S.RED, lw=0.8, label="controller abort"),
        Line2D([], [], color=S.GREEN, lw=1.2, label="goal succeeded")],
        loc="lower center", bbox_to_anchor=(0.53, 0.0), fontsize=6.5,
        ncol=6, columnspacing=0.9, handlelength=1.3, frameon=False)
    for ax in (a, b):
        ax.tick_params(labelsize=6.5)
    STATS["commands_run_final"] = dict(
        mppi_reverse_intervals=len(rev),
        mppi_reverse_time_s=round(sum(t1 - t0 for t0, t1 in rev), 1),
        mean_forward_speed=round(float(r["cmd_v"][r["cmd_v"] > 0.05].mean()), 3),
        max_cmd_v=round(float(r["cmd_v"].max()), 3),
        wall_to_sim_rate=round(rate, 4),
        events_sim_s=[(round(s_, 1), n) for s_, n in ev],
        stuck_position=[-22.8, 24.7], stuck_near="crate at (-21.82, 24.58), r 0.45",
        spin_duration_s=round(float(spin_end - spin), 1),
        goal_succeeded_sim_s=round(goal_t, 1),
        bt_recovery_count=RECOVERIES["run_final"])
    return S.save(fig, "ekf_commands")


def fig_deadreckon_reconstruction():
    r = RUNS["run_final"]
    gx, gy = r["gt_x"], r["gt_y"]
    dx, dy = np.diff(gx), np.diff(gy)
    ds = np.hypot(dx, dy)
    course = np.arctan2(dy, dx)
    s = np.concatenate([[0], np.cumsum(ds)])
    rate = math.radians(13.0) / 45.0                  # measured: 13 deg / 45 m
    drift = rate * s[:-1]
    slope = dem_at(gx[:-1], gy[:-1], SLOPE)
    over = np.where(slope > 5.0, 1.30, 1.0)

    def integrate(scale, dth):
        x = gx[0] + np.concatenate([[0], np.cumsum(ds * scale * np.cos(course + dth))])
        y = gy[0] + np.concatenate([[0], np.cumsum(ds * scale * np.sin(course + dth))])
        return np.hypot(x - gx, y - gy)

    e_gyro = integrate(1.0, drift)
    e_wheel = integrate(over, 0.0)
    e_both = integrate(over, drift)
    fig, ax = plt.subplots(figsize=(6.8, 2.9))
    fig.subplots_adjust(left=0.085, right=0.72, bottom=0.2, top=0.95)
    ax.semilogy(s, np.maximum(e_gyro, 1e-3), color=S.RED, lw=1.3,
                label="(i) gyro-only heading, 13°/45 m drift")
    ax.semilogy(s, np.maximum(e_wheel, 1e-3), color=S.ORANGE, lw=1.3,
                label="(ii) wheel distance +30 % on slopes > 5°")
    ax.semilogy(s, np.maximum(e_both, 1e-3), color=S.PURPLE, lw=1.0,
                ls=(0, (4, 2)), label="(i) + (ii) combined")
    ax.semilogy(r["dist"], np.maximum(r["err"], 1e-3), color=S.NAVY, lw=1.5,
                label="(iii) actual EKF: VIO-grade v + absolute yaw")
    ax.set_xlabel("distance driven (m)", fontsize=7)
    ax.set_ylabel("position error (m, log)", fontsize=7)
    ax.set_ylim(1e-2, 300)
    ax.set_xlim(0, s[-1])
    ax.grid(True, which="major", color="#EEEEEE", lw=0.6)
    ax.legend(loc="lower right", fontsize=6.6, frameon=True,
              facecolor="white", framealpha=0.92, edgecolor="#CCCCCC")
    frac = float((ds * (slope > 5.0)).sum() / ds.sum())
    fig.text(0.735, 0.93, f"End of run ({s[-1]:.0f} m):\n"
             f"(i)   {e_gyro[-1]:.1f} m\n(ii)  {e_wheel[-1]:.1f} m\n"
             f"(i+ii) {e_both[-1]:.1f} m\n(iii) {r['err'][-1]:.2f} m\n\n"
             f"{frac * 100:.1f} % of the path lies on\nslope > 5° (DEM)",
             fontsize=6.8, va="top", family="DejaVu Sans Mono", color=S.INK)
    fig.text(0.085, 0.035, "Reconstruction from the recorded run_final path "
             "using the drift rates measured on this platform; not a separate"
             " run.", **NOTE)
    STATS["deadreckon_reconstruction"] = dict(
        distance_m=round(float(s[-1]), 1),
        gyro_only_final_m=round(float(e_gyro[-1]), 2),
        wheel_overcount_final_m=round(float(e_wheel[-1]), 2),
        combined_final_m=round(float(e_both[-1]), 2),
        ekf_final_m=round(float(r["err"][-1]), 3),
        fraction_path_slope_gt5=round(frac, 4),
        gyro_heading_error_end_deg=round(math.degrees(drift[-1]), 1))
    return S.save(fig, "deadreckon_reconstruction")


def fig_stereo_depth_noise():
    B, SIG_D = 0.12, 0.16
    hf = math.radians(102.0)
    f_full = 640.0 / math.tan(hf / 2)
    fovy = 2 * math.atan(math.tan(hf / 2) / (640 / 360))
    f_proto = 180.0 / math.tan(fovy / 2)
    z = np.linspace(0.3, 20, 400)
    fig, ax = plt.subplots(figsize=(6.8, 2.2))
    fig.subplots_adjust(left=0.085, right=0.985, bottom=0.21, top=0.95)
    sp = z ** 2 * SIG_D / (f_proto * B)
    sf = z ** 2 * SIG_D / (f_full * B)
    ax.plot(z, sp, color=S.NAVY, lw=1.5,
            label=f"prototype render 640×360, f = {f_proto:.0f} px")
    ax.plot(z, sf, color=S.BLUE, lw=1.2, ls=(0, (4, 2)),
            label=f"full HD720 1280×720, f = {f_full:.0f} px")
    ax.axhline(0.8, color=S.GREY, lw=0.7, ls=":")
    ax.text(0.5, 0.82, "noise clip 0.8 m (sensors.py)", fontsize=6.5,
            color=S.GREY, va="bottom")
    ax.axvline(8.0, color=S.RED, lw=0.9)
    ax.text(8.15, 1.55, "obstacle evidence\nonly inside 8 m", color=S.RED,
            fontsize=6.6, va="top")
    s10 = 100 * SIG_D / (f_proto * B)
    ax.plot(10, s10, "o", color=S.NAVY, ms=4)
    ax.annotate(f"σz({10} m) = {s10:.2f} m", xy=(10, s10), xytext=(11.3, 0.25),
                fontsize=6.7, arrowprops=dict(arrowstyle="-|>", lw=0.8,
                                              color=S.GREY, mutation_scale=8))
    ax.text(14.5, 1.55, "σz = z²·σd / (f·B)\nB = 0.12 m, σd = 0.16 px",
            fontsize=7, va="top", color=S.INK,
            bbox=dict(fc="white", ec="#CCCCCC", lw=0.5, pad=2))
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 1.7)
    ax.set_xlabel("depth z (m)", fontsize=7)
    ax.set_ylabel("σz (m)", fontsize=7)
    ax.legend(loc="upper left", fontsize=6.6)
    STATS["stereo"] = dict(baseline_m=B, sigma_disp_px=SIG_D,
                           f_proto_px=round(f_proto, 1),
                           f_full_px=round(f_full, 1),
                           sigma_at_10m_proto=round(s10, 3),
                           sigma_at_8m_proto=round(64 * SIG_D / (f_proto * B), 3))
    return S.save(fig, "stereo_depth_noise")


def fig_loop_rates():
    items = [
        ("BT navigator tick (bt_loop_duration 10 ms)", 100, "nav2_params.yaml", S.BLUE),
        ("PhysX physics step", 60, "run_sim.py  physics_dt 1/60", S.GREY),
        ("slam_toolbox map→odom TF (slam:=true only)", 50, "slam_toolbox.yaml", "#BDBDBD"),
        ("Sensor render (camera, RTX lidar)", 30, "run_sim.py  rendering_dt 1/30", S.GREY),
        ("Control tick: /cmd_vel→wheels, /odom_wheel, /imu/data", 30, "run_sim.py  CTRL_DT", S.GREY),
        ("EKF (robot_localization)", 30, "ekf.yaml  frequency", S.GREEN),
        ("MPPI controller", 15, "nav2_params.yaml  controller_frequency", S.NAVY),
        ("Behaviour server (recoveries)", 10, "nav2_params.yaml  cycle_frequency", S.NAVY),
        ("Local costmap update", 8, "nav2_params.yaml", S.NAVY),
        ("Local costmap publish", 5, "nav2_params.yaml", S.NAVY),
        ("Mission logger (telemetry CSV)", 5, "mission_logger.py", S.ORANGE),
        ("Global costmap update / publish", 1, "nav2_params.yaml", S.NAVY),
        ("Global planner (expected rate)", 1, "nav2_params.yaml", S.NAVY),
    ]
    fig, ax = plt.subplots(figsize=(6.8, 2.3 + 0.0))
    fig.subplots_adjust(left=0.43, right=0.985, bottom=0.19, top=0.99)
    ys = np.arange(len(items))[::-1]
    for y, (lab, hz, src, col) in zip(ys, items):
        ax.barh(y, hz, color=col, height=0.62, alpha=0.9)
        ax.text(hz * 1.08, y, f"{hz:g} Hz", va="center", fontsize=6.5,
                color=S.INK)
        ax.text(-0.012, y, lab, transform=ax.get_yaxis_transform(),
                ha="right", va="center", fontsize=6.5, color=S.INK)
        ax.text(1.0, y, src, transform=ax.get_yaxis_transform(), ha="right",
                va="center", fontsize=6.5, color=S.GREY, style="italic")
    ax.set_xscale("log")
    ax.set_xlim(0.8, 1600)
    ax.set_yticks([])
    ax.set_ylim(-0.6, len(items) - 0.4)
    ax.set_xlabel("rate (Hz, log scale)", fontsize=7, labelpad=1)
    ax.tick_params(labelsize=6.5)
    for sp in ("top", "right", "left"):
        ax.spines[sp].set_visible(False)
    return S.save(fig, "loop_rates")


def fig_skid_steer():
    """Top view, x forward (right), y left (up); a CCW (left) turn."""
    fig = plt.figure(figsize=(6.8, 2.6))
    a = fig.add_axes([0.0, 0.0, 0.58, 1.0])
    a.set_aspect("equal")
    a.axis("off")
    a.add_patch(Rectangle((-0.35, -0.25), 0.70, 0.50, fc="#ECEFF1",
                          ec="#455A64", lw=1.2, zorder=2))
    a.text(0, 0.07, "chassis\n0.70 × 0.50 m", ha="center", va="center",
           fontsize=6.5, color="#37474F", zorder=3)
    for wx in (0.4, -0.4):
        for wy in (0.35, -0.35):
            a.add_patch(Rectangle((wx - 0.15, wy - 0.04), 0.30, 0.08,
                                  fc="#212121", ec="none", zorder=3))
    blue = dict(arrowstyle="-|>", color=S.BLUE, lw=1.6, mutation_scale=9)
    for wx in (0.4, -0.4):
        a.annotate("", xy=(wx + 0.12, 0.47), xytext=(wx - 0.12, 0.47),
                   arrowprops=blue, zorder=4)
        a.annotate("", xy=(wx + 0.26, -0.47), xytext=(wx - 0.12, -0.47),
                   arrowprops=blue, zorder=4)
    a.text(-0.6, 0.47, "v_l  (slower)", fontsize=6.6, color=S.BLUE,
           ha="right", va="center")
    a.text(-0.6, -0.47, "v_r  (faster)", fontsize=6.6, color=S.BLUE,
           ha="right", va="center")
    red = dict(arrowstyle="-|>", color=S.RED, lw=1.1, mutation_scale=7)
    for wy in (0.35, -0.35):
        a.annotate("", xy=(0.4, wy + 0.16), xytext=(0.4, wy), arrowprops=red,
                   zorder=5)
        a.annotate("", xy=(-0.4, wy - 0.16), xytext=(-0.4, wy),
                   arrowprops=red, zorder=5)
    a.text(0.6, -0.12, "lateral scrub:\nfront slides +y,\nrear slides −y",
           fontsize=6.5, color=S.RED, ha="left", va="center")
    a.plot([0, 0], [0.26, 0.86], color=S.GREY, lw=0.7, ls=(0, (3, 2)))
    a.plot(0, 0.86, "o", color=S.PURPLE, ms=5)
    a.text(0.06, 0.86, "ICR (turn centre, R = v/ω)", fontsize=6.5,
           color=S.PURPLE, va="center")
    a.add_patch(FancyArrowPatch((-0.17, -0.13), (0.17, -0.13),
                                connectionstyle="arc3,rad=0.45",
                                arrowstyle="-|>", mutation_scale=8,
                                color=S.PURPLE, lw=1.1, zorder=5))
    a.text(0.23, -0.17, "ω", fontsize=8, color=S.PURPLE, fontweight="bold",
           va="center")
    a.annotate("", xy=(-0.75, 0.35), xytext=(-0.75, -0.35),
               arrowprops=dict(arrowstyle="<->", color=S.GREY, lw=0.8))
    a.text(-0.8, 0.0, "track T\n0.70 m", fontsize=6.5, ha="right",
           va="center", color=S.GREY)
    a.annotate("", xy=(0.4, -0.66), xytext=(-0.4, -0.66),
               arrowprops=dict(arrowstyle="<->", color=S.GREY, lw=0.8))
    a.text(0.0, -0.71, "wheelbase 0.80 m", fontsize=6.5, ha="center",
           va="top", color=S.GREY)
    a.annotate("", xy=(1.2, -1.1), xytext=(1.0, -1.1),
               arrowprops=dict(arrowstyle="-|>", color=S.INK, lw=0.8))
    a.annotate("", xy=(1.0, -0.9), xytext=(1.0, -1.1),
               arrowprops=dict(arrowstyle="-|>", color=S.INK, lw=0.8))
    a.text(1.23, -1.1, "x fwd", fontsize=6.5, va="center")
    a.text(1.0, -0.87, "y left", fontsize=6.5, ha="center", va="bottom")
    a.text(-1.28, 1.2, "(a) 4-wheel skid steer, top view (left turn)",
           fontsize=8.5, fontweight="bold", color=S.NAVY, va="top")
    a.text(-1.28, -0.95, "ω_l,r = (v ∓ ω·G·T/2) / r\n"
           "G = SKID_YAW_GAIN 6.5, r = 0.15 m\n"
           "|ω_wheel| ≤ 30 rad/s   (run_sim.py)",
           fontsize=6.7, va="top", color=S.INK, family="DejaVu Sans Mono")
    a.set_xlim(-1.32, 1.62)
    a.set_ylim(-1.45, 1.22)

    b = fig.add_axes([0.675, 0.36, 0.31, 0.54])
    labels = ["stock\nfriction", "calibrated\nG = 2.6", "calibrated\nG = 6.5"]
    vals = [0.06, 0.398, 0.398 * 6.5 / 2.6]
    bars = b.bar(range(3), vals, color=[S.RED, S.ORANGE, S.GREEN], width=0.6)
    bars[2].set_hatch("////")
    bars[2].set_alpha(0.55)
    b.axhline(1.0, color=S.GREY, lw=0.8, ls=":")
    b.text(-0.45, 1.02, "ideal", fontsize=6.5, color=S.GREY, ha="left",
           va="bottom")
    for i, v in enumerate(vals):
        b.text(i, v + 0.03, ("≈" if i != 1 else "") + f"{v:.2f}", ha="center",
               fontsize=6.8)
    b.set_xticks(range(3))
    b.set_xticklabels(labels, fontsize=6.5)
    b.set_ylim(0, 1.2)
    b.set_ylabel("achieved / commanded\nyaw rate", fontsize=6.8)
    b.set_title("(b) Yaw tracking", loc="left", fontsize=8.5)
    b.tick_params(axis="y", labelsize=6.5)
    fig.text(0.62, 0.02, "Stock ≈ 0.06 and G = 2.6 → 0.398 are measured\n"
             "(calibrate_yaw.py); the hatched G = 6.5 bar is a\n"
             "linear scaling of the 2.6 measurement.", **NOTE)
    return S.save(fig, "skid_steer")


FIGS = [fig_site_map, fig_dem_features, fig_dem_oblique, fig_terrain_synthesis,
        fig_geotiff_structure, fig_slope_occupancy, fig_inflation_curve,
        fig_astar_search, fig_mppi_rollouts, fig_ekf_trajectory, fig_ekf_error,
        fig_ekf_commands, fig_deadreckon_reconstruction, fig_stereo_depth_noise,
        fig_loop_rates, fig_skid_steer]


def build(only=None):
    out = {}
    for f in FIGS:
        name = f.__name__[4:]
        if only and name not in only:
            continue
        t0 = time.perf_counter()
        out[name] = f()
        print(f"[figs-data] {name:28s} {time.perf_counter() - t0:5.1f} s")
    p = S.OUT_DIR / "data_stats.json"
    old = json.loads(p.read_text()) if p.exists() else {}
    old.update(STATS)
    p.write_text(json.dumps(old, indent=1, default=str))
    return out


if __name__ == "__main__":
    build(set(sys.argv[1:]) or None)
