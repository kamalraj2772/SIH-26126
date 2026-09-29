#!/usr/bin/env python3
"""QSLAM visual guide: whiteboard-style PNGs + a keyword-only PDF.

    .demoenv/bin/python src/sih_isaac/scripts/make_visual_guide.py \
        [--run mission_recordings/report_1401]

Writes SIH26126_Visual_Guide/NN_name.png (1920x1080 each) and
SIH26126_Visual_Guide.pdf in the workspace root, in the order of the SIH idea
template: 1 solution, 2 technical approach, 3 feasibility, 4 impact,
5 research and references.

Pictures over paragraphs: hand-drawn boxes and arrows, simple drawn icons,
real frames from a recorded run taped onto the board. Content follows the
team's registered QSLAM idea (Downloads/QSLAM_SIH26126_Idea_Presentation.pptx);
measured numbers are Isaac Sim testbed results and are labelled as such.
Every reference link was checked to resolve to the cited work.
"""
import argparse
import math
import pathlib
import subprocess
import sys

import numpy as np
import matplotlib
from matplotlib import font_manager
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import (Circle, FancyArrowPatch, FancyBboxPatch,
                                Polygon, Rectangle, Ellipse, Wedge)
from PIL import Image

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "sih_isaac"))
import video                                                    # noqa: E402

WS = video.WS
PKG = video.PKG
GEN = video.GEN
ap = argparse.ArgumentParser()
ap.add_argument("--run", default="mission_recordings/report_1401",
                help="recorded run to take photos and the route from")
ap.add_argument("--styles", default="all",
                help="comma-separated style folder names, or 'all'")
args = ap.parse_args()
RUN = (WS / args.run).resolve()
ROOT = WS / "SIH26126_Visual_Guide"
PDF = WS / "SIH26126_Visual_Guide.pdf"
ROOT.mkdir(exist_ok=True)
OUT = ROOT                                 # set per style by use_style()
CACHE = ROOT / ".frames"
CACHE.mkdir(exist_ok=True)
FONT = PKG / "assets/fonts/PatrickHand-Regular.ttf"   # the PDF's hand font
W, H = 160.0, 90.0                         # board units (16:9)
SYS = pathlib.Path("/usr/share/fonts/truetype")

# One entry per visual style; the 17 drawings are identical in content and
# layout, the style decides everything about how they look. `font` is a file;
# its text is scaled to the width of Patrick Hand, which the layouts were
# drawn for, so wider fonts never overflow.
STYLES = {
    "01_whiteboard_sketch": dict(
        font=FONT, sketch=(0.8, 90, 2.0), weight="normal",
        colors=dict(INK="#222222", BLUE="#1f5fbf", RED="#d23c3c", GREEN="#23875a",
                    ORANGE="#e07b00", PURPLE="#7a3fa0", TEAL="#138d90",
                    GREY="#8a8a8a", BOARD="#fdfdfb"),
        fill=0.10, lw=3.2, title="underline", tape=True),
    "02_modern_infographic": dict(
        font=SYS / "lato/Lato-Regular.ttf", bold=SYS / "lato/Lato-Bold.ttf",
        sketch=None, weight="bold",
        colors=dict(INK="#1f2a37", BLUE="#2563eb", RED="#dc2626", GREEN="#16a34a",
                    ORANGE="#ea8a0c", PURPLE="#7c3aed", TEAL="#0d9488",
                    GREY="#6b7280", BOARD="#f6f8fb"),
        fill=0.06, lw=2.2, header=True, shadow=0.07, title="accent"),
    "03_blueprint": dict(
        font=SYS / "dejavu/DejaVuSansCondensed.ttf",
        bold=SYS / "dejavu/DejaVuSansCondensed-Bold.ttf", sketch=None,
        weight="normal", upper_titles=True,
        colors=dict(INK="#eaf3ff", BLUE="#8fd0ff", RED="#ffa3a3", GREEN="#a8f5c8",
                    ORANGE="#ffd98a", PURPLE="#dcc2ff", TEAL="#8ff5ec",
                    GREY="#9fbbd9", BOARD="#0d3b66"),
        fill=0.05, lw=1.8, grid=True, title="blueprint", titleblock=True),
    "04_dark_neon": dict(
        font=SYS / "ubuntu/Ubuntu-R.ttf", sketch=None, weight="normal",
        colors=dict(INK="#e6edf3", BLUE="#3ea6ff", RED="#ff4d6d", GREEN="#3dffa0",
                    ORANGE="#ffae3d", PURPLE="#c77dff", TEAL="#2ef2e0",
                    GREY="#8b98a9", BOARD="#0b0f17"),
        fill=0.10, lw=2.2, glow=True, dots=True, title="glow"),
    "05_chalkboard": dict(
        font=PKG / "assets/fonts/ArchitectsDaughter-Regular.ttf",
        sketch=(1.3, 60, 3.0), weight="normal",
        colors=dict(INK="#f4f4ec", BLUE="#9cc9ff", RED="#ffa8a8", GREEN="#bdf5ad",
                    ORANGE="#ffd28f", PURPLE="#e2bdff", TEAL="#a8f0e8",
                    GREY="#b9c2b5", BOARD="#27342d"),
        fill=0.05, lw=3.0, chalk=True, title="underline", tape=False, frame=True),
    "06_flat_material": dict(
        font=SYS / "open-sans/OpenSans-Regular.ttf",
        bold=SYS / "open-sans/OpenSans-Bold.ttf", sketch=None, weight="bold",
        colors=dict(INK="#263238", BLUE="#1e88e5", RED="#e53935", GREEN="#43a047",
                    ORANGE="#fb8c00", PURPLE="#8e24aa", TEAL="#00897b",
                    GREY="#78909c", BOARD="#eceff1"),
        fill=0.16, lw=0, shadow=0.16, accent_bar=True, title="band"),
}
ST = {}                                    # the active style
FSCALE = 1.0
INK = BLUE = RED = GREEN = ORANGE = PURPLE = TEAL = GREY = BOARD = None


def _font_width(path, sample="Map anchoring: satellite / DEM map, 30 Hz"):
    from matplotlib.textpath import TextPath
    fp = font_manager.FontProperties(fname=str(path), size=24)
    return TextPath((0, 0), sample, prop=fp).get_extents().width


def use_style(name):
    """Make `name` the active style: colours, font, sketchiness, output dir."""
    global ST, FSCALE, OUT
    ST = STYLES[name]
    globals().update(ST["colors"])
    for f in (ST["font"], ST.get("bold")):
        if f:
            font_manager.fontManager.addfont(str(f))
    fam = font_manager.FontProperties(fname=str(ST["font"])).get_name()
    FSCALE = min(1.0, _font_width(FONT) / _font_width(ST["font"]))
    matplotlib.rcParams.update({
        "font.family": fam, "path.sketch": ST["sketch"],
        "text.color": INK, "axes.labelcolor": INK, "axes.edgecolor": INK,
        "xtick.color": INK, "ytick.color": INK})
    OUT = ROOT / name
    OUT.mkdir(exist_ok=True)


def on_colour():
    """Text colour on a solid accent fill: dark styles use light accents."""
    dark = np.mean(matplotlib.colors.to_rgb(BOARD)) < 0.5
    return BOARD if dark else "white"


def fs(size):
    return size * FSCALE


def C(value, name):
    """A colour argument, defaulting to the active style's `name` colour."""
    return value if value is not None else globals()[name]


# ------------------------------------------------------------ primitives ---
def board(title, sub=None):
    fig = Figure(figsize=(16, 9), dpi=120, facecolor=BOARD)
    FigureCanvasAgg(fig)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")
    ax.set_facecolor(BOARD)
    _backdrop(ax)
    if ST.get("upper_titles"):
        title, sub = title.upper(), (sub or "").upper() or None
    kind = ST.get("title")
    tcol, scol, weight = INK, GREY, ST.get("weight", "normal")
    tx = 5
    if kind == "band":                       # flat: title on a full-width bar
        ax.add_patch(Rectangle((0, 78), W, 12, facecolor=BLUE, edgecolor="none",
                               zorder=0.5))
        tcol = scol = "white"
    elif kind == "accent":                   # modern: colour tab before title
        ax.add_patch(FancyBboxPatch((4.2, 80.3), 1.4, 6.4, boxstyle="round,pad=0.3",
                                    facecolor=BLUE, edgecolor="none"))
        tx = 8
    tsz = 38 if ST.get("upper_titles") else 46      # capitals run wider
    t = ax.text(tx, 83.5, title, fontsize=fs(tsz), color=tcol, va="center",
                weight=weight, zorder=5)
    if kind == "glow":
        _glow(t, BLUE)
    if kind in ("underline", "blueprint"):
        fig.canvas.draw()
        bb = t.get_window_extent().transformed(ax.transData.inverted())
        ax.plot([5, bb.x1], [79.3, 79.5], color=BLUE, lw=5 if kind == "underline" else 1.6,
                solid_capstyle="round")
        if kind == "blueprint":
            ax.plot([5, bb.x1], [78.4, 78.4], color=BLUE, lw=0.9)
    if ST.get("titleblock"):                 # engineering-drawing title block
        ax.add_patch(Rectangle((101, 79.8), 57, 7.6, fill=False, edgecolor=INK,
                               lw=1.3, zorder=6))
        ax.plot([144, 144], [79.8, 87.4], color=INK, lw=1.3, zorder=6)
        # ~38 condensed capitals fit the cell at size 16; shrink longer ones
        ax.text(122.5, 83.6, sub or "",
                fontsize=fs(16) * min(1.0, 38 / max(len(sub or ""), 1)), color=scol,
                ha="center", va="center", zorder=6)
        ax.text(151, 83.6, f"SHEET {len(PNGS) + 1:02d}/17", fontsize=fs(16),
                color=INK, ha="center", va="center", zorder=6)
    elif sub:
        ax.text(W - 5, 83.5, sub, fontsize=fs(24), color=scol, va="center",
                ha="right", zorder=5)
    return fig, ax


def _glow(artist, color):
    from matplotlib import patheffects as pe
    artist.set_path_effects([pe.withStroke(linewidth=9, foreground=tint(color, 0.35)),
                             pe.Normal()])


def _backdrop(ax):
    """Style decoration drawn under everything."""
    if ST.get("grid"):                                   # blueprint paper
        for v in np.arange(0, W + 0.1, 2.5):
            ax.plot([v, v], [0, H], color=INK, lw=0.6 if v % 10 else 1.1,
                    alpha=0.08 if v % 10 else 0.16, zorder=0.1)
        for v in np.arange(0, H + 0.1, 2.5):
            ax.plot([0, W], [v, v], color=INK, lw=0.6 if v % 10 else 1.1,
                    alpha=0.08 if v % 10 else 0.16, zorder=0.1)
    if ST.get("dots"):                                   # neon dot grid
        xs, ys = np.meshgrid(np.arange(2, W, 4), np.arange(2, H, 4))
        ax.scatter(xs.ravel(), ys.ravel(), s=2.5, color=GREY, alpha=0.25,
                   zorder=0.1, linewidths=0)
    if ST.get("chalk"):                                  # chalk dust texture
        rng = np.random.default_rng(7)
        dust = rng.random((180, 320))
        ax.imshow(dust, extent=(0, W, 0, H), cmap="Greys_r", alpha=0.06,
                  zorder=0.1, aspect="auto", interpolation="bilinear")
    if ST.get("frame"):                                  # wooden frame
        ax.add_patch(Rectangle((0.3, 0.3), W - 0.6, H - 0.6, fill=False,
                               edgecolor="#8b5a2b", lw=14, zorder=6))


def save(fig, name):
    p = OUT / f"{name}.png"
    fig.savefig(p, dpi=120, facecolor=BOARD)
    PNGS.append(p)
    print("wrote", p.relative_to(ROOT))
    return p


def card(ax, x, y, w, h, color, title=None, lines=(), tsize=30, lsize=22,
         fill=None, align="center"):
    fill = ST["fill"] if fill is None else fill
    box = dict(boxstyle="round,pad=0.6")
    if ST.get("shadow"):
        ax.add_patch(FancyBboxPatch((x + 0.7, y - 0.8), w, h, facecolor="black",
                                    edgecolor="none", alpha=ST["shadow"],
                                    zorder=0.7, **box))
    if ST.get("glow"):
        for lw_, a in ((11, 0.07), (6.5, 0.16)):
            ax.add_patch(FancyBboxPatch((x, y), w, h, fill=False, edgecolor=color,
                                        lw=lw_, alpha=a, zorder=0.75, **box))
    ax.add_patch(FancyBboxPatch((x, y), w, h, facecolor=tint(color, fill),
                                edgecolor=color if ST["lw"] else "none",
                                lw=ST["lw"], zorder=0.8, **box))
    if ST.get("accent_bar"):
        ax.add_patch(Rectangle((x - 0.6, y - 0.6), 1.5, h + 1.2, facecolor=color,
                               edgecolor="none", zorder=0.85))
    if ST.get("titleblock"):                  # blueprint corner ticks
        for cx_, cy_ in ((x, y), (x + w, y), (x, y + h), (x + w, y + h)):
            ax.plot([cx_ - 1.2, cx_ + 1.2], [cy_, cy_], color=color, lw=1.2, zorder=0.9)
            ax.plot([cx_, cx_], [cy_ - 1.2, cy_ + 1.2], color=color, lw=1.2, zorder=0.9)
    ty = y + h - 4.2
    if title:
        band = tsize * 0.20 + 4.0
        if ST.get("header"):                  # modern: tinted header strip
            ax.add_patch(Rectangle((x - 0.35, y + h - band + 0.9), w + 0.7, band - 0.3,
                                   facecolor=tint(color, 0.16), edgecolor="none",
                                   zorder=0.82))
        if ST.get("upper_titles"):
            title = title.upper()
        ax.text(x + w / 2, ty, title, fontsize=fs(tsize), color=color,
                ha="center", va="center", zorder=5, weight=ST.get("weight", "normal"))
        ty -= tsize * 0.20 + 2.6
    for ln in lines:
        if align == "center":
            ax.text(x + w / 2, ty, ln, fontsize=fs(lsize), color=INK,
                    ha="center", va="center", zorder=5)
        else:
            ax.text(x + 2.5, ty, ln, fontsize=fs(lsize), color=INK, ha="left",
                    va="center", zorder=5)
        ty -= lsize * 0.19 + 1.6


def tint(color, amount):
    """Opaque pastel: `amount` of colour over the board."""
    c = np.array(matplotlib.colors.to_rgb(color))
    b = np.array(matplotlib.colors.to_rgb(BOARD))
    return tuple(amount * c + (1 - amount) * b)


def card_height(n_lines, title=True, tsize=30, lsize=22):
    """Height that fits a card's title and n lines (board units)."""
    return 4.2 + (tsize * 0.20 + 2.6 if title else 0) + n_lines * (lsize * 0.19 + 1.6) + 1.5


def arrow(ax, a, b, color=None, lw=3.2, rad=0.0, ms=28, style="-|>"):
    color = C(color, "INK")
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle=style, mutation_scale=ms,
                                 lw=lw, color=color,
                                 connectionstyle=f"arc3,rad={rad}",
                                 shrinkA=2, shrinkB=2))


def text(ax, x, y, s, size=24, color=None, ha="center", va="center", **kw):
    kw.setdefault("zorder", 5)
    ax.text(x, y, s, fontsize=fs(size), color=C(color, "INK"), ha=ha, va=va, **kw)


def tick(ax, x, y, s=2.2, color=None):
    color = C(color, "GREEN")
    ax.plot([x - s, x - s * 0.3, x + s * 1.1], [y, y - s * 0.8, y + s * 1.1],
            color=color, lw=5, solid_capstyle="round")


def cross(ax, x, y, s=2.0, color=None):
    color = C(color, "RED")
    ax.plot([x - s, x + s], [y - s, y + s], color=color, lw=5,
            solid_capstyle="round")
    ax.plot([x - s, x + s], [y + s, y - s], color=color, lw=5,
            solid_capstyle="round")


def badge(ax, x, y, n, color):
    ax.add_patch(Circle((x, y), 2.6, facecolor=color, edgecolor=color, lw=2))
    text(ax, x, y - 0.2, str(n), 24, on_colour())


# --------------------------------------------------------------- icons -----
def robot(ax, x, y, s=1.0, color=None):
    color = C(color, "BLUE")
    ax.add_patch(FancyBboxPatch((x - 5 * s, y), 10 * s, 4 * s,
                                boxstyle="round,pad=0.3", facecolor=color,
                                edgecolor=INK, lw=2.5))
    for dx in (-3.6, 3.6):
        ax.add_patch(Circle((x + dx * s, y - 0.2 * s), 1.7 * s,
                            facecolor=INK, edgecolor=INK))
    ax.plot([x, x], [y + 4 * s, y + 7.5 * s], color=INK, lw=3)
    ax.add_patch(Rectangle((x - 1.6 * s, y + 7.5 * s), 3.2 * s, 1.6 * s,
                           facecolor=GREY, edgecolor=INK, lw=2))


def satellite(ax, x, y, s=1.0, color=None):
    color = C(color, "GREY")
    ax.add_patch(Rectangle((x - 1.5 * s, y - 1.5 * s), 3 * s, 3 * s,
                           facecolor=color, edgecolor=INK, lw=2.5))
    for dx in (-6.5, 2.5):
        ax.add_patch(Rectangle((x + dx * s, y - 1 * s), 4 * s, 2 * s,
                               facecolor=tint(BLUE, 0.45), edgecolor=INK, lw=2))


def camera(ax, x, y, s=1.0, color=None):
    color = C(color, "TEAL")
    ax.add_patch(FancyBboxPatch((x - 3 * s, y - 2 * s), 6 * s, 4 * s,
                                boxstyle="round,pad=0.3", facecolor=color,
                                edgecolor=INK, lw=2.5))
    ax.add_patch(Circle((x, y), 1.3 * s, facecolor="white", edgecolor=INK,
                        lw=2.5))


def mapsheet(ax, x, y, s=1.0, color=None):
    color = C(color, "GREEN")
    xs = [x - 5 * s, x - 1.7 * s, x + 1.7 * s, x + 5 * s]
    for i in range(3):
        ax.add_patch(Polygon([(xs[i], y - 3.5 * s + (i % 2) * 0.8 * s),
                              (xs[i + 1], y - 3.5 * s + ((i + 1) % 2) * 0.8 * s),
                              (xs[i + 1], y + 3.5 * s + ((i + 1) % 2) * 0.8 * s),
                              (xs[i], y + 3.5 * s + (i % 2) * 0.8 * s)],
                             facecolor=matplotlib.colors.to_rgba(color, 0.25 + 0.15 * (i % 2)),
                             edgecolor=INK, lw=2))
    ax.plot([x - 3 * s, x, x + 3 * s], [y - 1 * s, y + 1.5 * s, y - 0.5 * s],
            color=RED, lw=2.5, ls="--")


def flag(ax, x, y, s=1.0, color=None):
    color = C(color, "RED")
    ax.plot([x, x], [y, y + 7 * s], color=INK, lw=3)
    ax.add_patch(Polygon([(x, y + 7 * s), (x + 4.5 * s, y + 5.7 * s),
                          (x, y + 4.4 * s)], facecolor=color, edgecolor=INK,
                         lw=2))


def pin(ax, x, y, s=1.0, color=None):
    color = C(color, "RED")
    ax.add_patch(Wedge((x, y + 3 * s), 2.2 * s, -20, 200, facecolor=color,
                       edgecolor=INK, lw=2))
    ax.add_patch(Polygon([(x - 2.05 * s, y + 2.3 * s), (x, y),
                          (x + 2.05 * s, y + 2.3 * s)], facecolor=color,
                         edgecolor=INK, lw=2))
    ax.add_patch(Circle((x, y + 3.1 * s), 0.8 * s, facecolor="white",
                        edgecolor=INK, lw=1.5))


def shield(ax, x, y, s=1.0, color=None):
    color = C(color, "GREEN")
    ax.add_patch(Polygon([(x - 3 * s, y + 3 * s), (x + 3 * s, y + 3 * s),
                          (x + 3 * s, y), (x, y - 3.5 * s), (x - 3 * s, y)],
                         facecolor=matplotlib.colors.to_rgba(color, 0.25),
                         edgecolor=color, lw=3))
    tick(ax, x, y + 0.2 * s, 1.2 * s, color)


def warn(ax, x, y, s=1.0, color=None):
    color = C(color, "ORANGE")
    ax.add_patch(Polygon([(x - 3 * s, y - 2.5 * s), (x + 3 * s, y - 2.5 * s),
                          (x, y + 2.8 * s)], facecolor=matplotlib.colors.to_rgba(color, 0.3),
                         edgecolor=color, lw=3))
    text(ax, x, y - 0.6 * s, "!", 26 * s, color)


def tree(ax, x, y, s=1.0):
    ax.plot([x, x], [y, y + 3 * s], color="#7a4b1e", lw=4)
    ax.add_patch(Circle((x, y + 5 * s), 2.6 * s, facecolor="#8fcf8f",
                        edgecolor=GREEN, lw=2))


def tunnel(ax, x, y, s=1.0):
    ax.add_patch(Rectangle((x - 7 * s, y), 14 * s, 7 * s, facecolor="#d9c9a8",
                           edgecolor=INK, lw=2.5))
    ax.add_patch(Wedge((x, y), 3.6 * s, 0, 180, facecolor="#333333",
                       edgecolor=INK, lw=2))


def bolt(ax, x, y, s=1.0, color=None):
    color = C(color, "ORANGE")
    ax.add_patch(Polygon([(x - 0.8 * s, y + 3 * s), (x + 1.2 * s, y + 3 * s),
                          (x + 0.1 * s, y + 0.6 * s), (x + 1.6 * s, y + 0.6 * s),
                          (x - 1.2 * s, y - 3.2 * s), (x - 0.2 * s, y - 0.2 * s),
                          (x - 1.6 * s, y - 0.2 * s)], facecolor=color,
                         edgecolor=INK, lw=2))


def person(ax, x, y, s=1.0, color=None):
    color = C(color, "INK")
    ax.add_patch(Circle((x, y + 6 * s), 1.3 * s, facecolor="white",
                        edgecolor=color, lw=2.5))
    ax.plot([x, x], [y + 4.7 * s, y + 1.8 * s], color=color, lw=3)
    ax.plot([x - 2 * s, x, x + 2 * s], [y + 3.2 * s, y + 4.2 * s, y + 3.2 * s],
            color=color, lw=3)
    ax.plot([x - 1.6 * s, x, x + 1.6 * s], [y, y + 1.8 * s, y], color=color,
            lw=3)


def chip(ax, x, y, s=1.0, color=None):
    color = C(color, "PURPLE")
    ax.add_patch(Rectangle((x - 3 * s, y - 3 * s), 6 * s, 6 * s,
                           facecolor=matplotlib.colors.to_rgba(color, 0.25),
                           edgecolor=color, lw=3))
    for k in (-1.5, 0, 1.5):
        for (a, b, c, d) in ((x + k * s, x + k * s, y + 3 * s, y + 4.3 * s),
                             (x + k * s, x + k * s, y - 3 * s, y - 4.3 * s),
                             (x - 3 * s, x - 4.3 * s, y + k * s, y + k * s),
                             (x + 3 * s, x + 4.3 * s, y + k * s, y + k * s)):
            ax.plot([a, b], [c, d], color=color, lw=2.5)


def photo(ax, img, x, y, w, h=None, tape=True, border=None):
    """Tape a picture onto the board at (x, y) lower-left, width w."""
    border = C(border, "INK")
    if isinstance(img, (str, pathlib.Path)):
        img = np.asarray(Image.open(img).convert("RGB"))
    ih, iw = img.shape[:2]
    h = h or w * ih / iw
    ax.imshow(img, extent=(x, x + w, y, y + h), zorder=2, aspect="auto",
              interpolation="bilinear")
    ax.add_patch(Rectangle((x, y), w, h, fill=False, edgecolor=border,
                           lw=3 if ST.get("tape") or ST.get("chalk") else 2,
                           zorder=3))
    if ST.get("glow"):
        ax.add_patch(Rectangle((x, y), w, h, fill=False, edgecolor=BLUE, lw=9,
                               alpha=0.25, zorder=2.9))
    if ST.get("shadow"):
        ax.add_patch(Rectangle((x + 0.7, y - 0.8), w, h, facecolor="black",
                               alpha=ST["shadow"], edgecolor="none", zorder=1.9))
    if tape and ST.get("tape"):
        for cx, cy, ang in ((x + 1.5, y + h - 0.5, 35), (x + w - 1.5, y + h - 0.5, -35)):
            t = Rectangle((cx - 3, cy - 0.9), 6, 1.8, angle=ang,
                          rotation_point="center", facecolor="#f5e27a",
                          edgecolor="none", alpha=0.85, zorder=4)
            ax.add_patch(t)
    return h


# ------------------------------------------------------------- real data ---
def frame(name, src, t):
    p = CACHE / f"{name}.png"
    if not p.exists():
        subprocess.run([video.ffmpeg_bin(), "-nostdin", "-loglevel", "error",
                        "-y", "-ss", str(t), "-i", str(RUN / src),
                        "-frames:v", "1", "-vf", "scale=960:540", str(p)],
                       check=True)
    return p


DEM = np.load(GEN / "dem.npy")
RES = video.world()["world_size"] / (DEM.shape[0] - 1)


def hillshade(z):
    dy, dx = np.gradient(z.astype(float), RES)
    sl = np.arctan(np.hypot(dx, dy))
    asp = np.arctan2(-dx, dy)
    a, h = math.radians(315), math.radians(45)
    return np.clip(math.sin(h) * np.cos(sl) + math.cos(h) * np.sin(sl)
                   * np.cos(a - asp), 0, 1)


def slope():
    dy, dx = np.gradient(DEM.astype(float), RES)
    return np.degrees(np.arctan(np.hypot(dx, dy)))


def cmap_img(arr, cmap, vmin=None, vmax=None):
    m = matplotlib.colormaps[cmap]
    lo = arr.min() if vmin is None else vmin
    hi = arr.max() if vmax is None else vmax
    return (m(np.clip((arr - lo) / max(hi - lo, 1e-9), 0, 1))[..., :3]
            * 255).astype(np.uint8)


HS = hillshade(DEM)
SL = slope()
LOG = video.read_log(RUN)
WORLD = video.world()
STATS = video.stats(RUN)
PNGS = []


# =================================================== 1. THE SOLUTION ======
def img_problem():
    fig, ax = board("Where GPS fails - and QSLAM does not",
                    "1 · the problem")
    # left: GPS-dependent vehicle
    card(ax, 4, 8, 70, 66, RED, "Today: GPS-dependent UGV")
    satellite(ax, 39, 60, 1.3)
    for i, (x, lbl) in enumerate(((16, "jammed"), (39, "spoofed"),
                                  (62, "lost in tunnels"))):
        ax.plot([39, x], [57, 33], color=GREY, lw=2.5, ls=":")
        cross(ax, (39 + x) / 2, 45, 1.8)
    bolt(ax, 16, 28, 1.3)
    text(ax, 16, 20, "jammed", 26, RED)
    person(ax, 39, 24, 0.9, RED)
    text(ax, 39, 20, "spoofed", 26, RED)
    tunnel(ax, 62, 24, 0.9)
    text(ax, 62, 20, "tunnel / canopy", 26, RED)
    text(ax, 39, 12.5, "465 interference incidents, Amritsar-Jammu", 22, INK)
    # arrow
    arrow(ax, (76, 41), (86, 41), BLUE, lw=5, ms=45)
    # right: QSLAM
    card(ax, 88, 8, 68, 66, GREEN, "QSLAM: no GPS at all")
    camera(ax, 100, 56, 1.2)
    text(ax, 100, 49.5, "vision", 24)
    mapsheet(ax, 122, 56, 1.0)
    text(ax, 122, 49.5, "preloaded map", 24)
    ax.add_patch(Circle((143, 56), 3.2, facecolor=tint(ORANGE, 0.3), edgecolor=INK,
                        lw=2.5))
    text(ax, 143, 55.8, "IMU", 20)
    text(ax, 143, 49.5, "motion", 24)
    for x in (100, 122, 143):
        arrow(ax, (x, 46.5), (117 + (x - 122) * 0.25, 36), GREEN, lw=2.5)
    robot(ax, 110, 25, 1.3)
    arrow(ax, (119, 28), (135, 28), GREEN, lw=4, ms=35)
    flag(ax, 140, 24, 1.2)
    text(ax, 142, 20.5, "UTM goal", 26, RED)
    tick(ax, 150, 33, 2.4)
    text(ax, 122, 12.5, "no GPS · no network · one UTM input", 24, GREEN)
    return save(fig, "01_problem_and_solution")


def img_how():
    fig, ax = board("How QSLAM knows where it is - without GPS",
                    "1 · the idea")
    steps = [
        (BLUE, "Surveyed datum", ["one lat / lon, fixed offline", "-> UTM anchor (E0, N0)",
                                  "goal = E - E0, N - N0"], "running"),
        (TEAL, "Visual-inertial EKF", ["camera motion + IMU heading", "+ wheel speed",
                                       "30 Hz pose, ~1 m error"], "running"),
        (GREEN, "One GeoTIFF", ["terrain for the simulator", "map for the planner",
                                "anchor for UTM"], "running"),
        (PURPLE, "Map anchoring", ["match camera view to", "satellite / DEM map",
                                   "removes long drift"], "planned"),
    ]
    x = 4
    for i, (c, t, lines, status) in enumerate(steps):
        card(ax, x, 16, 34, 52, c, t, lines, tsize=30, lsize=23)
        badge(ax, x + 1.2, 70.2, i + 1, c)
        chip_c = GREEN if status == "running" else ORANGE
        ax.add_patch(FancyBboxPatch((x + 9, 18.5), 16, 5, boxstyle="round,pad=0.5",
                                    facecolor=matplotlib.colors.to_rgba(chip_c, 0.18),
                                    edgecolor=chip_c, lw=2.5))
        text(ax, x + 17, 21, status, 22, chip_c)
        if i < 3:
            arrow(ax, (x + 34.8, 42), (x + 39.2, 42), INK, lw=3.5)
        x += 39
    # icons on each card
    pin(ax, 21, 28.5, 1.1, BLUE)
    camera(ax, 60, 31.5, 1.0)
    mapsheet(ax, 99, 31.5, 0.9)
    ax.add_patch(Circle((138, 31.5), 4, fill=False, edgecolor=PURPLE, lw=3))
    ax.plot([141, 144], [28.5, 25.5], color=PURPLE, lw=4)
    text(ax, 80, 8, "1-3 run today   ·   4 plays the role GPS used to play",
         24, GREY)
    return save(fig, "02_how_it_knows_where_it_is")


def img_innovation():
    fig, ax = board("What is new", "1 · innovation & uniqueness")
    items = [
        (GREEN, "Zero-GNSS by construction",
         ["no GPS topic, node or parameter", "nothing to jam or spoof"], "0",
         "GPS topics"),
        (BLUE, "UTM goal without GPS",
         ["datum -> anchor offline", "run time = one subtraction"], "0.000 mm",
         "round-trip error"),
        (ORANGE, "One GeoTIFF = one truth",
         ["simulator terrain, planner map", "and UTM anchor from one file"], "1",
         "source file"),
        (PURPLE, "Prior + live + semantic",
         ["DEM sees ditches cameras miss", "lidar sees people, AI sees water"], "3",
         "layers fused"),
    ]
    for i, (c, t, lines, big, unit) in enumerate(items):
        x = 4 + (i % 2) * 78
        y = 42 - (i // 2) * 36
        card(ax, x, y, 74, 32, c)
        text(ax, x + 16, y + 20, big, 50 if len(big) < 4 else 38, c)
        text(ax, x + 16, y + 9.5, unit, 22, GREY)
        text(ax, x + 48, y + 25, t, 30, c)
        for k, ln in enumerate(lines):
            text(ax, x + 48, y + 16 - k * 6, ln, 23)
    return save(fig, "03_innovation")


def img_compare():
    fig, ax = board("Existing ways vs QSLAM", "1 · how it addresses the problem")
    rows = [
        ("GNSS / RTK-GPS", "jammed, spoofed, lost in tunnels", "no receiver - immune"),
        ("wheel odometry + gyro", "slip & drift: 10-70 m", "EKF: VIO + heading, ~1 m"),
        ("2-D lidar SLAM", "breaks on slopes after ~50 m", "visual route + DEM prior"),
        ("pure visual odometry", "drift grows, low texture", "map anchoring + localizability"),
        ("camera-only map", "misses ditches, no geo-frame", "DEM shows ditches, goal in UTM"),
        ("tele-operation", "radio link + operator", "one UTM input, fully offline"),
    ]
    text(ax, 26, 73, "existing", 30, GREY)
    text(ax, 70, 73, "limitation", 30, RED)
    text(ax, 127, 73, "QSLAM", 30, GREEN)
    for i, (a, b, c) in enumerate(rows):
        y = 64 - i * 10.5
        ax.plot([4, 156], [y - 5.2, y - 5.2], color=tint(INK, 0.15), lw=1.5)
        text(ax, 26, y, a, 24)
        cross(ax, 47, y, 1.4)
        text(ax, 70, y, b, 23, RED)
        arrow(ax, (92, y), (100, y), INK, lw=2.5, ms=22)
        tick(ax, 104, y, 1.6)
        text(ax, 129, y, c, 23, GREEN)
    return save(fig, "04_existing_vs_qslam")


# ======================================== 2. TECHNICAL APPROACH ===========
def img_stack():
    fig, ax = board("Technology stack", "2 · frameworks")
    layers = [
        (PURPLE, "Compute", "RTX 4090 testbed  ->  Jetson AGX Orin onboard"),
        (GREY, "Simulation", "Isaac Sim 6  ·  Gazebo Harmonic  ·  MuJoCo"),
        (BLUE, "Middleware", "ROS 2 Jazzy (LTS)"),
        (TEAL, "Localization", "robot_localization EKF  ·  Isaac ROS cuVSLAM"),
        (GREEN, "Navigation", "Nav2: Smac A*  ·  MPPI  ·  behaviour tree"),
        (ORANGE, "Perception", "SegFormer (RELLIS-3D / RUGD)  ·  TensorRT"),
        (RED, "Geospatial", "GeoTIFF  ·  rasterio  ·  pyproj  ·  UTM 44N"),
    ]
    for i, (c, name, tech) in enumerate(layers):
        y = 6 + i * 10.3
        inset = (6 - i) * 0.0
        ax.add_patch(FancyBboxPatch((10 + inset, y), 140 - 2 * inset, 8.2,
                                    boxstyle="round,pad=0.5",
                                    facecolor=matplotlib.colors.to_rgba(c, 0.14),
                                    edgecolor=c, lw=3))
        text(ax, 27, y + 4.1, name, 30, c)
        text(ax, 98, y + 4.1, tech, 26)
    return save(fig, "05_technology_stack")


def img_arch():
    fig, ax = board("System architecture", "2 · offline preparation -> onboard loop")
    ax.add_patch(FancyBboxPatch((3, 52), 128, 23, boxstyle="round,pad=0.5",
                                facecolor=tint(BLUE, 0.04), edgecolor=BLUE, lw=2,
                                ls="--", zorder=0.6))
    text(ax, 11, 72, "offline", 26, BLUE)
    off = [("GeoTIFF DEM", GREEN), ("slope > 15°\n= no-go", ORANGE),
           ("prior map", GREEN), ("surveyed datum\n-> UTM anchor", BLUE)]
    xs = [7, 38, 69, 100]
    for (t, c), x in zip(off, xs):
        card(ax, x, 55, 27, 13, c, lines=t.split("\n"), lsize=24)
    for x in xs[:2]:
        arrow(ax, (x + 27.8, 61.5), (x + 30.2, 61.5), INK)
    # operator
    person(ax, 146, 58, 1.1, RED)
    text(ax, 146, 72, "operator", 26, RED)
    text(ax, 146, 54.5, "UTM E, N", 24, RED)
    # onboard loop
    text(ax, 16, 47, "onboard, real time", 26, TEAL)
    blocks = [("sensors", "camera · lidar\nIMU · wheels", TEAL, 5),
              ("EKF", "where am I?\n30 Hz", BLUE, 36),
              ("costmaps", "prior + lidar\nobstacles", ORANGE, 67),
              ("Smac A*", "route\n1 Hz", GREEN, 98),
              ("MPPI", "wheel commands\n15 Hz", PURPLE, 129)]
    for name, sub, c, x in blocks:
        card(ax, x, 20, 26, 20, c, name, sub.split("\n"), tsize=30, lsize=22)
    for i in range(4):
        x = blocks[i][3] + 26.8
        arrow(ax, (x, 30), (x + 3.4, 30), INK, lw=3.5)
    arrow(ax, (142, 18.5), (18, 18.5), GREY, lw=3, rad=-0.15)
    text(ax, 80, 4, "the rover moves -> the sensors see it -> the loop repeats",
         22, GREY)
    arrow(ax, (82.5, 54.2), (80, 41), GREEN, lw=3)          # prior map -> costmaps
    arrow(ax, (113.5, 54.2), (111, 41), BLUE, lw=3)         # anchor -> A*
    arrow(ax, (143, 53), (118, 41), RED, lw=3)              # goal -> A*
    return save(fig, "06_system_architecture")


def img_flow():
    fig, ax = board("Mission flow", "2 · methodology & process")
    steps = [("M0", "prepare", "map + anchor", BLUE),
             ("M1", "bring-up", "sensors, EKF", TEAL),
             ("M2", "goal", "UTM -> map", RED),
             ("M3", "plan", "A* route", GREEN),
             ("M4", "drive", "MPPI + EKF", PURPLE)]
    for i, (m, t, sub, c) in enumerate(steps):
        x = 4 + i * 23.5
        card(ax, x, 47, 19.5, card_height(2, True, 34, 22), c, m, [t, sub],
             tsize=34, lsize=22)
        if i < 4:
            arrow(ax, (x + 20.3, 60), (x + 23, 60), INK)
    cx, cy = 136, 60
    ax.add_patch(Polygon([(cx, cy + 11), (cx + 15, cy), (cx, cy - 11),
                          (cx - 15, cy)], facecolor=tint(ORANGE, 0.15), edgecolor=ORANGE,
                         lw=3, zorder=2))
    text(ax, cx, cy + 2, "goal", 28)
    text(ax, cx, cy - 3.5, "reached?", 28)
    arrow(ax, (98 + 20.3, 60), (cx - 15.2, 60), INK)
    arrow(ax, (cx, cy - 11.5), (cx, 38), GREEN, lw=3.5)
    text(ax, cx + 5, 43, "yes", 28, GREEN)
    card(ax, 123, 16, 26, 20, GREEN, "M5 arrive", ["within 0.7 m", "pose in UTM"],
         tsize=30, lsize=22)
    tick(ax, 153.5, 30, 1.8)
    arrow(ax, (cx - 8, cy - 6), (98, 37), ORANGE, lw=3, rad=-0.2)
    text(ax, 108, 45, "no", 28, ORANGE)
    card(ax, 60, 16, 40, 20, ORANGE, "stuck? recover",
         ["clear map · spin", "wait · back up"], tsize=30, lsize=22)
    arrow(ax, (60, 26), (79, 46.3), ORANGE, lw=3, rad=-0.5)
    text(ax, 40, 30, "re-plan &", 26, ORANGE)
    text(ax, 40, 25, "keep driving", 26, ORANGE)
    return save(fig, "07_mission_flow")


def img_geo():
    fig, ax = board("From GeoTIFF to a drivable route", "2 · map preparation")
    occ = SL > WORLD["max_slope_deg"]
    rgb_occ = np.empty(occ.shape + (3,), np.uint8)
    rgb_occ[...] = (235, 240, 246)
    rgb_occ[occ] = (40, 40, 40)
    terrain = (0.55 * cmap_img(DEM, "terrain") + 0.45 * (HS[..., None] * 255)).astype(np.uint8)
    # route panel: hillshade + no-go + route + start/goal
    route = np.repeat((HS * 200 + 40).astype(np.uint8)[..., None], 3, -1)
    route[occ] = (220, 70, 70)
    panels = [(terrain, "GeoTIFF", "elevation, UTM 44N"),
              (cmap_img(SL, "inferno", 0, 40), "slope", "steepness"),
              (rgb_occ, "no-go map", "slope > 15°"),
              (route, "route", "start -> goal")]
    w = 33
    for i, (img, t, sub) in enumerate(panels):
        x = 4 + i * 39.5
        photo(ax, img, x, 25, w, w)
        text(ax, x + w / 2, 20.5, t, 32, INK)
        text(ax, x + w / 2, 15.5, sub, 24, GREY)
        if i < 3:
            arrow(ax, (x + w + 0.8, 41.5), (x + 39, 41.5), BLUE, lw=4, ms=34)
    # draw the real route on the last panel
    x0 = 4 + 3 * 39.5
    half = WORLD["world_size"] / 2

    def to_px(mx, my):
        return x0 + (mx + half) / (2 * half) * w, 25 + (my + half) / (2 * half) * w
    if len(LOG) > 2:
        pts = np.array([to_px(a, b) for a, b in LOG[::10, 1:3]])
        ax.plot(pts[:, 0], pts[:, 1], color="#2fd07a", lw=3.5, zorder=5)
    sx, sy = to_px(WORLD["start"]["x"], WORLD["start"]["y"])
    g = STATS["goal"]
    gx, gy = to_px(g["map_x"], g["map_y"])
    ax.add_patch(Circle((sx, sy), 1.1, facecolor=BLUE, edgecolor="white", lw=2, zorder=6))
    ax.plot([gx], [gy], marker="*", ms=26, color="#ffd400", mec=INK, zorder=6)
    text(ax, 80, 6, "one file builds the world, the map and the UTM anchor", 24, GREEN)
    return save(fig, "08_geotiff_to_route")


def img_demo():
    fig, ax = board("Recorded GPS-free mission (Isaac Sim)", "2 · prototype")
    shots = [(frame("chase", "isaac_behind.mp4", 200), "rover driving"),
             (frame("onboard_tunnel", "isaac_onboard.mp4", 540), "rover camera: tunnel"),
             (frame("costmap", "costmap.mp4", 500), "costmaps + route"),
             (frame("rviz", "rviz.mp4", 1000), "RViz 3D view")]
    for i, (p, lbl) in enumerate(shots):
        x = 12 + (i % 2) * 74
        y = 42 - (i // 2) * 34.5
        photo(ax, p, x, y, 52, 29.25)
        text(ax, x + 26, y - 3, lbl, 26)
    text(ax, 80, 76, f"{STATS['path_len']:.0f} m driven  ·  through the tunnel  ·  "
         f"goal reached {STATS['final_dist']:.2f} m away  ·  no GPS", 25, GREEN)
    return save(fig, "09_recorded_mission")


def img_videos():
    fig, ax = board("Every run records itself", "2 · evidence pipeline")
    card(ax, 4, 30, 30, 34, BLUE, "3 terminals", ["1  simulation", "2  navigation",
                                                  "3  UTM goal"], lsize=24)
    rec = [("4 cameras", TEAL), ("RViz 3D view", PURPLE), ("costmap · EKF · SLAM", ORANGE),
           ("telemetry CSV", GREY)]
    for i, (t, c) in enumerate(rec):
        y = 64 - i * 12
        card(ax, 46, y - 8, 36, 9, c, lines=[t], lsize=24)
        arrow(ax, (35, 47), (45.3, y - 3.5), INK, lw=2.5)
    card(ax, 94, 30, 22, 34, GREEN, "goal", ["reached", "-> safe", "shutdown"], lsize=24)
    for i in range(4):
        arrow(ax, (82.8, 60 - i * 12), (93.3, 50), GREY, lw=2)
    outs = [("2-min highlight", RED), ("all videos", BLUE), ("PDF report", GREEN)]
    for i, (t, c) in enumerate(outs):
        y = 60 - i * 12
        card(ax, 126, y - 4, 30, 9, c, lines=[t], lsize=24)
        arrow(ax, (116.8, 47), (125.4, y + 0.5), INK, lw=2.5)
    text(ax, 80, 12, "pictures only - no words burned into any video", 24, GREY)
    text(ax, 80, 6.5, "a stray Ctrl-C cannot corrupt the recordings", 24, GREY)
    return save(fig, "10_recording_pipeline")


# ============================================= 3. FEASIBILITY =============
RUNS = [("reference", 123.5, 0.98), ("run_final", 189.0, 1.09),
        ("run_1080p", 295.1, 1.42), (RUN.name, STATS["path_len"], STATS["loc_err"])]


def img_results():
    fig, ax = board("Measured, not assumed", "3 · feasibility (Isaac Sim testbed)")
    tiles = [(f"{len(RUNS)} / {len(RUNS)}", "missions reached the goal\nthrough the tunnel", GREEN),
             ("~1 m", "EKF error vs 10-70 m\nwheel odometry", BLUE),
             ("0.000 mm", "UTM <-> map\nround trip", PURPLE),
             ("0 / 21", "contacts with unmapped\nobstacles (prototype)", ORANGE)]
    for i, (big, sub, c) in enumerate(tiles):
        x = 4 + i * 39
        card(ax, x, 56, 35, 20, c)
        text(ax, x + 17.5, 70.5, big, 40, c)
        for k, ln in enumerate(sub.split("\n")):
            text(ax, x + 17.5, 63 - k * 4.3, ln, 21)
    # bar chart: final localization error per run
    bx = fig.add_axes([0.04, 0.08, 0.42, 0.44])
    bx.set_facecolor(BOARD)
    names = [r[0] for r in RUNS]
    errs = [r[2] for r in RUNS]
    bars = bx.bar(range(len(RUNS)), errs, color=[BLUE, BLUE, BLUE, GREEN],
                  edgecolor=INK, lw=2)
    for b, (n, d, e) in zip(bars, RUNS):
        bx.text(b.get_x() + b.get_width() / 2, e + 0.05, f"{e:.2f} m", ha="center",
                fontsize=fs(26))
        bx.text(b.get_x() + b.get_width() / 2, 0.1, f"{d:.0f} m\ndriven",
                ha="center", fontsize=fs(21), color=on_colour())
    bx.set_xticks(range(len(RUNS)), names, fontsize=fs(22))
    bx.set_ylim(0, 1.8)
    bx.set_yticks([])
    for sp in ("top", "right", "left"):
        bx.spines[sp].set_visible(False)
    bx.set_title("final position error per run", fontsize=fs(28))
    # why EKF
    cx = fig.add_axes([0.62, 0.08, 0.34, 0.44])
    cx.set_facecolor(BOARD)
    vals = [("gyro-only heading", 57.0, RED), ("slipping wheels", 4.3, ORANGE),
            ("QSLAM EKF", 1.09, GREEN)]
    cx.barh(range(3), [v for _, v, _ in vals], color=[c for *_, c in vals],
            edgecolor=INK, lw=2)
    for i, (n, v, c) in enumerate(vals):
        cx.text(v + 1, i, f"{v:g} m", va="center", fontsize=fs(26))
    cx.set_yticks(range(3), [n for n, *_ in vals], fontsize=fs(23))
    cx.set_xlim(0, 70)
    cx.set_xticks([])
    for sp in ("top", "right", "bottom"):
        cx.spines[sp].set_visible(False)
    cx.set_title("error after the same 189 m drive", fontsize=fs(28))
    return save(fig, "11_measured_results")


def img_risks():
    fig, ax = board("Risks -> how we handle them", "3 · challenges")
    rows = [("low texture, dust, night", "localizability cost layer · IMU bridging · safe stop"),
            ("sim-to-real gap", "cuVSLAM with real noise · same ROS 2 interfaces"),
            ("ditches not in the DEM", "GPU elevation map from depth · SegFormer classes"),
            ("coarse national DEM (30 m)", "route from Bhuvan · drone DEM for key sites"),
            ("edge compute budget", "TensorRT on Jetson Orin · <= 100 ms to command")]
    for i, (r, s) in enumerate(rows):
        y = 67 - i * 13.2
        card(ax, 6, y - 5.5, 52, 11, ORANGE, fill=0.08)
        warn(ax, 11, y, 0.9)
        text(ax, 35, y, r, 25)
        arrow(ax, (60, y), (70, y), INK, lw=3.5)
        card(ax, 72, y - 5.5, 82, 11, GREEN, fill=0.08)
        shield(ax, 77.5, y + 0.3, 0.75)
        text(ax, 116, y, s, 23)
    return save(fig, "12_risks_and_mitigation")


def img_viability():
    fig, ax = board("Performance · safety · security · cost", "3 · viability")
    tiles = [(BLUE, "real time", ["MPPI 15 Hz", "EKF 30 Hz", "re-plan 1 Hz",
                                  "0.5 s stop watchdog"]),
             (ORANGE, "safety", ["0.95 m obstacle margin", "off-site goals rejected",
                                 "recovery behaviours", "collision monitor (next)"]),
             (GREEN, "security", ["no GNSS to spoof", "no network in mission",
                                  "sensors only receive", "no remote attack path"]),
             (PURPLE, "cost", ["Rs 0 software licences", "open source stack",
                               "one COTS sensor kit", "Jetson AGX Orin"])]
    for i, (c, t, lines) in enumerate(tiles):
        x = 4 + i * 39
        card(ax, x, 10, 35, 62, c, t, [], tsize=36)
        for k, ln in enumerate(lines):
            tick(ax, x + 4, 52 - k * 10.5, 1.2, c)
            text(ax, x + 7.5, 52 - k * 10.5, ln, 22, ha="left")
    return save(fig, "13_viability")


# ================================================ 4. IMPACT ===============
def img_users():
    fig, ax = board("Who benefits", "4 · impact on target audience")
    users = [(GREEN, "Army / CAPF", ["patrol & recce", "without risking people", "works under jamming"]),
             (BLUE, "BEL", ["indigenous nav module", "standard ROS 2 / Nav2", "hardware-agnostic"]),
             (RED, "NDRF / rescue", ["enters tunnels,", "collapse & canopy zones", "where GPS is lost"]),
             (ORANGE, "mining / tunnels", ["autonomous inspection", "underground", "no network needed"])]
    for i, (c, t, lines) in enumerate(users):
        x = 4 + i * 39
        card(ax, x, 8, 35, 64, c, None, [])
        text(ax, x + 17.5, 45, t, 32, c)
        for k, ln in enumerate(lines):
            text(ax, x + 17.5, 35 - k * 6.5, ln, 22)
    # icons
    person(ax, 14, 54, 1.2, GREEN); shield(ax, 29, 60, 0.9)
    chip(ax, 60.5, 60, 1.1, BLUE)
    person(ax, 94, 54, 1.2, RED); tunnel(ax, 105, 53, 0.7)
    robot(ax, 138, 55, 0.8, ORANGE)
    return save(fig, "14_target_users")


def img_benefits():
    fig, ax = board("Benefits", "4 · social · economic · environmental · strategic")
    quads = [(GREEN, "social", ["keeps soldiers & rescuers", "out of lead positions"]),
             (BLUE, "economic", ["no licence cost", "COTS sensors, not RTK / INS", "Atmanirbhar"]),
             (TEAL, "environmental", ["suits electric UGVs", "fewer manned sorties"]),
             (RED, "strategic", ["cannot be jammed,", "spoofed or cut off"])]
    for i, (c, t, lines) in enumerate(quads):
        x = 6 + (i % 2) * 76
        y = 42 - (i // 2) * 35
        card(ax, x, y, 72, 31, c, t, lines, tsize=36, lsize=25)
    return save(fig, "15_benefits")


def img_roadmap():
    fig, ax = board("Roadmap", "4 · prototype -> scale")
    ms = [("prototype", "3 simulators\nrecorded missions", GREEN, True),
          ("sim hardening", "M1-2: cuVSLAM,\nnoise, SegFormer", BLUE, False),
          ("pilot", "M3-5: Jetson + ZED X\non a real UGV", TEAL, False),
          ("validation", "M5-7: tunnel, canopy,\njammed-GNSS trials", ORANGE, False),
          ("deployment", "BEL integrates\ninto UGV platforms", PURPLE, False),
          ("scale", "new platforms,\nsites, multi-UGV", RED, False)]
    ax.plot([8, 152], [44, 44], color=INK, lw=4)
    arrow(ax, (148, 44), (156, 44), INK, lw=4)
    for i, (t, sub, c, done) in enumerate(ms):
        x = 14 + i * 26.5
        ax.add_patch(Circle((x, 44), 3.2, facecolor=c if done else BOARD,
                            edgecolor=c, lw=4, zorder=4))
        if done:
            tick(ax, x, 44, 1.3, "white")
        up = i % 2 == 0
        y = 60 if up else 20
        text(ax, x, y + 6, t, 30, c)
        for k, ln in enumerate(sub.split("\n")):
            text(ax, x, y - k * 5, ln, 21)
        ax.plot([x, x], [47.5 if up else 40.5, (y - 8) if up else (y + 9.5)],
                color=c, lw=2, ls=":")
    text(ax, 80, 6, "target: <= 2 % arrival error of distance · >= 90 % mission success",
         24, GREY)
    return save(fig, "16_roadmap")


# ======================================= 5. RESEARCH & REFERENCES =========
REFS = [
    ("1", "Hart, Nilsson & Raphael - A* (IEEE Trans. SSC, 1968)", "https://doi.org/10.1109/TSSC.1968.300136", "planning"),
    ("2", "Williams et al. - MPPI / information-theoretic MPC (ICRA 2017)", "https://doi.org/10.1109/ICRA.2017.7989202", "planning"),
    ("3", "Macenski et al. - The Marathon 2: Nav2 (IROS 2020)", "https://arxiv.org/abs/2003.00368", "planning"),
    ("4", "Moore & Stouch - robot_localization EKF (IAS-13, 2014)", "https://doi.org/10.1007/978-3-319-08338-4_25", "localization"),
    ("5", "Rublee et al. - ORB features (ICCV 2011)", "https://doi.org/10.1109/ICCV.2011.6126544", "localization"),
    ("6", "Labbe & Michaud - RTAB-Map (J. Field Robotics 2019)", "https://doi.org/10.1002/rob.21831", "localization"),
    ("7", "Xie et al. - SegFormer (NeurIPS 2021)", "https://arxiv.org/abs/2105.15203", "perception"),
    ("8", "Jiang et al. - RELLIS-3D dataset (ICRA 2021)", "https://arxiv.org/abs/2011.12954", "perception"),
    ("8b", "Wigness et al. - RUGD dataset (IROS 2019)", "https://doi.org/10.1109/IROS40897.2019.8968283", "perception"),
    ("9", "Miki et al. - GPU elevation mapping (IROS 2022)", "https://arxiv.org/abs/2204.12876", "perception"),
    ("10", "Shi & Li - vehicle localization from satellite images (CVPR 2022)", "https://arxiv.org/abs/2204.04752", "anchoring"),
    ("11", "Sarlin et al. - OrienterNet (CVPR 2023)", "https://arxiv.org/abs/2304.02009", "anchoring"),
    ("12", "Dolgov et al. - Hybrid-A* (IJRR 2010)", "https://doi.org/10.1177/0278364909359210", "planning"),
    ("13", "Fox, Burgard & Thrun - Dynamic Window Approach (IEEE RAM 1997)", "https://doi.org/10.1109/100.580977", "planning"),
    ("14", "Snyder - Map Projections: A Working Manual (USGS PP 1395, 1987)", "https://doi.org/10.3133/pp1395", "geodesy"),
]
TOOLS = [
    ("Nav2 documentation", "https://docs.nav2.org"),
    ("ROS 2 Jazzy", "https://docs.ros.org/en/jazzy/"),
    ("robot_localization", "https://github.com/cra-ros-pkg/robot_localization"),
    ("slam_toolbox", "https://github.com/SteveMacenski/slam_toolbox"),
    ("Isaac ROS Visual SLAM (cuVSLAM)", "https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html"),
    ("Isaac Sim documentation", "https://docs.isaacsim.omniverse.nvidia.com/"),
    ("ISRO Bhuvan (DEM)", "https://bhuvan.nrsc.gov.in"),
    ("rasterio (GeoTIFF)", "https://rasterio.readthedocs.io"),
    ("pyproj", "https://pyproj4.github.io/pyproj/stable/"),
    ("EPSG:32644 - WGS 84 / UTM 44N", "https://epsg.io/32644"),
    ("Our code: SIH-26126", "https://github.com/kamalraj2772/SIH-26126"),
]
EVIDENCE = [
    "Lok Sabha written reply (Mar 2025): 465 GPS interference / spoofing incidents, Amritsar-Jammu, Nov 2023 - Feb 2025",
    "DGCA SOP (10 Nov 2025): real-time reporting of GPS spoofing / GNSS interference",
    "SIH 2026 problem statement SIH26126 - Bharat Electronics Limited",
]


def img_refmap():
    fig, ax = board("Research behind each part", "5 · research & references")
    groups = [("planning", GREEN, (27, 60), "A* [1] · Hybrid-A* [12]\nMPPI [2] · DWA [13]\nNav2 [3]"),
              ("localization", TEAL, (133, 60), "EKF [4] · ORB [5]\nRTAB-Map [6]\ncuVSLAM"),
              ("perception", ORANGE, (27, 21), "SegFormer [7]\nRELLIS-3D / RUGD [8]\nGPU elevation [9]"),
              ("map anchoring", PURPLE, (133, 21), "satellite localization [10]\nOrienterNet [11]"),
              ("geodesy", RED, (80, 15), "UTM projection [14]\nEPSG:32644 · GeoTIFF")]
    for name, c, (x, y), refs in groups:          # connectors first, under the cards
        ax.plot([80, x], [44, y], color=c, lw=3.5, zorder=0.5)
    ax.add_patch(Circle((80, 44), 11, facecolor=tint(BLUE, 0.12), edgecolor=BLUE, lw=4,
                        zorder=3))
    text(ax, 80, 45.5, "QSLAM", 36, BLUE)
    text(ax, 80, 39.5, "no GPS", 24, GREY)
    for name, c, (x, y), refs in groups:
        lines = refs.split("\n")
        hh = card_height(len(lines), True, 30, 23)
        ww = 46 if name != "geodesy" else 40
        card(ax, x - ww / 2, y - hh / 2, ww, hh, c, name, lines, tsize=30,
             lsize=23)
    return save(fig, "17_research_map")


# ================================================================= PDF ====
def build_pdf(pngs):
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4, landscape
    from reportlab.lib.units import mm
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from reportlab.pdfgen import canvas

    pdfmetrics.registerFont(TTFont("Hand", str(FONT)))
    lato = pathlib.Path("/usr/share/fonts/truetype/lato/Lato-Regular.ttf")
    body = "Helvetica"
    if lato.exists():
        pdfmetrics.registerFont(TTFont("Lato", str(lato)))
        body = "Lato"
    pw, ph = landscape(A4)
    c = canvas.Canvas(str(PDF), pagesize=(pw, ph))
    c.setTitle("QSLAM - SIH26126 visual guide")
    c.setAuthor("Team QSLAM (ID 14)")
    col = {k: colors.HexColor(v) for k, v in
           dict(ink=INK, blue=BLUE, green=GREEN, red=RED, grey=GREY,
                board=BOARD, orange=ORANGE, purple=PURPLE, teal=TEAL).items()}
    page = [0]

    def frame_page(section, title, color):
        c.setFillColor(col["board"])
        c.rect(0, 0, pw, ph, stroke=0, fill=1)
        c.setFillColor(color)
        c.rect(0, ph - 16 * mm, pw, 16 * mm, stroke=0, fill=1)
        c.setFillColor(colors.white)
        c.setFont("Hand", 22)
        c.drawString(10 * mm, ph - 11 * mm, f"{section}   {title}")
        c.setFont("Hand", 13)
        c.drawRightString(pw - 10 * mm, ph - 11 * mm, "QSLAM · SIH26126 · Team 14")
        page[0] += 1
        c.setFillColor(col["grey"])
        c.setFont("Hand", 12)
        c.drawRightString(pw - 10 * mm, 6 * mm, str(page[0]))

    def chips(words, color, y=9 * mm):
        x = 10 * mm
        c.setFont("Hand", 14)
        for w in words:
            tw = c.stringWidth(w, "Hand", 14) + 8 * mm
            c.setStrokeColor(color)
            c.setFillColor(colors.Color(color.red, color.green, color.blue, 0.10))
            c.setLineWidth(1.4)
            c.roundRect(x, y - 2.5 * mm, tw, 8 * mm, 3 * mm, stroke=1, fill=1)
            c.setFillColor(col["ink"])
            c.drawString(x + 4 * mm, y, w)
            x += tw + 4 * mm

    def image_page(section, title, color, png, words):
        frame_page(section, title, color)
        iw = pw - 20 * mm
        ih = iw * 9 / 16
        top = ph - 20 * mm
        if ih > top - 20 * mm:
            ih = top - 20 * mm
            iw = ih * 16 / 9
        x = (pw - iw) / 2
        c.drawImage(str(png), x, top - ih, iw, ih)
        c.setStrokeColor(colors.HexColor("#dddddd"))
        c.rect(x, top - ih, iw, ih, stroke=1, fill=0)
        chips(words, color)
        c.showPage()

    # cover
    c.setFillColor(col["board"])
    c.rect(0, 0, pw, ph, stroke=0, fill=1)
    c.setFillColor(col["blue"])
    c.setFont("Hand", 64)
    c.drawCentredString(pw / 2, ph - 36 * mm, "QSLAM")
    c.setFillColor(col["ink"])
    c.setFont("Hand", 26)
    c.drawCentredString(pw / 2, ph - 50 * mm,
                        "GPS-free, vision-based UGV navigation to a UTM goal")
    c.setFont("Hand", 18)
    c.setFillColor(col["grey"])
    c.drawCentredString(pw / 2, ph - 62 * mm,
                        "SIH 2026 · SIH26126 · Vision Based Autonomous Navigation for "
                        "Unmanned Ground Vehicle for Outdoor environment")
    c.drawCentredString(pw / 2, ph - 71 * mm,
                        "Smart Automation · Software · Team QSLAM (ID 14) · Bharat Electronics Limited")
    c.drawImage(str(pngs["01"]), pw / 2 - 85 * mm, 15 * mm, 170 * mm, 95.6 * mm)
    c.setFillColor(col["green"])
    c.setFont("Hand", 18)
    c.drawCentredString(pw / 2, 7 * mm, "\"No GPS. No network. Just vision, a map and a UTM goal.\"")
    c.showPage()
    page[0] += 1

    # contents
    frame_page("", "Contents", col["blue"])
    sections = [("1", "Solution", "what it is · how it solves the problem · what is new", col["green"]),
                ("2", "Technical approach", "frameworks · architecture · flowcharts", col["blue"]),
                ("3", "Feasibility & viability", "measured results · risks & fixes", col["orange"]),
                ("4", "Impact & benefits", "target users · benefits · roadmap", col["red"]),
                ("5", "Research & references", "papers · tools · links", col["purple"])]
    for i, (n, t, sub, clr) in enumerate(sections):
        y = ph - 45 * mm - i * 28 * mm
        c.setFillColor(clr)
        c.circle(30 * mm, y + 3 * mm, 9 * mm, stroke=0, fill=1)
        c.setFillColor(colors.white)
        c.setFont("Hand", 28)
        c.drawCentredString(30 * mm, y - 0.5 * mm, n)
        c.setFillColor(col["ink"])
        c.setFont("Hand", 30)
        c.drawString(46 * mm, y + 2 * mm, t)
        c.setFillColor(col["grey"])
        c.setFont("Hand", 18)
        c.drawString(46 * mm, y - 6 * mm, sub)
    c.showPage()

    G, B, O, R, P = col["green"], col["blue"], col["orange"], col["red"], col["purple"]
    plan = [
        ("1", "Solution", G, "01", ["onboard ROS 2 module", "offline", "zero GNSS", "UTM goal in"]),
        ("1", "Solution - the idea", G, "02", ["surveyed datum", "visual-inertial EKF", "one GeoTIFF", "map anchoring"]),
        ("1", "Innovation & uniqueness", G, "03", ["immune to jamming", "exact UTM goal", "map = world", "prior + live + AI"]),
        ("1", "How it addresses the problem", G, "04", ["no receiver", "~1 m vs 10-70 m", "sees ditches", "fully autonomous"]),
        ("2", "Technologies used", B, "05", ["ROS 2 Jazzy", "Nav2", "Isaac Sim 6", "cuVSLAM", "SegFormer", "GeoTIFF"]),
        ("2", "System architecture", B, "06", ["offline map prep", "EKF 30 Hz", "A* 1 Hz", "MPPI 15 Hz"]),
        ("2", "Methodology - mission flow", B, "07", ["M0 prepare", "M1 bring-up", "M2 goal", "M3 plan", "M4 drive", "M5 arrive"]),
        ("2", "Process - map preparation", B, "08", ["GeoTIFF", "slope", "no-go map", "route"]),
        ("2", "Implementation - recorded prototype", B, "09", ["Isaac Sim", "4 cameras", "costmaps", "RViz"]),
        ("2", "Implementation - evidence pipeline", B, "10", ["auto recording", "safe shutdown", "2-min highlight", "PDF report"]),
        ("3", "Feasibility - measured results", O, "11", ["testbed numbers", "goal via tunnel", "~1 m EKF error"]),
        ("3", "Challenges & risks", O, "12", ["low texture", "sim-to-real", "hidden ditches", "coarse DEM", "compute"]),
        ("3", "Viability", O, "13", ["real time", "safe", "unspoofable", "Rs 0 licences"]),
        ("4", "Impact on target audience", R, "14", ["Army / CAPF", "BEL", "NDRF", "mining"]),
        ("4", "Benefits", R, "15", ["social", "economic", "environmental", "strategic"]),
        ("4", "Roadmap", R, "16", ["prototype done", "pilot", "validation", "deployment"]),
        ("5", "Research map", P, "17", ["planning", "localization", "perception", "anchoring", "geodesy"]),
    ]
    for sec, title, clr, key, words in plan:
        image_page(sec, title, clr, pngs[key], words)

    # references page(s): the one place text is unavoidable -- keep it a list
    def ref_page(title, rows, start_y):
        frame_page("5", title, P)
        y = start_y
        for tag, label, url in rows:
            if y < 16 * mm:
                c.showPage()
                frame_page("5", title + " (cont.)", P)
                y = ph - 26 * mm
            c.setFillColor(P)
            c.setFont("Hand", 15)
            c.drawString(12 * mm, y, tag)
            c.setFillColor(col["ink"])
            c.setFont(body, 10.5)
            c.drawString(26 * mm, y, label)
            if url:
                c.setFillColor(col["blue"])
                c.setFont(body, 9.5)
                c.drawString(26 * mm, y - 4.6 * mm, url)
                c.linkURL(url, (26 * mm, y - 6 * mm,
                                26 * mm + c.stringWidth(url, body, 9.5), y - 1 * mm),
                          relative=0)
                y -= 11.5 * mm
            else:
                y -= 7.5 * mm
        c.showPage()

    ref_page("Research papers", [(f"[{t}]", l, u) for t, l, u, _ in REFS], ph - 26 * mm)
    ref_page("Tools, data & problem evidence",
             [("tool", l, u) for l, u in TOOLS] + [("fact", e, None) for e in EVIDENCE],
             ph - 26 * mm)
    c.save()
    print("wrote", PDF.name, f"({page[0]} pages)")


def main():
    global PNGS
    makers = [img_problem, img_how, img_innovation, img_compare, img_stack,
              img_arch, img_flow, img_geo, img_demo, img_videos, img_results,
              img_risks, img_viability, img_users, img_benefits, img_roadmap,
              img_refmap]
    names = list(STYLES) if args.styles == "all" else args.styles.split(",")
    for old in ROOT.glob("[0-9][0-9]_*.png"):      # pre-style flat layout
        old.unlink()
    for name in names:
        use_style(name)
        PNGS = []
        pngs = {}
        for m in makers:
            p = m()
            pngs[p.name[:2]] = p
        if name == "01_whiteboard_sketch":
            build_pdf(pngs)                        # the PDF keeps this style
    import shutil
    shutil.rmtree(CACHE, ignore_errors=True)       # frames were only a cache


if __name__ == "__main__":
    main()
