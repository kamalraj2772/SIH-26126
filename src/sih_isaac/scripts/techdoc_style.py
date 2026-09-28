"""Shared look for the SIH26126 technical-description figures.

Every figure module imports this so diagrams, plots and the PDF share one
palette, one font and one box/arrow vocabulary.
"""
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

WS = pathlib.Path("/home/qbotix-rover/sih_ws")
PKG = WS / "src/sih_isaac"
GEN = PKG / "generated"
REC = WS / "mission_recordings"
OUT_DIR = WS / "build/tech_doc_figs"

NAVY = "#1F4E79"
BLUE = "#2E74B5"
INK = "#1A1A1A"
GREY = "#666666"
FAINT = "#9A9A9A"
LIGHT = "#F2F6FA"
ORANGE = "#C55A11"
GREEN = "#2E7D32"
RED = "#C62828"
PURPLE = "#6A1B9A"
TEAL = "#00796B"

# Status vocabulary used in every diagram and in the PDF text.
KINDS = {
    #            fill       edge       title      dashed
    "impl":    ("#EAF4EA", "#5B9B5F", "#1E5E23", False),   # implemented, Isaac testbed
    "proto":   ("#EEF2FA", "#7A93C2", NAVY,      False),   # implemented, MuJoCo prototype
    "design":  ("#FBF3E8", "#D9B98C", "#B0600F", False),   # QSLAM design, not in repo yet
    "exp":     ("#F4F4F4", "#AAAAAA", "#555555", True),    # experimental / installed, not wired
    "neutral": (LIGHT,     "#B9C9D9", NAVY,      False),
    "offline": ("#F5F5F0", "#BBBBAF", "#5A5A4A", False),
    "eval":    ("#FDECEC", "#D98C8C", "#A52A2A", False),   # evaluation only (ground truth)
    "white":   ("#FFFFFF", "#B9C9D9", NAVY,      False),
}
KIND_LABELS = {
    "impl": "Implemented (Isaac Sim testbed)",
    "proto": "Implemented (MuJoCo prototype)",
    "design": "QSLAM design (planned)",
    "exp": "Experimental / installed, not wired",
    "eval": "Evaluation only",
}

DPI = 220


def setup():
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 8.5,
        "axes.titlesize": 9.5,
        "axes.titleweight": "bold",
        "axes.titlecolor": NAVY,
        "axes.labelsize": 8.5,
        "axes.labelcolor": INK,
        "axes.edgecolor": "#888888",
        "axes.linewidth": 0.8,
        "axes.grid": False,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "xtick.color": "#444444",
        "ytick.color": "#444444",
        "legend.fontsize": 7.5,
        "legend.frameon": False,
        "figure.dpi": 100,
        "savefig.dpi": DPI,
        "mathtext.fontset": "dejavusans",
    })


setup()


def diagram(w_in, h_in):
    """Blank diagram canvas; data units are 1/100 inch, origin bottom-left."""
    fig = plt.figure(figsize=(w_in, h_in))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, w_in * 100)
    ax.set_ylim(0, h_in * 100)
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


def box(ax, x, y, w, h, title, sub=None, kind="neutral", ts=8.3, ss=6.8,
        radius=5, lw=1.1, title_color=None, align="center"):
    fc, ec, tc, dashed = KINDS[kind]
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle=f"round,pad=0,rounding_size={radius}",
        fc=fc, ec=ec, lw=lw, linestyle=(0, (4, 2.5)) if dashed else "-",
        zorder=2))
    cx = x + w / 2 if align == "center" else x + 7
    ha = "center" if align == "center" else "left"
    if sub:
        ax.text(cx, y + h - 4 - ts * 0.9, title, ha=ha, va="center",
                fontsize=ts, fontweight="bold", color=title_color or tc,
                zorder=3)
        ax.text(cx, (y + (y + h - 8 - ts * 1.6)) / 2, sub, ha=ha,
                va="center", fontsize=ss, color="#333333", zorder=3,
                linespacing=1.25)
    else:
        ax.text(cx, y + h / 2, title, ha=ha, va="center", fontsize=ts,
                fontweight="bold", color=title_color or tc, zorder=3,
                linespacing=1.2)


def arrow(ax, p1, p2, color="#7F7F7F", dashed=False, lw=1.2, label=None,
          lpos=None, lsize=6.6, lcolor=GREY, rad=0.0, style="-|>",
          ha="center"):
    ax.annotate("", xy=p2, xytext=p1, zorder=4, arrowprops=dict(
        arrowstyle=style, color=color, lw=lw, mutation_scale=11,
        linestyle=(0, (4, 2.5)) if dashed else "-", shrinkA=0, shrinkB=0,
        connectionstyle=f"arc3,rad={rad}"))
    if label:
        if lpos is None:
            lpos = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2 + 5)
        ax.text(lpos[0], lpos[1], label, fontsize=lsize, color=lcolor,
                ha=ha, va="center", style="italic", zorder=5,
                bbox=dict(fc="white", ec="none", pad=0.6, alpha=0.85))


def polyline(ax, pts, color="#7F7F7F", dashed=False, lw=1.2):
    """Multi-segment connector whose last segment carries the arrowhead."""
    xs, ys = zip(*pts[:-1])
    ax.plot(xs, ys, color=color, lw=lw, zorder=4,
            linestyle=(0, (4, 2.5)) if dashed else "-",
            solid_capstyle="butt")
    arrow(ax, pts[-2], pts[-1], color=color, dashed=dashed, lw=lw)


def band(ax, x, y, w, h, label, kind="offline", lsize=7.2):
    """Large background region with a caption in its top-left corner."""
    fc, ec, tc, _ = KINDS[kind]
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0,rounding_size=7",
                                fc=fc, ec=ec, lw=0.8, zorder=0))
    ax.text(x + 8, y + h - 9, label, fontsize=lsize, fontweight="bold",
            color=tc, ha="left", va="center", zorder=1)


def legend(ax, x, y, kinds, size=6.8, dx=None):
    """Horizontal status legend starting at (x, y)."""
    cx = x
    for k in kinds:
        fc, ec, tc, dashed = KINDS[k]
        ax.add_patch(FancyBboxPatch((cx, y - 5), 14, 10,
                                    boxstyle="round,pad=0,rounding_size=2",
                                    fc=fc, ec=ec, lw=1.0,
                                    linestyle=(0, (3, 2)) if dashed else "-"))
        label = KIND_LABELS[k]
        ax.text(cx + 19, y, label, fontsize=size, va="center", color=INK)
        cx += dx if dx else 34 + len(label) * size * 0.80


def save(fig, name, out_dir=OUT_DIR):
    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    p = out_dir / f"{name}.png"
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    return p
