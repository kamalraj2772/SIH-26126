"""Layout toolkit for the SIH26126 / QSLAM detailed technical PDF.

Fonts, paragraph styles, numbered equations (matplotlib mathtext), numbered
figures and tables, status badges, callouts, and a DocTemplate that builds
the table of contents and PDF bookmarks from the headings.
"""
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from fontTools.ttLib import TTFont  # noqa: E402
from reportlab.lib import colors  # noqa: E402
from reportlab.lib.enums import TA_CENTER, TA_LEFT  # noqa: E402
from reportlab.lib.pagesizes import A4  # noqa: E402
from reportlab.lib.styles import ParagraphStyle  # noqa: E402
from reportlab.pdfbase import pdfmetrics  # noqa: E402
from reportlab.pdfbase.ttfonts import TTFont as RLTTFont  # noqa: E402
from reportlab.platypus import (BaseDocTemplate, CondPageBreak, Flowable,  # noqa: E402
                                Frame, Image, KeepTogether, PageTemplate,
                                Paragraph, Spacer, Table, TableStyle,
                                XPreformatted)
from reportlab.platypus.tableofcontents import TableOfContents  # noqa: E402

WS = pathlib.Path("/home/qbotix-rover/sih_ws")
FIG_DIR = WS / "build/tech_doc_figs"
EQ_DIR = WS / "build/tech_doc_figs/eq"

# ------------------------------------------------------------------ fonts --
LATO = pathlib.Path("/usr/share/fonts/truetype/lato")
DJV = pathlib.Path("/usr/share/fonts/truetype/dejavu")
for name, f in [("Lato", "Lato-Regular.ttf"), ("Lato-Bold", "Lato-Bold.ttf"),
                ("Lato-Italic", "Lato-Italic.ttf"),
                ("Lato-BoldItalic", "Lato-BoldItalic.ttf"),
                ("Lato-Semibold", "Lato-Semibold.ttf"),
                ("Lato-Light", "Lato-Light.ttf")]:
    pdfmetrics.registerFont(RLTTFont(name, str(LATO / f)))
pdfmetrics.registerFontFamily("Lato", normal="Lato", bold="Lato-Bold",
                              italic="Lato-Italic",
                              boldItalic="Lato-BoldItalic")
pdfmetrics.registerFont(RLTTFont("DejaVu", str(DJV / "DejaVuSans.ttf")))
pdfmetrics.registerFont(RLTTFont("DejaVu-Bold", str(DJV / "DejaVuSans-Bold.ttf")))
pdfmetrics.registerFontFamily("DejaVu", normal="DejaVu", bold="DejaVu-Bold",
                              italic="DejaVu", boldItalic="DejaVu-Bold")
pdfmetrics.registerFont(RLTTFont("Mono", str(DJV / "DejaVuSansMono.ttf")))
pdfmetrics.registerFont(RLTTFont("Mono-Bold", str(DJV / "DejaVuSansMono-Bold.ttf")))
pdfmetrics.registerFontFamily("Mono", normal="Mono", bold="Mono-Bold",
                              italic="Mono", boldItalic="Mono-Bold")

_LATO_CMAP = set(TTFont(str(LATO / "Lato-Regular.ttf")).getBestCmap())


def fx(text: str) -> str:
    """Route characters Lato lacks (e.g. ∇ ∈ ✓) to DejaVu, outside tags."""
    out, in_tag = [], False
    for ch in text:
        if ch == "<":
            in_tag = True
        if not in_tag and ord(ch) > 127 and ord(ch) not in _LATO_CMAP:
            out.append(f'<font name="DejaVu">{ch}</font>')
        else:
            out.append(ch)
        if ch == ">":
            in_tag = False
    return "".join(out).replace("<sub>", '<sub rise="1.6" size="6.8">')


# ----------------------------------------------------------------- colours --
NAVY = colors.HexColor("#1F4E79")
BLUE = colors.HexColor("#2E74B5")
INK = colors.HexColor("#1A1A1A")
GREY = colors.HexColor("#5F5F5F")
FAINT = colors.HexColor("#8A8A8A")
RULE = colors.HexColor("#D5DDE5")
LIGHT = colors.HexColor("#F2F6FA")
ORANGE = colors.HexColor("#C55A11")
ORANGE_BG = colors.HexColor("#FBF3E8")
GREEN = colors.HexColor("#2E7D32")
GREEN_BG = colors.HexColor("#EAF4EA")
RED = colors.HexColor("#C62828")
RED_BG = colors.HexColor("#FDECEC")
GREY_BG = colors.HexColor("#F4F4F4")
CODE_BG = colors.HexColor("#F5F7F9")

PAGE_W, PAGE_H = A4
LM, RM, TM, BM = 52, 52, 58, 50
TEXT_W = PAGE_W - LM - RM

# ------------------------------------------------------------------ styles --
body = ParagraphStyle("body", fontName="Lato", fontSize=9.4, leading=13.1,
                      textColor=INK, spaceAfter=5, alignment=TA_LEFT)
body_tight = ParagraphStyle("body_tight", parent=body, spaceAfter=2.5)
small = ParagraphStyle("small", parent=body, fontSize=8.4, leading=11.4,
                       textColor=GREY, spaceAfter=3)
caption = ParagraphStyle("caption", parent=body, fontName="Lato-Italic",
                         fontSize=8.2, leading=11, textColor=GREY,
                         alignment=TA_CENTER, spaceBefore=3, spaceAfter=9)
cell = ParagraphStyle("cell", parent=body, fontSize=8.2, leading=10.8,
                      spaceAfter=0)
cell_b = ParagraphStyle("cell_b", parent=cell, fontName="Lato-Bold",
                        textColor=NAVY)
cell_h = ParagraphStyle("cell_h", parent=cell, fontName="Lato-Bold",
                        textColor=colors.white)
h1 = ParagraphStyle("H1", fontName="Lato-Bold", fontSize=15.5, leading=19,
                    textColor=NAVY, spaceBefore=4, spaceAfter=7,
                    keepWithNext=1)
h1_plain = ParagraphStyle("H1plain", parent=h1)
h2 = ParagraphStyle("H2", fontName="Lato-Bold", fontSize=11.6, leading=15,
                    textColor=BLUE, spaceBefore=8, spaceAfter=4,
                    keepWithNext=1)
h3 = ParagraphStyle("H3", fontName="Lato-Bold", fontSize=9.9, leading=13,
                    textColor=INK, spaceBefore=5, spaceAfter=2.5,
                    keepWithNext=1)
bullet = ParagraphStyle("bullet", parent=body, leftIndent=13, bulletIndent=3,
                        spaceAfter=2.6)
code = ParagraphStyle("code", fontName="Mono", fontSize=7.6, leading=10,
                      textColor=colors.HexColor("#222222"),
                      backColor=CODE_BG, borderPadding=(5, 6, 5, 6),
                      leftIndent=6, rightIndent=6, spaceBefore=3,
                      spaceAfter=9)
eqnum = ParagraphStyle("eqnum", parent=body, alignment=2, textColor=GREY,
                       fontSize=8.8, spaceAfter=0)

# Part B (plain language) uses larger, friendlier type.
pb_body = ParagraphStyle("pb_body", parent=body, fontSize=10.8, leading=15.6,
                         spaceAfter=7)
pb_bullet = ParagraphStyle("pb_bullet", parent=pb_body, leftIndent=15,
                           bulletIndent=4, spaceAfter=4)
pb_h1 = ParagraphStyle("PB1", parent=h1, fontSize=17, leading=21,
                       textColor=colors.HexColor("#B0600F"))
pb_h2 = ParagraphStyle("PB2", parent=h2, fontSize=12.5, leading=16,
                       textColor=colors.HexColor("#B0600F"))


def P(text, style=body):
    return Paragraph(fx(text), style)


def B(text, style=bullet, sym="•"):
    return Paragraph(fx(text), style, bulletText=sym)


def H1(text):
    return P(text, h1)


def H2(text):
    return P(text, h2)


def H3(text):
    return P(text, h3)


def mono(text):
    return f'<font name="Mono" size="8.2">{text}</font>'


def code_block(text):
    return XPreformatted(text, code)


# ------------------------------------------------------------------ badges --
BADGES = {
    "impl": ("IMPLEMENTED · Isaac testbed", "#1E5E23", "#DCEFDD"),
    "proto": ("IMPLEMENTED · MuJoCo prototype", "#1F4E79", "#E3EAF7"),
    "gz": ("IMPLEMENTED · Gazebo Phase 1", "#1F4E79", "#E3EAF7"),
    "design": ("QSLAM DESIGN · planned", "#9A5208", "#F8E6CF"),
    "exp": ("EXPERIMENTAL / NOT WIRED", "#4A4A4A", "#E9E9E9"),
    "eval": ("EVALUATION ONLY", "#A52A2A", "#F9DADA"),
}


def badge(kind):
    label, fg, bg = BADGES[kind]
    return (f'<font name="Lato-Bold" size="7" color="{fg}" '
            f'backColor="{bg}">&nbsp;{label}&nbsp;</font>')


# --------------------------------------------------------------- equations --
_eq = {"n": 0}


def eq(latex, size=12.5, number=True, width_scale=1.0):
    """Render a display equation with Computer Modern mathtext."""
    EQ_DIR.mkdir(parents=True, exist_ok=True)
    _eq["n"] += 1
    n = _eq["n"]
    p = EQ_DIR / f"eq_{n:03d}.png"
    with plt.rc_context({"mathtext.fontset": "cm", "text.color": "#1A1A1A"}):
        fig = plt.figure(figsize=(0.1, 0.1))
        fig.text(0, 0, f"${latex}$", fontsize=size)
        fig.savefig(p, dpi=300, bbox_inches="tight", pad_inches=0.025,
                    transparent=True)
        plt.close(fig)
    from PIL import Image as PILImage
    with PILImage.open(p) as im:
        wpx, hpx = im.size
    w, h = wpx / 300 * 72 * width_scale, hpx / 300 * 72 * width_scale
    if w > TEXT_W - 50:
        s = (TEXT_W - 50) / w
        w, h = w * s, h * s
    img = Image(str(p), width=w, height=h)
    if not number:
        t = Table([[img]], colWidths=[TEXT_W])
    else:
        t = Table([[img, Paragraph(f"({n})", eqnum)]],
                  colWidths=[TEXT_W - 40, 40])
    t.setStyle(TableStyle([("ALIGN", (0, 0), (0, 0), "CENTER"),
                           ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                           ("TOPPADDING", (0, 0), (-1, -1), 2),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
                           ("LEFTPADDING", (0, 0), (-1, -1), 0),
                           ("RIGHTPADDING", (0, 0), (-1, -1), 0)]))
    return t


def eq_no():
    """Number of the most recently rendered equation."""
    return _eq["n"]


# ----------------------------------------------------------------- figures --
_fig = {"n": 0}
FIG_NUMS = {}


def fig_ref(name):
    return f"Figure {FIG_NUMS[name]}" if name in FIG_NUMS else "Figure ?"


def figure(name_or_path, cap, width=TEXT_W, max_h=None, key=None):
    """Numbered figure + caption, kept together. Missing file → placeholder."""
    p = pathlib.Path(name_or_path)
    if not p.suffix:
        p = FIG_DIR / f"{name_or_path}.png"
    _fig["n"] += 1
    n = _fig["n"]
    FIG_NUMS[key or p.stem] = n
    if p.exists():
        from PIL import Image as PILImage
        with PILImage.open(p) as im:
            wpx, hpx = im.size
        w = width
        h = w * hpx / wpx
        if max_h and h > max_h:
            h = max_h
            w = h * wpx / hpx
        img = Image(str(p), width=w, height=h)
    else:
        img = Table([[Paragraph(f"[missing figure: {p.name}]", small)]],
                    colWidths=[width], rowHeights=[120])
        img.setStyle(TableStyle([("BOX", (0, 0), (-1, -1), 0.5, RED)]))
    return KeepTogether([img, P(f"<b>Figure {n}.</b> {cap}", caption)])


def reset_counters():
    """Between the numbering pass and the real pass: keep FIG_NUMS, so text
    written before a figure can cite its final number."""
    _fig["n"] = 0
    _eq["n"] = 0
    _tab["n"] = 0


# ------------------------------------------------------------------ tables --
_tab = {"n": 0}


def table(rows, widths, header=True, size=None, zebra=True, head_bg=NAVY,
          cap=None, first_col_bold=False, align_top=True):
    """Rows of strings (auto-wrapped) → styled Table (+ optional caption)."""
    st_cell = cell if size is None else ParagraphStyle(
        "c", parent=cell, fontSize=size, leading=size * 1.32)
    st_b = cell_b if size is None else ParagraphStyle(
        "cb", parent=cell_b, fontSize=size, leading=size * 1.32)
    st_h = cell_h if size is None else ParagraphStyle(
        "ch", parent=cell_h, fontSize=size, leading=size * 1.32)
    data = []
    for r, row in enumerate(rows):
        out = []
        for c, v in enumerate(row):
            if not isinstance(v, str):
                out.append(v)
                continue
            if header and r == 0:
                out.append(Paragraph(fx(v), st_h))
            elif first_col_bold and c == 0:
                out.append(Paragraph(fx(v), st_b))
            else:
                out.append(Paragraph(fx(v), st_cell))
        data.append(out)
    t = Table(data, colWidths=widths, repeatRows=1 if header else 0)
    style = [("GRID", (0, 0), (-1, -1), 0.4, RULE),
             ("VALIGN", (0, 0), (-1, -1), "TOP" if align_top else "MIDDLE"),
             ("TOPPADDING", (0, 0), (-1, -1), 3),
             ("BOTTOMPADDING", (0, 0), (-1, -1), 3.2),
             ("LEFTPADDING", (0, 0), (-1, -1), 5),
             ("RIGHTPADDING", (0, 0), (-1, -1), 5)]
    if header:
        style += [("BACKGROUND", (0, 0), (-1, 0), head_bg)]
    if zebra:
        for r in range(1 if header else 0, len(rows)):
            if (r % 2 == 0) == header:
                style.append(("BACKGROUND", (0, r), (-1, r),
                              colors.HexColor("#F7F9FB")))
    t.setStyle(TableStyle(style))
    if cap:
        _tab["n"] += 1
        return [CondPageBreak(80),
                P(f"<b>Table {_tab['n']}.</b> {cap}",
                  ParagraphStyle("tc", parent=caption, spaceBefore=2,
                                 spaceAfter=3)),
                t, Spacer(1, 8)]
    return t


def callout(flowables_or_text, kind="info", width=TEXT_W, pad=8):
    fills = {"info": (LIGHT, BLUE), "design": (ORANGE_BG, ORANGE),
             "ok": (GREEN_BG, GREEN), "warn": (RED_BG, RED),
             "grey": (GREY_BG, FAINT)}
    bg, bar = fills[kind]
    content = flowables_or_text
    if isinstance(content, str):
        content = [P(content, ParagraphStyle("co", parent=body,
                                             spaceAfter=0))]
    t = Table([[content]], colWidths=[width])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), bg),
                           ("LINEBEFORE", (0, 0), (0, -1), 2.6, bar),
                           ("TOPPADDING", (0, 0), (-1, -1), pad - 1),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), pad),
                           ("LEFTPADDING", (0, 0), (-1, -1), pad + 3),
                           ("RIGHTPADDING", (0, 0), (-1, -1), pad)]))
    return KeepTogether([t, Spacer(1, 7)])


def side_by_side(left, right, lw=0.5, gap=12):
    """Two flowable lists in columns (lw = left fraction of text width)."""
    wl = (TEXT_W - gap) * lw
    wr = TEXT_W - gap - wl
    t = Table([[left, "", right]], colWidths=[wl, gap, wr])
    t.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"),
                           ("LEFTPADDING", (0, 0), (-1, -1), 0),
                           ("RIGHTPADDING", (0, 0), (-1, -1), 0),
                           ("TOPPADDING", (0, 0), (-1, -1), 0),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 0)]))
    return t


# ---------------------------------------------------------- doc template --
class SetPart(Flowable):
    """Zero-size marker that changes the running header's part label."""

    def __init__(self, label):
        super().__init__()
        self.label = label
        self.width = self.height = 0

    def wrap(self, *a):
        return 0, 0

    def draw(self):
        self.canv._doctemplate_part = self.label


class DocTemplate(BaseDocTemplate):
    def __init__(self, path, **kw):
        super().__init__(str(path), pagesize=A4, leftMargin=LM,
                         rightMargin=RM, topMargin=TM, bottomMargin=BM, **kw)
        frame = Frame(LM, BM, TEXT_W, PAGE_H - TM - BM, id="f",
                      leftPadding=0, rightPadding=0, topPadding=0,
                      bottomPadding=0)
        cover_frame = Frame(40, 36, PAGE_W - 80, PAGE_H - 72, id="c",
                            leftPadding=0, rightPadding=0, topPadding=0,
                            bottomPadding=0)
        self.addPageTemplates([
            PageTemplate(id="cover", frames=[cover_frame],
                         onPageEnd=self._cover_deco),
            PageTemplate(id="body", frames=[frame], onPageEnd=self._deco),
        ])
        self._bm = 0

    def _cover_deco(self, canv, doc):
        canv.saveState()
        canv.setFillColor(NAVY)
        canv.rect(0, PAGE_H - 14, PAGE_W, 14, fill=1, stroke=0)
        canv.setFillColor(ORANGE)
        canv.rect(0, PAGE_H - 17, PAGE_W, 3, fill=1, stroke=0)
        canv.setFillColor(NAVY)
        canv.rect(0, 0, PAGE_W, 8, fill=1, stroke=0)
        canv.restoreState()

    def _deco(self, canv, doc):
        part = getattr(canv, "_doctemplate_part", "")
        canv.saveState()
        canv.setStrokeColor(RULE)
        canv.setLineWidth(0.6)
        canv.line(LM, PAGE_H - 38, PAGE_W - RM, PAGE_H - 38)
        canv.setFont("Lato-Bold", 7.8)
        canv.setFillColor(NAVY)
        canv.drawString(LM, PAGE_H - 33, "QSLAM")
        canv.setFont("Lato", 7.8)
        canv.setFillColor(FAINT)
        canv.drawString(LM + 30, PAGE_H - 33,
                        "·  SIH26126  ·  Detailed System Description")
        canv.setFillColor(ORANGE if "Part B" in part else BLUE)
        canv.setFont("Lato-Semibold", 7.8)
        canv.drawRightString(PAGE_W - RM, PAGE_H - 33, part)
        canv.line(LM, 36, PAGE_W - RM, 36)
        canv.setFont("Lato", 7.6)
        canv.setFillColor(FAINT)
        canv.drawString(LM, 25, "Team QSLAM (Team ID 14)  ·  SIH 2026  ·  "
                        "Smart Automation  ·  Software")
        canv.setFont("Lato-Bold", 8.4)
        canv.setFillColor(NAVY)
        canv.drawRightString(PAGE_W - RM, 25, f"{doc.page}")
        canv.restoreState()

    def beforeDocument(self):
        self._bm = 0

    def afterFlowable(self, flowable):
        if not isinstance(flowable, Paragraph):
            return
        name = flowable.style.name
        level = {"H1": 0, "H2": 1, "PB1": 0, "PB2": 1}.get(name)
        if level is None:
            return
        text = flowable.getPlainText()
        key = f"bm{self._bm}"
        self._bm += 1
        self.canv.bookmarkPage(key)
        self.canv.addOutlineEntry(text, key, level=level, closed=level > 0)
        if level == 0:
            self.notify("TOCEntry", (level, text, self.page, key))


def toc():
    t = TableOfContents()
    t.dotsMinLevel = 0
    t.levelStyles = [
        ParagraphStyle("toc0", fontName="Lato-Semibold", fontSize=9.4,
                       leading=11.6, textColor=NAVY, leftIndent=0,
                       firstLineIndent=0, spaceBefore=0),
        ParagraphStyle("toc1", fontName="Lato", fontSize=8.3, leading=10.4,
                       textColor=INK, leftIndent=14, firstLineIndent=0),
    ]
    return t


__all__ = [n for n in dir() if not n.startswith("_")] + ["CondPageBreak",
                                                         "Spacer", "KeepTogether"]
