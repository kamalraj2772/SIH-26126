"""Dashboard compositing for the demo video."""
from __future__ import annotations

import numpy as np
import cv2

W, H = 1920, 1080
BG = (18, 20, 24)
PANEL = (28, 31, 37)
LINE = (58, 63, 72)
TXT = (222, 226, 232)
DIM = (148, 155, 166)
ACCENT = (255, 176, 46)      # amber - vehicle / live
GOOD = (86, 204, 128)
WARN = (240, 96, 84)
BLUE = (96, 172, 255)
VIOLET = (186, 140, 255)

F = cv2.FONT_HERSHEY_DUPLEX
FS = cv2.FONT_HERSHEY_SIMPLEX

# Panel rectangles: (x0, y0, x1, y1)
R_HEAD = (0, 0, W, 58)
R_MAIN = (0, 58, 1160, 714)
R_GMAP = (0, 714, 580, 1080)
R_COST = (580, 714, 1160, 1080)
R_RGB = (1160, 58, 1920, 486)
R_DEPTH = (1160, 486, 1920, 914)
R_TEL = (1160, 914, 1920, 1080)


def new_canvas():
    c = np.zeros((H, W, 3), np.uint8)
    c[:] = BG
    return c


def panel(canvas, rect, title, subtitle=None):
    x0, y0, x1, y1 = rect
    cv2.rectangle(canvas, (x0 + 1, y0 + 1), (x1 - 2, y1 - 2), PANEL, -1)
    cv2.rectangle(canvas, (x0 + 1, y0 + 1), (x1 - 2, y1 - 2), LINE, 1)
    cv2.putText(canvas, title, (x0 + 14, y0 + 25), F, 0.52, TXT, 1, cv2.LINE_AA)
    if subtitle:
        cv2.putText(canvas, subtitle, (x0 + 14, y0 + 44), FS, 0.42, DIM, 1, cv2.LINE_AA)
    return (x0 + 10, y0 + (54 if subtitle else 36), x1 - 10, y1 - 10)


def blit(canvas, img, box, keep_aspect=True):
    bx0, by0, bx1, by1 = box
    bw, bh = bx1 - bx0, by1 - by0
    if bw <= 0 or bh <= 0:
        return box
    ih, iw = img.shape[:2]
    if keep_aspect:
        s = min(bw / iw, bh / ih)
        nw, nh = max(1, int(iw * s)), max(1, int(ih * s))
    else:
        nw, nh = bw, bh
    r = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    ox = bx0 + (bw - nw) // 2
    oy = by0 + (bh - nh) // 2
    canvas[oy:oy + nh, ox:ox + nw] = r
    return (ox, oy, ox + nw, oy + nh)


def text_rows(canvas, x, y, rows, dy=22, key_w=150):
    for k, v, col in rows:
        cv2.putText(canvas, k, (x, y), FS, 0.44, DIM, 1, cv2.LINE_AA)
        cv2.putText(canvas, v, (x + key_w, y), FS, 0.47, col, 1, cv2.LINE_AA)
        y += dy
    return y


def header(canvas, title, right_text, phase):
    cv2.rectangle(canvas, (0, 0), (W, 57), (12, 14, 17), -1)
    cv2.line(canvas, (0, 57), (W, 57), ACCENT, 2)
    cv2.putText(canvas, title, (18, 37), F, 0.72, TXT, 1, cv2.LINE_AA)
    cv2.putText(canvas, phase, (760, 36), FS, 0.52, ACCENT, 1, cv2.LINE_AA)
    (tw, _), _ = cv2.getTextSize(right_text, FS, 0.5, 1)
    cv2.putText(canvas, right_text, (W - tw - 18, 36), FS, 0.5, DIM, 1, cv2.LINE_AA)


def depth_colormap(depth, dmin, dmax):
    d = np.clip((depth - dmin) / (dmax - dmin), 0, 1)
    bad = ~np.isfinite(depth)
    d = np.nan_to_num(d, nan=1.0)
    img = cv2.applyColorMap((255 * (1.0 - d)).astype(np.uint8), cv2.COLORMAP_TURBO)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img[bad] = (26, 28, 33)
    return img


def hillshade(dem, res):
    gy, gx = np.gradient(dem, res)
    sh = np.clip(0.55 + 0.9 * (gx * 0.6 + gy * 0.6), 0.15, 1.0)
    base = np.clip((dem - dem.min()) / max(1e-6, float(np.ptp(dem))), 0, 1)
    img = np.stack([0.42 + 0.30 * base, 0.46 + 0.28 * base, 0.40 + 0.22 * base], -1)
    img *= sh[..., None]
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)


def badge(img, text, org, colour, scale=0.5):
    """Small dark plate behind text so it stays readable over bright imagery."""
    (tw, th), _ = cv2.getTextSize(text, FS, scale, 1)
    x, y = org
    ov = img.copy()
    cv2.rectangle(ov, (x, y), (x + tw + 14, y + th + 12), (10, 12, 15), -1)
    cv2.addWeighted(ov, 0.62, img, 0.38, 0, img)
    cv2.putText(img, text, (x + 7, y + th + 4), FS, scale, colour, 1, cv2.LINE_AA)


def card(lines, seconds, fps, title=None):
    """A full-frame title/summary card, returned as a list of identical frames."""
    c = new_canvas()
    cv2.rectangle(c, (0, 0), (W, 6), ACCENT, -1)
    y = 132
    if title:
        cv2.putText(c, title, (110, y), F, 1.35, TXT, 2, cv2.LINE_AA)
        cv2.line(c, (110, y + 26), (W - 110, y + 26), LINE, 1)
        y += 92
    for text, kind in lines:
        if kind == "h":
            y += 26
            cv2.putText(c, text, (110, y), F, 0.72, ACCENT, 1, cv2.LINE_AA)
            y += 40
        elif kind == "b":
            cv2.circle(c, (122, y - 6), 3, BLUE, -1, cv2.LINE_AA)
            cv2.putText(c, text, (142, y), FS, 0.62, TXT, 1, cv2.LINE_AA)
            y += 36
        elif kind == "k":
            # "LABEL|first line|second line" -- aligned two-column row.
            parts = text.split("|")
            cv2.putText(c, parts[0], (142, y), F, 0.60, ACCENT, 1, cv2.LINE_AA)
            for j, body in enumerate(parts[1:]):
                cv2.putText(c, body, (330, y + j * 30), FS,
                            0.58 if j == 0 else 0.54,
                            TXT if j == 0 else DIM, 1, cv2.LINE_AA)
            y += 30 * len(parts[1:]) + 14
        elif kind == "d":
            cv2.putText(c, text, (142, y), FS, 0.58, DIM, 1, cv2.LINE_AA)
            y += 32
        elif kind == "g":
            cv2.putText(c, text, (142, y), FS, 0.62, GOOD, 1, cv2.LINE_AA)
            y += 34
        else:
            y += 18
    return [c] * int(seconds * fps)
