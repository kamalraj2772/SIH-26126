"""A small whiteboard-animation engine: a real hand draws each element.

    sc = Scene()                     # 3840x2160, 60 fps, white board
    sc.stroke(points, color=INK)     # drawn along its path, marker tip on it
    sc.circle((x, y), r)
    sc.text("QSLAM", (x, y), size=300)   # written left to right
    sc.wait(0.5)
    sc.render("out.mp4", hold=1.0)

Timing is a cursor: each element starts after the previous one plus the
time the hand needs to travel between them, and draws at a steady speed
(stroke length / text width over a speed), the way a practised illustrator
works. Every element is rasterised once, with a per-pixel "when is this ink
laid down" map, so a frame is a threshold and a blend -- no re-drawing.

The hand is a cut-out from the team's own pitch video (assets/
whiteboard_hand.png): the same hand and marker viewers see in the rest of it.
"""
import json
import math
import pathlib
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

ASSETS = pathlib.Path(__file__).resolve().parents[1] / "assets"
FONT = str(ASSETS / "fonts/PatrickHand-Regular.ttf")

W, H, FPS = 3840, 2160, 60
BOARD = (252, 252, 250)
INK = (28, 28, 32)
RED = (212, 48, 46)
GREEN = (26, 138, 78)
BLUE = (24, 92, 190)
GREY = (120, 120, 126)

_HAND = None


def hand():
    """(sprite float32 RGBA, shadow alpha float32, tip (x, y))."""
    global _HAND
    if _HAND is None:
        spr = np.asarray(Image.open(ASSETS / "whiteboard_hand.png")).astype(np.float32) / 255
        tip = json.loads((ASSETS / "whiteboard_hand.json").read_text())["tip"]
        a = Image.fromarray((spr[..., 3] * 255).astype(np.uint8))
        sh = np.asarray(a.filter(ImageFilter.GaussianBlur(26)), np.float32) / 255 * 0.20
        _HAND = (spr, sh, tip)
    return _HAND


def ease(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


class Element:
    """Ink in a box: alpha, colour, and the time each pixel gets drawn."""

    def __init__(self, x0, y0, alpha, color, when, path, t0, t1):
        self.x0, self.y0 = x0, y0
        self.alpha = alpha                      # float32 (h, w), 0..1
        self.color = np.array(color, np.float32) / 255
        self.when = when                        # float32 (h, w), 0..1 progress
        self.path = path                        # progress 0..1 -> tip (x, y)
        self.t0, self.t1 = t0, t1

    def progress(self, t):
        if t <= self.t0:
            return 0.0
        if t >= self.t1:
            return 1.0
        return (t - self.t0) / (self.t1 - self.t0)


def _polyline_len(p):
    return float(np.sum(np.hypot(*np.diff(p, axis=0).T))) if len(p) > 1 else 0.0


def _resample(p, step=4.0):
    """Points every `step` px along a polyline (even pacing for the hand)."""
    p = np.asarray(p, np.float64)
    seg = np.hypot(*np.diff(p, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] == 0:
        return p[:1]
    n = max(int(s[-1] / step), 2)
    q = np.linspace(0, s[-1], n)
    return np.column_stack([np.interp(q, s, p[:, 0]), np.interp(q, s, p[:, 1])])


class Scene:
    def __init__(self, speed=2600.0, text_speed=2300.0, travel_speed=5200.0):
        self.els = []
        self.t = 0.35                           # the hand enters first
        self.speed = speed
        self.text_speed = text_speed
        self.travel_speed = travel_speed
        self.last_tip = None
        self.fixed = []                         # (t, callable(canvas)) extras

    # ---------------------------------------------------------- timeline --
    def _start(self, first_tip):
        if self.last_tip is not None:
            d = math.dist(self.last_tip, first_tip)
            self.t += min(max(d / self.travel_speed, 0.045), 0.28)
        return self.t

    def wait(self, s):
        self.t += s

    # ------------------------------------------------------------ strokes --
    def stroke(self, pts, color=INK, width=12, dur=None, speed=None):
        p = _resample(pts)
        L = _polyline_len(p)
        dur = dur or max(L / (speed or self.speed), 0.08)
        t0 = self._start(tuple(p[0]))
        pad = width + 6
        x0, y0 = np.floor(p.min(0) - pad).astype(int)
        x1, y1 = np.ceil(p.max(0) + pad).astype(int)
        w, h = x1 - x0, y1 - y0
        S = 3
        ink = Image.new("L", (w * S, h * S), 0)
        d = ImageDraw.Draw(ink)
        q = (p - (x0, y0)) * S
        d.line([tuple(v) for v in q], fill=255, width=int(width * S), joint="curve")
        r = width * S / 2
        for v in (q[0], q[-1]):
            d.ellipse([v[0] - r, v[1] - r, v[0] + r, v[1] + r], fill=255)
        alpha = np.asarray(ink.resize((w, h), Image.LANCZOS), np.float32) / 255
        # when: arc-length fraction of the nearest path sample, painted in order
        when = Image.new("I", (w, h), 1 << 20)
        dw = ImageDraw.Draw(when)
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
        s = (s / max(s[-1], 1e-6) * 65535).astype(int)
        qq = p - (x0, y0)
        rr = width / 2 + 3
        for i in range(len(qq) - 1):
            dw.line([tuple(qq[i]), tuple(qq[i + 1])], fill=int(s[i + 1]), width=int(width + 6))
            dw.ellipse([qq[i + 1][0] - rr, qq[i + 1][1] - rr, qq[i + 1][0] + rr,
                        qq[i + 1][1] + rr], fill=int(s[i + 1]))
        dw.ellipse([qq[0][0] - rr, qq[0][1] - rr, qq[0][0] + rr, qq[0][1] + rr], fill=0)
        when = np.asarray(when, np.float32) / 65535
        seg_s = s / 65535.0

        def path(u, p=p, seg_s=seg_s):
            i = int(np.searchsorted(seg_s, u))
            return tuple(p[min(i, len(p) - 1)])
        self.els.append(Element(x0, y0, alpha, color, when, path, t0, t0 + dur))
        self.t = t0 + dur
        self.last_tip = tuple(p[-1])
        return self

    def line(self, a, b, **kw):
        return self.stroke([a, b], **kw)

    def poly(self, pts, closed=False, **kw):
        pts = list(pts) + ([pts[0]] if closed else [])
        return self.stroke(pts, **kw)

    def circle(self, c, r, start=-90, sweep=360, **kw):
        n = max(int(abs(sweep) / 4), 8)
        a = np.radians(np.linspace(start, start + sweep, n))
        return self.stroke(np.column_stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)]), **kw)

    def rect(self, x0, y0, x1, y1, r=0, **kw):
        if r <= 0:
            return self.poly([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], closed=True, **kw)
        pts = []
        for cx, cy, a0 in ((x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0),
                           (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
            for a in np.radians(np.linspace(a0, a0 + 90, 10)):
                pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
        return self.poly(pts, closed=True, **kw)

    def arrow(self, a, b, head=46, **kw):
        self.line(a, b, **kw)
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        for s in (1, -1):
            e = (b[0] - head * math.cos(ang + s * 0.45), b[1] - head * math.sin(ang + s * 0.45))
            self.stroke([b, e], dur=0.12, **{k: v for k, v in kw.items() if k != "dur"})
        return self

    def tick(self, c, s=60, color=GREEN, width=16):
        return self.stroke([(c[0] - s, c[1]), (c[0] - s * 0.3, c[1] + s * 0.75),
                            (c[0] + s * 1.1, c[1] - s * 0.9)], color=color, width=width)

    def cross(self, c, s=50, color=RED, width=16):
        self.line((c[0] - s, c[1] - s), (c[0] + s, c[1] + s), color=color, width=width)
        return self.line((c[0] + s, c[1] - s), (c[0] - s, c[1] + s), color=color, width=width)

    # --------------------------------------------------------------- text --
    def text(self, s, xy, size=110, color=INK, anchor="mm", dur=None, speed=None,
             text_speed=None):
        speed = speed or text_speed
        font = ImageFont.truetype(FONT, int(size))
        l, t, r, b = font.getbbox(s, anchor=anchor)
        pad = 8
        x0, y0 = int(xy[0] + l - pad), int(xy[1] + t - pad)
        w, h = int(r - l + 2 * pad), int(b - t + 2 * pad)
        im = Image.new("L", (w, h), 0)
        ImageDraw.Draw(im).text((xy[0] - x0, xy[1] - y0), s, font=font, fill=255,
                                anchor=anchor)
        alpha = np.asarray(im, np.float32) / 255
        when = np.tile(np.linspace(0, 1, w, dtype=np.float32), (h, 1))
        dur = dur or max(w / (speed or self.text_speed), 0.25)
        ys = y0 + h * 0.62
        n_ch = max(len(s), 1)
        start = (x0 + pad, ys)
        t0 = self._start(start)

        def path(u, x0=x0, w=w, pad=pad, ys=ys, h=h, n_ch=n_ch):
            x = x0 + pad + u * (w - 2 * pad)
            y = ys - 0.28 * h * abs(math.sin(math.pi * u * n_ch))  # pen up/down per letter
            return (x, y)
        self.els.append(Element(x0, y0, alpha, color, when, path, t0, t0 + dur))
        self.t = t0 + dur
        self.last_tip = path(1.0)
        return self

    # ------------------------------------------------------------- render --
    def hand_at(self, t):
        """Tip position at time t, or None when the hand is off the board."""
        els = self.els
        if not els:
            return None
        off = (W + 400, H + 200)
        if t < els[0].t0:                            # entering
            u = ease((t - (els[0].t0 - 0.35)) / 0.35)
            a, b = off, els[0].path(0)
            return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u)
        for i, e in enumerate(els):
            if e.t0 <= t <= e.t1:
                return e.path(e.progress(t))
            nxt = els[i + 1] if i + 1 < len(els) else None
            if nxt is not None and e.t1 < t < nxt.t0:  # travelling between
                u = ease((t - e.t1) / (nxt.t0 - e.t1))
                a, b = e.path(1.0), nxt.path(0.0)
                lift = 60 * math.sin(math.pi * u)
                return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u - lift)
        last = els[-1]
        u = ease((t - last.t1) / 0.45)                  # leaving
        if u >= 1:
            return None
        a = last.path(1.0)
        return (a[0] + (off[0] - a[0]) * u, a[1] + (off[1] - a[1]) * u)

    @staticmethod
    def blend(canvas, x0, y0, rgb, alpha):
        """Alpha-blend a colour (or an image) into the canvas, clipped."""
        h, w = alpha.shape
        X0, Y0 = max(x0, 0), max(y0, 0)
        X1, Y1 = min(x0 + w, canvas.shape[1]), min(y0 + h, canvas.shape[0])
        if X1 <= X0 or Y1 <= Y0:
            return
        a = alpha[Y0 - y0:Y1 - y0, X0 - x0:X1 - x0][..., None]
        src = rgb if np.ndim(rgb) == 1 else rgb[Y0 - y0:Y1 - y0, X0 - x0:X1 - x0]
        reg = canvas[Y0:Y1, X0:X1]
        reg *= (1 - a)
        reg += a * src

    def draw_hand(self, canvas, tip):
        spr, sh, (tx, ty) = hand()
        x0, y0 = int(round(tip[0] - tx)), int(round(tip[1] - ty))
        self.blend(canvas, x0 + 38, y0 + 46, np.array([0.24, 0.24, 0.27], np.float32), sh)
        self.blend(canvas, x0, y0, spr[..., :3], spr[..., 3])

    def frames(self, hold=1.0, extra=None):
        """Yield float32 RGB frames. `extra(canvas, t)` may paint on top."""
        total = self.t + 0.45 + hold
        n = int(round(total * FPS))
        base = np.empty((H, W, 3), np.float32)
        base[:] = np.array(BOARD, np.float32) / 255
        done = [False] * len(self.els)
        for f in range(n):
            t = f / FPS
            canvas = None
            for i, e in enumerate(self.els):
                if done[i]:
                    continue
                if t >= e.t1:
                    self.blend(base, e.x0, e.y0, e.color, e.alpha)
                    done[i] = True
            canvas = base.copy()
            for i, e in enumerate(self.els):
                if not done[i] and e.t0 <= t < e.t1:
                    p = e.progress(t)
                    self.blend(canvas, e.x0, e.y0, e.color, e.alpha * (e.when <= p))
            if extra is not None:
                extra(canvas, t)
            tip = self.hand_at(t)
            if tip is not None:
                self.draw_hand(canvas, tip)
            yield canvas

    def render(self, out, ffmpeg, hold=1.0, extra=None, crf=17):
        proc = subprocess.Popen(
            [ffmpeg, "-y", "-nostdin", "-loglevel", "error", "-f", "rawvideo",
             "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-framerate", str(FPS), "-i", "-",
             "-c:v", "libx264", "-preset", "medium", "-crf", str(crf),
             "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out)],
            stdin=subprocess.PIPE)
        last = None
        for fr in self.frames(hold, extra):
            last = (np.clip(fr, 0, 1) * 255).astype(np.uint8)
            proc.stdin.write(last.tobytes())
        proc.stdin.close()
        proc.wait()
        return last
