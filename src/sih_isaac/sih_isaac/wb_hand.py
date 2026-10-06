"""A shaded right hand holding a black whiteboard marker, as an RGBA sprite.

    img, tip = make_hand(scale)    # tip = (x, y) of the marker nib in img

Built from smooth shapes with soft shading rather than a photo, so it is free
of licence questions and stays sharp at 4K. `scale` 1.0 suits a 1080p frame;
use 2.0 for 4K.
"""
import math

import numpy as np
from PIL import Image, ImageChops, ImageDraw, ImageFilter

SKIN = (226, 176, 146)
SKIN_DARK = (184, 126, 96)
SKIN_LIGHT = (242, 205, 180)
NAIL = (238, 205, 190)


def _mask(size, draw_fn):
    m = Image.new("L", size, 0)
    draw_fn(ImageDraw.Draw(m))
    return m


def _capsule(d, a, b, r, fill=255):
    d.line([a, b], fill=fill, width=int(2 * r))
    for p in (a, b):
        d.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill=fill)


def _shade(mask, base, light, dark, light_dir=(-1, -1), soft=40):
    """Fill `mask` with a lit gradient: light toward light_dir, dark opposite,
    plus darkening near the shape's edge (rounded-volume look)."""
    w, h = mask.size
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    lx, ly = light_dir
    n = math.hypot(lx, ly)
    proj = (xx * lx + yy * ly) / n
    m = np.asarray(mask, np.float32) / 255
    if m.max() == 0:
        return Image.new("RGBA", mask.size, (0, 0, 0, 0))
    sel = m > 0.5
    lo, hi = proj[sel].min(), proj[sel].max()
    t = np.clip((proj - lo) / max(hi - lo, 1), 0, 1)          # 1 = lit side
    # edge darkening: distance from the edge via a blurred mask
    inner = np.asarray(mask.filter(ImageFilter.GaussianBlur(soft)), np.float32) / 255
    vol = np.clip(inner * 1.6, 0, 1)
    b, l_, d_ = (np.array(c, np.float32) for c in (base, light, dark))
    col = d_ + (b - d_) * vol[..., None]
    col = col + (l_ - col) * (t[..., None] * 0.55 * vol[..., None])
    rgba = np.dstack([np.clip(col, 0, 255), m * 255]).astype(np.uint8)
    return Image.fromarray(rgba, "RGBA")


def make_hand(scale=1.0):
    S = 2                                   # supersample, downsampled at the end
    k = scale * S
    W, H = int(640 * k), int(760 * k)
    P = lambda x, y: (x * k, y * k)         # design units -> pixels
    out = Image.new("RGBA", (W, H), (0, 0, 0, 0))

    # marker axis: nib at (40, 40), running down-right
    ang = math.radians(33)
    ux, uy = math.cos(ang), math.sin(ang)
    px, py = -uy, ux                        # perpendicular (toward the palm)
    tip = (40, 40)

    def along(t, off=0.0):
        return (tip[0] + ux * t + px * off, tip[1] + uy * t + py * off)

    def quad(t0, t1, r0, r1):
        pts = [along(t0, -r0), along(t1, -r1), along(t1, r1), along(t0, r0)]
        return [P(*p) for p in pts]

    # ---- soft shadow of the whole hand on the board
    shadow = _mask((W, H), lambda d: (
        _capsule(d, P(*along(150, 60)), P(560, 740), 150 * k),
        _capsule(d, P(*along(90, 5)), P(*along(250, 40)), 46 * k),
        d.polygon(quad(20, 330, 16, 22), fill=255)))
    shadow = ImageChops.offset(shadow, int(34 * k), int(40 * k))
    shadow = shadow.filter(ImageFilter.GaussianBlur(28 * k))
    sh = Image.new("RGBA", (W, H), (60, 60, 70, 0))
    sh.putalpha(shadow.point(lambda v: int(v * 0.30)))
    out.alpha_composite(sh)

    # ---- forearm and back of the hand (behind the marker)
    arm = _mask((W, H), lambda d: _capsule(d, P(*along(185, 95)), P(600, 800), 118 * k))
    out.alpha_composite(_shade(arm, SKIN, SKIN_LIGHT, SKIN_DARK, (-1, -0.4), 60 * k))
    palm = _mask((W, H), lambda d: d.ellipse(
        [P(*along(170, 40))[0] - 150 * k, P(*along(170, 40))[1] - 125 * k,
         P(*along(170, 40))[0] + 150 * k, P(*along(170, 40))[1] + 125 * k], fill=255))
    out.alpha_composite(_shade(palm, SKIN, SKIN_LIGHT, SKIN_DARK, (-1, -1), 55 * k))

    # curled middle / ring / little fingers under the marker
    for i, (t, off, r) in enumerate(((150, 98, 34), (205, 118, 31), (255, 128, 27))):
        f = _mask((W, H), lambda d, t=t, off=off, r=r: _capsule(
            d, P(*along(t - 25, off - 40)), P(*along(t + 10, off)), r * k))
        out.alpha_composite(_shade(f, SKIN, SKIN_LIGHT, SKIN_DARK, (-1, -1), 16 * k))

    # ---- the marker: felt nib, cone, black barrel, white band, black cap end
    mk = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(mk)
    d.polygon(quad(0, 16, 4, 9), fill=(40, 40, 44, 255))              # nib
    d.polygon(quad(16, 44, 11, 17), fill=(70, 70, 76, 255))           # cone
    d.polygon(quad(44, 250, 19, 19), fill=(26, 26, 30, 255))          # barrel
    d.polygon(quad(250, 318, 19, 19), fill=(236, 236, 238, 255))      # white band
    d.polygon(quad(318, 350, 20, 20), fill=(22, 22, 26, 255))         # end cap
    # label stripes and a specular line along the barrel
    for t0 in (80, 104, 128):
        d.polygon(quad(t0, t0 + 12, 19, 19), fill=(58, 58, 64, 255))
    d.line([P(*along(48, -12)), P(*along(345, -12))], fill=(120, 120, 128, 200),
           width=int(4 * k))
    d.line([P(*along(250, -13)), P(*along(316, -13))], fill=(255, 255, 255, 230),
           width=int(5 * k))
    out.alpha_composite(mk)

    # ---- index finger over the barrel, pointing along it toward the nib
    idx = _mask((W, H), lambda d: _capsule(d, P(*along(78, -14)), P(*along(205, -46)), 27 * k))
    out.alpha_composite(_shade(idx, SKIN, SKIN_LIGHT, SKIN_DARK, (-1, -1), 18 * k))
    nail = _mask((W, H), lambda d: d.ellipse(
        [P(*along(80, -16))[0] - 17 * k, P(*along(80, -16))[1] - 13 * k,
         P(*along(80, -16))[0] + 15 * k, P(*along(80, -16))[1] + 11 * k], fill=255))
    out.alpha_composite(_shade(nail, NAIL, (250, 232, 224), (206, 170, 156), (-1, -1), 5 * k))

    # ---- thumb on the near side of the barrel
    th = _mask((W, H), lambda d: _capsule(d, P(*along(96, 26)), P(*along(215, 78)), 29 * k))
    out.alpha_composite(_shade(th, SKIN, SKIN_LIGHT, SKIN_DARK, (-1, -0.6), 18 * k))
    tnail = _mask((W, H), lambda d: d.ellipse(
        [P(*along(98, 26))[0] - 16 * k, P(*along(98, 26))[1] - 12 * k,
         P(*along(98, 26))[0] + 14 * k, P(*along(98, 26))[1] + 12 * k], fill=255))
    out.alpha_composite(_shade(tnail, NAIL, (250, 232, 224), (206, 170, 156), (-1, -1), 5 * k))

    # knuckle creases
    d2 = ImageDraw.Draw(out)
    for t, off in ((180, 58), (214, 76)):
        a, b = P(*along(t, off)), P(*along(t + 14, off + 16))
        d2.line([a, b], fill=(*SKIN_DARK, 150), width=int(3 * k))

    out = out.resize((W // S, H // S), Image.LANCZOS)
    return out, (tip[0] * scale, tip[1] * scale)


if __name__ == "__main__":
    import sys
    img, tip = make_hand(float(sys.argv[2]) if len(sys.argv) > 2 else 1.0)
    bg = Image.new("RGBA", img.size, (251, 251, 249, 255))
    bg.alpha_composite(img)
    ImageDraw.Draw(bg).ellipse([tip[0] - 4, tip[1] - 4, tip[0] + 4, tip[1] + 4],
                               outline=(255, 0, 0, 255))
    bg.convert("RGB").save(sys.argv[1])
    print(img.size, tip)
