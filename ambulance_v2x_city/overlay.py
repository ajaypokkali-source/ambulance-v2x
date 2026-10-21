"""
overlay.py

Draws text as small PNG images so the SUMO GUI can show it as POIs.
(SUMO-GUI's own text-label settings are unreliable, images always render.)

Images are cached in ./.overlay, keyed by their content.
"""

import hashlib
import os

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, ".overlay")
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

_fonts = {}


def _font(size):
    if size not in _fonts:
        try:
            _fonts[size] = ImageFont.truetype(FONT, size)
        except OSError:
            _fonts[size] = ImageFont.load_default()
    return _fonts[size]


def _luminance(rgb):
    r, g, b = rgb
    return 0.299 * r + 0.587 * g + 0.114 * b


def _render(lines, fg, bg, size, pad, min_w=0, min_h=0):
    key = hashlib.md5(repr((lines, fg, bg, size, pad, min_w, min_h))
                      .encode()).hexdigest()[:16]
    path = os.path.join(CACHE, f"{key}.png")
    font = _font(size)
    line_h = int(size * 1.3)
    probe = ImageDraw.Draw(Image.new("RGB", (1, 1)))
    w = max([int(probe.textlength(t, font=font)) for t in lines] + [1])
    w = max(w + 2 * pad, min_w)
    h = max(line_h * len(lines) + 2 * pad, min_h)
    if not os.path.exists(path):
        os.makedirs(CACHE, exist_ok=True)
        img = Image.new("RGBA", (w, h), bg)
        draw = ImageDraw.Draw(img)
        for i, text in enumerate(lines):
            draw.text((pad, pad + i * line_h), text, font=font, fill=fg)
        img.save(path)
    return path, w, h


def label(text, color=(255, 255, 255), size=30, m_per_px=0.5):
    """A small tag. Returns (png path, width in m, height in m)."""
    fg = (0, 0, 0, 255) if _luminance(color) > 150 else (255, 255, 255, 255)
    path, w, h = _render([text], fg, color + (225,), size, 8)
    return path, w * m_per_px, h * m_per_px


def junction_tag(text, m_per_px=0.5):
    path, w, h = _render([text], (255, 255, 255, 255), (30, 30, 30, 215),
                         30, 8)
    return path, w * m_per_px, h * m_per_px


def board(lines, width_px=1700, rows=6, m_per_px=0.6):
    """A fixed-size text board. Returns (png path, width in m, height in m)."""
    lines = list(lines)[:rows]
    lines += [""] * (rows - len(lines))
    size = 30
    path, w, h = _render(lines, (255, 255, 255, 255), (20, 20, 20, 225),
                         size, 14, min_w=width_px)
    return path, w * m_per_px, h * m_per_px
