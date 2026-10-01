#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2026 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

"""Theme, fonts and the two drawing back ends (PNG via Pillow, SVG as text)."""

import os

THEME = {
    "bg": "#0b0d14",
    "grid": "#141826",
    "card": "#151926",
    "card_border": "#262c3f",
    "card_shadow": "#06070c",
    "box": "#10131d",
    "box_border": "#3b4260",
    "text": "#e8ebf5",
    "muted": "#7d859e",
    "faint": "#4b5169",
    "divider": "#232839",
    "scalar": "#8d95ad",
    "message": "#7dd3fc",
    "enum": "#fbbf24",
    "neutron": "#c4b5fd",
    "value": "#fbbf24",
    "yaml": "#5eead4",
}

FONT_FILES = {
    ("sans", False): ["Inter-Regular.ttf", "Inter.ttf", "Ubuntu-R.ttf", "DejaVuSans.ttf"],
    ("sans", True): ["Inter-Bold.ttf", "Ubuntu-B.ttf", "DejaVuSans-Bold.ttf"],
    ("mono", False): ["JetBrainsMono-Regular.ttf", "JetBrainsMonoNerdFont-Regular.ttf", "DejaVuSansMono.ttf"],
    ("mono", True): ["JetBrainsMono-Bold.ttf", "JetBrainsMonoNerdFont-Bold.ttf", "DejaVuSansMono-Bold.ttf"],
}
FONT_FAMILIES = {
    "sans": "Inter, Ubuntu, 'DejaVu Sans', Helvetica, Arial, sans-serif",
    "mono": "'JetBrains Mono', 'DejaVu Sans Mono', Menlo, Consolas, monospace",
}


def _find_font_files():
    dirs = [
        "/usr/share/fonts",
        "/usr/local/share/fonts",
        os.path.expanduser("~/.fonts"),
        os.path.expanduser("~/.local/share/fonts"),
        os.path.expanduser("~/.nix-profile/share/fonts"),
        "/Library/Fonts",
        "C:/Windows/Fonts",
    ]
    found = {}
    for d in dirs:
        if os.path.isdir(d):
            for root, _, files in os.walk(d):
                for f in files:
                    found.setdefault(f, os.path.join(root, f))
    return found


class Fonts:
    def __init__(self, scale):
        from PIL import ImageFont

        self.ImageFont = ImageFont
        self.scale = scale
        self.available = _find_font_files()
        self.cache = {}
        self.paths = {}
        for key, names in FONT_FILES.items():
            self.paths[key] = next((self.available[n] for n in names if n in self.available), None)

    def get(self, family, size, bold=False, raw_scale=None):
        s = raw_scale if raw_scale is not None else self.scale
        px = max(1, int(round(size * s)))
        k = (family, px, bold)
        if k not in self.cache:
            path = self.paths[(family, bold)]
            try:
                self.cache[k] = self.ImageFont.truetype(path, px) if path else self.ImageFont.load_default(px)
            except Exception:
                self.cache[k] = self.ImageFont.load_default(px)
        return self.cache[k]

    def width(self, text, family, size, bold=False):
        """Width in logical pixels."""
        return self.get(family, size, bold, raw_scale=4).getlength(text) / 4

    def ascent(self, family, size, bold=False):
        a, _ = self.get(family, size, bold, raw_scale=4).getmetrics()
        return a / 4


def hex_rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i : i + 2], 16) for i in (0, 2, 4))


def mix(a, b, t):
    ra, rb = hex_rgb(a), hex_rgb(b)
    return "#%02x%02x%02x" % tuple(int(ra[i] * (1 - t) + rb[i] * t) for i in range(3))


def bezier(p0, p1, p2, p3, n=40):
    pts = []
    for i in range(n + 1):
        t = i / n
        u = 1 - t
        x = u**3 * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t**3 * p3[0]
        y = u**3 * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t**3 * p3[1]
        pts.append((x, y))
    return pts


class PngCanvas:
    """Draws at a supersampled resolution and downsamples for antialiasing."""

    def __init__(self, w, h, fonts, scale, ss=2):
        from PIL import Image, ImageDraw

        self.Image = Image
        self.s = scale * ss
        self.ss = ss
        self.fonts = fonts
        self.img = Image.new("RGB", (int(w * self.s), int(h * self.s)), THEME["bg"])
        self.d = ImageDraw.Draw(self.img)

    def _p(self, v):
        return int(round(v * self.s))

    def rect(self, x, y, w, h, fill=None, outline=None, width=1, radius=0):
        self.d.rounded_rectangle(
            [self._p(x), self._p(y), self._p(x + w), self._p(y + h)],
            radius=self._p(radius),
            fill=fill,
            outline=outline,
            width=self._p(width) if outline else 0,
        )

    def circle(self, cx, cy, r, fill, outline=None, width=1):
        self.d.ellipse(
            [self._p(cx - r), self._p(cy - r), self._p(cx + r), self._p(cy + r)],
            fill=fill,
            outline=outline,
            width=self._p(width) if outline else 0,
        )

    def line(self, pts, fill, width=1):
        self.d.line([(self._p(x), self._p(y)) for x, y in pts], fill=fill, width=max(1, self._p(width)), joint="curve")

    def text(self, x, y, s, family, size, fill, bold=False, anchor="la"):
        f = self.fonts.get(family, size, bold, raw_scale=self.s)
        self.d.text((self._p(x), self._p(y)), s, font=f, fill=fill, anchor=anchor)

    def save(self, path):
        if self.ss != 1:
            w, h = self.img.size
            self.img = self.img.resize((w // self.ss, h // self.ss), self.Image.Resampling.LANCZOS)
        self.img.save(path, optimize=True)


class SvgCanvas:
    def __init__(self, w, h, fonts, scale=1, ss=1):
        self.w, self.h = w, h
        self.fonts = fonts
        self.parts = [f'<rect width="{w}" height="{h}" fill="{THEME["bg"]}"/>']

    def rect(self, x, y, w, h, fill=None, outline=None, width=1, radius=0):
        self.parts.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{radius}" '
            f'fill="{fill or "none"}" stroke="{outline or "none"}" stroke-width="{width if outline else 0}"/>'
        )

    def circle(self, cx, cy, r, fill, outline=None, width=1):
        self.parts.append(
            f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r}" fill="{fill}" stroke="{outline or "none"}" '
            f'stroke-width="{width if outline else 0}"/>'
        )

    def line(self, pts, fill, width=1):
        d = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        self.parts.append(
            f'<polyline points="{d}" fill="none" stroke="{fill}" stroke-width="{width}" '
            f'stroke-linecap="round" stroke-linejoin="round"/>'
        )

    def text(self, x, y, s, family, size, fill, bold=False, anchor="la"):
        s = s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        ta = {"l": "start", "r": "end", "m": "middle"}[anchor[0]]
        y = y + self.fonts.ascent(family, size, bold)
        weight = "700" if bold else "400"
        self.parts.append(
            f'<text x="{x:.1f}" y="{y:.1f}" font-family="{FONT_FAMILIES[family]}" font-size="{size}" '
            f'font-weight="{weight}" fill="{fill}" text-anchor="{ta}">{s}</text>'
        )

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            f.write(
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
                f'viewBox="0 0 {self.w} {self.h}">\n' + "\n".join(self.parts) + "\n</svg>\n"
            )
