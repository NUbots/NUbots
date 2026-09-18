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

"""Lay out the cards around the module box and draw the whole diagram."""

import os
from dataclasses import dataclass

from .canvas import THEME, Fonts, PngCanvas, SvgCanvas, bezier, mix
from .cards import Row, build_card
from .cpp import ROLES

__all__ = ["Fonts", "RenderOptions", "render_module"]


@dataclass
class RenderOptions:
    fmt: str = "png"
    depth: int = 1
    max_fields: int = 28
    docs: bool = False
    usage: bool = True
    columns: int = 0
    scale: float = 2.0


# Layout constants (logical pixels)
L = dict(
    margin=56,
    header=64,
    card_w=372,
    card_gap=26,
    col_gap=34,
    box_w=340,
    gap=170,
    pad=16,
    row_h=21,
    doc_h=15,
    title=17,
    sub=10.5,
    field=12.5,
    tag=9.5,
    tag_h=17,
    max_col_h=2300,
)


def fit_text(fonts, s, family, size, max_w, bold=False):
    if fonts.width(s, family, size, bold) <= max_w:
        return s
    while len(s) > 1 and fonts.width(s + "…", family, size, bold) > max_w:
        s = s[:-1]
    return s + "…"


def measure_card(card, fonts, opts):
    """Compute card height, truncating rows to opts.max_fields."""
    pad, row_h = L["pad"], L["row_h"]
    h = pad + L["title"] + 6 + L["sub"] + 6
    if card.tags:
        h += L["tag_h"] + 8
    if card.note and opts.docs:
        h += L["doc_h"] + 2
    rows = card.rows
    if len(rows) > opts.max_fields:
        card.rows = rows[: opts.max_fields] + [Row(0, f"… {len(rows) - opts.max_fields} more", "", "scalar")]
    if card.rows:
        h += 8  # divider
        for r in card.rows:
            h += row_h + (L["doc_h"] if opts.docs and r.doc else 0)
    else:
        h += 4
    h += pad - 4
    card.w = L["card_w"]
    card.h = int(h)
    return card.h


def pack_columns(cards, n_cols):
    """Greedy shortest-column packing. Returns list of columns (lists of cards)."""
    cols = [[] for _ in range(n_cols)]
    heights = [0] * n_cols
    for c in cards:
        i = min(range(n_cols), key=lambda k: heights[k])
        cols[i].append(c)
        heights[i] += c.h + L["card_gap"]
    return cols, max(heights) - L["card_gap"] if cards else 0


def draw_tag(cv, fonts, x, y, text, accent):
    w = fonts.width(text, "sans", L["tag"], True) + 12
    cv.rect(
        x, y, w, L["tag_h"], fill=mix(accent, THEME["card"], 0.82), outline=mix(accent, THEME["card"], 0.55), radius=4
    )
    cv.text(x + 6, y + 3, text, "sans", L["tag"], accent, bold=True)
    return w


def draw_card(cv, fonts, card, opts):
    x, y, w, h = card.x, card.y, card.w, card.h
    pad = L["pad"]
    cv.rect(x + 3, y + 5, w, h, fill=THEME["card_shadow"], radius=10)
    cv.rect(x, y, w, h, fill=THEME["card"], outline=THEME["card_border"], width=1, radius=10)
    # accent stripe
    cv.rect(x, y + 10, 4, h - 20, fill=card.accent, radius=2)

    cy = y + pad
    cv.text(
        x + pad + 4,
        cy,
        fit_text(fonts, card.title, "sans", L["title"], w - 2 * pad - 8, True),
        "sans",
        L["title"],
        THEME["text"],
        bold=True,
    )
    cy += L["title"] + 6
    if card.subtitle:
        cv.text(
            x + pad + 4,
            cy,
            fit_text(fonts, card.subtitle, "mono", L["sub"], w - 2 * pad - 8),
            "mono",
            L["sub"],
            THEME["muted"],
        )
    cy += L["sub"] + 6
    if card.tags:
        tx = x + pad + 4
        for t in card.tags:
            if tx > x + w - pad - 40:
                break
            tx += draw_tag(cv, fonts, tx, cy, t, card.accent) + 6
        cy += L["tag_h"] + 8
    if card.note and opts.docs:
        cv.text(
            x + pad + 4,
            cy,
            fit_text(fonts, card.note, "sans", L["sub"], w - 2 * pad - 8),
            "sans",
            L["sub"],
            THEME["faint"],
        )
        cy += L["doc_h"] + 2
    if card.rows:
        cv.line([(x + pad, cy + 3), (x + w - pad, cy + 3)], THEME["divider"], 1)
        cy += 8
        for r in card.rows:
            ind = r.indent * 16
            tcol = THEME.get(r.kind, THEME["scalar"])
            tw = fonts.width(r.type, "mono", L["field"])
            max_name_w = w - 2 * pad - tw - 14 - ind
            name_col = THEME["text"] if r.kind != "value" else THEME["value"]
            if not r.used:
                tcol = name_col = THEME["faint"]
            cv.text(x + w - pad, cy + 3, r.type, "mono", L["field"], tcol, anchor="ra")
            if r.indent:
                cv.text(x + pad + 4 + ind - 12, cy + 3, "└", "mono", L["field"], THEME["faint"])
            name = fit_text(fonts, r.name, "mono", L["field"], max_name_w)
            nx = x + pad + 4 + ind
            cv.text(nx, cy + 3, name, "mono", L["field"], name_col)
            cy += L["row_h"]
            if opts.docs and r.doc:
                cv.text(
                    x + pad + 4 + ind,
                    cy - 3,
                    fit_text(fonts, r.doc, "sans", L["sub"], w - 2 * pad - 8 - ind),
                    "sans",
                    L["sub"],
                    THEME["faint"],
                )
                cy += L["doc_h"]
    else:
        cv.text(
            x + pad + 4,
            cy + 2,
            "no fields" if card.role != "unknown" else "type could not be resolved",
            "sans",
            L["sub"],
            THEME["faint"],
        )


def draw_box(cv, fonts, module, bx, by, bw, bh, n_in, n_out):
    cv.rect(bx + 4, by + 8, bw, bh, fill=THEME["card_shadow"], radius=16)
    cv.rect(bx, by, bw, bh, fill=THEME["box"], outline=THEME["box_border"], width=2, radius=16)
    # inner glow line
    cv.rect(bx + 6, by + 6, bw - 12, bh - 12, outline=mix(THEME["box_border"], THEME["box"], 0.6), radius=12)

    name = module.name.split("/")[-1]
    size = 26
    while fonts.width(name, "sans", size, True) > bw - 48 and size > 14:
        size -= 1
    cy = by + 30
    cv.text(bx + bw / 2, cy, name, "sans", size, THEME["text"], bold=True, anchor="ma")
    cy += size + 8
    cv.text(
        bx + bw / 2, cy, fit_text(fonts, module.namespace, "mono", 11, bw - 40), "mono", 11, THEME["muted"], anchor="ma"
    )
    cy += 20
    if module.director:
        tw = fonts.width("DIRECTOR", "sans", L["tag"], True) + 12
        draw_tag(cv, fonts, bx + bw / 2 - tw / 2, cy, "DIRECTOR", ROLES["provide"][0])
        cy += L["tag_h"] + 8
    cy += 8
    cv.line([(bx + 28, cy), (bx + bw - 28, cy)], THEME["divider"], 1)
    cy += 18

    stats = [("reactions", module.reactions), ("inputs", n_in), ("outputs", n_out)]
    slot = (bw - 40) / 3
    for i, (label, val) in enumerate(stats):
        cx = bx + 20 + slot * i + slot / 2
        cv.text(cx, cy, str(val), "sans", 22, THEME["text"], bold=True, anchor="ma")
        cv.text(cx, cy + 28, label, "sans", 10, THEME["muted"], anchor="ma")
    cy += 56
    cv.line([(bx + 28, cy), (bx + bw - 28, cy)], THEME["divider"], 1)
    cy += 16

    cv.text(bx + 28, cy, "SOURCES", "sans", 9, THEME["faint"], bold=True)
    cy += 16
    for f in module.files[:8]:
        cv.text(bx + 28, cy, fit_text(fonts, f, "mono", 10.5, bw - 56), "mono", 10.5, THEME["muted"])
        cy += 16
    if len(module.files) > 8:
        cv.text(bx + 28, cy, f"… {len(module.files) - 8} more", "mono", 10.5, THEME["faint"])
        cy += 16

    # side labels
    cv.text(bx + 14, by + bh - 22, "IN", "sans", 10, THEME["faint"], bold=True)
    cv.text(bx + bw - 14, by + bh - 22, "OUT", "sans", 10, THEME["faint"], bold=True, anchor="ra")


def render_module(module, index, opts, fonts, out_dir):
    in_cards = [build_card(p, index, module, opts.depth, usage=opts.usage) for p in module.inputs]
    out_cards = [build_card(p, index, module, opts.depth) for p in module.outputs]

    for c in in_cards + out_cards:
        measure_card(c, fonts, opts)

    def choose_cols(cards):
        if opts.columns:
            return opts.columns
        total = sum(c.h + L["card_gap"] for c in cards)
        return 1 if total <= L["max_col_h"] else 2

    left_cols, left_h = pack_columns(in_cards, choose_cols(in_cards))
    right_cols, right_h = pack_columns(out_cards, choose_cols(out_cards))

    side_w = lambda cols: len(cols) * L["card_w"] + (len(cols) - 1) * L["col_gap"]
    box_h = max(330, min(max(left_h, right_h), 700))
    content_h = max(left_h, right_h, box_h)
    width = L["margin"] * 2 + side_w(left_cols) + L["gap"] + L["box_w"] + L["gap"] + side_w(right_cols)
    height = L["margin"] + L["header"] + content_h + L["margin"]
    top = L["margin"] + L["header"]

    # Positions: left columns run outer -> inner, so the inner column touches the gap
    x = L["margin"]
    for col in reversed(left_cols):
        y = top
        for c in col:
            c.x, c.y = x, y
            y += c.h + L["card_gap"]
        x += L["card_w"] + L["col_gap"]
    x = L["margin"] + side_w(left_cols) + L["gap"]
    bx, by = x, top + (content_h - box_h) / 2
    x += L["box_w"] + L["gap"]
    for col in right_cols:
        y = top
        for c in col:
            c.x, c.y = x, y
            y += c.h + L["card_gap"]
        x += L["card_w"] + L["col_gap"]

    Canvas = SvgCanvas if opts.fmt == "svg" else PngCanvas
    cv = Canvas(int(width), int(height), fonts, opts.scale, 2)

    # subtle dot grid
    if opts.fmt != "svg":
        for gy in range(0, int(height), 28):
            for gx in range(0, int(width), 28):
                cv.circle(gx, gy, 0.8, THEME["grid"])

    # header
    cv.text(L["margin"], L["margin"] - 10, "NUbots · module black box", "sans", 12, THEME["muted"], bold=True)
    cv.text(L["margin"], L["margin"] + 10, module.name.replace("/", " / "), "mono", 11, THEME["faint"])
    cv.text(width - L["margin"], L["margin"] - 10, "./b blackbox", "mono", 11, THEME["faint"], anchor="ra")

    # connectors (under everything)
    def connect(cards, side):
        ordered = sorted(cards, key=lambda c: c.y + c.h / 2)
        n = len(ordered)
        for i, c in enumerate(ordered):
            py = by + box_h * (i + 1) / (n + 1)
            cy = c.y + min(c.h / 2, 60)
            if side == "in":
                p0, p3 = (c.x + c.w, cy), (bx, py)
            else:
                p0, p3 = (bx + L["box_w"], py), (c.x, cy)
            mid = (p0[0] + p3[0]) / 2
            pts = bezier(p0, (mid, p0[1]), (mid, p3[1]), p3)
            cv.line(pts, mix(c.accent, THEME["bg"], 0.55), 2.5)
            cv.line(pts, c.accent, 1.2)
            cv.circle(p0[0], p0[1], 4, THEME["bg"], outline=c.accent, width=1.5)
            cv.circle(p3[0], p3[1], 4, c.accent)

    connect(in_cards, "in")
    connect(out_cards, "out")

    draw_box(cv, fonts, module, bx, by, L["box_w"], box_h, len(in_cards), len(out_cards))
    for c in in_cards + out_cards:
        draw_card(cv, fonts, c, opts)

    if not in_cards:
        cv.text(L["margin"] + L["card_w"] / 2, top + 20, "no inputs found", "sans", 12, THEME["faint"], anchor="ma")
    if not out_cards:
        cv.text(
            bx + L["box_w"] + L["gap"] + L["card_w"] / 2,
            top + 20,
            "no outputs found",
            "sans",
            12,
            THEME["faint"],
            anchor="ma",
        )

    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, module.name.replace("/", "_") + "." + opts.fmt)
    cv.save(out)
    return out, (int(width), int(height))
