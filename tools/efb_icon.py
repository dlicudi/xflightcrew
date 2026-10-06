#!/usr/bin/env python3
"""The EFB's home-screen icons, rendered deterministically (no font, no
image library): a dark rounded square, a cyan ring, and "XVL" drawn as
bold strokes in the page's accent cyan.

    python3 tools/efb_icon.py

writes src/efb/icons/icon-180.png (apple-touch-icon, full bleed: iOS rounds
it itself), icon-192.png and icon-512.png (the manifest's, corners clear).
The same inputs always give the same bytes (zlib level 9, no timestamps), so
a re-run on an unchanged script leaves git clean. The server embeds them
with include_bytes! (src/efb/mod.rs).
"""

import os
import struct
import zlib

BG = (0x0D, 0x11, 0x17)
RING = (0x2E, 0x6E, 0x8E)
INK = (0x7F, 0xD3, 0xF7)

# The mark in a 100 x 100 box: (x0, y0, x1, y1) strokes, round capped.
STROKE = 7.4
LETTERS = [
    # X
    (18.0, 35.0, 34.0, 65.0),
    (34.0, 35.0, 18.0, 65.0),
    # V
    (42.0, 35.0, 50.5, 65.0),
    (50.5, 65.0, 59.0, 35.0),
    # L
    (67.0, 35.0, 67.0, 65.0),
    (67.0, 65.0, 82.0, 65.0),
]
RING_INSET = 10.0
RING_RADIUS = 14.0
RING_WIDTH = 3.2
CORNER = 22.0
SUPER = 4


def seg_dist(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
    cx, cy = x0 + t * dx, y0 + t * dy
    return ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5


def rrect_sd(px, py, x0, y0, x1, y1, r):
    """Signed distance to a rounded rectangle (negative inside)."""
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    hx, hy = (x1 - x0) / 2 - r, (y1 - y0) / 2 - r
    qx, qy = abs(px - cx) - hx, abs(py - cy) - hy
    ox, oy = max(qx, 0.0), max(qy, 0.0)
    return (ox * ox + oy * oy) ** 0.5 + min(max(qx, qy), 0.0) - r


def sample(u, v, rounded):
    """(r, g, b, a) at a point of the 100 box, no antialiasing."""
    if rounded and rrect_sd(u, v, 0, 0, 100, 100, CORNER) > 0:
        return (0, 0, 0, 0)
    for (x0, y0, x1, y1) in LETTERS:
        if seg_dist(u, v, x0, y0, x1, y1) <= STROKE / 2:
            return INK + (255,)
    i = RING_INSET
    if abs(rrect_sd(u, v, i, i, 100 - i, 100 - i, RING_RADIUS)) <= RING_WIDTH / 2:
        return RING + (255,)
    return BG + (255,)


def render(size, rounded):
    rows = []
    n = SUPER * SUPER
    for y in range(size):
        row = bytearray([0])  # filter: none
        for x in range(size):
            acc = [0, 0, 0, 0]
            for sy in range(SUPER):
                for sx in range(SUPER):
                    u = (x + (sx + 0.5) / SUPER) * 100.0 / size
                    v = (y + (sy + 0.5) / SUPER) * 100.0 / size
                    r, g, b, a = sample(u, v, rounded)
                    acc[0] += r * a
                    acc[1] += g * a
                    acc[2] += b * a
                    acc[3] += a
            a = acc[3]
            if a == 0:
                row += bytes(4)
            else:
                row += bytes([round(acc[0] / a), round(acc[1] / a), round(acc[2] / a), round(a / n)])
        rows.append(bytes(row))
    return png(size, size, b"".join(rows))


def chunk(kind, data):
    c = struct.pack(">I", len(data)) + kind + data
    return c + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)


def png(w, h, raw):
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0)  # 8-bit RGBA
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "..", "src", "efb", "icons")
    os.makedirs(out, exist_ok=True)
    for size, rounded in ((180, False), (192, True), (512, True)):
        path = os.path.join(out, "icon-%d.png" % size)
        with open(path, "wb") as f:
            f.write(render(size, rounded))
        print(path)


if __name__ == "__main__":
    main()
