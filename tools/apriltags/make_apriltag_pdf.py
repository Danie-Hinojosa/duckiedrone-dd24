#!/usr/bin/env python3
"""Generates the AprilTag (tag36h11) PDF to print and tape to the floor.

Usage: python3 tools/apriltags/make_apriltag_pdf.py [A4|A3]
The tag images are the official ones from AprilRobotics (apriltag-imgs, BSD-2 license).
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from PIL import Image

HERE = Path(__file__).resolve().parent
PAPER = {"A4": (210, 297, 160), "A3": (297, 420, 250)}  # width, height, side of the black tag square [mm]
MM = 1 / 25.4
IDS = range(12)


def page(pdf, w, h):
    fig = plt.figure(figsize=(w * MM, h * MM))
    return fig


def mm_axes(fig, w, h, x, y, ww, hh):
    """Axes in millimeters, measured from the bottom-left corner of the page."""
    return fig.add_axes([x / w, y / h, ww / w, hh / h])


def instructions(pdf, w, h, black):
    fig = page(pdf, w, h)
    ax = mm_axes(fig, w, h, 0, 0, w, h)
    ax.set_xlim(0, w); ax.set_ylim(0, h); ax.axis("off")
    s = black / 160
    txt = (
        "AprilTags for the Duckiedrone DD24-B\n\n"
        f"Family tag36h11, IDs 0 to 11. Side of the black square: {black} mm.\n\n"
        "How to print\n"
        "1. Print at 100 % (no \"fit to page\").\n"
        "2. Measure the check line on each page: it must be 100 mm.\n"
        f"   If not, measure the black square and write down the real side;\n"
        "   that value goes in the detector config (tag size).\n"
        "3. Use matte paper if possible. Lamp glare on glossy paper\n"
        "   makes the detector lose the tag.\n\n"
        "How to lay them out\n"
        "- Flat, no wrinkles, tape only on the white margins.\n"
        "- All with the \"front\" arrow pointing the same way:\n"
        "  the direction the drone faces at takeoff.\n"
        "- Spacing between centers: 40 cm, on the grid in the drawing.\n"
        "- The drone takes off over the X, camera facing IDs 0-2.\n"
        "- Measure and write down each tag's real position from the X.\n"
    )
    ax.text(15, h - 15, txt, va="top", ha="left", fontsize=10 * min(1.0, w / 210) + 1, family="DejaVu Sans")
    # Grid drawing (top view): drone x points toward the top of the page
    k = h / 297
    sc = 85 * k                       # mm on paper per real meter
    ox, oy = w / 2, 15 * k + 0.45 * sc  # position of the X
    grid = {0: (0.8, 0.4), 1: (0.8, 0.0), 2: (0.8, -0.4), 3: (0.4, 0.4), 4: (0.4, 0.0), 5: (0.4, -0.4),
            6: (0.0, 0.4), 7: (0.0, -0.4), 8: (-0.4, 0.4), 9: (-0.4, 0.0), 10: (-0.4, -0.4)}
    q = black / 1000 * sc
    for tid, (x, y) in grid.items():
        cx, cy = ox - y * sc, oy + x * sc
        ax.add_patch(plt.Rectangle((cx - q / 2, cy - q / 2), q, q, fill=False, lw=1))
        ax.text(cx, cy, str(tid), ha="center", va="center", fontsize=8)
    d = 4 * k
    ax.plot([ox - d, ox + d], [oy - d, oy + d], "r-", lw=2); ax.plot([ox - d, ox + d], [oy + d, oy - d], "r-", lw=2)
    ax.annotate("drone front", xy=(ox, oy + 1.15 * sc), xytext=(ox, oy + 0.95 * sc),
                ha="center", va="top", arrowprops=dict(arrowstyle="->"), fontsize=8)
    ax.text(ox, 6 * k, "X = takeoff point (0, 0). Centers every 40 cm. ID 11 is a spare.",
            ha="center", fontsize=7)
    pdf.savefig(fig); plt.close(fig)


def tag_page(pdf, w, h, black, tid):
    fig = page(pdf, w, h)
    full = black * 10 / 8  # the image includes the white border (1 cell per side)
    x0, y0 = (w - full) / 2, h - full - (h - full) * 0.35
    ax = mm_axes(fig, w, h, x0, y0, full, full)
    img = Image.open(HERE / "img" / f"tag36_11_{tid:05d}.png").convert("L")
    ax.imshow(img, cmap="gray", interpolation="nearest", vmin=0, vmax=255)
    ax.axis("off")
    t = mm_axes(fig, w, h, 0, 0, w, h); t.set_xlim(0, w); t.set_ylim(0, h); t.axis("off")
    t.annotate("front", xy=(w / 2, y0 + full + 12), xytext=(w / 2, y0 + full + 2), ha="center",
               arrowprops=dict(arrowstyle="->", lw=1.5), fontsize=9)
    t.text(w / 2, y0 - 8, f"tag36h11  ID {tid}   black side {black} mm", ha="center", fontsize=12, weight="bold")
    yl = max(8, y0 - 22)
    t.plot([w / 2 - 50, w / 2 + 50], [yl, yl], "k-", lw=1)
    t.plot([w / 2 - 50] * 2, [yl - 2, yl + 2], "k-", lw=1); t.plot([w / 2 + 50] * 2, [yl - 2, yl + 2], "k-", lw=1)
    t.text(w / 2, yl - 5, "check line: 100 mm", ha="center", fontsize=7)
    pdf.savefig(fig); plt.close(fig)


def main():
    size = (sys.argv[1] if len(sys.argv) > 1 else "A4").upper()
    w, h, black = PAPER[size]
    out = HERE / f"apriltags_tag36h11_{size}_{black}mm.pdf"
    with PdfPages(out) as pdf:
        instructions(pdf, w, h, black)
        for tid in IDS:
            tag_page(pdf, w, h, black, tid)
    print(out)


if __name__ == "__main__":
    main()
