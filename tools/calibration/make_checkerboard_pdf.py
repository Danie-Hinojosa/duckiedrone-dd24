#!/usr/bin/env python3
"""Generates the checkerboard used to calibrate the drone's fisheye camera.

Usage: python3 tools/calibration/make_checkerboard_pdf.py
Outputs a landscape A4 page with 9x6 squares of 30 mm (8x5 inner corners). Having an even number
of squares on one side and an odd number on the other keeps OpenCV from confusing the board with
itself rotated 180°.
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = Path(__file__).resolve().parent
W, H = 297, 210          # landscape A4 [mm]
COLS, ROWS, SQ = 9, 6, 30
MM = 1 / 25.4


def main():
    fig = plt.figure(figsize=(W * MM, H * MM))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")
    x0 = (W - COLS * SQ) / 2
    y0 = (H - ROWS * SQ) / 2 + 3          # shifted up a little to leave room for the note
    for i in range(COLS):
        for j in range(ROWS):
            if (i + j) % 2 == 0:
                ax.add_patch(Rectangle((x0 + i * SQ, y0 + j * SQ), SQ, SQ, color="black", lw=0))
    ax.text(W / 2, y0 - 6,
            f"Board of {COLS}x{ROWS} squares of {SQ} mm ({COLS - 1}x{ROWS - 1} inner corners). "
            "Print at 100 % (actual size), no \"fit to page\". "
            f"Measure one square with a ruler: it must be {SQ} mm.",
            ha="center", va="center", fontsize=6.5, color="0.35")
    out = HERE / f"checkerboard_{COLS}x{ROWS}_{SQ}mm_A4.pdf"
    fig.savefig(out)
    print(out)


if __name__ == "__main__":
    main()
