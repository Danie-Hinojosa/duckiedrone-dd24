#!/usr/bin/env python3
"""Genera el tablero de ajedrez para calibrar la cámara ojo de pez del dron.

Uso: python3 tools/calibration/make_checkerboard_pdf.py
Sale una hoja A4 horizontal con 9x6 cuadros de 30 mm (8x5 esquinas internas). Que el número de
cuadros sea par en un lado e impar en el otro evita que OpenCV confunda el tablero girado 180°.
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = Path(__file__).resolve().parent
W, H = 297, 210          # A4 horizontal [mm]
COLS, ROWS, SQ = 9, 6, 30
MM = 1 / 25.4


def main():
    fig = plt.figure(figsize=(W * MM, H * MM))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")
    x0 = (W - COLS * SQ) / 2
    y0 = (H - ROWS * SQ) / 2 + 3          # un poco arriba para dejar lugar a la nota
    for i in range(COLS):
        for j in range(ROWS):
            if (i + j) % 2 == 0:
                ax.add_patch(Rectangle((x0 + i * SQ, y0 + j * SQ), SQ, SQ, color="black", lw=0))
    ax.text(W / 2, y0 - 6,
            f"Tablero {COLS}x{ROWS} cuadros de {SQ} mm ({COLS - 1}x{ROWS - 1} esquinas internas). "
            "Imprimir al 100 % (tamaño real), sin \"ajustar a la página\". "
            f"Mide un cuadro con regla: debe dar {SQ} mm.",
            ha="center", va="center", fontsize=6.5, color="0.35")
    out = HERE / f"checkerboard_{COLS}x{ROWS}_{SQ}mm_A4.pdf"
    fig.savefig(out)
    print(out)


if __name__ == "__main__":
    main()
