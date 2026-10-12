#!/usr/bin/env python3
"""Genera el PDF de AprilTags (tag36h11) para imprimir y pegar en el piso.

Uso: python3 tools/apriltags/make_apriltag_pdf.py [A4|A3]
Las imágenes de los tags son las oficiales de AprilRobotics (apriltag-imgs, licencia BSD-2).
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from PIL import Image

HERE = Path(__file__).resolve().parent
PAPER = {"A4": (210, 297, 160), "A3": (297, 420, 250)}  # ancho, alto, lado negro del tag [mm]
MM = 1 / 25.4
IDS = range(12)


def page(pdf, w, h):
    fig = plt.figure(figsize=(w * MM, h * MM))
    return fig


def mm_axes(fig, w, h, x, y, ww, hh):
    """Ejes en milímetros medidos desde la esquina inferior izquierda de la hoja."""
    return fig.add_axes([x / w, y / h, ww / w, hh / h])


def instructions(pdf, w, h, black):
    fig = page(pdf, w, h)
    ax = mm_axes(fig, w, h, 0, 0, w, h)
    ax.set_xlim(0, w); ax.set_ylim(0, h); ax.axis("off")
    s = black / 160
    txt = (
        "AprilTags para el Duckiedrone DD24-B\n\n"
        f"Familia tag36h11, IDs 0 a 11. Lado del cuadro negro: {black} mm.\n\n"
        "Cómo imprimir\n"
        "1. Imprimir al 100 % (sin \"ajustar a la página\").\n"
        "2. Medir la línea de control de cada hoja: debe medir 100 mm.\n"
        f"   Si no, medir el cuadro negro y anotar el lado real; ese valor\n"
        "   va en la configuración del detector (tag size).\n"
        "3. Papel mate si se puede. El brillo de las lámparas en papel\n"
        "   satinado hace que el detector pierda el tag.\n\n"
        "Cómo pegarlos\n"
        "- Planos, sin arrugas, con cinta solo en las orillas blancas.\n"
        "- Todos con la flecha \"frente\" apuntando hacia el mismo lado:\n"
        "  la dirección hacia donde mira el dron al despegar.\n"
        "- Separación entre centros: 40 cm, en la cuadrícula del dibujo.\n"
        "- El dron despega sobre la X, con la cámara hacia los IDs 0-2.\n"
        "- Medir y anotar la posición real de cada tag respecto a la X.\n"
    )
    ax.text(15, h - 15, txt, va="top", ha="left", fontsize=10 * min(1.0, w / 210) + 1, family="DejaVu Sans")
    # Dibujo de la cuadrícula (vista desde arriba): x del dron hacia arriba de la hoja
    k = h / 297
    sc = 85 * k                       # mm de hoja por metro real
    ox, oy = w / 2, 15 * k + 0.45 * sc  # posición de la X
    grid = {0: (0.8, 0.4), 1: (0.8, 0.0), 2: (0.8, -0.4), 3: (0.4, 0.4), 4: (0.4, 0.0), 5: (0.4, -0.4),
            6: (0.0, 0.4), 7: (0.0, -0.4), 8: (-0.4, 0.4), 9: (-0.4, 0.0), 10: (-0.4, -0.4)}
    q = black / 1000 * sc
    for tid, (x, y) in grid.items():
        cx, cy = ox - y * sc, oy + x * sc
        ax.add_patch(plt.Rectangle((cx - q / 2, cy - q / 2), q, q, fill=False, lw=1))
        ax.text(cx, cy, str(tid), ha="center", va="center", fontsize=8)
    d = 4 * k
    ax.plot([ox - d, ox + d], [oy - d, oy + d], "r-", lw=2); ax.plot([ox - d, ox + d], [oy + d, oy - d], "r-", lw=2)
    ax.annotate("frente del dron", xy=(ox, oy + 1.15 * sc), xytext=(ox, oy + 0.95 * sc),
                ha="center", va="top", arrowprops=dict(arrowstyle="->"), fontsize=8)
    ax.text(ox, 6 * k, "X = punto de despegue (0, 0). Centros cada 40 cm. El ID 11 es de repuesto.",
            ha="center", fontsize=7)
    pdf.savefig(fig); plt.close(fig)


def tag_page(pdf, w, h, black, tid):
    fig = page(pdf, w, h)
    full = black * 10 / 8  # la imagen incluye el borde blanco (1 celda por lado)
    x0, y0 = (w - full) / 2, h - full - (h - full) * 0.35
    ax = mm_axes(fig, w, h, x0, y0, full, full)
    img = Image.open(HERE / "img" / f"tag36_11_{tid:05d}.png").convert("L")
    ax.imshow(img, cmap="gray", interpolation="nearest", vmin=0, vmax=255)
    ax.axis("off")
    t = mm_axes(fig, w, h, 0, 0, w, h); t.set_xlim(0, w); t.set_ylim(0, h); t.axis("off")
    t.annotate("frente", xy=(w / 2, y0 + full + 12), xytext=(w / 2, y0 + full + 2), ha="center",
               arrowprops=dict(arrowstyle="->", lw=1.5), fontsize=9)
    t.text(w / 2, y0 - 8, f"tag36h11  ID {tid}   lado negro {black} mm", ha="center", fontsize=12, weight="bold")
    yl = max(8, y0 - 22)
    t.plot([w / 2 - 50, w / 2 + 50], [yl, yl], "k-", lw=1)
    t.plot([w / 2 - 50] * 2, [yl - 2, yl + 2], "k-", lw=1); t.plot([w / 2 + 50] * 2, [yl - 2, yl + 2], "k-", lw=1)
    t.text(w / 2, yl - 5, "línea de control: 100 mm", ha="center", fontsize=7)
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
