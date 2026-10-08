#!/usr/bin/env python3
"""Vista superior y lateral del URDF (sin RViz), para revisar posiciones de piezas.
Uso: python3 tools/urdf_preview.py [salida.png]
"""
import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PKG = Path(__file__).resolve().parent.parent / "src" / "duckiedrone_dd24"
COLORS = {"carbon": "#333", "black": "#111", "prop": "#e6c200", "motor": "#c03020", "board": "#2a7",
          "hat": "#f90", "battery": "#36a", "sensor": "#29f"}


def stl_points(path, step=1):
    d = path.read_bytes()
    n = struct.unpack("<I", d[80:84])[0]
    a = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3)[::step] * 0.001


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin(el):
    o = el.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    ang = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    T = np.eye(4)
    T[:3, :3] = rpy(*ang)
    T[:3, 3] = xyz
    return T


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "urdf_preview.png"
    root = ET.parse(PKG / "urdf" / "dd24.urdf").getroot()
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}

    def link_tf(name):
        T = np.eye(4)
        while name in joints:
            j = joints[name]
            T = origin(j) @ T
            name = j.find("parent").get("link")
        return T

    fig, (top, side) = plt.subplots(1, 2, figsize=(14, 6.5))
    for link in root.findall("link"):
        T = link_tf(link.get("name"))
        for v in link.findall("visual"):
            TT = T @ origin(v)
            mat = v.find("material")
            c = COLORS.get(mat.get("name") if mat is not None else "", "#888")
            g = v.find("geometry")
            mesh, box, cyl = g.find("mesh"), g.find("box"), g.find("cylinder")
            if mesh is not None:
                f = PKG / "meshes" / mesh.get("filename").split("/")[-1]
                P = stl_points(f, step=6 if "prop" in f.name else 1)
            elif box is not None:
                sx, sy, sz = [float(x) / 2 for x in box.get("size").split()]
                P = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
            else:
                r, h = float(cyl.get("radius")), float(cyl.get("length")) / 2
                t = np.linspace(0, 2 * math.pi, 24)
                P = np.r_[np.c_[r * np.cos(t), r * np.sin(t), -h * np.ones(24)],
                          np.c_[r * np.cos(t), r * np.sin(t), h * np.ones(24)]]
            W = (TT[:3, :3] @ P.T).T + TT[:3, 3]
            top.scatter(W[:, 0], W[:, 1], s=0.15, c=c, alpha=0.5)
            side.scatter(W[:, 0], W[:, 2], s=0.15, c=c, alpha=0.5)
    for ax, title, ylab in ((top, "Vista superior (x adelante, y izquierda)", "y [m]"),
                            (side, "Vista lateral (x adelante, z arriba)", "z [m]")):
        ax.set_aspect("equal")
        ax.set_title(title)
        ax.grid(alpha=0.3)
        ax.set_xlabel("x [m]")
        ax.set_ylabel(ylab)
    plt.tight_layout()
    plt.savefig(out, dpi=90)
    print(out)


if __name__ == "__main__":
    main()
