#!/usr/bin/env python
"""Printable fold-up net that wraps a cube, with a tag correctly oriented per face.

    ./venv/bin/python make_cube_net.py --cube 0.060 --out cube_net.pdf

Reads cube_tags.yaml for the tag ids, sizes and orientations, so the print and
the map can never disagree.

Why the per-face rotation is derived, not typed
-----------------------------------------------
Folding rotates each face. A tag printed "upright" on the flat net does NOT end
up upright on the cube -- the top face in particular lands rotated 180 degrees.
Getting one face wrong yields a tag that decodes perfectly and reports a pose
rotated 90 or 180 degrees about its own normal, which is exactly the kind of
error that survives every reprojection check on that tag alone.

So for each face this computes the net's right/up directions AFTER folding
(expressed in cube coordinates), reads the tag's required right/up out of the
map, and solves for the rotation that reconciles them.

Net layout (landscape), sides wrapping left to right:

        +Z
    +X  +Y  -X  -Y
        -Z
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import tag_map as tmap
from make_apriltags import FAMILIES, tag_bits
from pdf_sheet import PDFPage, write_pdf

HERE = Path(__file__).resolve().parent

# After folding: (net-right, net-up) for each face, in cube coordinates.
# Verified elsewhere: cross(right, up) == the face's outward normal for all six.
NET = {
    "+X": ([0, 1, 0], [0, 0, 1]),
    "+Y": ([-1, 0, 0], [0, 0, 1]),
    "-X": ([0, -1, 0], [0, 0, 1]),
    "-Y": ([1, 0, 0], [0, 0, 1]),
    "+Z": ([0, 1, 0], [-1, 0, 0]),
    "-Z": ([0, 1, 0], [1, 0, 0]),
}
NORMAL = {"+X": [1, 0, 0], "-X": [-1, 0, 0], "+Y": [0, 1, 0],
          "-Y": [0, -1, 0], "+Z": [0, 0, 1], "-Z": [0, 0, -1]}
# grid cell (column, row); row 0 is the top of the sheet
GRID = {"+Z": (0, 0), "+X": (0, 1), "+Y": (1, 1), "-X": (2, 1),
        "-Y": (3, 1), "-Z": (0, 2)}


def fold_rotation(face: str, R_wt: np.ndarray) -> int:
    """Degrees to rotate the tag on the flat net so it lands right after folding."""
    u, v = (np.asarray(a, dtype=float) for a in NET[face])
    assert np.allclose(np.cross(u, v), NORMAL[face], atol=1e-6), face
    right = R_wt[:, 0]
    deg = int(round(np.degrees(np.arctan2(float(right @ v), float(right @ u))))) % 360
    if deg % 90:
        raise SystemExit(f"{face}: needs a {deg} deg rotation, not a multiple of 90")
    r = np.radians(deg)
    pred_r = np.cos(r) * u + np.sin(r) * v
    pred_u = -np.sin(r) * u + np.cos(r) * v
    # tolerance is loose because map rpy values are rounded (1.5708, not pi/2)
    assert np.allclose(pred_r, right, atol=1e-4), f"{face} right mismatch"
    assert np.allclose(pred_u, R_wt[:, 1], atol=1e-4), f"{face} up mismatch"
    return deg


def dashed(page, x1, y1, x2, y2, dash=2.0, width=0.2):
    n = max(1, int(np.hypot(x2 - x1, y2 - y1) / (dash * 2)))
    for i in range(n):
        t0, t1 = i / n, (i + 0.5) / n
        page.line(x1 + (x2 - x1) * t0, y1 + (y2 - y1) * t0,
                  x1 + (x2 - x1) * t1, y1 + (y2 - y1) * t1, width, 0.55)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--cube", type=float, default=0.060, help="cube edge, metres")
    p.add_argument("--map", type=Path, default=HERE / "cube_tags.yaml")
    p.add_argument("--family", default="tag36h11")
    p.add_argument("--page", choices=["a4", "letter"], default="a4")
    p.add_argument("--no-bottom", action="store_true",
                   help="omit the -Z square (it sits on the table)")
    p.add_argument("--out", type=Path, default=Path("cube_net.pdf"))
    args = p.parse_args(argv)

    T = tmap.load_tag_map(args.map, expected_frame="cube")
    by_face = {s.note.split()[0]: s for s in T.values()}
    cube = args.cube * 1000.0
    cells = FAMILIES[args.family][1] + 2

    faces = [f for f in GRID if f in by_face or (f == "-Z" and not args.no_bottom)]
    ncols = max(GRID[f][0] for f in faces) + 1
    nrows = max(GRID[f][1] for f in faces) + 1
    net_w, net_h = ncols * cube, nrows * cube

    pw, ph = ({"a4": (297.0, 210.0), "letter": (279.4, 215.9)})[args.page]  # landscape
    if net_w > pw - 16 or net_h > ph - 26:
        raise SystemExit(
            f"net is {net_w:.0f}x{net_h:.0f} mm, too big for {args.page} landscape "
            f"({pw:.0f}x{ph:.0f}). Use a smaller --cube or print on larger paper.")

    page = PDFPage(pw, ph)
    x0, y0 = (pw - net_w) / 2.0, 20.0
    print(f"  cube {cube:.1f} mm, net {net_w:.0f} x {net_h:.0f} mm on "
          f"{args.page.upper()} landscape ({pw:.0f} x {ph:.0f} mm)")
    print(f"\n  {'face':<5}{'id':>4}{'tag mm':>8}{'quiet mm':>10}{'rotation':>10}")

    for face in faces:
        c, r = GRID[face]
        fx, fy = x0 + c * cube, y0 + r * cube
        page.rect(fx, fy, cube, cube, 1.0)          # white face

        spec = by_face.get(face)
        if spec is None:
            page.text(fx + 6, fy + cube / 2, f"{face}  (no tag)", 9, 0.45)
            print(f"  {face:<5}{'-':>4}{'-':>8}{'-':>10}{'-':>10}")
            continue

        size = spec.size * 1000.0
        quiet = (cube - size) / 2.0
        if quiet < size / cells:
            print(f"    [warn] {face}: quiet zone {quiet:.1f} mm is under one "
                  f"cell ({size/cells:.1f} mm); detection will suffer")
        deg = fold_rotation(face, spec.R_wt)
        bits = np.rot90(tag_bits(args.family, spec.id), deg // 90)

        cell = size / cells
        page.rect(fx + quiet, fy + quiet, size, size, 0.0)
        for rr in range(cells):
            for cc in range(cells):
                if not bits[rr, cc]:
                    page.rect(fx + quiet + cc * cell, fy + quiet + rr * cell,
                              cell, cell, 1.0)
        page.text(fx + 2.5, fy + cube - 2.5, f"{face}  id {spec.id}", 7, 0.35)
        print(f"  {face:<5}{spec.id:>4}{size:>8.1f}{quiet:>10.1f}{deg:>9}d")

    # fold lines between adjacent faces, solid cut outline around the whole net
    occupied = {GRID[f] for f in faces}
    for (c, r) in occupied:
        fx, fy = x0 + c * cube, y0 + r * cube
        for dc, dr, seg in ((1, 0, "v"), (0, 1, "h")):
            if (c + dc, r + dr) in occupied:
                if seg == "v":
                    dashed(page, fx + cube, fy, fx + cube, fy + cube)
                else:
                    dashed(page, fx, fy + cube, fx + cube, fy + cube)
    for (c, r) in occupied:
        fx, fy = x0 + c * cube, y0 + r * cube
        if (c, r - 1) not in occupied: page.line(fx, fy, fx + cube, fy, 0.35)
        if (c, r + 1) not in occupied: page.line(fx, fy + cube, fx + cube, fy + cube, 0.35)
        if (c - 1, r) not in occupied: page.line(fx, fy, fx, fy + cube, 0.35)
        if (c + 1, r) not in occupied: page.line(fx + cube, fy, fx + cube, fy + cube, 0.35)

    page.text(8, 12, f"Cube net {cube:.0f} mm  {args.family}  ids "
              f"{sorted(s.id for s in T.values())}  -- PRINT AT 100% (actual size). "
              "Solid = cut, dashed = fold.", 8)
    ry = ph - 8
    page.line(8, ry, 108, ry, 0.3)
    page.line(8, ry - 2, 8, ry + 2, 0.3)
    page.line(108, ry - 2, 108, ry + 2, 0.3)
    page.text(112, ry + 2, "100 mm", 8)

    write_pdf(args.out, [page])
    print(f"\n  wrote {args.out}")
    print("  Cut the solid outline, crease the dashed lines, wrap with the")
    print("  printed side OUT. The +Z (top) tag is rotated 180 deg on purpose --")
    print("  it lands upright once folded.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
