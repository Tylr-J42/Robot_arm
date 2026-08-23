#!/usr/bin/env python
"""Generate a printable ChArUco board for intrinsics calibration.

    ./venv/bin/python make_charuco_board.py --out charuco_A4.png

Why ChArUco rather than a plain checkerboard
--------------------------------------------
A plain checkerboard must be ENTIRELY visible for findChessboardCorners to
return anything. That makes it very hard to get data near the image edges and
corners -- which is exactly where lens distortion is strongest and least
constrained. Calibrations shot this way end up with distortion coefficients
extrapolated into regions that were never observed, and a principal point
pulled toward wherever the board happened to spend its time.

ChArUco corners are individually identified by the ArUco markers around them,
so a PARTIALLY visible board still contributes. You can run the board off every
edge of the frame and still collect valid corners there. That single property
is what makes edge coverage achievable.

Printing
--------
Print at 100% / "actual size" -- NOT "fit to page", which silently rescales.

Absolute scale does NOT matter for intrinsics. The camera matrix and distortion
coefficients are invariant to board size: a board twice as large at twice the
distance produces an identical image. So do not agonise over the millimetres.

What DOES matter:
  * FLATNESS. Tape the print to glass, a clipboard, or foam board. A gentle
    curl is a systematic error the model cannot represent, and it will quietly
    bias your distortion coefficients.
  * UNIFORM scaling. Printers that stretch one axis make the squares
    non-square, which corrupts the fx/fy ratio. The ruler line printed on the
    sheet lets you check both axes with a tape measure.
  * MATTE paper. Glossy paper reflects the light source into the camera and
    ruins corner localisation.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

DICTS = {
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "5x5_100": cv2.aruco.DICT_5X5_100,
    "6x6_250": cv2.aruco.DICT_6X6_250,
}


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--squares-x", type=int, default=8)
    p.add_argument("--squares-y", type=int, default=11)
    p.add_argument("--square", type=float, default=0.022,
                   help="chessboard square size, metres")
    p.add_argument("--marker", type=float, default=0.017,
                   help="ArUco marker size, metres (must be < square)")
    p.add_argument("--dict", choices=list(DICTS), default="5x5_100")
    p.add_argument("--dpi", type=int, default=300)
    p.add_argument("--margin-mm", type=float, default=8.0)
    p.add_argument("--out", type=Path, default=Path("charuco_board.png"))
    args = p.parse_args(argv)

    if args.marker >= args.square:
        raise SystemExit("--marker must be smaller than --square")

    dictionary = cv2.aruco.getPredefinedDictionary(DICTS[args.dict])
    board = cv2.aruco.CharucoBoard(
        (args.squares_x, args.squares_y), args.square, args.marker, dictionary
    )

    px_per_m = args.dpi / 0.0254
    w = int(round(args.squares_x * args.square * px_per_m))
    h = int(round(args.squares_y * args.square * px_per_m))
    margin = int(round(args.margin_mm / 1000.0 * px_per_m))
    img = board.generateImage((w, h), marginSize=0)

    # Pad, then annotate with everything needed to verify the print.
    sheet = np.full((h + 2 * margin + int(0.9 * margin), w + 2 * margin), 255, np.uint8)
    sheet[margin:margin + h, margin:margin + w] = img

    n_corners = (args.squares_x - 1) * (args.squares_y - 1)
    label = (f"ChArUco {args.squares_x}x{args.squares_y}  dict {args.dict}  "
             f"square {args.square*1000:.1f} mm  marker {args.marker*1000:.1f} mm  "
             f"{n_corners} corners  -- print at 100%, keep FLAT")
    scale = max(0.4, w / 1600)
    cv2.putText(sheet, label, (margin, h + 2 * margin - int(0.15 * margin)),
                cv2.FONT_HERSHEY_SIMPLEX, scale, 0, max(1, int(scale * 2)), cv2.LINE_AA)

    # A 100 mm ruler so a tape measure can confirm the print scale on both axes.
    ruler_px = int(round(0.100 * px_per_m))
    y = h + 2 * margin + int(0.45 * margin)
    cv2.line(sheet, (margin, y), (margin + ruler_px, y), 0, max(2, int(scale * 3)))
    for x in (margin, margin + ruler_px):
        cv2.line(sheet, (x, y - int(0.12 * margin)), (x, y + int(0.12 * margin)),
                 0, max(2, int(scale * 3)))
    cv2.putText(sheet, "100 mm", (margin + ruler_px + int(0.2 * margin),
                                  y + int(0.12 * margin)),
                cv2.FONT_HERSHEY_SIMPLEX, scale, 0, max(1, int(scale * 2)), cv2.LINE_AA)

    Image.fromarray(sheet).save(args.out, dpi=(args.dpi, args.dpi))

    print(f"  wrote {args.out}")
    print(f"  board      {args.squares_x}x{args.squares_y} squares, "
          f"{n_corners} interior corners")
    print(f"  physical   {args.squares_x*args.square*1000:.1f} x "
          f"{args.squares_y*args.square*1000:.1f} mm  (+{args.margin_mm:.0f} mm margin)")
    print(f"  image      {sheet.shape[1]} x {sheet.shape[0]} px at {args.dpi} dpi")
    print(f"  dictionary {args.dict}")
    print("\n  Print at 100% / actual size, NOT fit-to-page.")
    print("  Then check the printed ruler line really measures 100 mm, both ways.")
    print("  Tape it FLAT to glass or a clipboard. Flatness matters; exact scale")
    print("  does not -- intrinsics are invariant to board size.")
    print(f"\n  Calibrate with:\n    ./venv/bin/python calibrate_intrinsics.py "
          f"--squares-x {args.squares_x} --squares-y {args.squares_y} "
          f"--square {args.square} --marker {args.marker} --dict {args.dict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
