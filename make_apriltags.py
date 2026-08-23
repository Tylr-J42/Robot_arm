#!/usr/bin/env python
"""Generate printable AprilTags at an exact physical size.

    ./venv/bin/python make_apriltags.py --family tag16h5 --ids 0,1,2,3

By default the tag size is read from tags_base_assy.yaml's ``default_size``, so
the printed tags match what the map already declares.

Size means the edge of the BLACK BORDER SQUARE -- the outer edge of the black
ring, not including the white quiet zone. That is the edge dt_apriltags reports
as ``det.corners``, and therefore the length ``size:`` refers to in a tag map.

The 180-degree rotation
-----------------------
cv2.aruco renders the DICT_APRILTAG_* dictionaries rotated 180 degrees relative
to the apriltag library's canonical orientation. Verified for both tag16h5 and
tag36h11 by rendering a tag at a known pose and comparing dt_apriltags' own
estimate_tag_pose output against ground truth: unrotated gives a 179.96 deg
error, rotated gives 0.07 deg. This tool applies the rotation, so a printed tag
matches TAG_CORNER_OFFSETS_UNIT in tag_map.py.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image

from pdf_sheet import PAGES_MM, PDFPage, write_pdf

HERE = Path(__file__).resolve().parent

# family -> (aruco dictionary, data grid cells, number of ids)
FAMILIES = {
    "tag16h5": (cv2.aruco.DICT_APRILTAG_16h5, 4, 30),
    "tag25h9": (cv2.aruco.DICT_APRILTAG_25h9, 5, 35),
    "tag36h11": (cv2.aruco.DICT_APRILTAG_36h11, 6, 587),
}
PAGES = {"a4": (210.0, 297.0), "letter": (215.9, 279.4), "none": None}


def default_size_from_map(path: Path) -> float | None:
    try:
        doc = yaml.safe_load(path.read_text())
        return float(doc["default_size"])
    except Exception:
        return None


def tag_image(family: str, tag_id: int, px: int) -> np.ndarray:
    dict_id, _, n_ids = FAMILIES[family]
    if not 0 <= tag_id < n_ids:
        raise SystemExit(f"{family} has ids 0..{n_ids-1}; {tag_id} is out of range")
    d = cv2.aruco.getPredefinedDictionary(dict_id)
    # ROTATE_180: see module docstring.
    return cv2.rotate(cv2.aruco.generateImageMarker(d, tag_id, px), cv2.ROTATE_180)


def tag_bits(family: str, tag_id: int) -> np.ndarray:
    """(cells, cells) bool grid, True = black. Includes the 1-cell border."""
    cells = FAMILIES[family][1] + 2
    img = tag_image(family, tag_id, cells * 20)
    step = img.shape[0] / cells
    grid = np.zeros((cells, cells), bool)
    for r in range(cells):
        for c in range(cells):
            patch = img[int(r * step) + 5:int((r + 1) * step) - 5,
                        int(c * step) + 5:int((c + 1) * step) - 5]
            grid[r, c] = patch.mean() < 128
    return grid


def build_pdf(family, ids, size_m, quiet_m, page_name, out_path):
    """Lay the tags out as VECTOR rectangles at absolute millimetre positions.

    Nothing here is a bitmap, so the printer rasterises at its own native
    resolution and the tag edges stay perfectly sharp -- which is what corner
    localization actually depends on.
    """
    pw, ph = PAGES_MM[page_name]
    size, quiet = size_m * 1000.0, quiet_m * 1000.0
    cells = FAMILIES[family][1] + 2
    cell = size / cells

    margin, header, caption, ruler_h = 12.0, 10.0, 8.0, 14.0
    tile_w = size + 2 * quiet
    tile_h = tile_w + caption
    cols = max(1, int((pw - 2 * margin) // tile_w))
    rows = max(1, int((ph - 2 * margin - header - ruler_h) // tile_h))
    per_page = cols * rows

    pages, placed = [], 0
    for start in range(0, len(ids), per_page):
        chunk = ids[start:start + per_page]
        page = PDFPage(pw, ph)
        page.text(margin, margin + 4,
                  f"{family}  ids {chunk}  black border {size:.1f} mm  "
                  f"-- PRINT AT 100% (actual size), keep FLAT", 8.5)
        x0 = max(margin, (pw - cols * tile_w) / 2.0)
        y0 = margin + header

        for k, tag_id in enumerate(chunk):
            r, c = divmod(k, cols)
            tx, ty = x0 + c * tile_w, y0 + r * tile_h
            # white quiet zone, then the tag drawn black-first and punched out
            page.rect(tx, ty, tile_w, tile_w, 1.0)
            page.rect(tx + quiet, ty + quiet, size, size, 0.0)
            bits = tag_bits(family, tag_id)
            for rr in range(cells):
                for cc in range(cells):
                    if not bits[rr, cc]:
                        page.rect(tx + quiet + cc * cell, ty + quiet + rr * cell,
                                  cell, cell, 1.0)
            # corner ticks in the quiet zone: what to check with calipers
            for cxm in (tx + quiet, tx + quiet + size):
                for cym in (ty + quiet, ty + quiet + size):
                    dx = -1 if cxm == tx + quiet else 1
                    dy = -1 if cym == ty + quiet else 1
                    page.line(cxm + dx * 2, cym, cxm + dx * 6, cym, 0.25)
                    page.line(cxm, cym + dy * 2, cxm, cym + dy * 6, 0.25)
            page.text(tx + quiet, ty + tile_w + 5.5,
                      f"{family}  id {tag_id}   {size:.1f} mm", 8)
            placed += 1

        ry = ph - margin
        page.line(margin, ry, margin + 100.0, ry, 0.3)
        page.line(margin, ry - 2, margin, ry + 2, 0.3)
        page.line(margin + 100.0, ry - 2, margin + 100.0, ry + 2, 0.3)
        page.text(margin + 104.0, ry + 2, "100 mm", 8)
        pages.append(page)

    write_pdf(out_path, pages)
    return len(pages), cols, rows


def build_tile(family, tag_id, size_m, quiet_m, dpi, label=True):
    """One tag plus its white quiet zone and a caption strip below it."""
    ppm = dpi / 0.0254
    tag_px = int(round(size_m * ppm))
    quiet_px = int(round(quiet_m * ppm))
    cap_px = int(round(0.010 * ppm)) if label else 0

    n = tag_px + 2 * quiet_px
    tile = np.full((n + cap_px, n), 255, np.uint8)
    tile[quiet_px:quiet_px + tag_px, quiet_px:quiet_px + tag_px] = tag_image(
        family, tag_id, tag_px)

    # Corner ticks marking the BLACK BORDER edge, drawn in the quiet zone so
    # they never touch the tag: these are what you measure with calipers.
    t = max(2, int(round(0.0015 * ppm)))
    arm = int(round(0.004 * ppm))
    gap = int(round(0.002 * ppm))
    for cx in (quiet_px, quiet_px + tag_px):
        for cy in (quiet_px, quiet_px + tag_px):
            dx = -1 if cx == quiet_px else 1
            dy = -1 if cy == quiet_px else 1
            cv2.line(tile, (cx + dx * gap, cy), (cx + dx * (gap + arm), cy), 0, t)
            cv2.line(tile, (cx, cy + dy * gap), (cx, cy + dy * (gap + arm)), 0, t)

    if label:
        s = max(0.35, tag_px / 900)
        cv2.putText(tile, f"{family}  id {tag_id}   {size_m*1000:.1f} mm",
                    (quiet_px, n + int(cap_px * 0.72)),
                    cv2.FONT_HERSHEY_SIMPLEX, s, 0, max(1, int(s * 2)), cv2.LINE_AA)
    return tile


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--family", choices=list(FAMILIES), default="tag16h5")
    p.add_argument("--ids", default="0,1,2,3", help="comma-separated tag ids")
    p.add_argument("--size", type=float, default=None,
                   help="black-border edge, metres (default: from --map)")
    p.add_argument("--map", type=Path, default=HERE / "tags_base_assy.yaml")
    p.add_argument("--quiet-frac", type=float, default=0.25,
                   help="white margin as a fraction of tag size")
    p.add_argument("--page", choices=list(PAGES), default="a4")
    p.add_argument("--dpi", type=int, default=600)
    p.add_argument("--format", choices=["pdf", "png", "both"], default="pdf",
                   help="pdf prints at exact scale; png DPI metadata is only "
                        "advisory and most print paths ignore it")
    p.add_argument("--separate", action="store_true",
                   help="one file per tag instead of a sheet")
    p.add_argument("--out", type=Path, default=Path("apriltags.png"))
    args = p.parse_args(argv)

    ids = [int(v) for v in args.ids.split(",") if v.strip() != ""]
    size = args.size or default_size_from_map(args.map)
    if size is None:
        raise SystemExit(f"no --size given and could not read default_size from {args.map}")
    quiet = size * args.quiet_frac
    cells = FAMILIES[args.family][1] + 2
    ppm = args.dpi / 0.0254

    print(f"  family     {args.family}  ({FAMILIES[args.family][2]} ids available)")
    print(f"  ids        {ids}")
    print(f"  tag size   {size*1000:.2f} mm black border  "
          f"({cells} cells -> {size/cells*1000:.2f} mm per cell)")
    print(f"  quiet zone {quiet*1000:.1f} mm  ({quiet/(size/cells):.1f} cells)")
    if quiet < size / cells:
        print("    [warn] quiet zone under one cell; detection will suffer")
    if args.size is None:
        print(f"  (size taken from {args.map})")

    if args.format in ("pdf", "both"):
        if args.page == "none":
            raise SystemExit("--format pdf needs a real --page (a4 or letter)")
        pdf_path = args.out.with_suffix(".pdf")
        n_pages, cols_, rows_ = build_pdf(args.family, ids, size, quiet,
                                          args.page, pdf_path)
        print(f"\n  wrote {pdf_path}   {n_pages} page(s), {cols_}x{rows_} per page, "
              f"{args.page.upper()}, VECTOR")
        print("    PDF carries absolute physical size -- print at 100% / actual size.")
        if args.format == "pdf":
            print("\n  Verify the 100 mm ruler with a tape measure, then check one")
            print("  tag's black border against the corner ticks with calipers.")
            return 0

    tiles = [(i, build_tile(args.family, i, size, quiet, args.dpi)) for i in ids]
    th, tw = tiles[0][1].shape

    if args.separate:
        for tag_id, tile in tiles:
            path = args.out.with_name(f"{args.out.stem}_{args.family}_{tag_id}.png")
            Image.fromarray(tile).save(path, dpi=(args.dpi, args.dpi))
            print(f"  wrote {path}")
        return 0

    page = PAGES[args.page]
    margin = int(round(0.012 * ppm))
    header = int(round(0.012 * ppm))
    if page:
        pw, ph = int(round(page[0] / 1000 * ppm)), int(round(page[1] / 1000 * ppm))
        cols = max(1, (pw - 2 * margin) // tw)
        rows_fit = max(1, (ph - 2 * margin - header) // th)
        if cols * rows_fit < len(tiles):
            print(f"    [warn] only {cols*rows_fit} tags fit on {args.page}; "
                  "use --separate or a smaller --size")
    else:
        cols = min(len(tiles), 2)
        rows_fit = 10**6
    rows = int(np.ceil(len(tiles) / cols))

    if page:
        sheet = np.full((ph, pw), 255, np.uint8)
        x0 = max(margin, (pw - cols * tw) // 2)
        y0 = margin + header
    else:
        sheet = np.full((rows * th + 2 * margin + header, cols * tw + 2 * margin),
                        255, np.uint8)
        x0, y0 = margin, margin + header

    for k, (_, tile) in enumerate(tiles):
        r, c = divmod(k, cols)
        y, x = y0 + r * th, x0 + c * tw
        if y + th > sheet.shape[0] or x + tw > sheet.shape[1]:
            print(f"    [warn] tag {ids[k]} does not fit on the page, skipped")
            continue
        sheet[y:y + th, x:x + tw] = tile

    s = max(0.4, sheet.shape[1] / 2600)
    cv2.putText(sheet, f"{args.family}  ids {ids}  black border {size*1000:.1f} mm  "
                f"-- PRINT AT 100% (actual size), keep FLAT",
                (margin, margin + int(header * 0.62)),
                cv2.FONT_HERSHEY_SIMPLEX, s, 0, max(1, int(s * 2)), cv2.LINE_AA)

    ruler = int(round(0.100 * ppm))
    ry = sheet.shape[0] - margin
    if ruler + 2 * margin < sheet.shape[1]:
        cv2.line(sheet, (margin, ry), (margin + ruler, ry), 0, max(2, int(s * 3)))
        for x in (margin, margin + ruler):
            cv2.line(sheet, (x, ry - int(0.003 * ppm)), (x, ry + int(0.003 * ppm)),
                     0, max(2, int(s * 3)))
        cv2.putText(sheet, "100 mm", (margin + ruler + int(0.004 * ppm), ry),
                    cv2.FONT_HERSHEY_SIMPLEX, s, 0, max(1, int(s * 2)), cv2.LINE_AA)

    Image.fromarray(sheet).save(args.out, dpi=(args.dpi, args.dpi))
    print(f"\n  wrote {args.out}   {sheet.shape[1]}x{sheet.shape[0]} px @ {args.dpi} dpi"
          + (f"  ({args.page.upper()})" if page else ""))
    print("\n  Print at 100% / actual size -- NOT fit-to-page.")
    print("  Verify the 100 mm ruler with a tape measure, then check one tag's")
    print("  black border against the corner ticks with calipers. Unlike the")
    print("  calibration board, THIS size matters: it is the scale of your map.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
