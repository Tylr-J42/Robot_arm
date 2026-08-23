"""Minimal dependency-free vector PDF writer, for printing at exact scale.

Why this exists
---------------
A PNG carries its physical size only as advisory metadata (the pHYs chunk).
Most print paths ignore it and scale the bitmap to fill the page, so "print
without scaling" still comes out the wrong size. A PDF instead defines its page
in POINTS (1/72 inch) and places content at absolute coordinates, so a printer
set to 100% / "actual size" reproduces it exactly.

Everything here is vector -- the tag cells are filled rectangles, not a raster
image -- so there is no resampling and edges stay perfectly crisp at whatever
resolution the printer runs at. That matters for fiducials: a resampled bitmap
softens the black/white border that corner localization depends on.

Coordinates are given in millimetres from the TOP-LEFT of the page, which is
how page layout is normally reasoned about; the conversion to PDF's bottom-left
origin in points happens internally.
"""

from __future__ import annotations

MM_TO_PT = 72.0 / 25.4
PAGES_MM = {"a4": (210.0, 297.0), "letter": (215.9, 279.4)}


def _esc(text: str) -> str:
    return text.replace("\\", r"\\").replace("(", r"\(").replace(")", r"\)")


class PDFPage:
    """One page of vector content, in millimetres from the top-left."""

    def __init__(self, width_mm: float, height_mm: float):
        self.w_mm, self.h_mm = float(width_mm), float(height_mm)
        self.ops: list[str] = []

    # -- internal: mm from top-left -> points from bottom-left ---------------
    def _x(self, mm: float) -> float:
        return mm * MM_TO_PT

    def _y(self, mm: float) -> float:
        return (self.h_mm - mm) * MM_TO_PT

    def rect(self, x, y, w, h, gray: float = 0.0) -> None:
        """Filled rectangle; (x, y) is its top-left corner."""
        self.ops.append(f"{gray:.4f} g")
        self.ops.append(
            f"{self._x(x):.4f} {self._y(y + h):.4f} "
            f"{w * MM_TO_PT:.4f} {h * MM_TO_PT:.4f} re f"
        )

    def line(self, x1, y1, x2, y2, width_mm: float = 0.3, gray: float = 0.0) -> None:
        self.ops.append(f"{gray:.4f} G {width_mm * MM_TO_PT:.4f} w")
        self.ops.append(
            f"{self._x(x1):.4f} {self._y(y1):.4f} m "
            f"{self._x(x2):.4f} {self._y(y2):.4f} l S"
        )

    def text(self, x, y, s: str, size_pt: float = 9.0, gray: float = 0.0) -> None:
        """Baseline-left text at (x, y)."""
        self.ops.append(
            f"BT {gray:.4f} g /F1 {size_pt:.2f} Tf "
            f"{self._x(x):.4f} {self._y(y):.4f} Td ({_esc(s)}) Tj ET"
        )

    def content(self) -> bytes:
        return ("\n".join(self.ops) + "\n").encode("latin-1")


def write_pdf(path, pages: list[PDFPage]) -> None:
    """Serialise pages to a PDF 1.4 file with a correct xref table."""
    out = bytearray(b"%PDF-1.4\n")
    offsets: dict[int, int] = {}

    def obj(num: int, body: bytes) -> None:
        offsets[num] = len(out)
        out.extend(f"{num} 0 obj\n".encode())
        out.extend(body)
        out.extend(b"\nendobj\n")

    n_pages = len(pages)
    font_num = 3 + 2 * n_pages
    page_nums = [3 + 2 * i for i in range(n_pages)]

    obj(1, b"<< /Type /Catalog /Pages 2 0 R >>")
    kids = " ".join(f"{n} 0 R" for n in page_nums)
    obj(2, f"<< /Type /Pages /Kids [{kids}] /Count {n_pages} >>".encode())

    for i, page in enumerate(pages):
        pnum, cnum = page_nums[i], page_nums[i] + 1
        obj(pnum, (
            f"<< /Type /Page /Parent 2 0 R /MediaBox "
            f"[0 0 {page.w_mm * MM_TO_PT:.4f} {page.h_mm * MM_TO_PT:.4f}] "
            f"/Resources << /Font << /F1 {font_num} 0 R >> >> "
            f"/Contents {cnum} 0 R >>"
        ).encode())
        data = page.content()
        obj(cnum, b"<< /Length " + str(len(data)).encode() + b" >>\nstream\n"
            + data + b"endstream")

    obj(font_num, b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica "
                  b"/Encoding /WinAnsiEncoding >>")

    total = font_num + 1
    xref_at = len(out)
    out.extend(f"xref\n0 {total}\n".encode())
    out.extend(b"0000000000 65535 f \n")
    for num in range(1, total):
        out.extend(f"{offsets[num]:010d} 00000 n \n".encode())
    out.extend(
        f"trailer\n<< /Size {total} /Root 1 0 R >>\nstartxref\n{xref_at}\n%%EOF\n"
        .encode()
    )
    with open(path, "wb") as fh:
        fh.write(bytes(out))
