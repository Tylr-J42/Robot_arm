#!/usr/bin/env python
"""Calibrate camera intrinsics (camera matrix + distortion) with guided capture.

    # 1. print a board
    ./venv/bin/python make_charuco_board.py --out charuco_A4.png

    # 2. guided live capture -> calibrate -> report
    ./venv/bin/python calibrate_intrinsics.py --out camera_intrinsics.yaml

    # or re-process an existing folder of images
    ./venv/bin/python calibrate_intrinsics.py --from-dir ./pics \
        --pattern checkerboard --grid 7x7

Why this exists
---------------
Intrinsics are now the ONLY stored calibration in this pipeline -- the camera's
extrinsic is re-solved from the base tags in every capture. So intrinsic error
is the dominant systematic, and it propagates straight into range: a 1% focal
length error is a 1% depth error at every distance.

What this adds over a plain calibrateCamera script:

  * GUIDED capture. It tracks which parts of the image and which board tilts
    you have already covered and auto-grabs frames that add something new, so
    you stop when the data is actually sufficient instead of guessing.
  * It PRINTS THE RMS. A calibration whose reprojection error you never looked
    at is not a calibration, it is a number of unknown quality.
  * Per-view errors and automatic outlier rejection. One blurred or
    mis-detected view can dominate the fit -- and typically does.
  * Parameter uncertainties (via calibrateCameraExtended), so you can see how
    well-determined fx and the principal point actually are.
  * A coverage map, because an empty image region means distortion coefficients
    extrapolated into territory that was never observed.
"""

from __future__ import annotations

import argparse
import datetime
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

import camera_capture
import constants

DICTS = {
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "5x5_100": cv2.aruco.DICT_5X5_100,
    "6x6_250": cv2.aruco.DICT_6X6_250,
}
SUBPIX_CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
GRID_X, GRID_Y = 6, 4


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------
class CharucoTarget:
    """ChArUco board: partial views still contribute, so edges are reachable."""

    def __init__(self, sx, sy, square, marker, dict_name):
        self.dictionary = cv2.aruco.getPredefinedDictionary(DICTS[dict_name])
        self.board = cv2.aruco.CharucoBoard((sx, sy), square, marker, self.dictionary)
        self.detector = cv2.aruco.CharucoDetector(self.board)
        self.all_obj = self.board.getChessboardCorners()
        self.name = f"charuco {sx}x{sy} ({(sx-1)*(sy-1)} corners)"

    def detect(self, gray):
        corners, ids, _, _ = self.detector.detectBoard(gray)
        if corners is None or ids is None or len(corners) < 4:
            return None, None
        obj = self.all_obj[ids.flatten()].astype(np.float32)
        return obj.reshape(-1, 3), corners.reshape(-1, 2).astype(np.float32)


class CheckerTarget:
    """Plain checkerboard: whole board must be visible. For legacy image sets."""

    def __init__(self, cols, rows, square):
        self.grid = (cols, rows)
        objp = np.zeros((cols * rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
        self.objp = objp
        self.name = f"checkerboard {cols}x{rows} inner corners"

    def detect(self, gray):
        ok, corners = cv2.findChessboardCorners(
            gray, self.grid,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not ok:
            return None, None
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRIT)
        return self.objp.copy(), corners.reshape(-1, 2)


# ---------------------------------------------------------------------------
# Coverage / pose-variety bookkeeping
# ---------------------------------------------------------------------------
class Coverage:
    """Tracks which image cells and which board poses have been sampled.

    Two things make a calibration well-conditioned, and neither is 'more
    images': spatial coverage (especially the edges, where distortion lives)
    and pose variety (tilt, which decorrelates focal length from distance).
    """

    def __init__(self, w, h, target_per_cell=25):
        self.w, self.h = w, h
        self.cells = np.zeros((GRID_Y, GRID_X), int)
        self.target = target_per_cell
        self.poses = []  # (cx_norm, cy_norm, area_frac, tilt_proxy)

    @staticmethod
    def _descriptor(pts, w, h):
        c = pts.mean(axis=0)
        span_x = np.ptp(pts[:, 0]) / w
        span_y = np.ptp(pts[:, 1]) / h
        # ratio of spans is a cheap proxy for out-of-plane tilt: a
        # fronto-parallel board keeps its aspect, a tilted one squashes.
        aspect = span_x / max(span_y, 1e-6)
        return np.array([c[0] / w, c[1] / h, span_x * span_y, aspect])

    def novelty(self, pts):
        """How different this view is from everything already accepted."""
        d = self._descriptor(pts, self.w, self.h)
        if not self.poses:
            return 1e9
        return float(min(np.linalg.norm(d - p) for p in self.poses))

    def gain(self, pts):
        """How many under-filled cells this view would contribute to."""
        g = 0
        for x, y in pts:
            cx = min(int(x / self.w * GRID_X), GRID_X - 1)
            cy = min(int(y / self.h * GRID_Y), GRID_Y - 1)
            if self.cells[cy, cx] < self.target:
                g += 1
        return g

    def add(self, pts):
        for x, y in pts:
            cx = min(int(x / self.w * GRID_X), GRID_X - 1)
            cy = min(int(y / self.h * GRID_Y), GRID_Y - 1)
            self.cells[cy, cx] += 1
        self.poses.append(self._descriptor(pts, self.w, self.h))

    @property
    def filled_fraction(self):
        return float((self.cells >= self.target).sum()) / (GRID_X * GRID_Y)

    def render(self, image):
        """Shade under-covered cells so the operator can see where to move."""
        overlay = image.copy()
        for cy in range(GRID_Y):
            for cx in range(GRID_X):
                x0 = int(cx * self.w / GRID_X)
                y0 = int(cy * self.h / GRID_Y)
                x1 = int((cx + 1) * self.w / GRID_X)
                y1 = int((cy + 1) * self.h / GRID_Y)
                frac = min(1.0, self.cells[cy, cx] / self.target)
                colour = (0, int(180 * frac), int(180 * (1 - frac)))
                if frac < 1.0:
                    cv2.rectangle(overlay, (x0, y0), (x1, y1), colour, -1)
                cv2.rectangle(image, (x0, y0), (x1, y1), (60, 60, 60), 1)
        cv2.addWeighted(overlay, 0.22, image, 0.78, 0, image)
        return image


# ---------------------------------------------------------------------------
# Calibration + diagnostics
# ---------------------------------------------------------------------------
def calibrate(obj_pts, img_pts, size, flags=0, drop_outliers=True, max_drop=5):
    """calibrateCameraExtended with iterative per-view outlier rejection."""
    kept = list(range(len(obj_pts)))
    dropped = []
    for _ in range(max_drop + 1):
        res = cv2.calibrateCameraExtended(
            [obj_pts[i] for i in kept], [img_pts[i] for i in kept],
            size, None, None, flags=flags,
        )
        rms, K, dist, rvecs, tvecs, sd_int, _, per_view = res
        pv = per_view.ravel()
        if not drop_outliers or len(kept) <= 6:
            break
        med = float(np.median(pv))
        worst = int(np.argmax(pv))
        if pv[worst] > max(3.0 * med, 1.0):
            dropped.append((kept[worst], float(pv[worst])))
            kept.pop(worst)
            continue
        break
    return dict(rms=rms, K=K, dist=dist, rvecs=rvecs, tvecs=tvecs,
                sd_int=sd_int.ravel(), per_view=pv, kept=kept, dropped=dropped)


def tilt_angles(rvecs):
    """Angle between each board's normal and the optical axis, degrees."""
    out = []
    for r in rvecs:
        R = cv2.Rodrigues(np.asarray(r))[0]
        out.append(float(np.degrees(np.arccos(min(1.0, abs(float(R[2, 2])))))))
    return out


def report(res, cov, size, target_name, n_input):
    K, dist, sd = res["K"], res["dist"].ravel(), res["sd_int"]
    w, h = size
    print("\n" + "=" * 72)
    print("  INTRINSIC CALIBRATION")
    print("=" * 72)
    print(f"\n  target      {target_name}")
    print(f"  resolution  {w} x {h}")
    print(f"  views       {len(res['kept'])} used of {n_input}")
    for idx, err in res["dropped"]:
        print(f"    dropped view {idx}: per-view error {err:.2f} px")

    print(f"\n  RMS reprojection error   {res['rms']:.4f} px")
    if res["rms"] < 0.3:
        print("    excellent")
    elif res["rms"] < 0.5:
        print("    good")
    elif res["rms"] < 1.0:
        print("    usable, but there is room to improve")
    else:
        print("    ** POOR. Expect blur, a non-flat board, or bad detections.")

    print(f"\n  fx {K[0,0]:9.3f}  +/- {sd[0]:6.3f}   ({sd[0]/K[0,0]*100:.2f} %)")
    print(f"  fy {K[1,1]:9.3f}  +/- {sd[1]:6.3f}   ({sd[1]/K[1,1]*100:.2f} %)")
    print(f"  cx {K[0,2]:9.3f}  +/- {sd[2]:6.3f}   (image centre {w/2:.0f}, "
          f"offset {K[0,2]-w/2:+.1f} px)")
    print(f"  cy {K[1,2]:9.3f}  +/- {sd[3]:6.3f}   (image centre {h/2:.0f}, "
          f"offset {K[1,2]-h/2:+.1f} px)")
    print(f"  fx/fy ratio {K[0,0]/K[1,1]:.5f}"
          + ("   ** far from 1.0: non-square pixels or a stretched print"
             if abs(K[0,0]/K[1,1] - 1) > 0.02 else ""))

    names = ["k1", "k2", "p1", "p2", "k3"]
    print("\n  distortion")
    for i, n in enumerate(names[:len(dist)]):
        s = sd[4 + i] if len(sd) > 4 + i else float("nan")
        print(f"    {n} {dist[i]:+10.6f}  +/- {s:.6f}")

    err_pct = sd[0] / K[0, 0] * 100
    print(f"\n  A {err_pct:.2f} % focal uncertainty is {err_pct/100*0.95*1000:.1f} mm "
          "of depth error at 0.95 m,")
    print("  and it scales linearly with range. This is the floor on your pose "
          "accuracy.")

    print(f"\n  coverage ({GRID_X}x{GRID_Y} cells, corners observed per cell):")
    for row in cov.cells:
        print("     " + " ".join(f"{v:5d}" for v in row))
    empty = int((cov.cells == 0).sum())
    if empty:
        print(f"    ** {empty} cell(s) with NO data. Distortion there is "
              "extrapolated,\n       and the principal point gets pulled toward "
              "wherever data exists.")
    elif (cov.cells < 10).any():
        print("    ** some cells are thin; edges are where distortion matters most.")
    else:
        print("    all cells covered")

    ang = tilt_angles([res["rvecs"][i] for i in range(len(res["kept"]))])
    steep = sum(a > 25 for a in ang)
    print(f"\n  board tilt   min {min(ang):.1f}  median {np.median(ang):.1f}  "
          f"max {max(ang):.1f} deg")
    print(f"  views tilted >25 deg: {steep}/{len(ang)}")
    if steep < max(3, len(ang) // 5):
        print("    ** Too few tilted views. Without tilt, focal length and board\n"
              "       distance are strongly correlated and fx is poorly pinned down.")


def write_outputs(res, size, target_name, out_path, n_input, settings=None):
    K, dist = res["K"], res["dist"].ravel()
    doc = {
        "schema": "camera_intrinsics/1",
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "resolution": [int(size[0]), int(size[1])],
        "target": target_name,
        "camera_matrix": [[float(v) for v in row] for row in K],
        "dist": [float(v) for v in dist],
        "rms_reprojection_px": round(float(res["rms"]), 5),
        "std_fx": float(res["sd_int"][0]),
        "std_fy": float(res["sd_int"][1]),
        "std_cx": float(res["sd_int"][2]),
        "std_cy": float(res["sd_int"][3]),
        "views_used": len(res["kept"]),
        "views_input": n_input,
        "note": "Valid ONLY at this resolution. Re-calibrate if the capture "
                "mode, lens, or focus changes.",
        # Recorded so the runtime can detect a geometry-critical setting drifting
        # away from what was calibrated. See camera_capture.check_against_calibration.
        "capture_settings": (
            {k: (float(v) if isinstance(v, (int, float)) and not isinstance(v, bool)
                 else v)
             for k, v in settings.items()} if settings else None),
    }
    Path(out_path).write_text(yaml.safe_dump(doc, sort_keys=False))
    print(f"\n  wrote {out_path}")

    print("\n" + "-" * 72)
    print("  Paste into constants.py to put this into service:\n")
    print("camera_matrix = np.array([")
    for row in K:
        print(f"    [{row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}],")
    print("    ])")
    print("\ndist = np.array([[" + ", ".join(f"{v:.8f}" for v in dist) + "]])")
    print(f"\ncamera_res = ({size[0]}, {size[1]})")
    print("-" * 72)


# ---------------------------------------------------------------------------
# Capture modes
# ---------------------------------------------------------------------------
def collect_live(target, args):
    """Guided capture: auto-grab frames that add coverage or pose variety."""
    cam = camera_capture.open_camera(args.device, args.exposure)
    w, h = constants.camera_res
    cov = Coverage(w, h, args.per_cell)
    obj_pts, img_pts, images = [], [], []
    settings = dict(cam.settings)
    camera_capture.settle(cam)

    print("\n  GUIDED CAPTURE")
    print("  Move the board so the shaded (red) cells fill in. Cover the EDGES "
          "and\n  CORNERS of the frame -- run the board off the sides; ChArUco "
          "still\n  contributes from a partial view. Tilt it 30-45 deg in "
          "varying directions.\n")
    print("  SPACE force-capture    c calibrate now    u undo last    q abort\n")

    try:
        while True:
            ok, frame = cam.read()
            if not ok:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            obj, img = target.detect(gray)

            view = cov.render(frame.copy())
            grabbed = False
            if obj is not None and len(img) >= args.min_corners:
                for pt in img:
                    cv2.circle(view, (int(pt[0]), int(pt[1])), 3, (0, 255, 255), -1)
                novel = cov.novelty(img)
                gain = cov.gain(img)
                if len(obj_pts) < args.max_views and (
                    novel > args.novelty and gain >= args.min_gain
                ):
                    obj_pts.append(obj)
                    img_pts.append(img)
                    images.append(frame.copy())
                    cov.add(img)
                    grabbed = True

            banner = (f"views {len(obj_pts)}/{args.max_views}   "
                      f"cells filled {cov.filled_fraction*100:.0f}%   "
                      f"corners {0 if img is None else len(img)}")
            cv2.putText(view, banner, (18, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (255, 255, 255), 2, cv2.LINE_AA)
            if grabbed:
                cv2.rectangle(view, (0, 0), (w - 1, h - 1), (0, 255, 0), 12)
            cv2.imshow("intrinsics capture", view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                obj_pts = []
                break
            if key == ord("c"):
                break
            if key == ord("u") and obj_pts:
                obj_pts.pop(); img_pts.pop(); images.pop()
                cov.poses.pop()
                print(f"  undid last view ({len(obj_pts)} remain); "
                      "coverage counts keep the old contribution")
            if key == ord(" ") and obj is not None and len(img) >= args.min_corners:
                obj_pts.append(obj); img_pts.append(img); images.append(frame.copy())
                cov.add(img)
            if len(obj_pts) >= args.max_views:
                print("  reached --max-views")
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()

    if args.save_dir and images:
        d = Path(args.save_dir); d.mkdir(parents=True, exist_ok=True)
        for i, im in enumerate(images):
            cv2.imwrite(str(d / f"view_{i:03d}.png"), im)
        print(f"  saved {len(images)} frames to {d}")
    return obj_pts, img_pts, cov, (w, h), settings


def collect_dir(target, args):
    """Re-process an existing folder of images."""
    files = sorted(
        f for ext in ("*.jpg", "*.jpeg", "*.png", "*.JPG", "*.PNG")
        for f in Path(args.from_dir).glob(ext)
    )
    if not files:
        raise SystemExit(f"no images found in {args.from_dir}")
    obj_pts, img_pts, size, cov = [], [], None, None
    for f in files:
        im = cv2.imread(str(f))
        if im is None:
            continue
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        if size is None:
            size = gray.shape[::-1]
            cov = Coverage(size[0], size[1], args.per_cell)
        obj, img = target.detect(gray)
        status = "no detection"
        if obj is not None and len(img) >= args.min_corners:
            obj_pts.append(obj); img_pts.append(img); cov.add(img)
            status = f"{len(img)} corners"
        print(f"    {f.name:<24} {status}")
    print(f"\n  {len(obj_pts)} of {len(files)} images usable")
    return obj_pts, img_pts, cov, size, None


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--pattern", choices=["charuco", "checkerboard"], default="charuco")
    p.add_argument("--squares-x", type=int, default=8)
    p.add_argument("--squares-y", type=int, default=11)
    p.add_argument("--square", type=float, default=0.022)
    p.add_argument("--marker", type=float, default=0.017)
    p.add_argument("--dict", choices=list(DICTS), default="5x5_100")
    p.add_argument("--grid", default="7x7",
                   help="checkerboard INNER corners, e.g. 7x7")
    p.add_argument("--from-dir", type=Path, default=None,
                   help="calibrate from a folder instead of the live camera")
    p.add_argument("--save-dir", type=Path, default=None,
                   help="save accepted live frames here")
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--exposure", type=int, default=None,
                   help="override constants.CAMERA_SETTINGS exposure")
    p.add_argument("--max-views", type=int, default=40)
    p.add_argument("--min-corners", type=int, default=12)
    p.add_argument("--per-cell", type=int, default=25,
                   help="corner observations per cell to call it covered")
    p.add_argument("--novelty", type=float, default=0.08,
                   help="how different a view must be to auto-capture")
    p.add_argument("--min-gain", type=int, default=3,
                   help="under-filled cells a view must touch to auto-capture")
    p.add_argument("--fix-k3", action="store_true",
                   help="hold k3 at zero; steadier for mild lenses")
    p.add_argument("--zero-tangent", action="store_true",
                   help="assume no sensor/lens misalignment (p1=p2=0)")
    p.add_argument("--no-drop", action="store_true",
                   help="keep outlier views instead of rejecting them")
    p.add_argument("--undistort-preview", action="store_true")
    p.add_argument("--out", type=Path, default=Path("camera_intrinsics.yaml"))
    args = p.parse_args(argv)

    if args.pattern == "charuco":
        target = CharucoTarget(args.squares_x, args.squares_y, args.square,
                               args.marker, args.dict)
    else:
        cols, rows = (int(v) for v in args.grid.lower().split("x"))
        target = CheckerTarget(cols, rows, args.square)
    print(f"  target: {target.name}")

    if args.from_dir:
        obj_pts, img_pts, cov, size, settings = collect_dir(target, args)
    else:
        obj_pts, img_pts, cov, size, settings = collect_live(target, args)

    if len(obj_pts) < 6:
        print(f"\n  only {len(obj_pts)} usable views -- need at least 6. Aborting.")
        return 1

    flags = 0
    if args.fix_k3:
        flags |= cv2.CALIB_FIX_K3
    if args.zero_tangent:
        flags |= cv2.CALIB_ZERO_TANGENT_DIST

    res = calibrate(obj_pts, img_pts, size, flags=flags,
                    drop_outliers=not args.no_drop)
    report(res, cov, size, target.name, len(obj_pts))
    write_outputs(res, size, target.name, args.out, len(obj_pts), settings)

    if args.undistort_preview and not args.from_dir:
        cam = camera_capture.open_camera(args.device, args.exposure)
        camera_capture.settle(cam)
        ok, frame = cam.read()
        cam.release()
        if ok:
            und = cv2.undistort(frame, res["K"], res["dist"])
            both = np.hstack([frame, und])
            cv2.putText(both, "raw", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 255), 2)
            cv2.putText(both, "undistorted", (size[0] + 20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("undistort preview (any key to close)",
                       cv2.resize(both, None, fx=0.5, fy=0.5))
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
