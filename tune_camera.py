#!/usr/bin/env python
"""Find camera settings that maximise AprilTag detection. Point it at the tags.

    ./venv/bin/python tune_camera.py --fourcc YUYV --sweep exposure
    ./venv/bin/python tune_camera.py --sweep brightness --live

Ranks settings by what actually matters -- how many mapped tags are found and
how confidently (decision margin) -- not by how the picture looks to a human.

Two hard-won notes about this class of webcam:

  * Some UVC firmware ignores manual exposure in MJPG and honours it only in
    YUYV. If an exposure sweep shows no change in mean brightness, re-run with
    --fourcc YUYV before concluding the control is broken.
  * V4L2 silently CLAMPS an out-of-range value to the nearest legal one, which
    is frequently the worst setting available, and reports success. Always read
    the "set/readback" columns rather than trusting the value you asked for.
"""

from __future__ import annotations

import argparse
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np

import apriltag_detect
import constants
import tag_map as tmap

HERE = Path(__file__).resolve().parent

# sweep name -> (v4l2 control, OpenCV prop, fallback range)
CONTROLS = {
    "exposure": ("exposure_time_absolute", cv2.CAP_PROP_EXPOSURE, (10, 400)),
    "brightness": ("brightness", cv2.CAP_PROP_BRIGHTNESS, (-100, 100)),
    "contrast": ("contrast", cv2.CAP_PROP_CONTRAST, (60, 255)),
    "gamma": ("gamma", cv2.CAP_PROP_GAMMA, (72, 500)),
    "sharpness": ("sharpness", cv2.CAP_PROP_SHARPNESS, (0, 255)),
}


def v4l_set(dev, ctl, val):
    subprocess.run(["v4l2-ctl", "-d", dev, "-c", f"{ctl}={val}"], capture_output=True)


def v4l_get(dev, ctl):
    r = subprocess.run(["v4l2-ctl", "-d", dev, "-C", ctl],
                       capture_output=True, text=True)
    try:
        return int(r.stdout.strip().split(":")[-1])
    except Exception:
        return None


def v4l_range(dev, ctl):
    r = subprocess.run(["v4l2-ctl", "-d", dev, "--list-ctrls"],
                       capture_output=True, text=True)
    for line in r.stdout.splitlines():
        if line.strip().startswith(ctl):
            lo = hi = None
            for tok in line.split():
                if tok.startswith("min="):
                    lo = int(tok[4:])
                if tok.startswith("max="):
                    hi = int(tok[4:])
            if lo is not None and hi is not None:
                return lo, hi
    return None


def settle(cam, frames=12, pause=0.35):
    """Flush the driver's queue after a control change.

    Without this you sample frames captured mid-transition and the numbers
    bounce wildly between the old and new setting.
    """
    for _ in range(frames):
        cam.read()
    time.sleep(pause)


def score(gray, detector, wanted):
    dets = apriltag_detect.detect(detector, gray)
    if wanted:
        dets = [d for d in dets if int(d.tag_id) in wanted]
    margins = sorted(float(d.decision_margin) for d in dets)
    clip = 100.0 * float((gray >= 250).sum()) / gray.size
    dark = 100.0 * float((gray <= 8).sum()) / gray.size
    return len(dets), margins, float(gray.mean()), float(gray.std()), clip, dark


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sweep", choices=list(CONTROLS), default="exposure")
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--fourcc", default=None,
                   help="YUYV or MJPG; defaults to constants.CAMERA_SETTINGS. Some "
                        "firmware only honours exposure in YUYV")
    p.add_argument("--steps", type=int, default=8)
    p.add_argument("--min", type=int, default=None)
    p.add_argument("--max", type=int, default=None)
    p.add_argument("--tags", type=Path, default=HERE / "tags_base_assy.yaml")
    p.add_argument("--all-ids", action="store_true",
                   help="count every tag, not only those in the map")
    p.add_argument("--live", action="store_true", help="show the feed while sweeping")
    args = p.parse_args(argv)

    import camera_capture as cc
    if args.fourcc is None:
        args.fourcc = cc.get_settings()["fourcc"]
    dev = f"/dev/video{args.device}"
    ctl, _, fallback = CONTROLS[args.sweep]
    wanted = set()
    if not args.all_ids:
        try:
            wanted = set(tmap.load_tag_map(args.tags))
            print(f"  scoring against {sorted(wanted)} from {args.tags.name}")
        except Exception as e:
            print(f"  [warn] no tag map ({e}); counting all detections")

    rng = v4l_range(dev, ctl) or fallback
    lo = args.min if args.min is not None else rng[0]
    hi = args.max if args.max is not None else rng[1]
    values = sorted({int(v) for v in np.linspace(lo, hi, args.steps)})
    print(f"  sweeping {ctl} over {lo}..{hi} ({len(values)} steps) in {args.fourcc}")

    W, H = constants.camera_res
    cam = cv2.VideoCapture(args.device)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    got = (int(cam.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if got != (W, H):
        cam.release()
        raise SystemExit(f"camera gave {got[0]}x{got[1]}, expected {W}x{H}")

    # deterministic baseline for everything not being swept
    v4l_set(dev, "auto_exposure", 1)
    v4l_set(dev, "white_balance_automatic", 0)
    v4l_set(dev, "focus_automatic_continuous", 0)
    v4l_set(dev, "sharpness", 0)

    detector = apriltag_detect.make_detector()
    print(f"\n{'set':>7}{'read':>7}{'mean':>7}{'std':>7}{'clip%':>7}{'dark%':>7}"
          f"{'tags':>6}  margins")
    rows = []
    try:
        for v in values:
            v4l_set(dev, ctl, v)
            settle(cam)
            ok, frame = cam.read()
            if not ok:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            n, margins, mean, std, clip, dark = score(gray, detector, wanted)
            rb = v4l_get(dev, ctl)
            print(f"{v:>7}{('-' if rb is None else rb):>7}{mean:>7.1f}{std:>7.1f}"
                  f"{clip:>7.2f}{dark:>7.2f}{n:>6}  "
                  f"{[int(m) for m in margins]}")
            rows.append((n, float(np.mean(margins)) if margins else 0.0, -clip, v))
            if args.live:
                cv2.putText(frame, f"{ctl}={v}  tags={n}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                cv2.imshow("tune", frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    finally:
        cam.release()
        cv2.destroyAllWindows()

    if not rows:
        print("\n  no frames captured")
        return 1
    means = [r[3] for r in rows]
    if len({round(m, 1) for m in [r[0] for r in rows]}) == 1 and args.sweep == "exposure":
        pass
    best = max(rows)
    print(f"\n  best {ctl} = {best[3]}  ({best[0]} tags, mean margin {best[1]:.0f}, "
          f"clip {-best[2]:.2f}%)")
    if best[0] == 0:
        print("  No tags found at ANY setting -- this is not an exposure problem.\n"
              "  Check the tags are in frame, in focus, and that the family in\n"
              "  the map matches what is printed.")
    else:
        print(f"\n  Put it into service in constants.py:\n"
              f"    CAMERA_SETTINGS = {{\"fourcc\": \"{args.fourcc}\", "
              f"\"{args.sweep}\": {best[3]}, ...}}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
