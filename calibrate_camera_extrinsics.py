#!/usr/bin/env python
"""Solve where the static camera sits in the arm's ``base_assy`` frame.

Point the camera at several AprilTags whose poses you have measured into
``tags_base_assy.yaml``, run this, and it writes ``camera_extrinsics.yaml``
containing the camera pose plus the diagnostics needed to judge whether to
believe it.

    # 1. FIRST, verify corner ordering -- everything else depends on this
    ./venv/bin/python calibrate_camera_extrinsics.py --corner-check --display

    # 2. Then calibrate
    ./venv/bin/python calibrate_camera_extrinsics.py --display --verify

ROLE: this is a SETUP AND VALIDATION tool, not part of the runtime path.
Nothing at pick time reads camera_extrinsics.yaml -- locate_cube.py and
cube_pick_target.py re-solve the camera pose from the base tags in each capture.
What this tool is for:

  * --corner-check, which must pass before any of the rest is trustworthy
  * proving a freshly measured tags_base_assy.yaml is self-consistent, via
    reprojection rms and the leave-one-tag-out table
  * a high-quality reference camera pose (averaged over many frames) to compare
    a single per-capture solve against

Fix your base tag map here, with these diagnostics, before relying on it.
"""

from __future__ import annotations

import argparse
import datetime
import sys
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import yaml

import apriltag_detect
import camera_capture
import camera_extrinsics as ext
import constants
import tag_map as tmap

HERE = Path(__file__).resolve().parent
DEFAULT_TAGS = HERE / "tags_base_assy.yaml"
DEFAULT_OUT = HERE / "camera_extrinsics.yaml"

# Colours (BGR) for the four corner indices, so the ordering is readable at a
# glance in --corner-check: 0 red, 1 green, 2 blue, 3 yellow.
CORNER_COLOURS = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]


# ---------------------------------------------------------------------------
# Overlays
# ---------------------------------------------------------------------------
def draw_tag_outline(image, det, colour=(0, 0, 255)) -> None:
    for i in range(4):
        j = (i + 1) % 4
        cv2.line(
            image,
            (int(det.corners[i][0]), int(det.corners[i][1])),
            (int(det.corners[j][0]), int(det.corners[j][1])),
            colour,
            2,
        )


def draw_corner_indices(image, det) -> None:
    """Label each det.corners[i] with its index.

    This is how the corner-ordering convention gets pinned down for good: hold a
    tag upright, read off which physical corner is index 0, and confirm it
    matches TAG_CORNER_OFFSETS_UNIT in tag_map.py.
    """
    for i, (cx, cy) in enumerate(det.corners):
        cv2.circle(image, (int(cx), int(cy)), 6, CORNER_COLOURS[i], -1)
        cv2.putText(
            image,
            str(i),
            (int(cx) + 8, int(cy) - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            CORNER_COLOURS[i],
            2,
            cv2.LINE_AA,
        )


def solve_single_tag(corners_img, size, K, dist):
    """Pose of one tag in the camera frame, from its 4 corners alone."""
    obj = tmap.tag_corner_offsets(size).reshape(-1, 1, 3)
    img = np.asarray(corners_img, dtype=np.float64).reshape(-1, 1, 2)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    if not ok or not len(rvecs):
        return None
    return rvecs[0], tvecs[0]


# ---------------------------------------------------------------------------
# Corner-ordering check
# ---------------------------------------------------------------------------
def run_corner_check(args) -> int:
    """Live view with per-tag axes and labelled corner indices.

    Gate for the whole pipeline. What you must see:
      * +X (red) points right along the tag as printed
      * +Y (green) points up as printed
      * +Z (blue) points OUT of the surface, toward the camera

    A triad rotated by 90 or 180 degrees means the corner order is cyclically
    shifted relative to TAG_CORNER_OFFSETS_UNIT. Mirrored axes, or +Z going into
    the surface, means the order is reversed (handedness flip). Either way, fix
    TAG_CORNER_OFFSETS_UNIT before calibrating -- the error is invisible in a
    single-tag solve but wrecks the multi-tag one.
    """
    K, dist = constants.camera_matrix, constants.dist
    detector = apriltag_detect.make_detector()
    cam = camera_capture.open_camera(args.device, args.exposure)

    print(__doc__ and "")
    print(run_corner_check.__doc__)
    print(f"  assumed local corner order (size {args.check_size} m):")
    for i, p in enumerate(tmap.tag_corner_offsets(args.check_size)):
        print(f"    corners[{i}] -> tag-frame {np.round(p, 4).tolist()}")
    print("\n  press q to quit\n")

    try:
        while True:
            ok, image = cam.read()
            if not ok:
                continue
            dets = apriltag_detect.detect(detector, image)
            for det in dets:
                draw_tag_outline(image, det)
                draw_corner_indices(image, det)
                pose = solve_single_tag(det.corners, args.check_size, K, dist)
                if pose is not None:
                    cv2.drawFrameAxes(
                        image, K, dist, pose[0], pose[1], args.check_size * 0.75, 3
                    )
                cv2.putText(
                    image,
                    f"#{det.tag_id}",
                    (int(det.center[0]) - 20, int(det.center[1]) + 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            cv2.imshow("corner check  (0=red 1=green 2=blue 3=yellow)", image)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()
    return 0


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
def collect_frames(cam, detector, tags, K, dist, args):
    """Capture, detect, per-frame solve, and gate. Returns accepted frames."""
    accepted: list[dict[int, np.ndarray]] = []
    poses: list[tuple[np.ndarray, np.ndarray]] = []
    rejects = Counter()
    captured = 0

    print(f"  settling ({args.settle} frames discarded for auto-exposure)...")
    for _ in range(args.settle):
        cam.read()

    print(f"  capturing up to {args.frames} usable frames (q to stop early)...")
    attempts = 0
    max_attempts = args.frames * 10
    while len(accepted) < args.frames and attempts < max_attempts:
        attempts += 1
        ok, image = cam.read()
        if not ok:
            rejects["read failed"] += 1
            continue
        captured += 1

        dets = apriltag_detect.detect(detector, image)
        obs = apriltag_detect.detections_to_corners(dets)
        corr = tmap.build_correspondences(obs, tags, warn=False)

        if corr.n_tags < args.min_tags:
            rejects[f"fewer than {args.min_tags} mapped tags"] += 1
        else:
            try:
                sol = ext.solve_correspondences(corr, K, dist)
            except (ext.ExtrinsicsError, cv2.error):
                rejects["solve failed"] += 1
            else:
                if sol.rms_px > args.max_rms:
                    rejects[f"rms > {args.max_rms} px"] += 1
                else:
                    accepted.append(
                        {tid: obs[tid] for tid in corr.tag_ids}
                    )
                    poses.append((sol.R_wc, sol.t_wc))

        if args.display:
            for det in dets:
                draw_tag_outline(image, det)
            cv2.putText(
                image,
                f"accepted {len(accepted)}/{args.frames}   tags {corr.n_tags}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("calibrating", image)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break

    return accepted, poses, captured, rejects


def report(sol, tags, held_out, corner_std, captured, rejects) -> None:
    print("\n" + "=" * 72)
    print("  CAMERA POSE IN base_assy")
    print("=" * 72)

    t = sol.t_wc
    print(f"\n  optical centre   x {t[0]:+.4f}  y {t[1]:+.4f}  z {t[2]:+.4f}   [m]")
    q = sol.quat_link
    r = sol.rpy_link
    print(f"  camera_link quat x {q[0]:+.5f}  y {q[1]:+.5f}  z {q[2]:+.5f}  w {q[3]:+.5f}")
    print(f"  camera_link rpy  {r[0]:+.4f} {r[1]:+.4f} {r[2]:+.4f}  [rad]")
    print(
        f"                   {np.degrees(r[0]):+.2f} {np.degrees(r[1]):+.2f} "
        f"{np.degrees(r[2]):+.2f}  [deg]"
    )
    axis = sol.R_wc[:, 2]
    print(f"  viewing direction {np.round(axis, 4).tolist()} (optical +Z in base_assy)")

    print(f"\n  frames captured  {captured}")
    print(f"  frames accepted  {sol.n_frames}")
    if rejects:
        for reason, n in rejects.most_common():
            print(f"    rejected {n:5d}  {reason}")

    print(f"\n  reprojection rms {sol.rms_px:.3f} px   over {sol.n_points} points")
    if sol.rms_px > 1.5:
        print(
            "    ** HIGH. Above ~3 px this is a wrong tag pose or a wrong corner\n"
            "       order, NOT noise. Run --corner-check and re-check the map."
        )
    for tid in sorted(sol.per_tag_rms):
        note = tags[tid].note if tid in tags else ""
        print(f"    tag {tid:3d}  {sol.per_tag_rms[tid]:6.3f} px   {note}")

    if corner_std:
        worst = max(float(np.max(v)) for v in corner_std.values())
        print(f"\n  worst per-corner pixel std across frames: {worst:.3f} px")
        if worst > 0.5:
            print("    ** >0.5 px means vibration, motion blur, or exposure flicker.")

    if sol.t_std is not None:
        print(
            f"  per-frame spread   translation std "
            f"[{sol.t_std[0]:.4f} {sol.t_std[1]:.4f} {sol.t_std[2]:.4f}] m"
        )
        print(f"                     rotation std    {sol.rot_std_rad:.5f} rad")

    print(f"\n  geometry         {'PLANAR' if sol.planar else 'non-planar'}")
    if sol.planar:
        ratio = sol.ambiguity_ratio
        print(
            f"    ** All tags are coplanar. Ambiguity ratio "
            f"{'n/a' if ratio is None else f'{ratio:.2f}'}."
        )
        if ratio is not None and ratio < 3.0:
            print(
                "    ** The two planar solutions fit nearly equally well. The wrong\n"
                "       one has a plausible position but the WRONG SIGN of tilt.\n"
                "       DO NOT TRUST THIS RESULT -- add an out-of-plane tag."
            )
        if sol.ambiguity_resolved_physically:
            print("    ** Tie broken by requiring the camera above the z=0 plane.")

    if held_out:
        suspect = ext.suspect_tag(sol.rms_px, held_out)
        print("\n  leave-one-tag-out:")
        print("    tag   held-out rms   rms of the remaining fit")
        for tid in sorted(held_out):
            r = held_out[tid]
            flag = "   <== SUSPECT: re-measure this tag" if tid == suspect else ""
            print(
                f"    {tid:3d}   {r.held_rms:9.3f} px   {r.remaining_rms:9.3f} px{flag}"
            )
        if suspect is not None:
            print(
                f"\n    Dropping tag {suspect} takes the fit from {sol.rms_px:.2f} px to "
                f"{held_out[suspect].remaining_rms:.2f} px.\n"
                "    Read the RIGHT column, not the left one: one bad tag inflates\n"
                "    every fit that still contains it, so all the held-out errors\n"
                "    rise together. The bad tag is the one whose REMOVAL makes the\n"
                "    rest snap clean. Re-measure its pose in the map."
            )
        elif sol.rms_px > 1.5:
            print(
                "\n    No single tag explains the residual, so the problem is global:\n"
                "    corner ordering, intrinsics, or units. Dropping tags will not\n"
                "    help. Run --corner-check first."
            )

    print("\n" + "-" * 72)
    print("  publish it with:\n")
    print("    " + ext.static_transform_publisher_cmd(sol))
    print(
        "\n  plus the fixed optical rotation:\n"
        "    ros2 run tf2_ros static_transform_publisher "
        "--roll -1.5708 --pitch 0 --yaw -1.5708 \\\n"
        "      --frame-id camera_link --child-frame-id camera_optical_frame"
    )
    print("-" * 72)


def run_calibration(args) -> int:
    K, dist = constants.camera_matrix, constants.dist
    tags = tmap.load_tag_map(args.tags)
    print(f"  loaded {len(tags)} tags from {args.tags}")
    for tid in sorted(tags):
        t = tags[tid]
        print(f"    tag {tid:3d}  size {t.size:.4f} m  at {np.round(t.t_w, 4).tolist()}"
              f"   {t.note}")

    if len(tags) < 3:
        print(
            "\n  ** Only {} tag(s) in the map. Three is the practical minimum and\n"
            "     four or five is recommended; a single tag is never enough."
            .format(len(tags))
        )

    detector = apriltag_detect.make_detector()
    cam = camera_capture.open_camera(args.device, args.exposure)
    try:
        accepted, poses, captured, rejects = collect_frames(
            cam, detector, tags, K, dist, args
        )
        last_frame = None
        if args.verify:
            ok, last_frame = cam.read()
            if not ok:
                last_frame = None
    finally:
        cam.release()
        if args.display:
            cv2.destroyAllWindows()

    if not accepted:
        print("\n  no usable frames.")
        for reason, n in rejects.most_common():
            print(f"    {n:5d}  {reason}")
        return 1

    # Average the corner PIXELS, not the poses: the scene is static, so every
    # frame observes the same 3D points and averaging cuts detector noise by
    # sqrt(N) before a single final solve.
    averaged, corner_std, n_used = ext.average_corners(accepted)
    corr = tmap.build_correspondences(averaged, tags)
    sol = ext.solve_correspondences(corr, K, dist, n_frames=n_used)

    # Pose spread across frames is an uncertainty estimate, not the estimator.
    t_std = rot_std = None
    modal = ext.modal_tag_set(accepted)
    modal_poses = [p for f, p in zip(accepted, poses) if frozenset(f) == modal]
    if len(modal_poses) > 1:
        _, _, t_std, rot_std = ext.average_poses(modal_poses)

    held_out = ext.leave_one_tag_out(corr, K, dist) if args.verify else {}

    dropped: list[int] = []
    if args.drop_bad_tags and held_out:
        # Drop one tag at a time and re-test: removing the worst offender can
        # reveal a second one that its residual was masking.
        while True:
            suspect = ext.suspect_tag(sol.rms_px, held_out)
            if suspect is None:
                break
            if corr.n_tags - 1 < max(3, args.min_tags):
                print(
                    f"\n  ** tag {suspect} looks bad, but dropping it would leave "
                    f"only {corr.n_tags - 1} tags. Keeping it -- re-measure instead. **"
                )
                break
            print(f"\n  ** DROPPING TAG {suspect} -- re-measure its pose. **")
            dropped.append(suspect)
            corr = corr.without(suspect)
            sol = ext.solve_correspondences(corr, K, dist, n_frames=n_used)
            held_out = ext.leave_one_tag_out(corr, K, dist)

    sol.t_std, sol.rot_std_rad = t_std, rot_std
    report(sol, tags, held_out, corner_std, captured, rejects)

    meta = {
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "camera": {
            "name": getattr(constants, "camera_color", "camera"),
            "resolution": list(constants.camera_res),
            "device": args.device,
        },
        "intrinsics": {
            "fx": float(K[0, 0]),
            "fy": float(K[1, 1]),
            "cx": float(K[0, 2]),
            "cy": float(K[1, 2]),
            "dist": [float(v) for v in np.asarray(dist).ravel()],
            "note": "These extrinsics are valid ONLY for these intrinsics at "
                    "this resolution. Re-calibrate if either changes.",
        },
        "tag_map": str(args.tags),
    }
    doc = ext.solution_to_dict(sol, meta)
    doc["solve"]["tags_dropped"] = dropped
    if held_out:
        doc["solve"]["held_out_rms_px"] = {
            k: (None if not np.isfinite(r.held_rms) else round(r.held_rms, 4))
            for k, r in held_out.items()
        }
        doc["solve"]["remaining_fit_rms_px"] = {
            k: (None if not np.isfinite(r.remaining_rms) else round(r.remaining_rms, 4))
            for k, r in held_out.items()
        }

    with Path(args.out).open("w") as fh:
        yaml.safe_dump(doc, fh, sort_keys=False, default_flow_style=False)
    print(f"\n  wrote {args.out}")

    if args.verify and last_frame is not None:
        path = Path(args.out).with_suffix(".verify.png")
        draw_verification(last_frame, sol, K, dist)
        cv2.imwrite(str(path), last_frame)
        print(f"  wrote {path}")
        print(
            "    Check it: the drawn triad must sit on the PHYSICAL arm base\n"
            "    origin with X/Y/Z pointing where the arm's axes really point.\n"
            "    This is the most convincing check available -- a human can see\n"
            "    whether it is right."
        )
    return 0


def draw_verification(image, sol, K, dist) -> None:
    """Project the base_assy origin and its axes into the image."""
    cv2.drawFrameAxes(image, K, dist, sol.rvec, sol.tvec, 0.10, 4)
    origin, _ = cv2.projectPoints(
        np.zeros((1, 1, 3)), sol.rvec, sol.tvec, K, dist
    )
    px, py = origin.reshape(2)
    cv2.circle(image, (int(px), int(py)), 8, (255, 255, 255), 2)
    cv2.putText(
        image,
        "base_assy origin",
        (int(px) + 12, int(py)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="Solve the static camera's pose in base_assy from known tags.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--tags", type=Path, default=DEFAULT_TAGS, help="tag map YAML")
    p.add_argument("--out", type=Path, default=DEFAULT_OUT, help="output YAML")
    p.add_argument("--device", type=int, default=0, help="VideoCapture index")
    p.add_argument("--exposure", type=int, default=60, help="manual exposure")
    p.add_argument("--frames", type=int, default=200, help="usable frames to collect")
    p.add_argument("--settle", type=int, default=10, help="frames discarded at start")
    p.add_argument("--max-rms", type=float, default=1.5, help="per-frame reject, px")
    p.add_argument("--min-tags", type=int, default=3, help="min mapped tags per frame")
    p.add_argument("--display", action="store_true", help="show the camera feed")
    p.add_argument("--verify", action="store_true", help="leave-one-out + overlay image")
    p.add_argument(
        "--drop-bad-tags",
        action="store_true",
        help="re-solve without tags that fail leave-one-out (needs --verify)",
    )
    p.add_argument(
        "--corner-check",
        action="store_true",
        help="live corner-ordering check; run this FIRST, before calibrating",
    )
    p.add_argument(
        "--check-size", type=float, default=0.060, help="tag size for --corner-check"
    )
    args = p.parse_args(argv)

    if args.corner_check:
        return run_corner_check(args)
    return run_calibration(args)


if __name__ == "__main__":
    sys.exit(main())
