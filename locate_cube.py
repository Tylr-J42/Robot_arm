#!/usr/bin/env python
"""Find the cube in base_assy from one capture, ready to hand to MoveIt.

    ./venv/bin/python locate_cube.py --display --verify

Each run re-solves the camera pose from the base tags in the same image that
sees the cube, so there is no stored extrinsic and nothing to go stale. The only
calibration this depends on is the camera's intrinsics + distortion in
constants.py.

Run from a saved still instead of the camera with --image scene.png.
"""

from __future__ import annotations

import argparse
import datetime
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

import apriltag_detect
import camera_capture
import constants
import scene_solve
import tag_map as tmap

HERE = Path(__file__).resolve().parent


def draw_overlay(image, sol, obs, base_map, object_map, K, dist):
    """Outline every tag, then draw the base_assy and cube frames."""
    for tag_id, corners in obs.items():
        colour = (0, 200, 255) if tag_id in base_map else (255, 120, 0)
        pts = corners.astype(int)
        for i in range(4):
            cv2.line(image, tuple(pts[i]), tuple(pts[(i + 1) % 4]), colour, 2)
        cv2.putText(image, f"#{tag_id}", tuple(pts[0] + np.array([4, -6])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2, cv2.LINE_AA)

    # base_assy frame: rvec/tvec of the camera solve are already T_cam<-base
    cv2.drawFrameAxes(image, K, dist, sol.camera.rvec, sol.camera.tvec, 0.10, 4)
    org, _ = cv2.projectPoints(np.zeros((1, 1, 3)), sol.camera.rvec,
                               sol.camera.tvec, K, dist)
    px, py = org.reshape(2)
    cv2.putText(image, "base_assy", (int(px) + 12, int(py)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

    # cube frame
    cv2.drawFrameAxes(image, K, dist, sol.obj.rvec, sol.obj.tvec, 0.04, 3)
    org, _ = cv2.projectPoints(np.zeros((1, 1, 3)), sol.obj.rvec, sol.obj.tvec,
                               K, dist)
    px, py = org.reshape(2)
    cv2.putText(image, "cube", (int(px) + 10, int(py)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return image


def report(sol, stds, grasp, base_map, object_map) -> None:
    print("\n" + "=" * 70)
    print("  SCENE SOLVED  (camera pose re-derived from this capture)")
    print("=" * 70)

    print(f"\n  frames averaged   {sol.n_frames}")
    if stds:
        worst = max(float(np.max(v)) for v in stds.values())
        print(f"  worst corner std  {worst:.3f} px"
              + ("   ** >0.5 px: vibration or blur" if worst > 0.5 else ""))

    print(f"\n  camera in base_assy   "
          f"x {sol.t_bc[0]:+.4f}  y {sol.t_bc[1]:+.4f}  z {sol.t_bc[2]:+.4f}  [m]")
    print(f"    from {sol.camera.n_tags if hasattr(sol.camera,'n_tags') else len(sol.camera.tag_ids)}"
          f" base tags {sol.camera.tag_ids}, rms {sol.camera.rms_px:.3f} px")
    for tid in sorted(sol.camera.per_tag_rms):
        print(f"      tag {tid:3d}  {sol.camera.per_tag_rms[tid]:6.3f} px")

    print(f"\n  cube in base_assy     "
          f"x {sol.t_bo[0]:+.4f}  y {sol.t_bo[1]:+.4f}  z {sol.t_bo[2]:+.4f}  [m]")
    q = sol.quat_bo()
    print(f"    quat  x {q[0]:+.5f}  y {q[1]:+.5f}  z {q[2]:+.5f}  w {q[3]:+.5f}")
    print(f"    from {len(sol.obj.tag_ids)} cube tags {sol.obj.tag_ids}, "
          f"rms {sol.obj.rms_px:.3f} px")
    for tid in sorted(sol.obj.per_tag_rms):
        note = object_map[tid].note if tid in object_map else ""
        print(f"      tag {tid:3d}  {sol.obj.per_tag_rms[tid]:6.3f} px   {note}")
    print(f"    range from camera  {sol.distance_cam_to_object:.3f} m")

    if sol.obj.planar:
        print(
            "\n  ** The visible cube tags are COPLANAR -- only one face is in\n"
            "     view. The pose has a two-fold ambiguity and the tilt may have\n"
            "     the wrong sign. Move the camera to see a second face."
        )

    t_g, R_g = sol.grasp_in_base(grasp)
    print(f"\n  GRASP target in base_assy  "
          f"x {t_g[0]:+.4f}  y {t_g[1]:+.4f}  z {t_g[2]:+.4f}  [m]")
    if grasp is None:
        print("    (no grasp: block in the object map -- this is the cube centre)")
    print("\n" + "-" * 70)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="Locate the cube in base_assy from a single capture.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--base-tags", type=Path, default=HERE / "tags_base_assy.yaml")
    p.add_argument("--cube-tags", type=Path, default=HERE / "cube_tags.yaml")
    p.add_argument("--out", type=Path, default=None, help="write the pose to YAML")
    p.add_argument("--image", type=Path, default=None,
                   help="solve a saved still instead of opening the camera")
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--exposure", type=int, default=60)
    p.add_argument("--burst", type=int, default=20,
                   help="frames per capture, averaged to cut corner noise")
    p.add_argument("--min-base-tags", type=int, default=3)
    p.add_argument("--min-cube-tags", type=int, default=2)
    p.add_argument("--max-rms", type=float, default=1.5)
    p.add_argument("--display", action="store_true")
    p.add_argument("--verify", action="store_true",
                   help="write an overlay image next to --out")
    args = p.parse_args(argv)

    K, dist = constants.camera_matrix, constants.dist
    base_map = tmap.load_tag_map(args.base_tags)
    object_map = tmap.load_tag_map(args.cube_tags, expected_frame="cube")
    grasp = tmap.load_grasp(args.cube_tags)
    print(f"  base map  {len(base_map)} tags {sorted(base_map)} in base_assy")
    print(f"  cube map  {len(object_map)} tags {sorted(object_map)} in cube frame")
    scene_solve.check_disjoint(base_map, object_map)

    if args.image:
        image = cv2.imread(str(args.image))
        if image is None:
            raise SystemExit(f"could not read image {args.image}")
        images = [image]
    else:
        cam = camera_capture.open_camera(args.device, args.exposure)
        try:
            images = camera_capture.capture_burst(cam, args.burst)
        finally:
            cam.release()

    detector = apriltag_detect.make_detector()
    try:
        sol, stds = scene_solve.solve_scene_from_images(
            images, base_map, object_map, K, dist, detector,
            min_base_tags=args.min_base_tags,
            min_object_tags=args.min_cube_tags,
            max_rms_px=args.max_rms,
        )
    except scene_solve.SceneError as e:
        print(f"\n  FAILED: {e}")
        return 1

    obs, _, _ = scene_solve.observe(images, detector)
    report(sol, stds, grasp, base_map, object_map)

    t_g, R_g = sol.grasp_in_base(grasp)
    from scipy.spatial.transform import Rotation
    q_g = Rotation.from_matrix(R_g).as_quat()

    if args.out:
        doc = {
            "schema": "object_pose/1",
            "generated": datetime.datetime.now().isoformat(timespec="seconds"),
            "frame_id": "base_assy",
            "object_frame": "cube",
            "cube": {
                "translation": {k: float(v) for k, v in zip("xyz", sol.t_bo)},
                "rotation": {k: float(v) for k, v in zip("xyzw", sol.quat_bo())},
            },
            "grasp": {
                "translation": {k: float(v) for k, v in zip("xyz", t_g)},
                "rotation": {k: float(v) for k, v in zip("xyzw", q_g)},
            },
            "camera_this_capture": {
                "translation": {k: float(v) for k, v in zip("xyz", sol.t_bc)},
                "note": "re-solved from the base tags in this image; not cached",
            },
            "solve": {
                "base_tag_ids": sol.camera.tag_ids,
                "base_rms_px": round(sol.camera.rms_px, 4),
                "cube_tag_ids": sol.obj.tag_ids,
                "cube_rms_px": round(sol.obj.rms_px, 4),
                "cube_planar": bool(sol.obj.planar),
                "n_frames": sol.n_frames,
            },
        }
        args.out.write_text(yaml.safe_dump(doc, sort_keys=False))
        print(f"  wrote {args.out}")

    if args.display or args.verify:
        canvas = draw_overlay(images[-1].copy(), sol, obs, base_map,
                              object_map, K, dist)
        if args.verify:
            path = (args.out.with_suffix(".verify.png") if args.out
                    else HERE / "locate_cube.verify.png")
            cv2.imwrite(str(path), canvas)
            print(f"  wrote {path}")
        if args.display:
            cv2.imshow("scene", canvas)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
