"""Locate an object in ``base_assy`` from a single capture of the scene.

Pure solver + composition. Imports numpy/cv2/scipy only -- no camera, no
argparse, no ROS -- so a ROS2 node and a CLI can share it unchanged.

The idea
--------
One image contains two kinds of tag: BASE tags at known poses in ``base_assy``,
and OBJECT tags at known poses in the object's own frame. Two independent PnP
solves come out of that single image::

    base tags   -> T_base<-cam    (where the camera is, right now)
    object tags -> T_cam<-object  (where the object is, relative to the camera)

Compose them and the camera drops out::

    T_base<-object = T_base<-cam @ T_cam<-object

Why re-solve the camera every time
----------------------------------
The camera extrinsic is not cached. Re-deriving it from the same frame that
sees the object buys two things:

1. Camera drift stops being a failure mode. A bumped tripod changes the base
   tag geometry in the very image being used, so the answer follows the camera
   instead of silently going wrong. Nothing to re-calibrate, ever.

2. Common-mode errors partially cancel. The composed transform depends on the
   *relative* geometry of base tags and object within one image. A small scale
   error in the intrinsics, say, pushes the camera and the object the same
   direction, and part of that cancels in the composition -- more so the closer
   the object sits to the base tags. Relative geometry is better determined
   than either absolute pose.

The cost is that a single capture is noisier than a 200-frame calibration, which
is why callers should hand this a burst (see camera_capture.capture_burst) and
let ``observe`` average the corners.

Note that ``solve_camera_pose`` does not care which frame its object points are
in -- it returns the transform between that frame and the camera. Feed it base
tags and its ``R_wc/t_wc`` is the camera in base_assy; feed it object tags and
its ``rvec/tvec`` is T_cam<-object directly. Same function, both jobs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import apriltag_detect
import camera_extrinsics as ext
import tag_map as tmap
from camera_extrinsics import ExtrinsicSolution
from tag_map import TagSpec


class SceneError(RuntimeError):
    """Raised when a scene cannot be solved honestly."""


def compose(R_a, t_a, R_b, t_b):
    """(R_a,t_a) o (R_b,t_b) -- i.e. A<-B composed with B<-C gives A<-C."""
    return R_a @ R_b, R_a @ np.asarray(t_b).reshape(3) + np.asarray(t_a).reshape(3)


def invert(R, t):
    """Inverse of a rigid transform."""
    return R.T, -R.T @ np.asarray(t).reshape(3)


@dataclass
class SceneSolution:
    """Where the camera and the object are, from one capture."""

    R_bc: np.ndarray  # base_assy <- camera optical
    t_bc: np.ndarray
    R_bo: np.ndarray  # base_assy <- object
    t_bo: np.ndarray
    R_co: np.ndarray  # camera optical <- object
    t_co: np.ndarray
    camera: ExtrinsicSolution
    obj: ExtrinsicSolution
    n_frames: int = 1

    @property
    def distance_cam_to_object(self) -> float:
        return float(np.linalg.norm(self.t_co))

    def quat_bo(self) -> np.ndarray:
        """(x, y, z, w) of the object's orientation in base_assy."""
        return Rotation.from_matrix(self.R_bo).as_quat()

    def grasp_in_base(self, grasp) -> tuple[np.ndarray, np.ndarray]:
        """Apply an object-frame grasp offset -> (t, R) in base_assy."""
        if grasp is None:
            return self.t_bo, self.R_bo
        t_og, R_og = grasp
        R, t = compose(self.R_bo, self.t_bo, R_og, t_og)
        return t, R


def check_disjoint(base_map: Mapping[int, TagSpec],
                   object_map: Mapping[int, TagSpec]) -> None:
    """Base and object tag ids must not overlap.

    A shared id is silently catastrophic: the same detection would be paired
    against two different world positions, and the solver would return a
    confident, wrong answer with a plausible reprojection error. Use disjoint
    ranges (e.g. base 0-9, object 10-19).
    """
    clash = sorted(set(base_map) & set(object_map))
    if clash:
        raise SceneError(
            f"tag ids {clash} appear in BOTH the base map and the object map. "
            "Ids must be disjoint -- a shared id pairs one detection against "
            "two different world poses and produces a confidently wrong result."
        )


def observe(
    images: Sequence[np.ndarray],
    detector,
    *,
    min_decision_margin: float = apriltag_detect.DEFAULT_MIN_DECISION_MARGIN,
) -> tuple[dict[int, np.ndarray], dict[int, np.ndarray], int]:
    """Detect over a burst of a static scene and average the corner pixels.

    Returns (averaged observations, per-corner pixel std, frames contributing).
    Only frames seeing the modal tag set contribute, so a frame that briefly
    lost or gained a tag cannot skew the average.
    """
    per_frame = []
    for image in images:
        dets = apriltag_detect.detect(
            detector, image, min_decision_margin=min_decision_margin
        )
        obs = apriltag_detect.detections_to_corners(dets)
        if obs:
            per_frame.append(obs)
    if not per_frame:
        raise SceneError("no tags detected in any frame of the burst")
    return ext.average_corners(per_frame)


def solve_scene(
    observations: Mapping[int, np.ndarray],
    base_map: Mapping[int, TagSpec],
    object_map: Mapping[int, TagSpec],
    K: np.ndarray,
    dist: np.ndarray,
    *,
    min_base_tags: int = 3,
    min_object_tags: int = 2,
    max_rms_px: float = 4.0,
    n_frames: int = 1,
) -> SceneSolution:
    """Solve camera pose and object pose from one set of observations.

    ``min_object_tags`` defaults to 2 for a reason. Tags on a single cube face
    are coplanar, so one face gives the two-fold planar ambiguity -- a pose that
    looks plausible but can have the wrong sign of tilt. Two visible faces make
    the point set genuinely three-dimensional and the ambiguity disappears.
    Aim the camera at a corner of the cube, not square at one face.

    ``max_rms_px`` is 4.0, not the 1.5 it started at. The gate is in pixels but
    the requirement is in millimetres, and the conversion is range/focal: at
    0.6-0.85 m with f = 1007 px, one pixel is 0.6-0.83 mm, so 1.5 px demanded
    0.9-1.25 mm of agreement. That is 3-4x stricter than the ~4 mm the rig
    actually needs, and it rejected maps that were measurably fine.

    4.0 px is ~3.3 mm at the far tag. THAT IS CLOSE TO THE EDGE: the rig's ~4 mm
    budget is 4.8 px at that range, so this gate is now spending most of the
    error budget on tag-map disagreement alone, leaving little for everything
    downstream. Do not raise it again without re-doing the arithmetic -- past
    ~4.8 px the gate stops enforcing the requirement at all and a genuinely
    displaced tag sails through.

    The honest fix in that direction is not a bigger number: it is more pixels
    on the tag (move the camera closer, print the base tags larger) or a
    cleaner image (the captures behind this were 17% clipped), either of which
    lets the gate come back DOWN. Re-derive it if the geometry changes.

    Note what this number can and cannot do. It measures whether the tags agree
    with EACH OTHER. A common offset of the whole base map moves the solved
    camera with it and never shows up here at any threshold (see the z-offset
    warning in the module docs); only physical measurement or --verify finds
    that.
    """
    check_disjoint(base_map, object_map)

    base_obs = {k: v for k, v in observations.items() if k in base_map}
    obj_obs = {k: v for k, v in observations.items() if k in object_map}

    if len(base_obs) < min_base_tags:
        raise SceneError(
            f"saw {len(base_obs)} base tag(s) {sorted(base_obs)}, need "
            f"{min_base_tags}. Without them the camera pose is unknown for "
            "this image and the object cannot be placed in base_assy."
        )
    if len(obj_obs) < min_object_tags:
        raise SceneError(
            f"saw {len(obj_obs)} object tag(s) {sorted(obj_obs)}, need "
            f"{min_object_tags}. One face of tags is coplanar and suffers the "
            "two-fold planar ambiguity; move the camera so it sees two faces."
        )

    corr_b = tmap.build_correspondences(base_obs, base_map, warn=False)
    cam_sol = ext.solve_correspondences(corr_b, K, dist, n_frames=n_frames)
    if cam_sol.rms_px > max_rms_px:
        raise SceneError(
            f"base-tag fit is {cam_sol.rms_px:.2f} px (limit {max_rms_px}). "
            "The camera pose for this image is not trustworthy: check the base "
            "tag map, or whether a tag has been moved."
        )

    corr_o = tmap.build_correspondences(obj_obs, object_map, warn=False)
    obj_sol = ext.solve_correspondences(corr_o, K, dist, n_frames=n_frames)
    if obj_sol.rms_px > max_rms_px:
        raise SceneError(
            f"object-tag fit is {obj_sol.rms_px:.2f} px (limit {max_rms_px}). "
            "Check the object map's tag size and face layout."
        )

    # solve_camera_pose returns T_cam<-frame as rvec/tvec, so for the object map
    # that IS the object's pose in the camera frame -- no inversion needed.
    R_co = cv2.Rodrigues(obj_sol.rvec)[0]
    t_co = obj_sol.tvec.reshape(3)

    R_bc, t_bc = cam_sol.R_wc, cam_sol.t_wc
    R_bo, t_bo = compose(R_bc, t_bc, R_co, t_co)

    return SceneSolution(
        R_bc=R_bc, t_bc=t_bc,
        R_bo=R_bo, t_bo=t_bo,
        R_co=R_co, t_co=t_co,
        camera=cam_sol, obj=obj_sol, n_frames=n_frames,
    )


def solve_scene_from_images(
    images: Sequence[np.ndarray],
    base_map: Mapping[int, TagSpec],
    object_map: Mapping[int, TagSpec],
    K: np.ndarray,
    dist: np.ndarray,
    detector=None,
    **kwargs,
) -> tuple[SceneSolution, dict[int, np.ndarray]]:
    """Burst -> averaged observations -> scene solution. Returns (sol, stds)."""
    detector = detector or apriltag_detect.make_detector()
    obs, stds, n = observe(images, detector)
    return solve_scene(obs, base_map, object_map, K, dist, n_frames=n, **kwargs), stds


def pose_delta(
    R_a: np.ndarray, t_a: np.ndarray, R_b: np.ndarray, t_b: np.ndarray
) -> tuple[float, float]:
    """(translation metres, rotation radians) between two poses.

    Handy for comparing this image's camera pose against a previous one: a jump
    means the camera moved between captures.
    """
    d_t = float(np.linalg.norm(np.asarray(t_a).reshape(3) - np.asarray(t_b).reshape(3)))
    d_R = float((Rotation.from_matrix(R_a) * Rotation.from_matrix(R_b).inv()).magnitude())
    return d_t, d_R
