"""Solve a static camera's pose in ``base_assy`` from known-pose AprilTags.

Pure solver. Imports numpy/cv2/scipy/PyYAML only -- no VideoCapture, no
argparse, no ROS -- so ``camera_tf_publisher.py`` can import it unchanged.

The maths
---------
``cv2.solvePnP(obj_pts_in_base_assy, img_pts, K, dist)`` returns the transform
*world -> camera*::

    X_cam = R_cw @ X_world + tvec        R_cw = cv2.Rodrigues(rvec)[0]

The camera's pose is the inverse::

    R_wc = R_cw.T
    t_wc = -R_cw.T @ tvec                # optical centre in base_assy, metres

Because the object points are already expressed in ``base_assy``, solvePnP
absorbs the frame change and **no axis remap is applied to the position**.
``t_wc`` is a plain base_assy 3-vector.

The columns of ``R_wc`` are the camera's *optical* axes in base_assy: X image-
right, Y image-**down**, Z along the view direction. That is a correct and
publishable ``base_assy -> camera_optical_frame`` rotation. Converting to a
ROS-body ``camera_link`` (X forward, Y left, Z up) is a fixed post-rotation --
see ``optical_to_link``. Intrinsics and distortion are defined in the optical
frame, so the optical pose is the source of truth and camera_link is derived.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, Sequence

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from tag_map import Correspondences

SCHEMA = "camera_extrinsics/1"

# Rotation whose columns are the camera optical axes expressed in camera_link
# coordinates. Equals Rotation.from_euler("xyz", [-pi/2, 0, -pi/2]).as_matrix(),
# the fixed camera_link -> camera_optical_frame rotation every ROS camera driver
# publishes as rpy="-1.5708 0 -1.5708".
R_LINK_OPT = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)

OPTICAL_FROM_LINK_RPY = (-np.pi / 2, 0.0, -np.pi / 2)


class ExtrinsicsError(RuntimeError):
    """Raised when the geometry is too degenerate to give an honest answer."""


@dataclass
class ExtrinsicSolution:
    """Camera pose in base_assy, plus everything needed to judge it."""

    R_wc: np.ndarray  # (3,3) base_assy <- camera OPTICAL frame
    t_wc: np.ndarray  # (3,) optical centre in base_assy, metres
    rvec: np.ndarray  # (3,1) world -> camera, as returned by solvePnP
    tvec: np.ndarray  # (3,1)
    rms_px: float = 0.0
    per_tag_rms: dict[int, float] = field(default_factory=dict)
    tag_ids: list[int] = field(default_factory=list)
    n_points: int = 0
    n_frames: int = 1
    planar: bool = False
    ambiguity_ratio: float | None = None
    ambiguity_resolved_physically: bool = False
    t_std: np.ndarray | None = None
    rot_std_rad: float | None = None

    @property
    def quat_optical(self) -> np.ndarray:
        """(x, y, z, w) for base_assy -> camera_optical_frame."""
        return Rotation.from_matrix(self.R_wc).as_quat()

    @property
    def R_w_link(self) -> np.ndarray:
        return optical_to_link(self.R_wc)

    @property
    def quat_link(self) -> np.ndarray:
        """(x, y, z, w) for base_assy -> camera_link."""
        return Rotation.from_matrix(self.R_w_link).as_quat()

    @property
    def rpy_link(self) -> np.ndarray:
        return Rotation.from_matrix(self.R_w_link).as_euler("xyz")


def optical_to_link(R_w_opt: np.ndarray) -> np.ndarray:
    """base_assy <- camera_link, given base_assy <- camera_optical_frame."""
    return R_w_opt @ R_LINK_OPT.T


def invert_pose(rvec: np.ndarray, tvec: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(rvec, tvec) world->camera  ->  (R_wc, t_wc) camera pose in world."""
    R_cw = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))[0]
    t = np.asarray(tvec, dtype=np.float64).reshape(3)
    R_wc = R_cw.T
    t_wc = -R_cw.T @ t
    return R_wc, t_wc


def reprojection_error(
    obj_pts: np.ndarray,
    img_pts: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
) -> tuple[float, np.ndarray]:
    """(rms_pixels, per_point_pixel_error)."""
    proj, _ = cv2.projectPoints(
        np.asarray(obj_pts, dtype=np.float64).reshape(-1, 1, 3), rvec, tvec, K, dist
    )
    per_point = np.linalg.norm(
        proj.reshape(-1, 2) - np.asarray(img_pts, dtype=np.float64).reshape(-1, 2),
        axis=1,
    )
    return float(np.sqrt(np.mean(per_point**2))), per_point


def is_planar(obj_pts: np.ndarray, tol: float = 1e-2) -> bool:
    """True when the world points are (near-)coplanar.

    Ratio of smallest to largest singular value of the mean-centred points. A
    coplanar configuration is solvable but suffers the two-fold IPPE ambiguity
    and couples depth error to focal-length error.
    """
    pts = np.asarray(obj_pts, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] < 4:
        return True
    centred = pts - pts.mean(axis=0)
    s = np.linalg.svd(centred, compute_uv=False)
    if s[0] <= 0:
        return True
    return bool(s[-1] / s[0] < tol)


def _candidate_solutions(
    obj_pts, img_pts, K, dist, planar
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Initial pose candidates. Never uses cold SOLVEPNP_ITERATIVE.

    ITERATIVE with no extrinsic guess bootstraps via DLT, which is degenerate
    for coplanar points -- the likely cause of the garbage poses in the FRC
    reference implementation.
    """
    obj = obj_pts.reshape(-1, 1, 3)
    img = img_pts.reshape(-1, 1, 2)

    if planar:
        # IPPE returns both solutions of the planar ambiguity explicitly.
        ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE
        )
        if ok and len(rvecs):
            return [(r, t) for r, t in zip(rvecs, tvecs)]

    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        obj, img, K, dist, flags=cv2.SOLVEPNP_SQPNP
    )
    if not ok or not len(rvecs):
        raise ExtrinsicsError("solvePnP found no solution")
    return [(r, t) for r, t in zip(rvecs, tvecs)]


def solve_camera_pose(
    obj_pts: np.ndarray,
    img_pts: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    *,
    initial: tuple[np.ndarray, np.ndarray] | None = None,
    planar_check: bool = True,
    refine: bool = True,
    prefer_camera_above: bool = True,
    allow_single_tag: bool = False,
    tag_ids: Sequence[int] | None = None,
    tag_slices: Sequence[slice] | None = None,
    n_frames: int = 1,
) -> ExtrinsicSolution:
    """Solve for the camera pose from world/image point correspondences.

    Strategy: SQPNP (or IPPE when coplanar) for a globally-optimal starting
    point, then VVS refinement for a true least-squares minimum of reprojection
    error. ``prefer_camera_above`` breaks a planar tie by discarding candidates
    that place the camera below the base_assy z=0 plane.
    """
    obj_pts = np.asarray(obj_pts, dtype=np.float64).reshape(-1, 3)
    img_pts = np.asarray(img_pts, dtype=np.float64).reshape(-1, 2)
    if obj_pts.shape[0] < 4:
        raise ExtrinsicsError(f"need at least 4 points, got {obj_pts.shape[0]}")
    if tag_ids is not None and len(tag_ids) < 2 and not allow_single_tag:
        raise ExtrinsicsError(
            "a single tag is not enough for a trustworthy extrinsic: its 4 "
            "coplanar corners suffer the two-fold planar ambiguity, worst "
            "exactly at the near-frontal views you are most likely to set up. "
            "Use 3+ tags (pass allow_single_tag=True only for debug overlays)."
        )

    planar = is_planar(obj_pts) if planar_check else False

    if initial is not None:
        rvec0 = np.asarray(initial[0], dtype=np.float64).reshape(3, 1).copy()
        tvec0 = np.asarray(initial[1], dtype=np.float64).reshape(3, 1).copy()
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts.reshape(-1, 1, 3),
            img_pts.reshape(-1, 1, 2),
            K,
            dist,
            rvec0,
            tvec0,
            useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            raise ExtrinsicsError("guided solvePnP failed to converge")
        candidates = [(rvec, tvec)]
        ambiguity_ratio = None
    else:
        candidates = _candidate_solutions(obj_pts, img_pts, K, dist, planar)
        scored = sorted(
            (reprojection_error(obj_pts, img_pts, r, t, K, dist)[0], i)
            for i, (r, t) in enumerate(candidates)
        )
        ambiguity_ratio = (
            float(scored[1][0] / scored[0][0])
            if len(scored) > 1 and scored[0][0] > 1e-9
            else None
        )
        candidates = [candidates[i] for _, i in scored]

    resolved_physically = False
    if prefer_camera_above and len(candidates) > 1:
        above = [c for c in candidates if invert_pose(*c)[1][2] > 0.0]
        if above and above[0] is not candidates[0]:
            resolved_physically = True
        if above:
            candidates = above

    rvec, tvec = candidates[0]
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1).copy()
    tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1).copy()

    if refine:
        rvec, tvec = cv2.solvePnPRefineVVS(
            obj_pts.reshape(-1, 1, 3),
            img_pts.reshape(-1, 1, 2),
            K,
            dist,
            rvec,
            tvec,
        )

    rms, per_point = reprojection_error(obj_pts, img_pts, rvec, tvec, K, dist)
    R_wc, t_wc = invert_pose(rvec, tvec)

    per_tag: dict[int, float] = {}
    if tag_ids is not None and tag_slices is not None:
        for tag_id, sl in zip(tag_ids, tag_slices):
            e = per_point[sl]
            per_tag[int(tag_id)] = float(np.sqrt(np.mean(e**2)))

    return ExtrinsicSolution(
        R_wc=R_wc,
        t_wc=t_wc,
        rvec=rvec,
        tvec=tvec,
        rms_px=rms,
        per_tag_rms=per_tag,
        tag_ids=[int(t) for t in (tag_ids or [])],
        n_points=int(obj_pts.shape[0]),
        n_frames=n_frames,
        planar=planar,
        ambiguity_ratio=ambiguity_ratio,
        ambiguity_resolved_physically=resolved_physically,
    )


def solve_correspondences(
    corr: Correspondences, K: np.ndarray, dist: np.ndarray, **kwargs
) -> ExtrinsicSolution:
    """solve_camera_pose() driven directly by a Correspondences bundle."""
    return solve_camera_pose(
        corr.obj_pts,
        corr.img_pts,
        K,
        dist,
        tag_ids=corr.tag_ids,
        tag_slices=corr.slices,
        **kwargs,
    )


@dataclass
class LeaveOneOut:
    """Result of excluding one tag from the solve."""

    tag_id: int
    held_rms: float  # reprojection error of the excluded tag
    remaining_rms: float  # rms of the fit that excluded it


def leave_one_tag_out(
    corr: Correspondences, K: np.ndarray, dist: np.ndarray
) -> dict[int, LeaveOneOut]:
    """{tag_id: LeaveOneOut} -- the best diagnostic available without hardware.

    Errors here arrive as whole mis-measured tags (four correlated points at
    once), which is exactly the structure RANSAC's minimal sample sets cannot
    see and exactly what this test localises.

    Read ``remaining_rms``, not ``held_rms``, to find the culprit. With only a
    few tags, one bad tag contaminates every solve that still contains it, so
    *all* the held-out errors inflate together and the largest is not reliably
    the guilty one. But the fit that excludes the bad tag is the one that snaps
    clean -- so the bad tag is the one whose removal *minimises*
    ``remaining_rms``. ``held_rms`` then quantifies how far off that tag is.
    """
    out: dict[int, LeaveOneOut] = {}
    if corr.n_tags < 3:
        return out
    for tag_id in corr.tag_ids:
        subset = corr.without(tag_id)
        try:
            sol = solve_correspondences(subset, K, dist)
        except (ExtrinsicsError, cv2.error):
            out[int(tag_id)] = LeaveOneOut(int(tag_id), float("inf"), float("inf"))
            continue
        sl = corr.slices[corr.tag_ids.index(tag_id)]
        held, _ = reprojection_error(
            corr.obj_pts[sl], corr.img_pts[sl], sol.rvec, sol.tvec, K, dist
        )
        out[int(tag_id)] = LeaveOneOut(int(tag_id), held, sol.rms_px)
    return out


def suspect_tag(
    full_rms: float, loo: Mapping[int, LeaveOneOut], factor: float = 3.0
) -> int | None:
    """The tag whose removal most improves the fit, if the gain is decisive.

    Returns None when no single tag explains the residual -- in which case the
    problem is global (corner ordering, intrinsics, units) rather than one bad
    measurement, and dropping tags will not help.
    """
    if not loo:
        return None
    best = min(loo.values(), key=lambda r: r.remaining_rms)
    if best.remaining_rms <= 0:
        return best.tag_id
    if full_rms / best.remaining_rms >= factor:
        return best.tag_id
    return None


def modal_tag_set(frames: Sequence[Mapping[int, np.ndarray]]) -> frozenset[int]:
    """The most frequently observed set of tag ids across frames."""
    counts: dict[frozenset[int], int] = {}
    for obs in frames:
        key = frozenset(obs)
        counts[key] = counts.get(key, 0) + 1
    return max(counts, key=lambda k: (counts[k], len(k)))


def average_corners(
    frames: Sequence[Mapping[int, np.ndarray]]
) -> tuple[dict[int, np.ndarray], dict[int, np.ndarray], int]:
    """Average corner pixels across frames of a static scene.

    Camera and tags are both fixed, so every frame observes the same 3D points.
    Averaging the *pixels* cuts detector noise by sqrt(N) before a single final
    solve -- strictly better than averaging poses, which is only used here to
    estimate spread.

    Returns (averaged observations, per-corner pixel std, frames used). Only
    frames matching the modal tag set contribute.
    """
    if not frames:
        raise ExtrinsicsError("no frames to average")
    modal = modal_tag_set(frames)
    kept = [f for f in frames if frozenset(f) == modal]

    averaged: dict[int, np.ndarray] = {}
    stds: dict[int, np.ndarray] = {}
    for tag_id in sorted(modal):
        stack = np.stack([np.asarray(f[tag_id], dtype=np.float64) for f in kept])
        averaged[tag_id] = stack.mean(axis=0)
        stds[tag_id] = stack.std(axis=0)
    return averaged, stds, len(kept)


def average_poses(
    poses: Sequence[tuple[np.ndarray, np.ndarray]]
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """(R_mean, t_mean, t_std, rot_std_rad) from per-frame camera poses.

    Rotations are averaged on SO(3) via scipy's chordal L2 mean; the spread is
    the RMS geodesic deviation from that mean. Never average Euler angles or
    rotation matrices elementwise.
    """
    if not poses:
        raise ExtrinsicsError("no poses to average")
    Rs = Rotation.from_matrix(np.stack([np.asarray(R) for R, _ in poses]))
    ts = np.stack([np.asarray(t, dtype=np.float64).reshape(3) for _, t in poses])

    R_mean = Rs.mean()
    deviations = (Rs * R_mean.inv()).magnitude()
    return (
        R_mean.as_matrix(),
        ts.mean(axis=0),
        ts.std(axis=0),
        float(np.sqrt(np.mean(deviations**2))),
    )


def _xyz(v: np.ndarray) -> dict[str, float]:
    return {"x": float(v[0]), "y": float(v[1]), "z": float(v[2])}


def _quat(q: np.ndarray) -> dict[str, float]:
    return {"x": float(q[0]), "y": float(q[1]), "z": float(q[2]), "w": float(q[3])}


def solution_to_dict(sol: ExtrinsicSolution, meta: Mapping | None = None) -> dict:
    """YAML-ready representation of a solved extrinsic."""
    meta = dict(meta or {})
    doc = {
        "schema": SCHEMA,
        "parent_frame": "base_assy",
        "child_frame": "camera_link",
        "optical_frame": "camera_optical_frame",
    }
    doc.update(meta)
    doc["_frames_note"] = (
        "camera_optical_frame is the source of truth (direct solvePnP output; "
        "x right, y down, z forward -- the frame the intrinsics are defined in). "
        "camera_link is derived from it by the fixed ROS rotation "
        "rpy=(-pi/2, 0, -pi/2). Both frames share ONE origin: the translations "
        "are identical by construction, not duplicated by mistake."
    )
    doc["camera_optical_frame"] = {
        "translation": _xyz(sol.t_wc),
        "rotation": _quat(sol.quat_optical),
    }
    doc["camera_link"] = {
        "translation": _xyz(sol.t_wc),
        "rotation": _quat(sol.quat_link),
        "rpy": [float(v) for v in sol.rpy_link],
    }
    doc["solve"] = {
        "n_tags": len(sol.tag_ids),
        "tag_ids": list(sol.tag_ids),
        "n_points": sol.n_points,
        "n_frames_used": sol.n_frames,
        "rms_reprojection_px": round(sol.rms_px, 4),
        "per_tag_rms_px": {int(k): round(v, 4) for k, v in sol.per_tag_rms.items()},
        "planar": bool(sol.planar),
        "ambiguity_ratio": (
            None if sol.ambiguity_ratio is None else round(sol.ambiguity_ratio, 4)
        ),
        "ambiguity_resolved_physically": bool(sol.ambiguity_resolved_physically),
        "translation_std_m": (
            None if sol.t_std is None else [round(float(v), 6) for v in sol.t_std]
        ),
        "rotation_std_rad": (
            None if sol.rot_std_rad is None else round(float(sol.rot_std_rad), 6)
        ),
    }
    return doc


def save_solution(path: str | Path, sol: ExtrinsicSolution, meta=None) -> None:
    doc = solution_to_dict(sol, meta)
    with Path(path).open("w") as fh:
        yaml.safe_dump(doc, fh, sort_keys=False, default_flow_style=False)


def load_solution(path: str | Path) -> dict:
    """Read a camera_extrinsics.yaml. The ROS node's single entry point."""
    path = Path(path)
    if not path.is_file():
        raise ExtrinsicsError(f"extrinsics file not found: {path}")
    with path.open() as fh:
        doc = yaml.safe_load(fh)
    if doc.get("schema") != SCHEMA:
        raise ExtrinsicsError(
            f"{path}: schema is {doc.get('schema')!r}, expected {SCHEMA!r}"
        )
    return doc


def static_transform_publisher_cmd(sol: ExtrinsicSolution) -> str:
    """Paste-ready ros2 command for base_assy -> camera_link."""
    t, q = sol.t_wc, sol.quat_link
    return (
        "ros2 run tf2_ros static_transform_publisher "
        f"--x {t[0]:.6f} --y {t[1]:.6f} --z {t[2]:.6f} "
        f"--qx {q[0]:.6f} --qy {q[1]:.6f} --qz {q[2]:.6f} --qw {q[3]:.6f} "
        "--frame-id base_assy --child-frame-id camera_link"
    )
