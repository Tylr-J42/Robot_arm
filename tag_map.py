"""AprilTag world-pose map: loading, validation, and corner geometry.

Pure data + geometry. Imports numpy/scipy/PyYAML only -- no cv2, no camera, no
ROS -- so this module is safe to import from a ROS2 node, a test, or the CLI
calibration runner alike.

Frame conventions
-----------------
World frame is the arm's URDF root link, ``base_assy`` (X forward, Y left,
Z up; right-handed). All lengths are metres, all angles radians.

Tag frame: origin at the tag centre, +X to the right of the tag *as printed*,
+Y up as printed, +Z out of the tag surface toward a viewer. ``rpy`` in the YAML
is the extrinsic-XYZ rotation from base_assy to the tag frame, i.e. exactly the
convention used by URDF ``<origin rpy="..."/>``.

Corner ordering: see TAG_CORNER_OFFSETS_UNIT. This MUST match the order in which
dt_apriltags reports ``det.corners``, or the solve silently returns a pose that
is rotated about each tag's normal. Verify it empirically with
``calibrate_camera_extrinsics.py --corner-check`` before trusting any result.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Mapping

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

SCHEMA = "apriltag_map/1"
WORLD_FRAME = "base_assy"

# Canonical tag-local corner offsets for a tag of edge length 1.0, in the tag
# frame (X right, Y up, Z out of the surface). Index i here pairs with
# det.corners[i] from dt_apriltags.
#
#   3 ----- 2        +Y
#   |       |         ^
#   |   o   |         |
#   |       |         +--> +X
#   0 ----- 1
#
# Verified against the real detector on a synthetic scene of tag36h11 markers
# rendered at known poses: detected corners matched the projected world corners
# index-for-index to within 1.2 px, and a full solve recovered the camera to
# 1.7 mm / 0.13 deg. Deliberately shifting the image corners relative to these
# by one index drives reprojection rms from 0.26 px to 43-259 px, which is why
# rms is a reliable detector of an ordering mistake.
#
# Trap for anyone building synthetic tests: cv2.aruco.generateImageMarker with
# DICT_APRILTAG_36h11 renders the tag rotated 180 deg from the apriltag
# library's canonical orientation, so a naive rendered scene will appear to
# prove this constant wrong by exactly a 2-index roll. It isn't; the renderer
# is. A real printed tag under --corner-check is the only final authority.
#
TAG_CORNER_OFFSETS_UNIT = np.array(
    [
        [-0.5, -0.5, 0.0],
        [+0.5, -0.5, 0.0],
        [+0.5, +0.5, 0.0],
        [-0.5, +0.5, 0.0],
    ],
    dtype=np.float64,
)


class TagMapError(ValueError):
    """Raised when a tag map file is malformed or inconsistent."""


@dataclass(frozen=True, eq=False)
class TagSpec:
    """One tag's known pose in the world frame.

    ``corners_override``, when present, is a (4, 3) array of directly-measured
    world corner coordinates that takes precedence over pose + size. Use it only
    for a tag whose corners were measured individually; the derived form is
    preferred because it enforces planar-and-square.
    """

    id: int
    size: float
    t_w: np.ndarray  # (3,) tag centre in world frame, metres
    R_wt: np.ndarray  # (3,3) world <- tag rotation
    note: str = ""
    corners_override: np.ndarray | None = None


@dataclass
class Correspondences:
    """Paired 3D world points and 2D image points, grouped by tag."""

    obj_pts: np.ndarray  # (N, 3) float64, world frame, metres
    img_pts: np.ndarray  # (N, 2) float64, pixels
    tag_ids: list[int] = field(default_factory=list)  # one entry per tag, in row order
    slices: list[slice] = field(default_factory=list)  # rows belonging to each tag

    @property
    def n_tags(self) -> int:
        return len(self.tag_ids)

    @property
    def n_points(self) -> int:
        return int(self.obj_pts.shape[0])

    def without(self, tag_id: int) -> "Correspondences":
        """Return a copy with one tag's rows removed (for leave-one-out)."""
        keep = [i for i, tid in enumerate(self.tag_ids) if tid != tag_id]
        if not keep:
            raise TagMapError(f"cannot drop tag {tag_id}: nothing would remain")
        return _stack_correspondences(
            [(self.tag_ids[i], self.obj_pts[self.slices[i]], self.img_pts[self.slices[i]])
             for i in keep]
        )


def tag_corner_offsets(size: float) -> np.ndarray:
    """(4, 3) tag-local corner coordinates for a tag of the given edge length."""
    return TAG_CORNER_OFFSETS_UNIT * float(size)


def world_corners(tag: TagSpec) -> np.ndarray:
    """(4, 3) world-frame coordinates of one tag's corners, metres."""
    if tag.corners_override is not None:
        return tag.corners_override
    local = tag_corner_offsets(tag.size)
    return local @ tag.R_wt.T + tag.t_w


def load_tag_map(
    path: str | Path, *, expected_frame: str | None = WORLD_FRAME
) -> dict[int, TagSpec]:
    """Parse and validate a tag map YAML file. Returns {tag_id: TagSpec}.

    ``expected_frame`` is the frame the poses must be measured in. It defaults
    to the world frame for a base map; pass the object's own frame name (e.g.
    "cube") for an object map, or None to accept whatever the file declares.
    """
    path = Path(path)
    if not path.is_file():
        raise TagMapError(f"tag map not found: {path}")
    with path.open() as fh:
        doc = yaml.safe_load(fh)

    if not isinstance(doc, dict):
        raise TagMapError(f"{path}: top level must be a mapping")

    schema = doc.get("schema")
    if schema != SCHEMA:
        raise TagMapError(f"{path}: schema is {schema!r}, expected {SCHEMA!r}")

    frame_id = doc.get("frame_id")
    if not frame_id:
        raise TagMapError(f"{path}: no frame_id declared")
    if expected_frame is not None and frame_id != expected_frame:
        raise TagMapError(
            f"{path}: frame_id is {frame_id!r}, expected {expected_frame!r}. "
            "Tag poses must be measured in that frame."
        )

    units = doc.get("units") or {}
    if units.get("length") != "m" or units.get("angle") != "rad":
        raise TagMapError(
            f"{path}: units must be {{length: m, angle: rad}}, got {units!r}. "
            "This pipeline is metres/radians end to end -- convert your "
            "measurements in the file, not in the code."
        )

    default_size = doc.get("default_size")
    entries = doc.get("tags")
    if not entries:
        raise TagMapError(f"{path}: no tags defined")

    tags: dict[int, TagSpec] = {}
    for i, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise TagMapError(f"{path}: tags[{i}] must be a mapping")
        if "id" not in entry:
            raise TagMapError(f"{path}: tags[{i}] has no id")
        tag_id = int(entry["id"])
        if tag_id in tags:
            raise TagMapError(f"{path}: duplicate tag id {tag_id}")

        size = entry.get("size", default_size)
        if size is None:
            raise TagMapError(
                f"{path}: tag {tag_id} has no size and no default_size is set"
            )
        size = float(size)
        if not size > 0:
            raise TagMapError(f"{path}: tag {tag_id} has non-positive size {size}")
        if size > 1.0:
            raise TagMapError(
                f"{path}: tag {tag_id} size is {size} m. That is almost certainly "
                "millimetres or centimetres -- sizes are metres."
            )

        override = entry.get("corners_xyz")
        if override is not None:
            corners = np.asarray(override, dtype=np.float64)
            if corners.shape != (4, 3):
                raise TagMapError(
                    f"{path}: tag {tag_id} corners_xyz must be 4x3, got {corners.shape}"
                )
            tags[tag_id] = TagSpec(
                id=tag_id,
                size=size,
                t_w=corners.mean(axis=0),
                R_wt=np.eye(3),
                note=str(entry.get("note", "")),
                corners_override=corners,
            )
            continue

        if "xyz" not in entry or "rpy" not in entry:
            raise TagMapError(
                f"{path}: tag {tag_id} needs both xyz and rpy (or corners_xyz)"
            )
        xyz = np.asarray(entry["xyz"], dtype=np.float64)
        rpy = np.asarray(entry["rpy"], dtype=np.float64)
        if xyz.shape != (3,):
            raise TagMapError(f"{path}: tag {tag_id} xyz must have 3 elements")
        if rpy.shape != (3,):
            raise TagMapError(f"{path}: tag {tag_id} rpy must have 3 elements")
        if np.any(np.abs(rpy) > 2 * np.pi + 1e-6):
            raise TagMapError(
                f"{path}: tag {tag_id} rpy {rpy.tolist()} exceeds 2*pi -- "
                "these look like degrees, but rpy is radians."
            )

        # Lowercase 'xyz' is EXTRINSIC in scipy, matching URDF <origin rpy=...>.
        R_wt = Rotation.from_euler("xyz", rpy).as_matrix()
        tags[tag_id] = TagSpec(
            id=tag_id,
            size=size,
            t_w=xyz,
            R_wt=R_wt,
            note=str(entry.get("note", "")),
        )

    return tags


def _stack_correspondences(
    groups: Iterable[tuple[int, np.ndarray, np.ndarray]]
) -> Correspondences:
    tag_ids: list[int] = []
    slices: list[slice] = []
    obj_chunks: list[np.ndarray] = []
    img_chunks: list[np.ndarray] = []
    row = 0
    for tag_id, obj, img in groups:
        n = obj.shape[0]
        tag_ids.append(tag_id)
        slices.append(slice(row, row + n))
        obj_chunks.append(obj)
        img_chunks.append(img)
        row += n
    if not obj_chunks:
        return Correspondences(
            obj_pts=np.zeros((0, 3), dtype=np.float64),
            img_pts=np.zeros((0, 2), dtype=np.float64),
        )
    return Correspondences(
        obj_pts=np.vstack(obj_chunks).astype(np.float64),
        img_pts=np.vstack(img_chunks).astype(np.float64),
        tag_ids=tag_ids,
        slices=slices,
    )


def build_correspondences(
    observations: Mapping[int, np.ndarray],
    tag_map: Mapping[int, TagSpec],
    *,
    warn: bool = True,
) -> Correspondences:
    """Pair observed image corners with known world corners.

    ``observations`` maps tag id -> (4, 2) pixel corners, ordered as
    dt_apriltags reports them. Tags absent from ``tag_map`` are skipped.

    This is the single place where the image-corner order and the world-corner
    order are joined; keeping it in one function is what makes the ordering
    convention auditable.
    """
    unknown = sorted(set(observations) - set(tag_map))
    unseen = sorted(set(tag_map) - set(observations))
    if warn and unknown:
        print(f"  [warn] detected tags not in map, ignored: {unknown}")
    if warn and unseen:
        print(f"  [warn] mapped tags not detected this frame: {unseen}")

    groups = []
    for tag_id in sorted(set(observations) & set(tag_map)):
        img = np.asarray(observations[tag_id], dtype=np.float64)
        if img.shape != (4, 2):
            raise TagMapError(
                f"tag {tag_id}: expected (4, 2) image corners, got {img.shape}"
            )
        groups.append((tag_id, world_corners(tag_map[tag_id]), img))

    return _stack_correspondences(groups)


def load_grasp(path: str | Path) -> tuple[np.ndarray, np.ndarray] | None:
    """Optional ``grasp:`` block from an object map -> (t_og, R_og), else None.

    The grasp pose is where the gripper should end up, expressed in the
    OBJECT's own frame. It is kept in the map file because it is a property of
    the physical object, not of any particular pick.

    An object's frame origin is usually its centre (that is what the tag
    geometry defines), which is inside the object and not somewhere a gripper
    can go. This offset is what turns "where the cube is" into "where to put
    the gripper".
    """
    path = Path(path)
    with path.open() as fh:
        doc = yaml.safe_load(fh)
    grasp = (doc or {}).get("grasp")
    if not grasp:
        return None
    xyz = np.asarray(grasp.get("xyz", [0.0, 0.0, 0.0]), dtype=np.float64)
    rpy = np.asarray(grasp.get("rpy", [0.0, 0.0, 0.0]), dtype=np.float64)
    if xyz.shape != (3,) or rpy.shape != (3,):
        raise TagMapError(f"{path}: grasp xyz and rpy must each have 3 elements")
    if np.any(np.abs(rpy) > 2 * np.pi + 1e-6):
        raise TagMapError(
            f"{path}: grasp rpy {rpy.tolist()} exceeds 2*pi -- rpy is radians."
        )
    return xyz, Rotation.from_euler("xyz", rpy).as_matrix()
