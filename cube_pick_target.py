"""Turn a camera capture into pick waypoints for moveit_pick_and_pour.py.

This is the seam between vision and motion. It is deliberately a separate
module so the working pick-and-pour script needs only a couple of lines
changed, and so a failed capture is a caught exception rather than a crash
halfway through a motion.

Usage from moveit_pick_and_pour.py -- replace the hardcoded waypoints:

    import cube_pick_target

    def run(self):
        try:
            target = cube_pick_target.get_pick_target()
        except cube_pick_target.NoTargetError as e:
            self.logger.error(f"Vision failed, not moving: {e}")
            return
        self.gripper(GRIPPER_OPEN)
        if not self.move_to_pose(target.approach): return
        if not self.move_to_pose(target.grasp):    return
        self.gripper(GRIPPER_CLOSE)
        if not self.move_to_pose(target.lift):     return
        ...

Capture BEFORE the arm moves. The camera is fixed and looking at the workspace,
so once the arm is in flight it will sooner or later occlude the cube or the
base tags. Solve once, then execute the whole pick open-loop -- the cube is not
going anywhere.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import apriltag_detect
import camera_capture
import camera_extrinsics as ext
import constants
import scene_solve
import tag_map as tmap

HERE = Path(__file__).resolve().parent
DEFAULT_BASE_TAGS = HERE / "tags_base_assy.yaml"
DEFAULT_CUBE_TAGS = HERE / "cube_tags.yaml"


class NoTargetError(RuntimeError):
    """The scene could not be solved well enough to move the arm.

    Carries the evidence with it. ``frames`` is the burst that was captured and
    ``observations`` the tags that were found in it (possibly empty), so a
    caller can show the operator what the camera actually saw instead of only
    a sentence about it. Both are None when the failure happened before or
    during capture. See ``locate_cube.annotate_failure``.
    """

    def __init__(self, message, *, frames=None, observations=None,
                 base_map=None, cube_map=None):
        super().__init__(message)
        self.frames = frames
        self.observations = observations
        self.base_map = base_map
        self.cube_map = cube_map


@dataclass
class PickTarget:
    """Waypoints in base_assy, metres -- ready for move_to_pose()."""

    grasp: np.ndarray
    approach: np.ndarray
    lift: np.ndarray
    quat: np.ndarray  # grasp orientation, for when you stop using a fixed one
    cube_xyz: np.ndarray
    base_rms_px: float
    cube_rms_px: float
    cube_tag_ids: list[int]
    camera_xyz: np.ndarray
    # Orientations for the TF frames a consumer will want to publish, so the
    # SAME solve that moves the arm is the one drawn in RViz. camera_quat is
    # the ROS camera_link convention (x forward), not the optical frame --
    # camera_tf_publisher.py derives it the same way.
    cube_quat: np.ndarray
    camera_quat: np.ndarray

    def describe(self) -> str:
        return (
            f"cube [{self.cube_xyz[0]:+.3f} {self.cube_xyz[1]:+.3f} "
            f"{self.cube_xyz[2]:+.3f}] m, grasp [{self.grasp[0]:+.3f} "
            f"{self.grasp[1]:+.3f} {self.grasp[2]:+.3f}] m, from tags "
            f"{self.cube_tag_ids} (base_rms {self.base_rms_px:.2f} px, "
            f"cube_rms {self.cube_rms_px:.2f} px)"
        )


def get_pick_target(
    *,
    base_tags: Path = DEFAULT_BASE_TAGS,
    cube_tags: Path = DEFAULT_CUBE_TAGS,
    device: int = 0,
    exposure: int | None = None,
    burst: int = 20,
    approach_height: float = 0.08,
    lift_height: float = 0.30,
    min_base_tags: int = 3,
    min_cube_tags: int = 2,
    max_rms_px: float = 4.0,
    max_reach: float = 0.9,
) -> PickTarget:
    """Capture once, solve the scene, and return pick waypoints.

    Raises NoTargetError on any doubt. Refusing to move is always cheaper than
    moving to a pose derived from a bad solve.
    """
    K, dist = constants.camera_matrix, constants.dist
    base_map = tmap.load_tag_map(base_tags)
    cube_map = tmap.load_tag_map(cube_tags, expected_frame="cube")
    grasp = tmap.load_grasp(cube_tags)
    scene_solve.check_disjoint(base_map, cube_map)

    # Retries with a fresh open: the camera intermittently streams frames that
    # carry no image at all, and a reopen usually clears it. Without this the
    # flat frame reaches the detector and reads as "no tags visible".
    try:
        frames, _ = camera_capture.open_and_capture(device, exposure, burst)
    except camera_capture.DeadCaptureError as e:
        raise NoTargetError(str(e)) from e

    detector = apriltag_detect.make_detector()

    def no_target(message, cause=None):
        """Fail WITH the picture. Re-detecting is cheap next to a bad move."""
        try:
            obs, _, _ = scene_solve.observe(frames, detector)
        except Exception:                                       # noqa: BLE001
            obs = {}
        err = NoTargetError(message, frames=frames, observations=obs,
                            base_map=base_map, cube_map=cube_map)
        if cause is not None:
            raise err from cause
        raise err

    try:
        sol, _ = scene_solve.solve_scene_from_images(
            frames, base_map, cube_map, K, dist, detector,
            min_base_tags=min_base_tags,
            min_object_tags=min_cube_tags,
            max_rms_px=max_rms_px,
        )
    except scene_solve.SceneError as e:
        no_target(str(e), e)

    if sol.obj.planar:
        no_target(
            "only one cube face is visible, so the cube's orientation is "
            "two-fold ambiguous. Move the camera to see a second face."
        )

    t_g, R_g = sol.grasp_in_base(grasp)

    # Sanity-check against the workspace before handing anything to a planner.
    # A number that is physically impossible means the solve is wrong, and it
    # is much better to say so here than to watch MoveIt fail obscurely.
    reach = float(np.linalg.norm(t_g))
    if reach > max_reach:
        no_target(
            f"grasp point is {reach:.3f} m from the base, beyond the {max_reach} m "
            "sanity limit -- the solve is almost certainly wrong."
        )
    if t_g[2] < -0.05:
        no_target(
            f"grasp point is {t_g[2]:.3f} m, below the base plane -- the solve "
            "is almost certainly wrong (check for a flipped planar solution)."
        )

    up = np.array([0.0, 0.0, 1.0])
    return PickTarget(
        grasp=t_g,
        approach=t_g + up * approach_height,
        lift=t_g + up * lift_height,
        quat=Rotation.from_matrix(R_g).as_quat(),
        cube_xyz=sol.t_bo,
        base_rms_px=sol.camera.rms_px,
        cube_rms_px=sol.obj.rms_px,
        cube_tag_ids=list(sol.obj.tag_ids),
        camera_xyz=sol.t_bc,
        cube_quat=sol.quat_bo(),
        camera_quat=Rotation.from_matrix(ext.optical_to_link(sol.R_bc)).as_quat(),
    )


if __name__ == "__main__":
    try:
        print(get_pick_target().describe())
    except NoTargetError as e:
        raise SystemExit(f"no target: {e}")
