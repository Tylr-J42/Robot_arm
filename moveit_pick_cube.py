#!/usr/bin/env python3
"""
Pick the tagged cube where VISION says it is, planned with MoveIt 2.

This is the seam described in VISION_PIPELINE.md finally wired up: one camera
capture produces a cube pose in base_assy, that pose becomes the pick
waypoints, and the SAME solve is broadcast into TF and the planning scene so
what you see in RViz is exactly what the arm is about to act on. If the picture
looks wrong, the motion is wrong -- there is no second, hidden number.

SEQUENCE
    rotate base   --start-base-deg (default 180) before anything else
    approach      grasp point + 0.15 m up, planner picks the path
    descend       STRAIGHT down onto the cube (Pilz LIN)
    close
    retreat       STRAIGHT back up to the approach point (Pilz LIN)
    lift          grasp point + 0.30 m

    The staging height and the straight descent go together: a joint-space plan
    between two nearby points is free to bow sideways and clip the cube on the
    way in. Stage well clear, then come down the one axis that cannot.

GRASP ORIENTATION
    Default is --orientation down: the tool points straight at the table, which
    is what a cube lying on it wants. The inherited pick-and-pour orientation
    (--orientation fixed) holds the tool HORIZONTAL for reaching round a
    bottle; it is kept only for comparison. The spin about the vertical is the
    remaining freedom -- --yaw azimuth (default) follows the turret bearing and
    is the most reachable, --yaw cube squares the fingers to the cube's faces.

It deliberately does NOT reuse moveit_pick_and_pour.py's bottle waypoints; it
subclasses that script purely to inherit the tuned move/gripper/plan helpers.

WHY ONE CAPTURE, NOT A LOOP
    The camera is static and looks across the workspace, so the arm occludes
    the cube (or the base tags) as soon as it moves. Solve once, before any
    motion, then execute open loop. The cube is not going anywhere. This is
    also why the TF frames below are republished from a STORED solve rather
    than re-measured -- re-capturing mid-motion would just fail.

WHAT LANDS IN RViz
    TF:      base_assy -> camera_link -> camera_optical_frame
             base_assy -> cube, base_assy -> cube_grasp
    Topics:  /cube_pose            (geometry_msgs/PoseStamped)
             /display_planned_path (each trajectory, before it executes)
    Scene:   "table" and "cube" collision objects, so the planner knows the
             table is solid and you can SEE the measurement as a box.
    Markers: /vision_markers -- the cube, the grasp point, EVERY base tag with
             its id, the camera body and its view frustum. Add a MarkerArray
             display on that topic. If a tag marker does not land on the real
             printed tag, the map is wrong and so is everything downstream.

RUN
    # terminal 1 -- planner + robot state (starts move_group and RViz)
    ros2 launch tyler_arm bringup_real.launch.py
    # terminal 2 -- the Pi
    python3 arm_pi_node.py
    # terminal 3 -- look before you leap: plan and draw, do not move
    ros2 launch /home/tyler/Desktop/Robot_arm/pick_cube.launch.py plan_only:=true
    # then for real
    ros2 launch /home/tyler/Desktop/Robot_arm/pick_cube.launch.py

Do NOT run camera_tf_publisher.py at the same time -- it opens the same
/dev/video device and one of the two will fail. This script publishes the same
frames itself, from the solve it actually used.
"""

from __future__ import annotations

import argparse
import sys
import time
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from moveit_msgs.msg import CollisionObject, DisplayTrajectory
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time as RclpyTime
from rclpy.utilities import remove_ros_args
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

import camera_extrinsics as ext
import constants
import cube_pick_target
import tag_map as tmap
from moveit_pick_and_pour import (
    ARM_GROUP,
    BASE_FRAME,
    GRIPPER_CLOSE,
    GRIPPER_OPEN,
    PickAndPour,
    _EE_FRAME_OFFSET_RPY,
)

TABLE_ID = "table"
CUBE_ID = "cube"
BASE_JOINT_NAME = "1st"      # the turret; revolute, limits +-pi (robot.urdf)


def wrist_quat_pointing_down(azimuth_rad: float) -> np.ndarray:
    """Wrist orientation whose TOOL axis points straight down (-Z in base).

    The tool tip sits at (-0.0095, 0, 0.06275) off ``6th_wrist_joint``, i.e.
    essentially along the wrist's +Z, so "tool down" means wrist +Z -> base -Z.
    ``Ry(pi)`` does exactly that, and the leftover freedom is a spin about the
    vertical, which ``Rz(azimuth)`` fixes.

    Compare the inherited pick-and-pour orientation, which puts the tool
    HORIZONTAL (+Y, 0 deg elevation) -- right for reaching round a bottle,
    wrong for dropping onto a cube.

    Returns the WRIST orientation, matching TARGET_ORIENTATION_RPY's convention;
    move_to_pose() composes the ee_frame offset itself.
    """
    return (Rotation.from_euler("z", azimuth_rad)
            * Rotation.from_euler("y", np.pi)).as_quat()


def _pose(xyz, quat=(0.0, 0.0, 0.0, 1.0)) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    (p.orientation.x, p.orientation.y,
     p.orientation.z, p.orientation.w) = (float(v) for v in quat)
    return p


def _point(xyz) -> Point:
    p = Point()
    p.x, p.y, p.z = (float(v) for v in xyz)
    return p


def _tf(stamp, parent: str, child: str, t, q) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(t[0])
    msg.transform.translation.y = float(t[1])
    msg.transform.translation.z = float(t[2])
    msg.transform.rotation.x = float(q[0])
    msg.transform.rotation.y = float(q[1])
    msg.transform.rotation.z = float(q[2])
    msg.transform.rotation.w = float(q[3])
    return msg


class CubePicker(PickAndPour):
    """A pick driven by one vision measurement, published as it is used."""

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.logger = get_logger("pick_cube")
        self.target = None

        # A second node purely for the things RViz reads. MoveItPy owns its own
        # node and we do not want to fight it for callbacks.
        self.vis = Node("cube_vision")
        self.tf_bc = TransformBroadcaster(self.vis)
        self.pose_pub = self.vis.create_publisher(PoseStamped, "/cube_pose", 10)
        self.display_pub = self.vis.create_publisher(
            DisplayTrajectory, "/display_planned_path", 1)
        # Transient-local so the markers are still there for an RViz that
        # subscribes after we have published, which is the normal case.
        self.marker_pub = self.vis.create_publisher(
            MarkerArray, "/vision_markers",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # TF is not latched: a transform published once ages out of RViz in
        # seconds. The measurement is genuinely fixed for the whole task, but
        # it still has to be re-stamped and re-sent, so keep a stored copy and
        # republish it on a timer from a background executor.
        self._tf_msgs: list[TransformStamped] = []
        self._tf_lock = threading.Lock()
        self.vis.create_timer(0.1, self._republish_tf)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.vis)
        threading.Thread(target=self._executor.spin, daemon=True).start()

    def shutdown(self) -> None:
        """Stop the background executor before rclpy goes away.

        Tearing rclpy down underneath a spinning executor produces a spray of
        exceptions from the timer thread that look like a crash but are just
        an untidy exit.
        """
        try:
            self._executor.shutdown()
        except Exception:                                       # noqa: BLE001
            pass
        for node in (getattr(self, "vis", None),
                     getattr(self, "gripper_node", None)):
            if node is None:
                continue
            try:
                node.destroy_node()
            except Exception:                                   # noqa: BLE001
                pass

    # -- vision -------------------------------------------------------------
    def capture(self) -> bool:
        """Solve the scene once. Must happen before the arm moves."""
        if self.args.cube_xyz is not None:
            # Bench mode: no camera, no tags. Lets you exercise the planning and
            # the RViz path when the rig is not set up.
            xyz = np.array(self.args.cube_xyz, dtype=float)
            up = np.array([0.0, 0.0, 1.0])
            self.target = cube_pick_target.PickTarget(
                grasp=xyz,
                approach=xyz + up * self.args.approach_height,
                lift=xyz + up * self.args.lift_height,
                quat=np.array([0.0, 0.0, 0.0, 1.0]),
                cube_xyz=xyz,
                base_rms_px=float("nan"), cube_rms_px=float("nan"),
                cube_tag_ids=[], camera_xyz=np.zeros(3),
                cube_quat=np.array([0.0, 0.0, 0.0, 1.0]),
                camera_quat=np.array([0.0, 0.0, 0.0, 1.0]),
            )
            self.logger.warn(f"--cube-xyz given: skipping the camera, using {xyz}")
            return True

        self.logger.info("Capturing (arm must be clear of the camera's view)...")
        try:
            self.target = cube_pick_target.get_pick_target(
                device=self.args.device,
                exposure=self.args.exposure,
                burst=self.args.burst,
                approach_height=self.args.approach_height,
                lift_height=self.args.lift_height,
                max_rms_px=self.args.max_rms,
            )
        except cube_pick_target.NoTargetError as e:
            self.logger.error(f"Vision failed, NOT moving: {e}")
            return False

        self.logger.info(self.target.describe())
        return True

    def publish_vision(self) -> None:
        """Store the solve's frames so the timer keeps them alive in RViz."""
        t = self.target
        msgs = [
            ("base_assy", "camera_link", t.camera_xyz, t.camera_quat),
            ("camera_link", "camera_optical_frame", (0.0, 0.0, 0.0),
             Rotation.from_euler("xyz", ext.OPTICAL_FROM_LINK_RPY).as_quat()),
            ("base_assy", "cube", t.cube_xyz, t.cube_quat),
            ("base_assy", "cube_grasp", t.grasp, t.quat),
        ]
        stamp = self.vis.get_clock().now().to_msg()
        with self._tf_lock:
            self._tf_msgs = [_tf(stamp, p, c, tr, q) for p, c, tr, q in msgs]

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = BASE_FRAME
        msg.pose = _pose(t.cube_xyz, t.cube_quat)
        self.pose_pub.publish(msg)

    def publish_markers(self) -> None:
        """Draw the cube, every base tag, and the camera into RViz.

        The collision objects already make the cube plannable, but they say
        nothing about the tags or the camera -- the two things you actually
        want to eyeball, because if a tag marker does not sit on the real
        printed tag then the map is wrong and everything downstream is too.

        Markers are stamped with time 0 ("latest transform") and sit in the
        frames this node publishes, so they follow the solve automatically.
        """
        if self.args.no_markers:
            return
        t = self.target
        arr = MarkerArray()
        zero = RclpyTime().to_msg()

        def base(mid, kind, frame=BASE_FRAME):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = zero
            m.ns = "vision"
            m.id = mid
            m.type = kind
            m.action = Marker.ADD
            m.pose = _pose((0.0, 0.0, 0.0))
            return m

        # --- the cube, where vision says it is
        cube_map = tmap.load_tag_map(
            cube_pick_target.DEFAULT_CUBE_TAGS, expected_frame="cube")
        edge = 2.0 * max(float(np.abs(s.t_w).max()) for s in cube_map.values())
        m = base(0, Marker.CUBE)
        m.pose = _pose(t.cube_xyz, t.cube_quat)
        m.scale.x = m.scale.y = m.scale.z = edge
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.2, 0.55
        arr.markers.append(m)

        # --- the grasp point the arm is actually aiming at
        m = base(1, Marker.SPHERE)
        m.pose = _pose(t.grasp)
        m.scale.x = m.scale.y = m.scale.z = 0.012
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.1, 0.95
        arr.markers.append(m)

        # --- every base tag, as it is mapped
        base_map = tmap.load_tag_map(cube_pick_target.DEFAULT_BASE_TAGS)
        for i, (tag_id, spec) in enumerate(sorted(base_map.items())):
            q = Rotation.from_matrix(spec.R_wt).as_quat()
            m = base(100 + i, Marker.CUBE)
            m.pose = _pose(spec.t_w, q)
            m.scale.x = m.scale.y = float(spec.size)
            m.scale.z = 0.0012
            m.color.r, m.color.g, m.color.b, m.color.a = 0.95, 0.85, 0.1, 0.9
            arr.markers.append(m)

            m = base(200 + i, Marker.TEXT_VIEW_FACING)
            m.pose = _pose(spec.t_w + np.array([0.0, 0.0, 0.035]))
            m.scale.z = 0.030
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.text = f"tag {tag_id}"
            arr.markers.append(m)

        # --- the camera body, in its own frame so it tracks the solve
        m = base(300, Marker.CUBE, frame="camera_link")
        m.scale.x, m.scale.y, m.scale.z = 0.030, 0.090, 0.026
        m.color.r, m.color.g, m.color.b, m.color.a = 0.15, 0.15, 0.18, 0.95
        arr.markers.append(m)

        # --- what the camera can actually see. Occlusion is the failure mode
        #     this rig hits most, so drawing the frustum earns its place.
        depth = float(np.linalg.norm(np.asarray(t.cube_xyz) - np.asarray(t.camera_xyz)))
        depth = float(np.clip(depth, 0.20, 3.0)) if np.isfinite(depth) and depth > 0 else 1.0
        w, h = constants.camera_res
        K = constants.camera_matrix
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        corners = [((u - cx) / fx * depth, (v - cy) / fy * depth, depth)
                   for u, v in ((0, 0), (w, 0), (w, h), (0, h))]
        m = base(301, Marker.LINE_LIST, frame="camera_optical_frame")
        m.scale.x = 0.002
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.9, 1.0, 0.5
        for i, c in enumerate(corners):
            m.points.append(_point((0.0, 0.0, 0.0)))
            m.points.append(_point(c))
            m.points.append(_point(c))
            m.points.append(_point(corners[(i + 1) % 4]))
        arr.markers.append(m)

        self.marker_pub.publish(arr)
        self.logger.info(
            f"RViz markers: cube ({edge*1000:.0f} mm), {len(base_map)} base tags, "
            f"camera + {depth:.2f} m frustum on /vision_markers")

    def _republish_tf(self) -> None:
        with self._tf_lock:
            if not self._tf_msgs:
                return
            stamp = self.vis.get_clock().now().to_msg()
            for m in self._tf_msgs:
                m.header.stamp = stamp
            self.tf_bc.sendTransform(list(self._tf_msgs))

    # -- planning scene -----------------------------------------------------
    def _apply(self, co: CollisionObject) -> bool:
        """Push one collision object, surviving a binding that misbehaves.

        moveit_py's scene API is the shakiest part of this build (see the
        RobotState note in moveit_pick_and_pour._joint_goal), so a failure here
        downgrades to a warning: losing the table in RViz is not a reason to
        refuse to pick.
        """
        try:
            psm = self.moveit.get_planning_scene_monitor()
            with psm.read_write() as scene:
                scene.apply_collision_object(co)
                scene.current_state.update()
            return True
        except Exception as e:                                  # noqa: BLE001
            self.logger.warn(f"could not update the planning scene ({co.id}): {e}")
            return False

    def add_scene(self) -> None:
        """Put the table and the measured cube into the world."""
        if self.args.no_scene:
            return

        # The table top is where the base tags lie -- they are taped flat to
        # it, so their z IS the surface. Reading it from the map means this
        # cannot drift out of sync with the calibration.
        base_map = tmap.load_tag_map(cube_pick_target.DEFAULT_BASE_TAGS)
        table_z = float(np.median([s.t_w[2] for s in base_map.values()]))

        table = CollisionObject()
        table.header.frame_id = BASE_FRAME
        table.id = TABLE_ID
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [2.0, 2.0, 0.02]
        table.primitives = [prim]
        table.primitive_poses = [_pose((0.0, 0.0, table_z - 0.01))]
        table.operation = CollisionObject.ADD
        if self._apply(table):
            self.logger.info(f"planning scene: table top at z={table_z:+.4f} m")

        # The cube's edge comes from its own map: the face tags sit at +-edge/2.
        cube_map = tmap.load_tag_map(
            cube_pick_target.DEFAULT_CUBE_TAGS, expected_frame="cube")
        edge = 2.0 * max(float(np.abs(s.t_w).max()) for s in cube_map.values())

        cube = CollisionObject()
        cube.header.frame_id = BASE_FRAME
        cube.id = CUBE_ID
        cprim = SolidPrimitive()
        cprim.type = SolidPrimitive.BOX
        cprim.dimensions = [edge, edge, edge]
        cube.primitives = [cprim]
        cube.primitive_poses = [
            _pose(self.target.cube_xyz, self.target.cube_quat)]
        cube.operation = CollisionObject.ADD
        if self._apply(cube):
            self.logger.info(f"planning scene: cube {edge*1000:.0f} mm at "
                             f"{np.round(self.target.cube_xyz, 3).tolist()}")

    def remove_cube(self) -> None:
        """Drop the cube from the world just before the gripper occupies it.

        Keeping it would make the descent self-colliding by construction: we
        are deliberately moving the gripper to where the cube is. It stays in
        the scene for the approach, which is the move that actually benefits
        from knowing it is there.
        """
        if self.args.no_scene:
            return
        co = CollisionObject()
        co.header.frame_id = BASE_FRAME
        co.id = CUBE_ID
        co.operation = CollisionObject.REMOVE
        self._apply(co)

    # -- motion -------------------------------------------------------------
    def _plan_and_execute(self, straight_line=False):
        """As the parent, plus draw the trajectory and honour --plan-only."""
        self.arm.set_start_state_to_current_state()
        if straight_line:
            from moveit.planning import PlanRequestParameters
            params = PlanRequestParameters(self.moveit, "pilz_lin")
            plan = self.arm.plan(single_plan_parameters=params)
        else:
            plan = self.arm.plan()

        if not plan:
            self.logger.error("Planning FAILED -- aborting task.")
            return False

        self._display(plan.trajectory)
        if self.args.plan_only:
            self.logger.info("--plan-only: planned but NOT executing.")
            return True
        self.moveit.execute(plan.trajectory, controllers=[])
        return True

    def _display(self, trajectory) -> None:
        """Publish for RViz's "Planned Path" display."""
        try:
            msg = DisplayTrajectory()
            msg.trajectory.append(trajectory.get_robot_trajectory_msg())
            self.display_pub.publish(msg)
        except Exception as e:                                  # noqa: BLE001
            self.logger.warn(f"could not publish the planned path: {e}")

    def move_straight(self, xyz, quat=None, what="move"):
        """A Cartesian straight line, falling back to joint space if LIN fails.

        Pilz LIN maps a straight Cartesian path onto the joints, and near a
        singularity that demands joint accelerations past the limits -- exactly
        why moveit_pick_and_pour.py gave up on LIN for its long lateral moves.
        A short vertical hop is a much easier case, but "much easier" is not
        "guaranteed", so a LIN failure degrades to an OMPL plan with a loud
        warning rather than aborting the pick.

        The fallback is NOT a straight line. If that matters for this move --
        and for the descent onto the cube it might -- use --require-straight to
        make a LIN failure fatal instead.
        """
        self.logger.info(f"{what}: straight line to {np.round(xyz, 3).tolist()}")
        if self.move_to_pose(xyz, straight_line=True, quat=quat):
            return True

        if self.args.require_straight:
            self.logger.error(
                f"{what}: LIN planning failed and --require-straight is set, so "
                "stopping rather than approaching along an unknown path.")
            return False

        self.logger.warn(
            f"{what}: LIN planning failed -- falling back to a joint-space plan. "
            "This path is NOT straight and may bow sideways; watch it. Raising "
            "--approach-height or lowering the pilz_lin scaling factors in "
            "pick_cube.launch.py usually gets LIN working.")
        return self.move_to_pose(xyz, straight_line=False, quat=quat)

    def rotate_base(self) -> bool:
        """Spin the turret by --start-base-deg before doing anything else.

        Same bounded-move shape as PickAndPour.pour(): the base joint "1st" is
        revolute with limits [-pi, +pi], NOT continuous, so a blind +180 deg
        can walk straight past the upper stop. Try +, then -, then clamp.
        """
        deg = self.args.start_base_deg
        if not deg:
            return True

        psm = self.moveit.get_planning_scene_monitor()
        with psm.read_only() as scene:
            current = list(scene.current_state.get_joint_group_positions(ARM_GROUP))

        jmg = self.robot_model.get_joint_model_group(ARM_GROUP)
        names = jmg.active_joint_model_names
        bounds = jmg.active_joint_model_bounds
        if BASE_JOINT_NAME not in names:
            self.logger.error(
                f"no joint {BASE_JOINT_NAME!r} in group {ARM_GROUP} "
                f"(have {list(names)}) -- skipping the base rotation.")
            return True
        idx = names.index(BASE_JOINT_NAME)
        lower = bounds[idx][0].min_position
        upper = bounds[idx][0].max_position

        delta = np.radians(deg)
        now = current[idx]
        margin = 1e-3
        if lower + margin <= now + delta <= upper - margin:
            target = now + delta
        elif lower + margin <= now - delta <= upper - margin:
            target = now - delta
            self.logger.info(f"base: {deg:+.0f} deg would leave the joint range, "
                             f"going the other way instead.")
        else:
            # Both directions overshoot. Go as far as the stop allows and say
            # exactly how far that was, rather than silently doing nothing or
            # silently doing something different.
            target = (upper - margin) if delta > 0 else (lower + margin)
            if abs(target - now) < abs((lower + margin) - now):
                target = lower + margin
            self.logger.warn(
                f"base: cannot rotate {deg:+.1f} deg from {np.degrees(now):+.1f} "
                f"deg within limits [{np.degrees(lower):+.1f}, "
                f"{np.degrees(upper):+.1f}]; clamping to "
                f"{np.degrees(target):+.1f} deg "
                f"({np.degrees(target - now):+.1f} deg of rotation).")

        self.logger.info(
            f"base: rotating {BASE_JOINT_NAME} {np.degrees(now):+.1f} -> "
            f"{np.degrees(target):+.1f} deg")
        goal = list(current)
        goal[idx] = float(target)
        self.arm.set_goal_state(
            motion_plan_constraints=[self._joint_goal(names, goal)])
        if not self._plan_and_execute():
            self.logger.error("base rotation failed to plan -- stopping.")
            return False
        return True

    def _grasp_quat(self):
        """The wrist orientation to grasp with, or None to keep the tuned one."""
        mode = self.args.orientation
        if mode == "fixed":
            self.logger.info(
                "orientation 'fixed': inherited pick-and-pour wrist pose, tool "
                "HORIZONTAL. Fine for a side grasp, not for a cube on a table.")
            return None

        if mode == "vision":
            self.logger.warn(
                "orientation 'vision': taken from cube_tags.yaml's grasp: block, "
                "which is a TOOL-frame pose being used where a WRIST pose is "
                "expected, and is untuned against this gripper. Experimental.")
            return self.target.quat

        # "down"
        t = self.target
        if self.args.yaw == "cube":
            # Square the fingers to the cube's faces.
            yaw = float(Rotation.from_quat(t.cube_quat).as_euler("xyz")[2])
            how = "cube faces"
        else:
            # Point the wrist along the turret's own bearing to the target. The
            # arm already lies in that vertical plane to reach, so this is the
            # yaw least likely to twist it into a corner of its joint range.
            yaw = float(np.arctan2(t.grasp[1], t.grasp[0]))
            how = "turret bearing"
        yaw += np.radians(self.args.yaw_offset)
        self.logger.info(
            f"orientation 'down': tool straight down, yaw {np.degrees(yaw):+.1f} "
            f"deg from {how}"
            + (f" (incl. {self.args.yaw_offset:+.1f} deg offset)"
               if self.args.yaw_offset else ""))
        return wrist_quat_pointing_down(yaw)

    # -- the task -----------------------------------------------------------
    def run(self):
        self.logger.info("Starting vision-guided cube pick.")

        if not self.capture():
            return
        self.publish_vision()
        self.publish_markers()
        self.add_scene()

        t = self.target
        # Worth saying out loud: every waypoint in the tuned pick-and-pour task
        # sits at z=0.175, and a cube on the table is far below that. The fixed
        # orientation was tuned up there, so a low grasp is the most likely
        # thing to fail to plan. Better to name it now than to read it out of a
        # bare "Planning FAILED".
        if t.grasp[2] < 0.05:
            self.logger.warn(
                f"grasp z is {t.grasp[2]:+.3f} m -- low. The tuned orientation "
                "came from waypoints at z=0.175; if planning fails here that "
                "is the first thing to suspect, not the vision.")

        self.logger.info(f"approach {np.round(t.approach, 3).tolist()}")
        self.logger.info(f"grasp    {np.round(t.grasp, 3).tolist()}")
        self.logger.info(f"lift     {np.round(t.lift, 3).tolist()}")

        quat = self._grasp_quat()

        if not self.rotate_base():
            return

        self.gripper(GRIPPER_OPEN)

        # Stage high, well clear of the cube, by whatever path the planner
        # likes -- nothing is near the cube yet.
        if not self.move_to_pose(t.approach, quat=quat):
            return

        # The cube has to leave the world before the descent: we are moving the
        # gripper to exactly where it is, so keeping it would make the move
        # self-colliding by construction.
        self.remove_cube()

        # Straight DOWN onto the cube. A joint-space plan between these two
        # points is free to bow sideways and clip the cube on the way in, which
        # is the whole reason to stage above it first.
        if not self.move_straight(t.grasp, quat=quat, what="descent"):
            return

        self.gripper(GRIPPER_CLOSE)

        # Straight back UP before anything else moves, so the cube is lifted
        # clear rather than dragged across the table.
        if not self.move_straight(t.approach, quat=quat, what="retreat"):
            return
        if not self.move_to_pose(t.lift, quat=quat):
            return

        self.logger.info("Cube pick complete.")

    def move_to_pose(self, xyz, straight_line=False, quat=None):
        """Parent's move, with an optional orientation override.

        quat=None keeps the tuned fixed orientation, which is what
        VISION_PIPELINE.md's integration plan asks for: position from vision
        first, orientation later.
        """
        if quat is None:
            return super().move_to_pose(xyz, straight_line=straight_line)

        from moveit_pick_and_pour import EE_FRAME, _EE_FRAME_OFFSET_RPY, quat_mult, rpy_to_quat
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose = _pose(xyz, quat_mult(tuple(float(v) for v in quat),
                                         rpy_to_quat(*_EE_FRAME_OFFSET_RPY)))
        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link=EE_FRAME)
        return self._plan_and_execute(straight_line=straight_line)


def _xyz(text: str) -> list[float]:
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected three numbers, e.g. -0.1,0.5,0.02")
    return [float(p) for p in parts]


def main(argv=None):
    argv = remove_ros_args(sys.argv if argv is None else argv)[1:]
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--exposure", type=int, default=None,
                   help="override constants.CAMERA_SETTINGS exposure")
    p.add_argument("--burst", type=int, default=20)
    p.add_argument("--max-rms", type=float, default=4.0,
                   help="reject the solve above this reprojection rms, px")
    p.add_argument("--approach-height", type=float, default=0.15,
                   help="how far above the grasp point to stage before the "
                        "straight descent, metres")
    p.add_argument("--lift-height", type=float, default=0.30,
                   help="how far above the grasp point to lift, metres")
    p.add_argument("--require-straight", action="store_true",
                   help="treat a failed straight-line (Pilz LIN) plan as fatal "
                        "instead of falling back to a joint-space path")
    p.add_argument("--start-base-deg", type=float, default=180.0,
                   help="rotate the turret this many degrees before the pick; "
                        "0 disables. The joint is limited to +-180 deg, so this "
                        "may be flipped or clamped to stay in range")
    p.add_argument("--plan-only", action="store_true",
                   help="plan and draw in RViz, but do not move the arm")
    p.add_argument("--no-scene", action="store_true",
                   help="skip the table/cube collision objects")
    p.add_argument("--no-markers", action="store_true",
                   help="skip the cube/base-tag/camera markers on /vision_markers")
    p.add_argument("--orientation", choices=("down", "fixed", "vision"),
                   default="down",
                   help="wrist pose to grasp with. 'down' points the tool "
                        "straight at the table (right for a cube); 'fixed' is "
                        "the inherited pick-and-pour side grasp; 'vision' uses "
                        "cube_tags.yaml's grasp block and is experimental")
    p.add_argument("--yaw", choices=("azimuth", "cube"), default="azimuth",
                   help="with --orientation down, what sets the spin about the "
                        "vertical: 'azimuth' follows the turret bearing (most "
                        "reachable), 'cube' squares the fingers to the faces")
    p.add_argument("--yaw-offset", type=float, default=0.0,
                   help="degrees added to the chosen yaw, for tuning finger "
                        "alignment without editing code")
    p.add_argument("--cube-xyz", type=_xyz, default=None,
                   help="bench mode: skip the camera and pick here instead. "
                        "Use the = form -- --cube-xyz=-0.1,0.5,0.02 -- because "
                        "a leading minus reads as an option name otherwise")
    p.add_argument("--hold", action=argparse.BooleanOptionalAction, default=None,
                   help="after the task, keep republishing TF so the frames stay "
                        "visible in RViz until Ctrl-C. %(default)s = auto: on "
                        "with --plan-only (which otherwise exits before you can "
                        "look), off for a real pick")
    args = p.parse_args(argv)
    if args.hold is None:
        # The whole point of --plan-only is to LOOK at the result. Exiting
        # immediately drops the TF frames and leaves an empty RViz, which reads
        # as "it didn't work" when in fact it worked and then quit.
        args.hold = args.plan_only

    rclpy.init()
    task = CubePicker(args)
    try:
        task.run()
        if args.hold:
            task.logger.info(
                "Holding: TF frames stay published so you can inspect them in "
                "RViz. The collision objects live in move_group and persist on "
                "their own. Ctrl-C when done.")
            while rclpy.ok():
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        task.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
