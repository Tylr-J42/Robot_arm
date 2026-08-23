#!/usr/bin/env python
"""Broadcast the camera and cube frames into TF from live captures.

    ros2 run tf2_ros ...        # not needed -- this node does it
    ./venv/bin/python camera_tf_publisher.py --rate 0.5

Publishes, per capture:

    base_assy   -> camera_link            (re-solved from the base tags)
    camera_link -> camera_optical_frame   (fixed ROS convention)
    base_assy   -> cube                   (when the cube is visible)
    base_assy   -> cube_grasp             (when the map has a grasp: block)

plus a PoseStamped on ~/cube_pose for anything that would rather subscribe.

These are DYNAMIC transforms, not static ones. The camera extrinsic is derived
fresh from every capture, so a bumped camera simply produces a new, correct
transform instead of a stale, wrong one. That is the whole point of the design;
publishing them as static would throw the property away.

Mostly this exists so you can SEE the calibration in RViz against the robot
model -- an overlay that lines up with the real workspace is far more convincing
than a column of numbers.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster

import apriltag_detect
import camera_capture
import camera_extrinsics as ext
import constants
import scene_solve
import tag_map as tmap

HERE = Path(__file__).resolve().parent


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


class SceneTFPublisher(Node):
    def __init__(self, args):
        super().__init__("camera_tf_publisher")
        self.args = args
        self.K, self.dist = constants.camera_matrix, constants.dist
        self.base_map = tmap.load_tag_map(args.base_tags)
        self.object_map = tmap.load_tag_map(args.cube_tags, expected_frame="cube")
        self.grasp = tmap.load_grasp(args.cube_tags)
        scene_solve.check_disjoint(self.base_map, self.object_map)

        self.detector = apriltag_detect.make_detector()
        self.cam = camera_capture.open_camera(args.device, args.exposure)
        camera_capture.settle(self.cam)

        self.bc = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, "~/cube_pose", 10)
        self.last_t_bc = None

        self.get_logger().info(
            f"base tags {sorted(self.base_map)}, cube tags {sorted(self.object_map)}"
        )
        self.timer = self.create_timer(1.0 / args.rate, self.tick)

    def tick(self):
        frames = camera_capture.capture_burst(self.cam, self.args.burst, settle_frames=0)
        try:
            sol, _ = scene_solve.solve_scene_from_images(
                frames, self.base_map, self.object_map, self.K, self.dist,
                self.detector,
                min_base_tags=self.args.min_base_tags,
                min_object_tags=self.args.min_cube_tags,
                max_rms_px=self.args.max_rms,
            )
        except scene_solve.SceneError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()
        out = []

        # base_assy -> camera_link (derived from the optical pose)
        q_link = Rotation.from_matrix(ext.optical_to_link(sol.R_bc)).as_quat()
        out.append(_tf(stamp, "base_assy", "camera_link", sol.t_bc, q_link))

        # camera_link -> camera_optical_frame: the fixed ROS convention
        q_opt = Rotation.from_euler("xyz", ext.OPTICAL_FROM_LINK_RPY).as_quat()
        out.append(_tf(stamp, "camera_link", "camera_optical_frame",
                       (0.0, 0.0, 0.0), q_opt))

        # base_assy -> cube
        out.append(_tf(stamp, "base_assy", "cube", sol.t_bo, sol.quat_bo()))

        if self.grasp is not None:
            t_g, R_g = sol.grasp_in_base(self.grasp)
            out.append(_tf(stamp, "base_assy", "cube_grasp", t_g,
                           Rotation.from_matrix(R_g).as_quat()))

        self.bc.sendTransform(out)

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_assy"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(v) for v in sol.t_bo)
        q = sol.quat_bo()
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = (float(v) for v in q)
        self.pose_pub.publish(msg)

        # A jump in the camera pose between captures means the camera moved.
        # Worth saying out loud: it invalidates nothing (each solve stands on
        # its own) but it usually means someone knocked the mount.
        if self.last_t_bc is not None:
            d = float(np.linalg.norm(sol.t_bc - self.last_t_bc))
            if d > self.args.move_warn:
                self.get_logger().warn(
                    f"camera moved {d*1000:.1f} mm since the last capture")
        self.last_t_bc = sol.t_bc

        self.get_logger().info(
            f"cube [{sol.t_bo[0]:+.3f} {sol.t_bo[1]:+.3f} {sol.t_bo[2]:+.3f}] m  "
            f"base_rms {sol.camera.rms_px:.2f} px  cube_rms {sol.obj.rms_px:.2f} px",
            throttle_duration_sec=2.0,
        )

    def destroy_node(self):
        try:
            self.cam.release()
        finally:
            super().destroy_node()


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--base-tags", type=Path, default=HERE / "tags_base_assy.yaml")
    p.add_argument("--cube-tags", type=Path, default=HERE / "cube_tags.yaml")
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--exposure", type=int, default=60)
    p.add_argument("--burst", type=int, default=10)
    p.add_argument("--rate", type=float, default=1.0, help="captures per second")
    p.add_argument("--min-base-tags", type=int, default=3)
    p.add_argument("--min-cube-tags", type=int, default=2)
    p.add_argument("--max-rms", type=float, default=1.5)
    p.add_argument("--move-warn", type=float, default=0.005,
                   help="warn if the camera shifts more than this, metres")
    args = p.parse_args(argv)

    rclpy.init()
    node = SceneTFPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
