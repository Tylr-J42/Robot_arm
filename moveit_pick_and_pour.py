#!/usr/bin/env python3
"""
Pick-and-pour task, planned with MoveIt 2  (runs on the DEV MACHINE).

This replaces the PyTorch IK solver + manual line-interpolation in
pytorch_ik_to_robot.py. MoveIt does the IK, the collision-aware planning, and
the time parameterization. The resulting trajectory is sent to the Pi in ONE
FollowJointTrajectory goal (see arm_pi_node.py) — not streamed point by point.

PREREQUISITES (one-time, not code in this file):
  * A MoveIt config package generated from robot.urdf with:
        - planning group "arm"  = the 6 stepper joints
        - a tool/EE link ("ee") with the -0.0095/0/0.06275 tool offset baked
          in via a fixed joint off "6th_wrist_joint" (so we plan for the tool
          frame directly, no manual offset math)
        - moveit_controllers.yaml pointing "arm_controller" at the Pi's
          FollowJointTrajectory action server
  * move_group + robot_state_publisher running (from that package's launch)
  * The Pi running arm_pi_node.py, reachable over the same ROS_DOMAIN_ID

LAUNCH: this script must be started with the MoveIt config parameters loaded
(robot_description, _semantic, kinematics.yaml, planning pipelines). The usual
way is a launch file built with MoveItConfigsBuilder that adds this file as a
Node — see the note at the bottom.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint
from std_msgs.msg import Int32

from moveit.planning import MoveItPy
# Not used directly, but importing it registers the pybind11 type caster for
# moveit::core::RobotModel -- without it, MoveItPy.get_robot_model() raises
# "TypeError: Unregistered type : moveit::core::RobotModel".
import moveit.core.robot_model  # noqa: F401

# ----------------------------------------------------------------------------
# Task definition  (mirrors pytorch_ik_to_robot.py)
# ----------------------------------------------------------------------------
ARM_GROUP = "arm"
BASE_FRAME = "base_assy"        # URDF root link (per Robot_URDF.srdf)
EE_FRAME = "ee"    # tool-tip frame (fixed joint off 6th_wrist_joint, baked
                    # into the URDF -- see ee_frame in robot.urdf).

TARGET_ORIENTATION_RPY = [math.pi/2, math.pi/2, -math.pi]   # roll, pitch, yaw
                    # ^ tuned against the old EE_FRAME="6th_wrist_joint".

# "ee"'s fixed rotation relative to "6th_wrist_joint" (the ee_frame joint's
# <origin rpy=...> in robot.urdf). TARGET_ORIENTATION_RPY above still
# describes the intended 6th_wrist_joint orientation; move_to_pose() composes
# it with this offset so pose goals against EE_FRAME="ee" keep that same
# physical wrist/gripper orientation instead of pointing "ee" itself there.
_EE_FRAME_OFFSET_RPY = (-1.5708, -2.9713e-15, 1.5708)

# [x, y, z] in metres, same waypoints as before.To answer the question, move_group will run on the current development machine and it will talk to the pi over ros 2. Can you implement these changes, but make a new python file that contains those changes for both the pi and the planning host? Also am I correct in interpreting the planning host stream the angles to the pi in real time, and would it make sense to send all the angles at once?

WAYPOINT_APPROACH = [-0.250, -0.040, 0.175]   # above bottle, gripper open
                    # ^ y shifted +0.06m from the original -0.100: now that
                    # "ee" models the true tool-tip offset (previously
                    # unmodeled), the original xyz was just outside reach at
                    # the required orientation. Verified this variant plans;
                    # re-check it still lines up with the bottle physically.
WAYPOINT_PICKUP   = [-0.250,  0.150, 0.175]   # at bottle
WAYPOINT_LIFT     = [-0.250,  0.150, 0.500]   # straight up
WAYPOINT_OVER_POT = [-0.250,  0.800, 0.500]   # over the plant pot

GRIPPER_OPEN = 0
GRIPPER_CLOSE = 100
GRIPPER_STOP = -1

POUR_JOINT_NAME = "6th"        # wrist joint to rotate for the pour
POUR_JOINT_INDEX = 5           # its index within the "arm" group's joint vector
POUR_ANGLE_RAD = math.radians(90)


def rpy_to_quat(roll, pitch, yaw):
    """Return (x, y, z, w)."""
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    w = cr*cp*cy + sr*sp*sy
    return x, y, z, w


def quat_mult(q1, q2):
    """Hamilton product q1 (x,y,z,w) * q2 (x,y,z,w) -> (x,y,z,w).

    Composes rotations the same way matrix multiplication does: applying the
    result to a vector is equivalent to applying q2 first, then q1.
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


class PickAndPour:
    def __init__(self):
        self.logger = get_logger("pick_and_pour")
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component(ARM_GROUP)
        self.robot_model = self.moveit.get_robot_model()

        # Small side node just to command the gripper (not a MoveIt joint).
        self.gripper_node = Node("gripper_commander")
        self.gripper_pub = self.gripper_node.create_publisher(
            Int32, "/gripper_command", 10)

    # -- helpers ------------------------------------------------------------
    def gripper(self, value):
        msg = Int32()
        msg.data = value
        self.gripper_pub.publish(msg)
        self.logger.info(f"Gripper command: {value}")
        time.sleep(1.0)   # give the servos time to actuate

    def _plan_and_execute(self, straight_line=False):
        """Plan from the current state to the already-set goal, then execute.

        Default: OMPL, via the node's top-level `plan_request_params`.
        straight_line: Pilz LIN, via the `pilz_lin` param namespace.
        Both param blocks are supplied by pick_and_pour.launch.py.
        """
        self.arm.set_start_state_to_current_state()
        if straight_line:
            from moveit.planning import PlanRequestParameters
            params = PlanRequestParameters(self.moveit, "pilz_lin")
            plan = self.arm.plan(single_plan_parameters=params)
        else:
            plan = self.arm.plan()

        if not plan:
            self.logger.error("Planning FAILED — aborting task.")
            return False
        self.moveit.execute(plan.trajectory, controllers=[])
        return True

    def move_to_pose(self, xyz, straight_line=False):
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        qx, qy, qz, qw = quat_mult(
            rpy_to_quat(*TARGET_ORIENTATION_RPY),
            rpy_to_quat(*_EE_FRAME_OFFSET_RPY))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link=EE_FRAME)

        return self._plan_and_execute(straight_line=straight_line)

    def pour(self):
        """Rotate the wrist +-90 deg in joint space, then rotate back.

        The wrist ("6th") joint isn't continuous -- it only has ~350 deg of
        range (see robot.urdf), so a blind +90 deg offset can walk past its
        upper limit depending on where the arm happens to be for
        WAYPOINT_OVER_POT. Pick whichever direction actually stays in bounds;
        pouring works the same either way.
        """
        # Read current joint values for the group.
        psm = self.moveit.get_planning_scene_monitor()
        with psm.read_only() as scene:
            current = scene.current_state.get_joint_group_positions(ARM_GROUP)

        jmg = self.robot_model.get_joint_model_group(ARM_GROUP)
        names = jmg.active_joint_model_names
        bounds = jmg.active_joint_model_bounds
        idx = names.index(POUR_JOINT_NAME)
        # active_joint_model_bounds is a list of per-variable-bounds lists
        # (one entry per joint, itself a list of VariableBounds -- multiple
        # for e.g. a floating joint). Our joints are revolute: 1 variable each.
        lower = bounds[idx][0].min_position
        upper = bounds[idx][0].max_position

        margin = 1e-3
        current_angle = current[POUR_JOINT_INDEX]
        if current_angle + POUR_ANGLE_RAD <= upper - margin:
            target_angle = current_angle + POUR_ANGLE_RAD
        elif current_angle - POUR_ANGLE_RAD >= lower + margin:
            target_angle = current_angle - POUR_ANGLE_RAD
        else:
            self.logger.error(
                f"Pour aborted: neither +/-{POUR_ANGLE_RAD:.3f} rad from "
                f"current {POUR_JOINT_NAME} angle {current_angle:.3f} fits "
                f"within joint bounds [{lower:.3f}, {upper:.3f}].")
            return False

        self.logger.info(
            f"Pour: {POUR_JOINT_NAME} {current_angle:.3f} -> {target_angle:.3f} "
            f"rad (bounds [{lower:.3f}, {upper:.3f}]).")

        poured = list(current)
        poured[POUR_JOINT_INDEX] = target_angle

        self.arm.set_goal_state(
            motion_plan_constraints=[self._joint_goal(names, poured)])
        if not self._plan_and_execute():
            return False

        # Rotate back.
        self.arm.set_goal_state(
            motion_plan_constraints=[self._joint_goal(names, list(current))])
        return self._plan_and_execute()

    @staticmethod
    def _joint_goal(joint_names, positions, tolerance=1e-3):
        """Build a joint-space goal as Constraints/JointConstraint messages.

        RobotState.set_joint_group_positions() segfaults in this moveit_py
        build (confirmed pre-existing ABI-level bug, reproduces even on an
        unmodified URDF/robot model -- not specific to this robot). The
        message-based goal API (PlanningComponent.set_goal_state(
        motion_plan_constraints=...)) doesn't go through that binding, so use
        it instead of building a RobotState goal.
        """
        constraints = Constraints()
        for name, position in zip(joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    # -- the task -----------------------------------------------------------
    def run(self):
        self.logger.info("Starting pick-and-pour.")

        # NOTE: these were Pilz LIN (straight_line=True) moves, but the LIN
        # Cartesian path crosses a near-singular region that forces infeasible
        # 4th-joint acceleration, so LIN planning fails. We use OMPL joint-space
        # planning instead (as the original waypoint code effectively did). To
        # revisit true straight-line moves, avoid the singular region and/or
        # raise the joint acceleration limits, then set straight_line=True.
        self.gripper(GRIPPER_OPEN)
        if not self.move_to_pose(WAYPOINT_APPROACH):
            return
        if not self.move_to_pose(WAYPOINT_PICKUP):
            return
        self.gripper(GRIPPER_CLOSE)

        if not self.move_to_pose(WAYPOINT_LIFT):
            return
        if not self.move_to_pose(WAYPOINT_OVER_POT):
            return

        # Pour and return.
        if not self.pour():
            return
        self.gripper(GRIPPER_STOP)

        self.logger.info("Pick-and-pour complete.")


def main():
    rclpy.init()
    task = PickAndPour()
    try:
        task.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# ---------------------------------------------------------------------------
# Example launch file (place in your moveit_config package, e.g.
#   launch/pick_and_pour.launch.py):
#
#   from moveit_configs_utils import MoveItConfigsBuilder
#   from launch import LaunchDescription
#   from launch_ros.actions import Node
#
#   def generate_launch_description():
#       moveit_config = (
#           MoveItConfigsBuilder("robot", package_name="robot_moveit_config")
#           .to_moveit_configs()
#       )
#       return LaunchDescription([
#           Node(
#               package="robot_moveit_config",  # or wherever you install this
#               executable="moveit_pick_and_pour.py",
#               output="screen",
#               parameters=[moveit_config.to_dict()],
#           ),
#       ])
#
# move_group + robot_state_publisher come from the package's own
# demo/move_group launch; run that first (with the real controllers, not the
# fake ones), then run this.
# ---------------------------------------------------------------------------
