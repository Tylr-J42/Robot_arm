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
        - a tool/EE link that already includes the -0.0095/0/0.06275 offset
          (so we plan for the tool frame directly, no manual offset math)
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
from std_msgs.msg import Int32

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

# ----------------------------------------------------------------------------
# Task definition  (mirrors pytorch_ik_to_robot.py)
# ----------------------------------------------------------------------------
ARM_GROUP = "arm"
BASE_FRAME = "base_assy"        # URDF root link (per Robot_URDF.srdf)
EE_FRAME = "6th_wrist_joint"    # tip link. NOTE: the -0.0095/0/0.06275 tool
                                # offset is NOT yet modeled — add a tool link to
                                # the URDF and point EE_FRAME at it when ready.

TARGET_ORIENTATION_RPY = [math.pi/2, math.pi/2, -math.pi]   # roll, pitch, yaw

# [x, y, z] in metres, same waypoints as before.To answer the question, move_group will run on the current development machine and it will talk to the pi over ros 2. Can you implement these changes, but make a new python file that contains those changes for both the pi and the planning host? Also am I correct in interpreting the planning host stream the angles to the pi in real time, and would it make sense to send all the angles at once?

WAYPOINT_APPROACH = [-0.250, -0.100, 0.175]   # above bottle, gripper open
WAYPOINT_PICKUP   = [-0.250,  0.150, 0.175]   # at bottle
WAYPOINT_LIFT     = [-0.250,  0.150, 0.500]   # straight up
WAYPOINT_OVER_POT = [-0.250,  0.800, 0.500]   # over the plant pot

GRIPPER_OPEN = 0
GRIPPER_CLOSE = 100
GRIPPER_STOP = -1

POUR_JOINT_INDEX = 5          # wrist joint to rotate for the pour
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

    def _plan_and_execute(self, pipeline=None, planner_id=None):
        """Plan from the current state to the already-set goal, then execute."""
        self.arm.set_start_state_to_current_state()
        if pipeline:
            # e.g. pipeline="pilz_industrial_motion_planner", planner_id="LIN"
            # for a straight-line Cartesian move. Requires the Pilz pipeline to
            # be enabled in your MoveIt config; otherwise use the default.
            plan = self.arm.plan(
                single_plan_parameters=self._plan_params(pipeline, planner_id))
        else:
            plan = self.arm.plan()

        if not plan:
            self.logger.error("Planning FAILED — aborting task.")
            return False
        self.moveit.execute(plan.trajectory, controllers=[])
        return True

    def _plan_params(self, pipeline, planner_id):
        from moveit.planning import PlanRequestParameters
        params = PlanRequestParameters(self.moveit, pipeline)
        if planner_id:
            params.planner_id = planner_id
        params.planning_pipeline = pipeline
        return params

    def move_to_pose(self, xyz, straight_line=False):
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        qx, qy, qz, qw = rpy_to_quat(*TARGET_ORIENTATION_RPY)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link=EE_FRAME)

        if straight_line:
            return self._plan_and_execute(
                pipeline="pilz_industrial_motion_planner", planner_id="LIN")
        return self._plan_and_execute()

    def pour(self):
        """Rotate the wrist +90 deg in joint space, then rotate back."""
        # Read current joint values for the group.
        psm = self.moveit.get_planning_scene_monitor()
        with psm.read_only() as scene:
            current = scene.current_state.get_joint_group_positions(ARM_GROUP)

        poured = list(current)
        poured[POUR_JOINT_INDEX] += POUR_ANGLE_RAD

        goal = RobotState(self.robot_model)
        goal.set_joint_group_positions(ARM_GROUP, poured)
        self.arm.set_goal_state(robot_state=goal)
        if not self._plan_and_execute():
            return False

        # Rotate back.
        goal.set_joint_group_positions(ARM_GROUP, list(current))
        self.arm.set_goal_state(robot_state=goal)
        return self._plan_and_execute()

    # -- the task -----------------------------------------------------------
    def run(self):
        self.logger.info("Starting pick-and-pour.")

        self.gripper(GRIPPER_OPEN)
        if not self.move_to_pose(WAYPOINT_APPROACH):
            return
        # Straight-line down onto the bottle.
        if not self.move_to_pose(WAYPOINT_PICKUP, straight_line=True):
            return
        self.gripper(GRIPPER_CLOSE)

        # Lift straight up, then translate over the pot.
        if not self.move_to_pose(WAYPOINT_LIFT, straight_line=True):
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
