#!/usr/bin/env python3
# file: pybullet_ik_to_robot.py

import pybullet as p
import pybullet_data
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
import threading
import os

class PyBulletIKNode(Node):
    def __init__(self):
        super().__init__('pybullet_ik_node')

        self.tool_down_orn = p.getQuaternionFromEuler([np.pi, 0.0, 0.0])   # 180° about X

        # Measured once in OnShape: centre of 6th_wrist_joint → gripper tip
        self.tcp_offset = np.array([0.0, 0.0, -0.152])   # ← change only if you re-measure

        self.waypoints = [
            [0.30, 0.45, 0.00],   # X, Y, Z in CAD
            [0.40, 0.50, 0.20],
            [0.35, 0.55, -0.15],
            [0.25, 0.45, 0.00],
        ]

        p.connect(p.GUI)
        p.setRealTimeSimulation(0)
        p.setGravity(0, 0, -9.81)
        camera_distance = 1.5
        camera_yaw = 45
        camera_pitch = -30
        p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, [0.3, 0, 0.5])

        urdf_path = "/home/tyler/Desktop/Robot_URDF/robot.urdf"
        if not os.path.isfile(urdf_path):
            self.get_logger().error(f"URDF not found: {urdf_path}")
            raise FileNotFoundError(urdf_path)

        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True)
        time.sleep(0.1)                         # let PyBullet finish loading

        self.joint_names = []
        self.joint_indices = []                 # PyBullet joint indices
        self.joint_lower = []
        self.joint_upper = []

        self.tcp_offset = np.array([0.0, 0.0, -0.152])

        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            name = info[1].decode('utf-8')
            jtype = info[2]
            if jtype == p.JOINT_REVOLUTE:
                self.joint_names.append(name)
                self.joint_indices.append(i)
                self.joint_lower.append(info[8])
                self.joint_upper.append(info[9])

        self.get_logger().info(f"Found {len(self.joint_names)} revolute joints: {self.joint_names}")

        # End-effector is the **last link** (6th_wrist_joint)
        self.ee_index = p.getNumJoints(self.robot_id) - 1
        ee_name = p.getJointInfo(self.robot_id, self.ee_index)[12].decode('utf-8')
        self.get_logger().info(f"End-effector link index {self.ee_index} ('{ee_name}')")

        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        self.current_joints = None               # will be filled by callback
        self.state_received = threading.Event()

        # ------------------------------------------------------------------
        # 4. Trajectory parameters (safe for your arm)
        # ------------------------------------------------------------------
        # Reach ≈ 0.8 m → keep targets inside a sphere of radius 0.6 m
        self.waypoints = [
            [0.30,  0.00, 0.45],   # close to home
            [0.40,  0.20, 0.50],
            [0.35, -0.15, 0.55],
            [0.25,  0.00, 0.45],
        ]
        self.time_per_waypoint = 3.0          # seconds

    def _joint_state_cb(self, msg: JointState):
        if self.current_joints is None:
            self.current_joints = [0.0] * len(self.joint_names)

        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.current_joints[idx] = msg.position[i]

        self.state_received.set()

    def _wait_for_state(self, timeout=5.0):
        return self.state_received.wait(timeout)

    def _solve_ik(self, xyz_urdf):
        # xyz_urdf = desired TIP position in world coordinates (CAD/world)
        desired_tip = np.array(xyz_urdf)

        # Get rotation matrix for the desired tool orientation (world <- tool)
        R_flat = p.getMatrixFromQuaternion(self.tool_down_orn)  # length-9 flat list
        R = np.array(R_flat).reshape(3, 3)

        # tcp_offset is measured from wrist joint -> gripper tip in the TOOL frame.
        # To get wrist (link) position in world coordinates:
        wrist_target = desired_tip - R.dot(self.tcp_offset)

        # Solve IK for the wrist link position and tool orientation
        q_all = p.calculateInverseKinematics(
            self.robot_id,
            self.ee_index,
            targetPosition=wrist_target.tolist(),
            targetOrientation=self.tool_down_orn,
            maxNumIterations=5000,
            residualThreshold=1e-7
        )

        q_rev = [q_all[i] for i in self.joint_indices]
        return np.clip(np.array(q_rev), np.array(self.joint_lower), np.array(self.joint_upper)).tolist()


    def build_trajectory(self):
        if not self._wait_for_state():
            self.get_logger().warn("No joint state received – starting from zero.")
            start_q = [0.0] * len(self.joint_names)
        else:
            start_q = self.current_joints.copy()

        traj = [start_q]
        times = [0.0]

        t = 0.0
        for i, pos in enumerate(self.waypoints):
            q = self._solve_ik(pos)
            if q is None:
                self.get_logger().error(f"Stopping trajectory at waypoint {i+1}")
                break
            traj.append(q)
            t += self.time_per_waypoint
            times.append(t)
            self.get_logger().info(
                f"Waypoint {i+1} → {pos}  joints(deg): {[round(np.degrees(v),1) for v in q[:6]]}"
            )
        return traj, times

    def publish_trajectory(self):
        points, times = self.build_trajectory()
        if len(points) < 2:
            self.get_logger().error("Trajectory too short – aborting.")
            return

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = []

        for q, t in zip(points, times):
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            msg.points.append(pt)

        self.get_logger().info(f"Publishing {len(msg.points)} points …")
        self.traj_pub.publish(msg)

        # Wait for execution + a little safety margin
        final_t = times[-1] + 1.0
        self.get_logger().info(f"Waiting {final_t:.1f} s for robot to finish …")
        time.sleep(final_t)

def main():
    rclpy.init()
    node = PyBulletIKNode()

    # Give ROS a moment to discover the topics
    time.sleep(2.0)

    try:
        node.publish_trajectory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        p.disconnect()

if __name__ == '__main__':
    main()