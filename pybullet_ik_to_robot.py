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

        # 1. TCP offset – 7 mm from wrist centre to tip
        self.tcp_offset = np.array([0.0, 0.0, 0.0])

        # 2. Gripper points straight down (-Z)
        self.tool_down_orn = p.getQuaternionFromEuler([np.pi, 0.0, 0.0])

        # 3. SET YOUR STARTING POSE (degrees)
        self.start_joints_deg = [
            0.0,    # 1st
            -45.0,   # 2nd
            -90.0,  # 3rd
            0.0,    # 4th
            -96.0,  # 5th
            0.0    # 6th
        ]

        # 4. Waypoints in URDF frame (Z-up, X-forward, Y-left)
        self.waypoints = [
            [0.30,  0.00, 0.45],
            [0.40,  0.20, 0.50],
            [0.35, -0.15, 0.55],
            [0.25,  0.00, 0.45],
        ]
        self.time_per_waypoint = 2.0

        p.connect(p.GUI)
        p.setRealTimeSimulation(0)
        p.setGravity(0, 0, -9.81)
        p.resetDebugVisualizerCamera(1.5, 45, -30, [0.3, 0, 0.5])

        # Load URDF
        urdf_path = "/home/tyler/Desktop/Robot_URDF/robot.urdf"
        if not os.path.isfile(urdf_path):
            self.get_logger().error(f"URDF not found: {urdf_path}")
            raise FileNotFoundError(urdf_path)

        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True)
        time.sleep(0.1)

        self.joint_names = []
        self.joint_indices = []
        self.joint_lower = []
        self.joint_upper = []
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            if info[2] == p.JOINT_REVOLUTE:
                self.joint_names.append(info[1].decode())
                self.joint_indices.append(i)
                self.joint_lower.append(info[8])
                self.joint_upper.append(info[9])

        self.get_logger().info(f"Found {len(self.joint_names)} revolute joints: {self.joint_names}")

        # End-effector
        self.ee_index = p.getNumJoints(self.robot_id) - 1
        ee_name = p.getJointInfo(self.robot_id, self.ee_index)[12].decode()
        self.get_logger().info(f"End-effector link index {self.ee_index} ('{ee_name}')")

        # ROS
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.state_sub = self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.current_joints = None
        self.state_received = threading.Event()

        # 5. Start visualization thread
        self.vis_thread = threading.Thread(target=self._visualize_loop, daemon=True)
        self.vis_thread.start()

    def _joint_state_cb(self, msg):
        if self.current_joints is None:
            self.current_joints = [0.0] * len(self.joint_names)
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joints[self.joint_names.index(name)] = msg.position[i]
        self.state_received.set()

    def _wait_for_state(self, timeout=5.0):
        return self.state_received.wait(timeout)

    def _solve_ik(self, xyz_tip_urdf):
        tip = np.array(xyz_tip_urdf)
        wrist_target = tip - self.tcp_offset

        q_all = p.calculateInverseKinematics(
            self.robot_id,
            self.ee_index,
            targetPosition=wrist_target.tolist(),
            targetOrientation=self.tool_down_orn,
            maxNumIterations=5000,
            residualThreshold=1e-7
        )

        q_rev = [q_all[i] for i in self.joint_indices]
        return np.clip(np.array(q_rev), self.joint_lower, self.joint_upper).tolist()

    def _set_robot_pose(self, joint_angles_rad):
        """Update PyBullet simulation to match joint angles"""
        for i, joint_idx in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, joint_idx, joint_angles_rad[i])
        # Step once to update visuals
        p.stepSimulation()
        time.sleep(0.01)  # Small delay for smooth motion

    def _visualize_loop(self):
        """Background thread: update PyBullet GUI"""
        rate = 120  # Hz
        dt = 1.0 / rate
        while rclpy.ok():
            p.stepSimulation()
            time.sleep(dt)

    def build_trajectory(self):
        # Use custom start pose
        start_q_deg = self.start_joints_deg
        if len(start_q_deg) != len(self.joint_names):
            self.get_logger().warn("start_joints_deg length mismatch! Using zero.")
            start_q = [0.0] * len(self.joint_names)
        else:
            start_q = [np.radians(deg) for deg in start_q_deg]
            start_q = np.clip(start_q, self.joint_lower, self.joint_upper).tolist()

        # Show start pose in GUI
        self._set_robot_pose(start_q)
        self.get_logger().info(f"Starting pose (deg): {[round(np.degrees(v),1) for v in start_q]}")

        traj = [start_q]
        times = [0.0]

        t = self.time_per_waypoint
        for i, pos in enumerate(self.waypoints):
            q = self._solve_ik(pos)
            traj.append(q)
   
            times.append(t)

            # Update GUI
            self._set_robot_pose(q)

            self.get_logger().info(
                f"Waypoint {i+1} → {pos}  joints(deg): {[round(np.degrees(v),1) for v in q[:6]]}"
            )
        return traj, times

    def publish_trajectory(self):
        points, times = self.build_trajectory()
        if len(points) < 2: return

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        for q, t in zip(points, times):
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t%1)*1e9))
            msg.points.append(pt)

        self.traj_pub.publish(msg)
        final_t = times[-1] + 1.0
        self.get_logger().info(f"Published {len(msg.points)} points. Waiting {final_t:.1f}s …")
        time.sleep(final_t)

def main():
    rclpy.init()
    node = PyBulletIKNode()
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