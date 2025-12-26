#!/usr/bin/env python3


import sys
from pathlib import Path
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from typing import List, Tuple
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
import threading

from urdf_parser_py.urdf import URDF

TIME_BETWEEN_POINTS = 1

START_JOINTS_RAD = [
    0.0*(np.pi/180.0),    # 1st
    -45.0*(np.pi/180.0),   # 2nd
    -90.0*(np.pi/180.0),  # 3rd
    0.0*(np.pi/180.0),    # 4th
    -96.0*(np.pi/180.0),  # 5th
    0.0*(np.pi/180.0)    # 6th
]

TARGET_POSITION = [0, -0.5, 0.5]
TARGET_ORIENTATION = [-np.pi/2, np.pi/2, -np.pi]

GRIPPER_OPEN = 0
GRIPPER_CLOSED = 100

WAYPOINT_1 = [250, 0, 250]
WAYPOINT_2 = [250, 0, 500]
WAYPOINT_3 = [250, -400, 500]

def load_urdf_joints(urdf_path: Path):
    robot = URDF.from_xml_file(str(urdf_path))
    outgoing = {j.parent for j in robot.joints if j.type != "fixed"}
    all_links = {l.name for l in robot.links}
    tip = (all_links - outgoing).pop()
    print(f"Tip link: {tip}")

    chain = []
    limits = []
    current = robot.get_root()
    while current != tip:
        joint = next((j for j in robot.joints if j.parent == current and j.type == "revolute"), None)
        if not joint:
            raise RuntimeError(f"No joint from {current}")

        lower = joint.limit.lower if joint.limit else -np.pi
        upper = joint.limit.upper if joint.limit else  np.pi
        limits.append((lower, upper))

        o = joint.origin
        chain.append((
            joint.name,
            joint.axis or [0, 0, 1],
            o.rotation or [0, 0, 0],
            o.position or [0, 0, 0]
        ))
        current = joint.child
    return chain, limits


def se3_exp(xi: torch.Tensor) -> torch.Tensor:
    rho, phi = xi[:3], xi[3:]
    theta = torch.norm(phi)
    device = xi.device

    if theta < 1e-8:
        T = torch.eye(4, device=device)
        skew = torch.zeros(3, 3, device=device)
        skew[0, 1] = -phi[2]; skew[0, 2] = phi[1]
        skew[1, 0] = phi[2];  skew[1, 2] = -phi[0]
        skew[2, 0] = -phi[1]; skew[2, 1] = phi[0]
        T[:3, :3] = torch.eye(3, device=device) + skew
        T[:3, 3] = rho
        return T

    a = phi / theta
    zeros = torch.zeros_like(a[0])

    a_cross = torch.stack([
        torch.stack([zeros, -a[2], a[1]]),
        torch.stack([a[2], zeros, -a[0]]),
        torch.stack([-a[1], a[0], zeros])
    ])

    sin_t, cos_t = torch.sin(theta), torch.cos(theta)
    eye = torch.eye(3, device=device)
    R = eye + sin_t * a_cross + (1 - cos_t) * (a_cross @ a_cross)

    V = eye + (1 - cos_t) / (theta ** 2) * a_cross + (theta - sin_t) / (theta ** 3) * (a_cross @ a_cross)
    t = V @ rho

    T = torch.eye(4, device=device)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

class TorchKinematics(nn.Module):
    def __init__(self, joint_data, limits, device, initial_q=None):
        super().__init__()
        self.n_dof = len(joint_data)
        if initial_q is None:
            self.q = nn.Parameter(torch.randn(self.n_dof, device=device) * 0.1)
        else:
            self.q = nn.Parameter(torch.tensor(initial_q, dtype=torch.float32, device=device))

        T_list = []
        axes_list = []
        for name, axis, rpy, xyz in joint_data:
            axis_t = torch.tensor(axis, dtype=torch.float32, device=device)
            r, p, y = [torch.tensor(v, dtype=torch.float32, device=device) for v in rpy]
            xyz_t = torch.tensor(xyz, dtype=torch.float32, device=device)

            cr, sr = torch.cos(r), torch.sin(r)
            cp, sp = torch.cos(p), torch.sin(p)
            cy, sy = torch.cos(y), torch.sin(y)
            ones = torch.ones_like(cr)
            zeros = torch.zeros_like(cr)

            Rx = torch.stack([ones, zeros, zeros, zeros, cr, -sr, zeros, sr, cr]).reshape(3, 3)
            Ry = torch.stack([cp, zeros, sp, zeros, ones, zeros, -sp, zeros, cp]).reshape(3, 3)
            Rz = torch.stack([cy, -sy, zeros, sy, cy, zeros, zeros, zeros, ones]).reshape(3, 3)
            R = Rz @ Ry @ Rx

            T = torch.eye(4, device=device)
            T[:3, :3] = R
            T[:3, 3] = xyz_t

            T_list.append(T)
            axes_list.append(axis_t)

        self.register_buffer('T_fixed', torch.stack(T_list))
        self.register_buffer('axes', torch.stack(axes_list))

        # ----- joint limits -----
        lower = torch.tensor([l[0] for l in limits], device=device)
        upper = torch.tensor([l[1] for l in limits], device=device)
        self.register_buffer('lower', lower)
        self.register_buffer('upper', upper)
        # -------------------------

    def forward(self):
        T = torch.eye(4, device=self.q.device)
        for i in range(self.n_dof):
            T = T @ self.T_fixed[i]
            xi = torch.cat([torch.zeros(3, device=self.q.device), self.axes[i] * self.q[i]])
            T = T @ se3_exp(xi)
        return T

def pose_error(pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    assert pred.shape == (4, 4) and target.shape == (4, 4)
    dp = target[:3, 3] - pred[:3, 3]

    R_err = target[:3, :3] @ pred[:3, :3].transpose(0, 1)
    trace = torch.trace(R_err)
    theta = torch.acos(torch.clamp((trace - 1) / 2, -1.0, 1.0))

    if theta < 1e-8:
        dphi = torch.zeros(3, device=pred.device)
    else:
        sin_t = torch.sin(theta)
        if sin_t < 1e-8:
            dphi = torch.zeros(3, device=pred.device)
        else:
            w = torch.stack([
                R_err[2,1] - R_err[1,2],
                R_err[0,2] - R_err[2,0],
                R_err[1,0] - R_err[0,1]
            ]) / (2 * sin_t)
            dphi = w * theta
    return torch.cat([dp, dphi])

def solve_ik(model, target_pose, max_steps=3000, lr=0.15):
    opt = optim.Adam(model.parameters(), lr=lr)
    for i in range(max_steps):
        opt.zero_grad()
        pred = model()
        err  = pose_error(pred, target_pose)
        loss = torch.sum(err[:3]**2) + 0.3 * torch.sum(err[3:]**2)
        loss.backward()
        opt.step()

        # ----- clamp to joint limits -----
        with torch.no_grad():
            model.q.clamp_(model.lower, model.upper)
        # ---------------------------------

        if i % 300 == 0 or i == max_steps - 1:
            print(f"Step {i:4d} | loss: {loss.item():.6f} | pos_err: {torch.norm(err[:3]).item():.5f}")
        if loss < 1e-7:
            print(f"Converged at step {i}")
            break
    return model.q.detach()

class Arm_Node(Node):
    
    def __init__(self):
        super().__init__('pytorch_ik_node')

        # ROS
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        self.joint_names = ['1st', '2nd', '3rd', '4th', '5th', '6th', 'gripper']

    def publish_trajectory(self, points, times):

        if len(points) < 2: return

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        for q, t in zip(points, times):
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t%1)*1e9))
            msg.points.append(pt)

        self.traj_pub.publish(msg)
        return

def points_along_line(p1, p2, num_points=10):

    if num_points < 1:
        raise ValueError("num_points must be at least 1")
    
    p1 = np.array(p1)
    p2 = np.array(p2)
    
    # Generate parameter t from 0 to 1
    t = np.linspace(0, 1, num_points)
    
    # Linear interpolation: points = p1 + t * (p2 - p1)
    points = p1 + t[:, np.newaxis] * (p2 - p1)
    
    return points.tolist()

def point_to_IK(target_pos, target_orientation, joints, limits, prev_solution):
    device = torch.device("cpu")

    # Target pose
    target_pos = torch.tensor(target_pos, device=device)
    target_rpy = torch.tensor(target_orientation, device=device)

    def rpy_to_mat(r, p, y):
        cr, sr = torch.cos(r), torch.sin(r)
        cp, sp = torch.cos(p), torch.sin(p)
        cy, sy = torch.cos(y), torch.sin(y)
        ones = torch.ones_like(cr)
        zeros = torch.zeros_like(cr)

        Rx = torch.stack([ones, zeros, zeros, zeros, cr, -sr, zeros, sr, cr]).reshape(3, 3)
        Ry = torch.stack([cp, zeros, sp, zeros, ones, zeros, -sp, zeros, cp]).reshape(3, 3)
        Rz = torch.stack([cy, -sy, zeros, sy, cy, zeros, zeros, zeros, ones]).reshape(3, 3)
        R = Rz @ Ry @ Rx
        T = torch.eye(4, device=r.device)
        T[:3, :3] = R
        T[:3, 3] = target_pos
        return T

    # start = time.perf_counter()
    target = rpy_to_mat(*target_rpy)

    # Adjust for EE offset (from robot_ee.urdf)
    offset_pos = torch.tensor([-0.0095, 0.0, 0.06275], device=device)
    offset_rpy = torch.tensor([0.0, 0.0, 0.0], device=device)#[-np.pi/2, 0.0, np.pi/2], device=device)
    T_offset = rpy_to_mat(*offset_rpy)
    T_offset[:3, 3] = offset_pos

    # Compute inverse of T_offset
    R_offset = T_offset[:3, :3]
    t_offset = T_offset[:3, 3]
    inv_R_offset = R_offset.transpose(0, 1)
    inv_t_offset = -inv_R_offset @ t_offset
    inv_T_offset = torch.eye(4, device=device)
    inv_T_offset[:3, :3] = inv_R_offset
    inv_T_offset[:3, 3] = inv_t_offset

    # Adjust target to account for offset (target_for_6th = target_ee @ inv_T_offset)
    target_adjusted = target @ inv_T_offset

    model = TorchKinematics(joints, limits, device, prev_solution)
    q_sol = solve_ik(model, target_adjusted)

    # print("\nIK Solution:")
    # for (name, _, _, _), q in zip(joints, q_sol.cpu().numpy()):
    #     print(f"  {name:8s}: {q:+.6f} rad ({np.degrees(q):+7.2f} degrees)")

    with torch.no_grad():
        pred = model()
    # print(f"\nFinal pose: {pred[:3,3].cpu().numpy()}")
    # print(f"Target:     {target[:3,3].cpu().numpy()}")
    # print(time.perf_counter()-start)

    joint_output = q_sol.cpu().numpy().tolist()
    # print(joint_output)
    return joint_output

def main():

    rclpy.init()
    arm_node = Arm_Node()

    urdf_path = "/home/tyler/Desktop/Robot_URDF/robot.urdf"

    print("Loading URDF...")
    joints, limits = load_urdf_joints(urdf_path)
    print(f"Found {len(joints)} joints: {[n for n, _, _, _ in joints]}")

    device = torch.device("cpu")
    print(f"Device: {device}")

    linear_waypoints = points_along_line([0.0, -0.5, 0.25], [0.0, -0.75, 0.25], 10)

    final_output = [START_JOINTS_RAD + [GRIPPER_OPEN]]
    time_to_goals = [0.0]

    prev_solution = START_JOINTS_RAD

    for i in range(len(linear_waypoints)):
        IK_solution = point_to_IK(linear_waypoints[i], TARGET_ORIENTATION, joints, limits, prev_solution)
        
        final_output.append(IK_solution + [GRIPPER_OPEN])
        if(i==0):
            time_to_goals.append(5)
        else:
            time_to_goals.append(TIME_BETWEEN_POINTS)
        prev_solution = IK_solution
    
    final_output.append(prev_solution + [GRIPPER_CLOSED])
    time_to_goals.append(2)

    arm_node.publish_trajectory(
        final_output,
        time_to_goals
    )

    print(np.degrees(final_output))
    


if __name__ == "__main__":
    main()