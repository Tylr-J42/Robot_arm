#!/usr/bin/env python3
"""
torch_inverse_kinematics.py

Inverse kinematics for the robot defined in robot.urdf using
differentiable forward kinematics in PyTorch + Adam optimisation.

Author:  ChatGPT (2025-11-09)
"""

import sys
from pathlib import Path
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from typing import List, Tuple

# ----------------------------------------------------------------------
# 1. URDF parsing (only revolute joints are kept)
# ----------------------------------------------------------------------
try:
    from urdf_parser_py.urdf import URDF
except ImportError:
    sys.exit("ERROR: Install urdf_parser_py: pip install urdf-parser-py")


def load_urdf_joints(urdf_path: Path):
    robot = URDF.from_xml_file(str(urdf_path))

    # ---- find the tip link (last link that has a child joint) ----
    tip = None
    for j in robot.joints:
        if j.type != "fixed":
            tip = j.child

    chain = []
    current = robot.get_root()
    while current != tip:
        # find the *next* revolute joint leaving this link
        joint = next((j for j in robot.joints
                      if j.parent == current and j.type == "revolute"), None)
        if joint is None:
            raise RuntimeError(f"Cannot reach tip {tip} from {current}")

        axis = torch.tensor(joint.axis or [0, 0, 1], dtype=torch.float32)
        o = joint.origin
        pos = torch.tensor(o.position or [0, 0, 0], dtype=torch.float32)
        rpy = torch.tensor(o.rotation or [0, 0, 0], dtype=torch.float32)

        R = rpy_to_rot(*rpy)                # you can reuse the helper above
        T = torch.eye(4)
        T[:3, :3] = R
        T[:3, 3] = pos

        chain.append((joint.name, axis, T))
        current = joint.child
    return chain

# ----------------------------------------------------------------------
# 2. Differentiable forward kinematics
# ----------------------------------------------------------------------
def se3_exp(xi: torch.Tensor) -> torch.Tensor:
    """
    Exponential map for se(3): xi = [ρ; φ]  (3×1 + 3×1)
    Returns 4×4 homogeneous transformation.
    """
    rho = xi[:3]
    phi = xi[3:]
    theta = torch.norm(phi)
    if theta < 1e-8:
        # small angle → linear approximation
        R = torch.eye(3, device=xi.device)
        V = torch.eye(3, device=xi.device)
    else:
        a = phi / theta
        a_cross = torch.tensor([[0, -a[2], a[1]],
                                [a[2], 0, -a[0]],
                                [-a[1], a[0], 0]], device=xi.device)
        R = (torch.eye(3, device=xi.device) +
             torch.sin(theta) * a_cross +
             (1 - torch.cos(theta)) * (a_cross @ a_cross))
        V = (torch.eye(3, device=xi.device) +
             (1 - torch.cos(theta)) / (theta ** 2) * a_cross +
             (theta - torch.sin(theta)) / (theta ** 3) * (a_cross @ a_cross))

    t = V @ rho
    T = torch.eye(4, device=xi.device)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


class TorchKinematics(nn.Module):
    def __init__(self, joint_data: List[Tuple[str, torch.Tensor, torch.Tensor]]):
        super().__init__()
        self.joint_data = joint_data                     # (name, axis, parent→joint)
        self.n_dof = len(joint_data)

        # Learnable joint angles (initialised at zero)
        self.q = nn.Parameter(torch.zeros(self.n_dof, dtype=torch.float32))

        # Register fixed transforms (parent→joint)
        self.register_buffer('T_fixed', torch.stack([T for _, _, T in joint_data]))

        # Joint axes (in child frame, will be rotated into parent frame later)
        self.register_buffer('axes', torch.stack([axis for _, axis, _ in joint_data]))

    def forward(self) -> torch.Tensor:
        """
        Returns the 4×4 homogeneous matrix of the end-effector w.r.t. base.
        """
        T = torch.eye(4, device=self.q.device)      # base frame
        for i in range(self.n_dof):
            # fixed transform up to the joint
            T = T @ self.T_fixed[i]

            # rotation around the joint axis
            theta = self.q[i]
            axis = self.axes[i]
            # twist = [0,0,0, axis] because pure rotation
            xi = torch.cat([torch.zeros(3, device=theta.device), axis * theta])
            T_joint = se3_exp(xi)
            T = T @ T_joint
        return T


# ----------------------------------------------------------------------
# 3. Pose error (position + orientation)
# ----------------------------------------------------------------------
def pose_error(pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    """
    pred, target: 4×4 homogeneous matrices.
    Returns a 6-vector [Δp; Δφ] where Δφ is the axis-angle error.
    """
    # position error
    dp = target[:3, 3] - pred[:3, 3]

    # rotation error (axis-angle)
    R_err = target[:3, :3] @ pred[:3, :3].T
    trace = torch.trace(R_err)
    theta = torch.acos(torch.clamp((trace - 1) / 2, -1.0, 1.0))
    if theta < 1e-8:
        dphi = torch.zeros(3, device=pred.device)
    else:
        # Rodrigues formula for axis
        w = torch.stack([R_err[2, 1] - R_err[1, 2],
                         R_err[0, 2] - R_err[2, 0],
                         R_err[1, 0] - R_err[0, 1]]) / (2 * torch.sin(theta))
        dphi = w * theta

    return torch.cat([dp, dphi])


# ----------------------------------------------------------------------
# 4. IK solver (Adam)
# ----------------------------------------------------------------------
def solve_ik_torch(model: TorchKinematics,
                   target_pose: torch.Tensor,
                   max_steps: int = 3000,
                   lr: float = 0.05,
                   pos_weight: float = 1.0,
                   rot_weight: float = 1.0,
                   verbose: bool = True) -> torch.Tensor:
    """
    Returns the joint angles (torch tensor) that minimise the pose error.
    """
    optimizer = optim.Adam(model.parameters(), lr=lr)
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=800, gamma=0.5)

    for step in range(max_steps):
        optimizer.zero_grad()
        pred = model()
        err = pose_error(pred, target_pose)
        loss = pos_weight * torch.sum(err[:3] ** 2) + rot_weight * torch.sum(err[3:] ** 2)
        loss.backward()
        optimizer.step()
        scheduler.step()

        if verbose and (step % 200 == 0 or step == max_steps - 1):
            pos_err = torch.norm(err[:3]).item()
            rot_err = torch.norm(err[3:]).item()
            print(f"Step {step:4d} | loss {loss.item():.6f} | "
                  f"pos_err {pos_err:.5f} m | rot_err {rot_err:.5f} rad")

        if loss.item() < 1e-10:
            break

    return model.q.detach().clone()


# ----------------------------------------------------------------------
# 5. Example / demo
# ----------------------------------------------------------------------
def main():
    urdf_path = Path("/home/tyler/Desktop/Robot_URDF/robot.urdf")
    if not urdf_path.is_file():
        sys.exit(f"ERROR: {urdf_path} not found")

    print("Parsing URDF ...")
    joints = load_urdf_joints(urdf_path)
    print(f"  → {len(joints)} revolute joints found:")
    for name, _, _ in joints:
        print(f"      {name}")

    # ------------------------------------------------------------------
    # Desired end-effector pose (base frame)
    # ------------------------------------------------------------------
    # Edit these values freely.
    target_pos = torch.tensor([0.50, 0.00, 0.25], dtype=torch.float32).to(device)
    target_rpy = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32).to(device)
    target_pose = rpy_to_matrix(*target_rpy)

    # Build homogeneous matrix
    def rpy_to_matrix(r: torch.Tensor, p: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        """Convert roll-pitch-yaw (rad) to a 4×4 SE(3) matrix."""
        cr, sr = torch.cos(r), torch.sin(r)
        cp, sp = torch.cos(p), torch.sin(p)
        cy, sy = torch.cos(y), torch.sin(y)

        # ---- Rx (roll) ----
        Rx = torch.stack([torch.ones_like(cr), torch.zeros_like(cr), torch.zeros_like(cr),
                        torch.zeros_like(cr), cr, -sr,
                        torch.zeros_like(cr), sr, cr]).reshape(3, 3)

        # ---- Ry (pitch) ----
        Ry = torch.stack([cp, torch.zeros_like(cp), sp,
                        torch.zeros_like(cp), torch.ones_like(cp), torch.zeros_like(cp),
                        -sp, torch.zeros_like(cp), cp]).reshape(3, 3)

        # ---- Rz (yaw) ----
        Rz = torch.stack([cy, -sy, torch.zeros_like(cy),
                        sy, cy, torch.zeros_like(cy),
                        torch.zeros_like(cy), torch.zeros_like(cy), torch.ones_like(cy)]).reshape(3, 3)

        R = Rz @ Ry @ Rx
        T = torch.eye(4, device=r.device)
        T[:3, :3] = R
        T[:3, 3] = target_pos          # target_pos is already a tensor on the right device
        return T

    target_pose = rpy_to_matrix(*target_rpy)

    # ------------------------------------------------------------------
    # Build the differentiable model
    # ------------------------------------------------------------------
    device = torch.device("rocm" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    model = TorchKinematics(joints).to(device)
    target_pose = target_pose.to(device)

    # ------------------------------------------------------------------
    # Solve IK
    # ------------------------------------------------------------------
    print("\nSolving inverse kinematics with Adam ...")
    q_sol = solve_ik_torch(model, target_pose,
                           max_steps=4000,
                           lr=0.08,
                           pos_weight=1.0,
                           rot_weight=0.5)

    # ------------------------------------------------------------------
    # Print results
    # ------------------------------------------------------------------
    print("\nIK solution (radians / degrees):")
    for (name, _, _), val in zip(joints, q_sol.cpu().numpy()):
        deg = np.degrees(val)
        print(f"  {name:12s} : {val: .6f} rad  ({deg: .2f}° )")

    # ------------------------------------------------------------------
    # Verify with forward kinematics
    # ------------------------------------------------------------------
    with torch.no_grad():
        pred = model()
    print("\nVerification (FK of solution):")
    print(f"  position : {pred[:3, 3].cpu().numpy()} m")
    rpy_pred = pred[:3, :3].cpu().numpy()
    from scipy.spatial.transform import Rotation as R
    euler = R.from_matrix(rpy_pred).as_euler('xyz')
    print(f"  RPY      : {euler} rad  ({np.degrees(euler)}°)")


if __name__ == "__main__":
    main()