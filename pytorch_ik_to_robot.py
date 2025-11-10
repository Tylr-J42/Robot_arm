#!/usr/bin/env python3


import sys
from pathlib import Path
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from typing import List, Tuple

from urdf_parser_py.urdf import URDF


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
    def __init__(self, joint_data, limits, device):
        super().__init__()
        self.n_dof = len(joint_data)
        self.q = nn.Parameter(torch.randn(self.n_dof, device=device) * 0.1)

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

def main():
    urdf_path = Path("/home/tyler/Desktop/Robot_URDF/robot.urdf")
    if not urdf_path.exists():
        sys.exit("URDF not found")

    print("Loading URDF...")
    joints, limits = load_urdf_joints(urdf_path)
    print(f"Found {len(joints)} joints: {[n for n, _, _, _ in joints]}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # Target pose
    target_pos = torch.tensor([0.5, 0.0, 0.5], device=device)
    target_rpy = torch.tensor([0.0, 0.0, 0.0], device=device)

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

    target = rpy_to_mat(*target_rpy)

    model = TorchKinematics(joints, limits, device)
    q_sol = solve_ik(model, target)

    print("\nIK Solution:")
    for (name, _, _, _), q in zip(joints, q_sol.cpu().numpy()):
        print(f"  {name:8s}: {q:+.6f} rad ({np.degrees(q):+7.2f} degrees)")

    with torch.no_grad():
        pred = model()
    print(f"\nFinal pose: {pred[:3,3].cpu().numpy()}")
    print(f"Target:     {target[:3,3].cpu().numpy()}")


if __name__ == "__main__":
    main()